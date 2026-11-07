#include "core/ZmcController.h"
#include "core/ScopeExit.h"
#include "core/MachineCalibration.h"
#include "core/TableRingRead.h"
#include "utils/GlobalTimeSync.h"
#include "utils/LogManager.h"
#include "calibration/Small7HeightPlan.h"

#include <QDebug>
#include <QThread>
#include <QElapsedTimer>
#include <QStringList>

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <limits>

#if defined(Q_OS_WIN) && !defined(MLS_PORTABLE_TRANSPORT_TEST)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#endif

namespace {

#if defined(Q_OS_WIN) && !defined(MLS_PORTABLE_TRANSPORT_TEST)

// ==================== 禾川绝对值伺服 RS485 配置 ====================
// IMPORTANT: keep the Win32 device path byte-for-byte equivalent to the
// standalone test program: "\\\\.\\COM3" in C++ source -> "\\.\COM3" at runtime.
// V4 accidentally lost one escape level ("\\.\COM3" in source), which can open
// the wrong Win32 path and then time out while waiting for a Modbus reply.
constexpr const char* kAbsServoPortName = "\\\\.\\COM3";
constexpr DWORD kAbsServoBaudRate = 9600;
constexpr int kAbsServoDataFormat = 0; // 0=8N2, 1=8E1, 2=8O1, 3=8N1

constexpr uint8_t kCServoSlaveId = 2;
constexpr uint8_t kBServoSlaveId = 1;

// 现场已与厂家确认：B/C 均为 23bit 绝对值编码器。
// 机械零点、减速比与方向统一集中到 MachineCalibration.h，
// 以后重新用千分表标定零位时只需修改该文件中的 kZeroCountsC / kZeroCountsB。
constexpr int kEncoderBitsC = MachineCalibration::kEncoderBits;
constexpr int kEncoderBitsB = MachineCalibration::kEncoderBits;
constexpr int64_t kEncoderCprC = MachineCalibration::kEncoderCpr;
constexpr int64_t kEncoderCprB = MachineCalibration::kEncoderCpr;

constexpr uint16_t kAddrAbsTurns = 0x1520;      // P21.32，多圈圈数，有符号32位
constexpr uint16_t kAddrSingleTurn = 0x1522;   // P21.34，单圈位置，无符号32位

constexpr int64_t kZeroCountsC = MachineCalibration::kZeroCountsC;
constexpr double kReducerRatioC = MachineCalibration::kReducerRatioC;
constexpr int kAbsDirectionC = MachineCalibration::kAbsoluteCountToControllerSignC;
constexpr int64_t kZeroCountsB = MachineCalibration::kZeroCountsB;
constexpr double kReducerRatioB = MachineCalibration::kReducerRatioB;
constexpr int kAbsDirectionB = MachineCalibration::kAbsoluteCountToControllerSignB;

constexpr int kAbsReadRetries = 3;
constexpr DWORD kAbsReadTimeoutMs = 1000;

uint16_t crc16Modbus(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void appendModbusCrc(std::vector<uint8_t>& frame)
{
    const uint16_t crc = crc16Modbus(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
}

std::string win32ErrorText(const char* prefix)
{
    return std::string(prefix) + ", GetLastError=" + std::to_string(GetLastError());
}

#if defined(_MSC_VER)
// Some vendor DLL builds may raise a Windows SEH exception instead of returning
// an error code on a failed/opening network path.  Keep the wrapper deliberately
// C-style (no local C++ objects that need unwinding) so MSVC permits __try/__except.
static bool safeZAuxOpenEth(char* ip, ZMC_HANDLE* handle, int32* retCode, DWORD* sehCode)
{
    if (retCode) *retCode = -1;
    if (sehCode) *sehCode = 0;
    if (handle) *handle = nullptr;

    __try {
        const int32 ret = ZAux_OpenEth(ip, handle);
        if (retCode) *retCode = ret;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER) {
        if (sehCode) *sehCode = GetExceptionCode();
        if (handle) *handle = nullptr;
        return false;
    }
}
#endif

QString formatHexFrame(const std::vector<uint8_t>& frame)
{
    QStringList parts;
    parts.reserve(static_cast<int>(frame.size()));
    for (const uint8_t b : frame) {
        parts.push_back(QStringLiteral("%1").arg(static_cast<unsigned int>(b), 2, 16, QLatin1Char('0')).toUpper());
    }
    return parts.join(QLatin1Char(' '));
}

class AbsServoSerialPort
{
public:
    AbsServoSerialPort(const std::string& portName, DWORD baudRate, int dataFormat)
    {
        // Keep this sequence identical to the user's standalone reader, which is
        // known to communicate successfully with the X3E drives.
        m_handle = CreateFileA(
            portName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );

        if (m_handle == INVALID_HANDLE_VALUE) {
            throw std::runtime_error(win32ErrorText("Open absolute-servo serial port failed"));
        }

        SetupComm(m_handle, 4096, 4096);
        PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);

        if (!GetCommState(m_handle, &dcb)) {
            throw std::runtime_error(win32ErrorText("GetCommState failed"));
        }

        dcb.BaudRate = baudRate;
        dcb.ByteSize = 8;
        dcb.fBinary = TRUE;

        switch (dataFormat) {
        case 0: // 8N2
            dcb.Parity = NOPARITY;
            dcb.fParity = FALSE;
            dcb.StopBits = TWOSTOPBITS;
            break;
        case 1: // 8E1
            dcb.Parity = EVENPARITY;
            dcb.fParity = TRUE;
            dcb.StopBits = ONESTOPBIT;
            break;
        case 2: // 8O1
            dcb.Parity = ODDPARITY;
            dcb.fParity = TRUE;
            dcb.StopBits = ONESTOPBIT;
            break;
        case 3: // 8N1
            dcb.Parity = NOPARITY;
            dcb.fParity = FALSE;
            dcb.StopBits = ONESTOPBIT;
            break;
        default:
            throw std::runtime_error("Unsupported absolute-servo serial data format");
        }

        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;

        if (!SetCommState(m_handle, &dcb)) {
            throw std::runtime_error(win32ErrorText("SetCommState failed"));
        }

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = 20;
        timeouts.ReadTotalTimeoutMultiplier = 2;
        timeouts.ReadTotalTimeoutConstant = 1000;
        timeouts.WriteTotalTimeoutMultiplier = 2;
        timeouts.WriteTotalTimeoutConstant = 1000;

        if (!SetCommTimeouts(m_handle, &timeouts)) {
            throw std::runtime_error(win32ErrorText("SetCommTimeouts failed"));
        }
    }

    ~AbsServoSerialPort()
    {
        if (m_handle != INVALID_HANDLE_VALUE) {
            CloseHandle(m_handle);
        }
    }

    void purge()
    {
        PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }

    void writeAll(const std::vector<uint8_t>& data)
    {
        DWORD written = 0;

        if (!WriteFile(m_handle, data.data(), static_cast<DWORD>(data.size()), &written, nullptr)) {
            throw std::runtime_error(win32ErrorText("Absolute-servo serial write failed"));
        }

        if (written != data.size()) {
            throw std::runtime_error("Absolute-servo serial write was incomplete");
        }

        FlushFileBuffers(m_handle);
    }

    std::vector<uint8_t> readExact(size_t n, DWORD timeoutMs = kAbsReadTimeoutMs)
    {
        std::vector<uint8_t> out;
        out.reserve(n);

        ULONGLONG start = GetTickCount64();

        while (out.size() < n) {
            uint8_t buf[128];
            DWORD want = static_cast<DWORD>(std::min<size_t>(sizeof(buf), n - out.size()));
            DWORD got = 0;

            if (!ReadFile(m_handle, buf, want, &got, nullptr)) {
                throw std::runtime_error(win32ErrorText("Absolute-servo serial read failed"));
            }

            if (got > 0) {
                out.insert(out.end(), buf, buf + got);
            } else {
                if (GetTickCount64() - start > timeoutMs) {
                    throw std::runtime_error("Absolute-servo serial read timeout");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }

        return out;
    }

private:
    HANDLE m_handle{INVALID_HANDLE_VALUE};
};

void checkModbusCrc(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 4) {
        throw std::runtime_error("Modbus response frame too short");
    }

    const uint16_t received =
        static_cast<uint16_t>(frame[frame.size() - 2]) |
        static_cast<uint16_t>(frame[frame.size() - 1] << 8);
    const uint16_t calculated = crc16Modbus(frame.data(), frame.size() - 2);

    if (received != calculated) {
        std::ostringstream oss;
        oss << "Modbus CRC check failed, received=0x"
            << std::hex << std::uppercase << received
            << ", calculated=0x" << calculated;
        throw std::runtime_error(oss.str());
    }
}

uint32_t modbusReadU32X3EOnce(AbsServoSerialPort& sp, uint8_t slaveId, uint16_t regAddr)
{
    std::vector<uint8_t> req;
    req.push_back(slaveId);
    req.push_back(0x03);
    req.push_back(static_cast<uint8_t>((regAddr >> 8) & 0xFF));
    req.push_back(static_cast<uint8_t>(regAddr & 0xFF));
    req.push_back(0x00);
    req.push_back(0x02);
    appendModbusCrc(req);

    sp.purge();
    LOG_INFO(LogCategory::Controller,
             QStringLiteral("RS485 TX slave=%1 reg=0x%2: %3")
                  .arg(slaveId)
                  .arg(regAddr, 4, 16, QLatin1Char('0'))
                  .arg(formatHexFrame(req)));
    sp.writeAll(req);

    std::vector<uint8_t> head = sp.readExact(3, kAbsReadTimeoutMs);

    if (head[0] != slaveId) {
        throw std::runtime_error("Modbus slave ID mismatch");
    }

    if (head[1] & 0x80) {
        std::vector<uint8_t> tail = sp.readExact(2, kAbsReadTimeoutMs);
        std::vector<uint8_t> resp = head;
        resp.insert(resp.end(), tail.begin(), tail.end());
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("RS485 RX: %1").arg(formatHexFrame(resp)));
        checkModbusCrc(resp);

        std::ostringstream oss;
        oss << "Modbus exception response, code=" << static_cast<int>(head[2]);
        throw std::runtime_error(oss.str());
    }

    if (head[1] != 0x03) {
        throw std::runtime_error("Modbus response function code is not 03");
    }

    const uint8_t byteCount = head[2];
    if (byteCount != 4) {
        throw std::runtime_error("Modbus response byte count is not 4 for a 32-bit parameter");
    }

    std::vector<uint8_t> tail = sp.readExact(byteCount + 2, kAbsReadTimeoutMs);
    std::vector<uint8_t> resp = head;
    resp.insert(resp.end(), tail.begin(), tail.end());
    LOG_INFO(LogCategory::Controller,
             QStringLiteral("RS485 RX: %1").arg(formatHexFrame(resp)));
    checkModbusCrc(resp);

    const uint16_t lowWord = static_cast<uint16_t>(
        (static_cast<uint16_t>(resp[3]) << 8) | resp[4]);
    const uint16_t highWord = static_cast<uint16_t>(
        (static_cast<uint16_t>(resp[5]) << 8) | resp[6]);

    return (static_cast<uint32_t>(highWord) << 16) | static_cast<uint32_t>(lowWord);
}

uint32_t modbusReadU32X3E(AbsServoSerialPort& sp, uint8_t slaveId, uint16_t regAddr)
{
    std::string lastError;
    for (int attempt = 1; attempt <= kAbsReadRetries; ++attempt) {
        try {
            return modbusReadU32X3EOnce(sp, slaveId, regAddr);
        } catch (const std::exception& e) {
            lastError = e.what();
            LOG_WARNING(LogCategory::Controller,
                        QStringLiteral("RS485 read retry %1/%2, slave=%3, reg=0x%4: %5")
                            .arg(attempt)
                            .arg(kAbsReadRetries)
                            .arg(slaveId)
                            .arg(regAddr, 4, 16, QLatin1Char('0'))
                            .arg(QString::fromLocal8Bit(e.what())));
            if (attempt < kAbsReadRetries) {
                sp.purge();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }

    std::ostringstream oss;
    oss << "Modbus read failed after " << kAbsReadRetries
        << " attempts (slave=" << static_cast<int>(slaveId)
        << ", reg=0x" << std::hex << std::uppercase << regAddr << "): " << lastError;
    throw std::runtime_error(oss.str());
}

struct AbsServoPosition
{
    int32_t turns = 0;
    uint32_t singleTurn = 0;
    int64_t totalCounts = 0;
    double axisDeg = 0.0;
};

AbsServoPosition readAbsServoPosition(AbsServoSerialPort& sp, uint8_t slaveId,
                                      int64_t zeroCounts, double reducerRatio, int direction,
                                      int encoderBits, int64_t encoderCpr)
{
    if (reducerRatio <= 0.0) {
        throw std::runtime_error("Rotary reducer ratio must be greater than zero");
    }
    if (direction != 1 && direction != -1) {
        throw std::runtime_error("Rotary absolute-position direction must be +1 or -1");
    }
    if (encoderBits <= 0 || encoderBits >= 63 || encoderCpr <= 0) {
        throw std::runtime_error("Invalid absolute-encoder resolution configuration");
    }

    // Keep the user's proven cross-turn consistency sequence:
    // turns1 -> single-turn -> turns2. Only combine the values when turns1==turns2.
    for (int retry = 0; retry < 3; ++retry) {
        const int32_t turns1 = static_cast<int32_t>(modbusReadU32X3E(sp, slaveId, kAddrAbsTurns));
        const uint32_t single = modbusReadU32X3E(sp, slaveId, kAddrSingleTurn);
        const int32_t turns2 = static_cast<int32_t>(modbusReadU32X3E(sp, slaveId, kAddrAbsTurns));

        if (turns1 == turns2) {
            if (single >= static_cast<uint64_t>(encoderCpr)) {
                std::ostringstream oss;
                oss << "P21.34 single-turn value exceeds configured " << encoderBits
                    << "-bit range, single=" << single << ", CPR=" << encoderCpr;
                throw std::runtime_error(oss.str());
            }

            AbsServoPosition pos;
            pos.turns = turns2;
            pos.singleTurn = single;
            pos.totalCounts = static_cast<int64_t>(turns2) * encoderCpr
                            + static_cast<int64_t>(single);
            // P21.32/P21.34 follow the servo encoder's native count direction.
            // Axis4/5 use ENCODER_RATIO(1,-1) in controller Basic, so controller MPOS/DPOS
            // use the opposite sign. Apply the same inversion here so the software machine
            // coordinate and controller coordinate increase in the same physical direction.
            pos.axisDeg = static_cast<double>(direction)
                        * static_cast<double>(pos.totalCounts - zeroCounts)
                        * 360.0 / static_cast<double>(encoderCpr) / reducerRatio;

            if (!std::isfinite(pos.axisDeg)) {
                throw std::runtime_error("Absolute-position conversion produced an invalid value");
            }
            return pos;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    throw std::runtime_error("Absolute-position read crossed a turn boundary and did not stabilize after retries");
}

#endif // Q_OS_WIN

QString jointAxisName(int axis)
{
    switch (axis) {
    case AxisX: return QStringLiteral("X");
    case AxisY: return QStringLiteral("Y");
    case AxisZ: return QStringLiteral("Z");
    case AxisC: return QStringLiteral("C");
    case AxisB: return QStringLiteral("B");
    default: return QStringLiteral("?");
    }
}

double jointValue(const JointPose5D& q, int axis)
{
    switch (axis) {
    case AxisX: return q.x;
    case AxisY: return q.y;
    case AxisZ: return q.z;
    case AxisC: return q.c;
    case AxisB: return q.b;
    default: return 0.0;
    }
}

void setJointValue(JointPose5D& q, int axis, double value)
{
    switch (axis) {
    case AxisX: q.x = value; break;
    case AxisY: q.y = value; break;
    case AxisZ: q.z = value; break;
    case AxisC: q.c = value; break;
    case AxisB: q.b = value; break;
    default: break;
    }
}

void updateMaximumValue(double& current, double candidate)
{
    if (candidate > current) current = candidate;
}

} // namespace

// ==================== 构造 / 析构 ====================

ZmcController::ZmcController(QObject* parent)
    : QObject(parent)
{
    // 初始化所有轴任务状态为空闲
    for (auto& task : m_axisTask) {
        task.store(AxisTaskNone);
    }
    for (auto& stop : m_stopRequested) {
        stop.store(false);
    }
    for (auto& referenced : m_linearMposReferenced) {
        referenced.store(false);
    }
    for (auto& generation : m_linearMposSetGeneration) {
        generation.store(0);
    }
    for (auto& ready : m_rotaryAbsReady) {
        ready.store(false);
    }
    for (auto& enabled : m_rotaryServoEnabled) {
        enabled.store(false);
    }
    for (auto& homed : m_rotaryHomeCompleted) {
        homed.store(false);
    }
    for (auto& offset : m_rotaryMachineOffset) {
        offset.store(0.0);
    }
    for (auto& limit : m_rotaryMachineSoftLimitPos) {
        limit.store(90.0);
    }
    for (auto& limit : m_rotaryMachineSoftLimitNeg) {
        limit.store(-90.0);
    }
}

ZmcController::~ZmcController()
{
    disconnect();
    m_io.shutdown(); // join before any state captured by queued work is destroyed
}

bool ZmcController::runIo(IoPriority priority, const char* name, std::function<bool()> fn,
                          QString* err, unsigned domain,
                          const eci::EciIoScheduler::Token* operationToken, unsigned invalidateDomains) const
{
    if (m_io.isWorkerThread()) return fn();
    if (m_lifecycleChanging.load()) {
        if (err) *err = QStringLiteral("控制卡连接切换中，请求未下发: %1").arg(QString::fromLatin1(name));
        return false;
    }
    const auto token = operationToken ? *operationToken : m_io.token();
    try {
        auto options = eci::EciIoScheduler::options(priority, name, domain);
        options.invalidateDomains = invalidateDomains;
        return m_io.invoke(options, token, std::move(fn));
    } catch (const std::exception& e) {
        if (err) *err = QStringLiteral("ECI I/O 请求未正常完成: %1").arg(QString::fromUtf8(e.what()));
        return false;
    }
}

bool ZmcController::runLifecycle(const char* name, std::function<bool()> fn, QString* err)
{
    auto options = eci::EciIoScheduler::options(IoPriority::Emergency, name);
    options.sessionBound = false;
    try { return m_io.invoke(options, m_io.token(), std::move(fn)); }
    catch (const std::exception& e) {
        if (err) *err = QStringLiteral("ECI 连接生命周期操作失败: %1").arg(QString::fromUtf8(e.what()));
        return false;
    }
}

bool ZmcController::postStop(const char* name, std::function<bool()> fn)
{
    auto options = eci::EciIoScheduler::options(IoPriority::Emergency, name);
    options.invalidateDomains = eci::EciIoScheduler::Motion | eci::EciIoScheduler::Acquisition;
    return m_io.post(options, m_io.token(), std::move(fn), [this](const std::string& e) {
        emit errorOccurred(QStringLiteral("软件停止请求失败/未执行: %1；请检查硬件停止状态")
                               .arg(QString::fromUtf8(e.c_str())));
    });
}
bool ZmcController::requestStopAxis(int axis, bool emergency)
{
    return postStop("STOP.axis", [this, axis, emergency] {
        QString err; const bool ok = stopAxis(axis, emergency, &err);
        if (!ok && !err.isEmpty()) emit errorOccurred(err);
        return ok;
    });
}
bool ZmcController::requestStopJointMove5D(bool emergency)
{
    return postStop("STOP.joint5D", [this, emergency] {
        QString err; const bool ok = stopJointMove5D(emergency, &err);
        if (!ok && !err.isEmpty()) emit errorOccurred(err);
        return ok;
    });
}
bool ZmcController::requestStopAllAxes(bool emergency)
{
    Q_UNUSED(emergency)
    return postStop("STOP.allAxes", [this] {
        p0::IoLock lock(m_ioMutex);
        m_mergePath.clear();m_mergeIndex=0;++m_mergeRevision;m_mergePaused=false;
        invalidateLinearTarget();
        if(!m_handle)return false;
        for(int axis=AxisX;axis<=AxisB;++axis)m_stopRequested[axis].store(true);
        int axes[5]={AxisX,AxisY,AxisZ,AxisC,AxisB};
        const int32 r=ZAux_Direct_CancelAxisList(m_handle,5,axes,2);
        if(r!=ERR_OK)emit errorOccurred(errorTextFromCode(r,"CancelAxisList(5)"));
        return r==ERR_OK;
    });
}


QString ZmcController::errorTextFromCode(int32 code, const char* func) const
{
    if (code == ERR_OK || code == ERR_SUCCESS) {
        return {};
    }
    return QStringLiteral("%1 调用失败，错误码 = %2")
        .arg(QString::fromLatin1(func))
        .arg(code);
}

int ZmcController::rotaryEnableOutputForAxis(int axis) const
{
    if (axis == AxisB) return Constants::kBServoEnableOutput;
    if (axis == AxisC) return Constants::kCServoEnableOutput;
    return -1;
}

bool ZmcController::isRotaryAbsoluteReady(int axis) const
{
    if (axis != AxisB && axis != AxisC) return true;
    return m_rotaryAbsReady[axis].load();
}

bool ZmcController::isRotaryServoEnabled(int axis) const
{
    if (axis != AxisB && axis != AxisC) return true;
    return m_rotaryServoEnabled[axis].load();
}

bool ZmcController::isLinearMposReferenced(int axis) const
{
    if (axis < AxisX || axis > AxisZ) return false;
    return m_linearMposReferenced[axis].load();
}

bool ZmcController::areLinearMposReferenced() const
{
    return m_linearMposReferenced[AxisX].load()
        && m_linearMposReferenced[AxisY].load()
        && m_linearMposReferenced[AxisZ].load();
}

bool ZmcController::isAxisHomed(int axis) const
{
    if (axis >= AxisX && axis <= AxisZ) return m_linearMposReferenced[axis].load();
    if (axis == AxisC || axis == AxisB) {
        // A persisted C/B Home bit is meaningful only after this host session has
        // re-established the absolute mechanical frame from RS485.
        return m_rotaryHomeCompleted[axis].load() && m_rotaryAbsReady[axis].load();
    }
    return false;
}

bool ZmcController::areAllAxesHomed() const
{
    return isAxisHomed(AxisX) && isAxisHomed(AxisY) && isAxisHomed(AxisZ)
        && isAxisHomed(AxisC) && isAxisHomed(AxisB);
}

bool ZmcController::readHardwareTriggerFrequencyHz(int& requestedHz, int& appliedHz, QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Diagnostic, "readHardwareTriggerFrequencyHz",
                     [&] { return readHardwareTriggerFrequencyHz(requestedHz, appliedHz, err); }, err);
    }
    p0::IoLock lock(m_ioMutex);
    requestedHz = appliedHz = 0;
    if (!m_handle) { if (err) *err = QStringLiteral("尚未连接控制器"); return false; }
    using T = mls::runtime::Table5D;
    float v[3] = {0,0,0};
    const int32 r = ZAux_Direct_GetTable(m_handle, T::base + T::runtimeCapabilityOffset, 3, v);
    if (r != ERR_OK) { if (err) *err = errorTextFromCode(r, "GetTable(runtime PWM config)"); return false; }
    if (!T::integer(v[0], T::runtimeCapabilityMagic, T::runtimeCapabilityMagic)) {
        if (err) *err = QStringLiteral("控制器BASIC不是V32.16 H1/H3协议；请下载controller/ECI3808_V32_16_H1_H3_ONLY.bas");
        return false;
    }
    if (!T::integer(v[1], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz)
        || !T::integer(v[2], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz)) {
        if (err) *err = QStringLiteral("控制器频率请求/ACK不是合法的25~2000Hz整数，拒绝读取");
        return false;
    }
    requestedHz = static_cast<int>(v[1]);
    appliedHz = static_cast<int>(v[2]);
    return true;
}

bool ZmcController::setHardwareTriggerFrequencyHz(int frequencyHz, QString* err)
{
    return setHardwareAcquisitionConfig(frequencyHz, mls::runtime::HardwareSyncTopology::EciPwmShared,
                                        mls::runtime::Table5D::h1R0Falling, err);
}

bool ZmcController::setHardwareAcquisitionConfig(int frequencyHz,
        mls::runtime::HardwareSyncTopology topology, int cmmR1Edge, QString* err)
{
    using T = mls::runtime::Table5D;
    const int selectedMode=static_cast<int>(topology);
    if(!T::validTopologyEdge(selectedMode,cmmR1Edge)){
        if(err)*err=QStringLiteral("非法硬件同步拓扑/锁存极性：mode=%1 edge=%2")
            .arg(selectedMode).arg(cmmR1Edge);
        return false;
    }
    if(m_io.isWorkerThread()){
        if(err)*err=QStringLiteral("频率配置握手必须由外部线程发起，禁止在I/O工作线程等待ACK");
        return false;
    }
    if(frequencyHz<T::minTriggerFrequencyHz || frequencyHz>T::maxTriggerFrequencyHz){
        if(err)*err=QStringLiteral("PWM频率必须在%1~%2Hz范围内")
            .arg(T::minTriggerFrequencyHz).arg(T::maxTriggerFrequencyHz);
        return false;
    }
    const auto token=m_io.token();
    int expectedSeq=0;
    if(!runIo(IoPriority::Control,"pwm.frequency.requestV3213",[&]{
        p0::IoLock lock(m_ioMutex);
        if(!m_handle){if(err)*err=QStringLiteral("控制器未连接");return false;}
        float v[T::runtimeStatusWords]={};
        int32 r=ZAux_Direct_GetTable(m_handle,T::base+T::runtimeCapabilityOffset,T::runtimeStatusWords,v);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetTable(PWM runtime state)");return false;}
        if(!T::integer(v[0],T::runtimeCapabilityMagic,T::runtimeCapabilityMagic)){
            if(err)*err=QStringLiteral("BASIC不是V32.16 H1/H3协议(TABLE[354])");return false;
        }
        float enable=0;
        r=ZAux_Direct_GetTable(m_handle,T::base+4,1,&enable);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetTable(PWM enable)");return false;}
        // Fail closed on corrupted table words BEFORE any float-to-int conversion.
        // A ConfigError may have an invalid requested Hz (the reason for the error),
        // but the BASIC-applied frequency, both sequences and state must remain sound.
        if (!T::integer(v[2], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz) ||
            !T::integer(v[3], 0, static_cast<int>(T::allAxesMask)) ||
            !T::integer(v[4], T::IdleReady, T::Fault) ||
            !T::integer(v[5], 0, T::maxRequestSequence) ||
            !T::integer(v[6], 0, T::maxRequestSequence) ||
            !T::integer(v[7], 0, 12) || !T::integer(enable, 0, 3)) {
            if (err) *err=QStringLiteral("BASIC PWM状态/序号/HomeMask损坏或非整数，拒绝提交新频率");
            return false;
        }
        const int state=static_cast<int>(v[4]);
        if(enable!=0 || m_hwSampling || (state!=T::IdleReady && state!=T::ConfigError)){
            if(err)*err=QStringLiteral("BASIC尚未真实IdleReady，禁止改频：EN=%1 state=%2")
                .arg(int(enable)).arg(state);
            return false;
        }
        if(state==T::IdleReady && v[5]!=v[6]){
            if(err)*err=QStringLiteral("上一请求尚未被ACK，不覆盖未完成PWM配置");return false;
        }
        if(!T::integer(v[5],0,T::maxRequestSequence)){
            if(err)*err=QStringLiteral("控制器频率请求序号非法");return false;
        }
        expectedSeq=int(v[5])+1;
        if(expectedSeq>T::maxRequestSequence)expectedSeq=1;
        float requested=float(frequencyHz),seq=float(expectedSeq);
        r=ZAux_Direct_SetTable(m_handle,T::base+T::requestedFrequencyHzOffset,1,&requested);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"SetTable(PWM Hz)");return false;}
        // Commit protocol: [355] Hz, [365..366] topology/edge, then [359] sequence LAST.
        // No BASIC hardware configuration is applied before the new sequence is visible.
        float cfg[2]={float(selectedMode),float(cmmR1Edge)};
        r=ZAux_Direct_SetTable(m_handle,T::topologyRequestIndex,2,cfg);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"SetTable(topology/edge)");return false;}
        r=ZAux_Direct_SetTable(m_handle,T::base+T::frequencyRequestSequenceOffset,1,&seq);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"SetTable(PWM config sequence)");return false;}
        return true;
    },err,eci::EciIoScheduler::Acquisition,&token))return false;

    // Poll from the caller thread. This keeps the ECI I/O worker available for
    // emergency STOP while BASIC finishes its asynchronous configuration change.
    QElapsedTimer timer;timer.start();
    while(timer.elapsed()<1200){
        bool done=false,rejected=false;
        if(!runIo(IoPriority::Diagnostic,"pwm.frequency.awaitAckV3213",[&]{
            p0::IoLock lock(m_ioMutex);
            if(!m_handle){if(err)*err=QStringLiteral("等待频率ACK时控制器已断开");return false;}
            float v[T::runtimeStatusWords]={};
            int32 r=ZAux_Direct_GetTable(m_handle,T::base+T::runtimeCapabilityOffset,T::runtimeStatusWords,v);
            if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetTable(PWM request ACK)");return false;}
            if (!T::integer(v[0], T::runtimeCapabilityMagic, T::runtimeCapabilityMagic) ||
                !T::integer(v[2], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz) ||
                !T::integer(v[3], 0, static_cast<int>(T::allAxesMask)) ||
                !T::integer(v[4], T::IdleReady, T::Fault) ||
                !T::integer(v[5], 0, T::maxRequestSequence) ||
                !T::integer(v[6], 0, T::maxRequestSequence) ||
                !T::integer(v[7], 0, 12)) {
                if (err) *err=QStringLiteral("等待PWM ACK时BASIC状态数据无效，拒绝确认频率");
                return false;
            }
            if(T::runtimeReady(v,frequencyHz,T::IdleReady) && v[5]==expectedSeq){
                float modes[4]={};
                r=ZAux_Direct_GetTable(m_handle,T::topologyRequestIndex,4,modes);
                if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetTable(topology ACK)");return false;}
                if(!T::modeReady(modes,selectedMode,cmmR1Edge)){
                    if(err)*err=QStringLiteral("拓扑ACK与Host请求不一致，拒绝启用硬件同步");
                    return false;
                }
                done=true;return true;
            }
            if(v[4]==T::ConfigError && v[5]==expectedSeq){
                if(err)*err=QStringLiteral("BASIC拒绝配置：Hz=%1 seq=%2 error=%3")
                    .arg(frequencyHz).arg(expectedSeq).arg(int(v[7]));
                rejected=true;
            } else if(v[4]==T::Fault){
                if(err)*err=QStringLiteral("控制器已发生硬件同步故障，PWM配置停止");
                rejected=true;
            }
            return true;
        },err,eci::EciIoScheduler::Acquisition,&token)) return false;
        if(rejected)return false;
        if(done){
            m_hwTriggerFrequencyHz.store(frequencyHz);
            m_hwTopology.store(selectedMode);
            m_hwCmmEdge.store(cmmR1Edge);
            LOG_INFO(LogCategory::Controller,
                QStringLiteral("V32.16 H1/H3 PWM配置ACK完成: %1Hz seq=%2 period=%3ms")
                    .arg(frequencyHz).arg(expectedSeq)
                    .arg(1000.0/static_cast<double>(frequencyHz),0,'f',6));
            return true;
        }
        QThread::msleep(2);
    }
    if(err)*err=QStringLiteral("V32.13 PWM频率ACK超时：freq=%1Hz seq=%2，不允许启动")
        .arg(frequencyHz).arg(expectedSeq);
    return false;
}

bool ZmcController::hwTableWaitIdleReady(int timeoutMs, QString* err)
{
    if(m_io.isWorkerThread()){
        if(err)*err=QStringLiteral("等待BASIC真实空闲状态不能阻塞I/O工作线程");
        return false;
    }
    using T=mls::runtime::Table5D;
    QElapsedTimer timer;timer.start();
    while(timer.elapsed()<timeoutMs){
        int state=-1,configError=-1,enable=-1,fault=-1;
        bool ok=runIo(IoPriority::Measurement,"table.awaitActualIdle",[&]{
            p0::IoLock lock(m_ioMutex);
            if(!m_handle){if(err)*err=QStringLiteral("控制器已断开");return false;}
            float h[5]={},v[T::runtimeStatusWords]={};
            int32 r=ZAux_Direct_GetTable(m_handle,T::base,5,h);
            if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetTable(stop header)");return false;}
            r=ZAux_Direct_GetTable(m_handle,T::base+T::runtimeCapabilityOffset,T::runtimeStatusWords,v);
            if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetTable(stop actual state)");return false;}
            if (!T::integer(v[0], T::runtimeCapabilityMagic, T::runtimeCapabilityMagic) ||
                !T::integer(h[3], 0, T::maxOrdinal) || !T::integer(h[4], 0, 3) ||
                !T::integer(v[2], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz) ||
                !T::integer(v[3], 0, static_cast<int>(T::allAxesMask)) ||
                !T::integer(v[4], T::IdleReady, T::Fault) ||
                !T::integer(v[5], 0, T::maxRequestSequence) ||
                !T::integer(v[6], 0, T::maxRequestSequence) ||
                !T::integer(v[7], 0, 12)) {
                if (err) *err=QStringLiteral("等待真实停机状态时TABLE协议数据非整数、越界或版本不匹配");
                return false;
            }
            enable=static_cast<int>(h[4]);fault=static_cast<int>(h[3]);
            state=static_cast<int>(v[4]);configError=static_cast<int>(v[7]);
            return true;
        },err);
        if(!ok)return false;
        if(state==T::IdleReady && enable==0 && fault==0 && configError==0)return true;
        if(state==T::Fault || state==T::ConfigError || fault!=0 || configError!=0){
            if(err)*err=QStringLiteral("BASIC未能安全停止：state=%1 fault=%2 cfgErr=%3")
                .arg(state).arg(fault).arg(configError);return false;
        }
        QThread::msleep(5);
    }
    if(err)*err=QStringLiteral("等待PWM关闭/REGIST Pump完成并进入IdleReady超时");
    return false;
}

bool ZmcController::restoreMachineHomeStateFromController(QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Diagnostic, "restoreMachineHomeState",
                     [&] { return restoreMachineHomeStateFromController(err); }, err);
    }
    p0::IoLock lock(m_ioMutex);
    using T = mls::runtime::Table5D;
    for (int axis = AxisX; axis <= AxisZ; ++axis) m_linearMposReferenced[axis].store(false);
    m_rotaryHomeCompleted[AxisC].store(false);
    m_rotaryHomeCompleted[AxisB].store(false);
    if (!m_handle) { if (err) *err = QStringLiteral("尚未连接控制器"); return false; }

    float v[4] = {0,0,0,0};
    int32 r = ZAux_Direct_GetTable(m_handle, T::base + T::runtimeCapabilityOffset, 4, v);
    if (r != ERR_OK) { if (err) *err = errorTextFromCode(r, "GetTable(five-axis home state)"); return false; }
    if (!T::integer(v[0], T::runtimeCapabilityMagic, T::runtimeCapabilityMagic)) {
        if (err) *err = QStringLiteral("控制器未运行V32.13 BASIC，不能恢复XYZBC回零状态；按未回零处理");
        return false;
    }
    if (!T::integer(v[3], 0, static_cast<int>(T::allAxesMask))) {
        if (err) *err = QStringLiteral("控制器Home mask不为0..31内的整数，按未回零处理: %1").arg(v[3]);
        return false;
    }
    const int mask = static_cast<int>(v[3]);

    // V27.6.55 reconnect fix: TABLE[357] is the explicitly committed reference
    // for THIS BASIC power session (set only after successful DATUM + SetMpos).
    // GetHomeStatus is a DATUM operation-result indicator, not an independently
    // proved durable coordinate reference after other axis commands.  Therefore
    // it is diagnostic only during reconnect; it must never clear TABLE[357].
    // Refuse local recovery if current MPOS, axis status or idle state is unsafe.
    // A temporary read/axis fault must not destroy the controller-resident bit:
    // the operator can remedy the fault and reconnect without a needless DATUM.
    int verifiedMask = mask & ((1 << (AxisC - AxisX)) | (1 << (AxisB - AxisX)));
    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        const int bit = 1 << (axis - AxisX);
        if ((mask & bit) == 0) continue;
        uint32 homeStatus = 0;
        float mpos = 0.0f;
        int axisStatus = 0;
        int idle = 0;
        const int32 hr = ZAux_Direct_GetHomeStatus(m_handle, static_cast<uint32>(axis), &homeStatus);
        const int32 mr = ZAux_Direct_GetMpos(m_handle, axis, &mpos);
        const int32 sr = ZAux_Direct_GetAxisStatus(m_handle, axis, &axisStatus);
        const int32 ir = ZAux_Direct_GetIfIdle(m_handle, axis, &idle);
        const bool valid = mr == ERR_OK && std::isfinite(mpos)
            && sr == ERR_OK && axisStatus == 0
            && ir == ERR_OK && idle != 0;
        m_linearMposReferenced[axis].store(valid);
        if (valid) verifiedMask |= bit;
        if (hr != ERR_OK || homeStatus != 1u) {
            LOG_INFO(LogCategory::Controller,
                QStringLiteral("%1重连诊断：历史HOME_STATUS返回 code=%2 value=%3；"
                               "该值不用于撤销TABLE[357]的上电会话Home认证")
                    .arg(jointAxisName(axis)).arg(hr).arg(homeStatus));
        }
        if (!valid) {
            LOG_WARNING(LogCategory::Controller,
                QStringLiteral("%1重连安全检查未通过：MPOS(code=%2,value=%3) "
                               "AXISSTATUS(code=%4,value=%5) IDLE(code=%6,value=%7)；"
                               "本地禁止该轴运动，控制器TABLE[357]不修改")
                    .arg(jointAxisName(axis)).arg(mr).arg(mpos)
                    .arg(sr).arg(axisStatus).arg(ir).arg(idle));
        }
    }

    // C/B bits are effective only after this host session rebuilds the RS485
    // absolute reference. Restore their committed history, not readiness.
    const int cBit = 1 << (AxisC - AxisX);
    const int bBit = 1 << (AxisB - AxisX);
    m_rotaryHomeCompleted[AxisC].store((mask & cBit) != 0);
    m_rotaryHomeCompleted[AxisB].store((mask & bBit) != 0);

    // Never auto-write a downgraded mask during connect. Transient SDK reads,
    // HOME_STATUS and active limit indications are not evidence that the prior
    // successful DATUM + SetMpos commit should be permanently deleted.
    if (verifiedMask != mask) {
        if (err) *err = QStringLiteral(
            "部分XYZ轴未通过重连安全检查：controller mask=%1, currently verified=%2；"
            "未改写控制器Home mask。请检查轴状态/MPOS后重连，勿直接自动运动")
                .arg(mask).arg(verifiedMask);
        return false;
    }

    LOG_INFO(LogCategory::Controller,
             QStringLiteral("V32.13控制器上电会话五轴回零状态恢复: mask=%1 verified=%2 "
                            "(X=%3 Y=%4 Z=%5 Cbit=%6 Bbit=%7；C/B需absolute-ready后才生效)")
                 .arg(mask).arg(verifiedMask)
                 .arg(m_linearMposReferenced[AxisX].load()?1:0)
                 .arg(m_linearMposReferenced[AxisY].load()?1:0)
                 .arg(m_linearMposReferenced[AxisZ].load()?1:0)
                 .arg(m_rotaryHomeCompleted[AxisC].load()?1:0)
                 .arg(m_rotaryHomeCompleted[AxisB].load()?1:0));
    return true;
}

bool ZmcController::updateControllerHomeMaskBit(int axis, bool homed, QString* err)
{
    if (axis < AxisX || axis > AxisB) {
        if (err) *err = QStringLiteral("五轴Home状态轴号非法: %1").arg(axis);
        return false;
    }
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "updateFiveAxisHomeMask",
                     [&] { return updateControllerHomeMaskBit(axis, homed, err); }, err);
    }
    p0::IoLock lock(m_ioMutex);
    using T = mls::runtime::Table5D;
    if (!m_handle) { if (err) *err = QStringLiteral("尚未连接控制器"); return false; }
    float v[4] = {0,0,0,0};
    int32 r = ZAux_Direct_GetTable(m_handle, T::base + T::runtimeCapabilityOffset, 4, v);
    if (r != ERR_OK) { if (err) *err = errorTextFromCode(r, "GetTable(five-axis home mask)"); return false; }
    if (!T::integer(v[0], T::runtimeCapabilityMagic, T::runtimeCapabilityMagic)) {
        if (err) *err = QStringLiteral("控制器未运行V32.13 BASIC，不能安全维护XYZBC回零状态");
        return false;
    }
    if (!T::integer(v[3], 0, static_cast<int>(T::allAxesMask))) {
        if (err) *err = QStringLiteral("控制器Home mask非法，拒绝依据损坏状态改写: %1").arg(v[3]);
        return false;
    }
    int mask = static_cast<int>(v[3]);
    const int bit = 1 << (axis - AxisX);
    mask = homed ? (mask | bit) : (mask & ~bit);
    float updated = static_cast<float>(mask);
    r = ZAux_Direct_SetTable(m_handle, T::base + T::machineHomeMaskOffset, 1, &updated);
    if (r != ERR_OK) { if (err) *err = errorTextFromCode(r, "SetTable(five-axis home mask)"); return false; }
    float readback = -1.0f;
    r = ZAux_Direct_GetTable(m_handle, T::base + T::machineHomeMaskOffset, 1, &readback);
    if (r != ERR_OK) { if (err) *err = errorTextFromCode(r, "GetTable(five-axis home mask readback)"); return false; }
    if (readback != updated) {
        if (err) *err = QStringLiteral("控制器Home mask写入后回读不一致，拒绝继续回零/参数更改: 期望=%1 实际=%2")
            .arg(mask).arg(readback);
        return false;
    }
    return true;
}

std::uint64_t ZmcController::linearMposSetGeneration(int axis) const
{
    if (axis < AxisX || axis > AxisZ) return 0;
    return m_linearMposSetGeneration[axis].load();
}

double ZmcController::rotaryMachineOffset(int axis) const
{
    if (axis != AxisB && axis != AxisC) return 0.0;
    return m_rotaryMachineOffset[axis].load();
}

float ZmcController::controllerToMachinePosition(int axis, float controllerPos) const
{
    if ((axis == AxisB || axis == AxisC) && m_rotaryAbsReady[axis].load()) {
        return controllerPos + static_cast<float>(m_rotaryMachineOffset[axis].load());
    }
    return controllerPos;
}

float ZmcController::machineToControllerPosition(int axis, float machinePos) const
{
    if (axis == AxisB || axis == AxisC) {
        return machinePos - static_cast<float>(m_rotaryMachineOffset[axis].load());
    }
    return machinePos;
}

bool ZmcController::readAxisEncoder(int axis, float& encoder, QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "readAxisEncoder", [&] { return readAxisEncoder(axis, encoder, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    const int32 ret = ZAux_Direct_GetEncoder(m_handle, axis, &encoder);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "GetEncoder");
        return false;
    }
    return true;
}

bool ZmcController::getAxisPositionPairRaw(int axis, float& dpos, float& mpos,
                                            bool& idle, QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "getAxisPositionPairRaw", [&] { return getAxisPositionPairRaw(axis, dpos, mpos, idle, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    int32 ret = ERR_OK;
    // V22: B/C mechanical pose is encoder MPOS only.  Do not read DPOS(C/B) in
    // upper-level state paths; keep the legacy dpos output equal to MPOS so old
    // display/struct APIs cannot accidentally reintroduce command-position pose.
    if (axis == AxisC || axis == AxisB) {
        ret = ZAux_Direct_GetMpos(m_handle, axis, &mpos);
        dpos = mpos;
    } else {
        ret = ZAux_Direct_GetDpos(m_handle, axis, &dpos);
        if (ret == ERR_OK) ret = ZAux_Direct_GetMpos(m_handle, axis, &mpos);
    }
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, axis==AxisC||axis==AxisB ? "GetMpos(rotary)" : "GetDpos/GetMpos");
        return false;
    }
    int idleInt = 0;
    ret = ZAux_Direct_GetIfIdle(m_handle, axis, &idleInt);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "GetIfIdle");
        return false;
    }
    idle = (idleInt != 0);
    return true;
}

bool ZmcController::applyRotaryControllerSoftLimits(int axis, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "applyRotaryControllerSoftLimits", [&] { return applyRotaryControllerSoftLimits(axis, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (axis != AxisB && axis != AxisC) return true;
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    const double offset = m_rotaryMachineOffset[axis].load();
    const float machineFs = static_cast<float>(m_rotaryMachineSoftLimitPos[axis].load());
    const float machineRs = static_cast<float>(m_rotaryMachineSoftLimitNeg[axis].load());
    const float controllerFs = machineFs - static_cast<float>(offset);
    const float controllerRs = machineRs - static_cast<float>(offset);

    int32 ret = ZAux_Direct_SetFsLimit(m_handle, axis, controllerFs);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "SetFsLimit(rotary transformed)");
        return false;
    }
    ret = ZAux_Direct_SetRsLimit(m_handle, axis, controllerRs);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "SetRsLimit(rotary transformed)");
        return false;
    }

    LOG_INFO(LogCategory::Controller,
             QStringLiteral("%1 soft-limit frame: machine=[%2,%3], controller=[%4,%5], offset=%6")
                 .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                 .arg(machineRs, 0, 'f', 4).arg(machineFs, 0, 'f', 4)
                 .arg(controllerRs, 0, 'f', 4).arg(controllerFs, 0, 'f', 4)
                 .arg(offset, 0, 'f', 6));
    return true;
}

bool ZmcController::checkRotaryMotionAllowed(int axis, QString* err) const
{
    if (axis != AxisB && axis != AxisC) return true;

    if (!m_rotaryAbsReady[axis].load()) {
        if (err) {
            *err = QStringLiteral("%1轴尚未从禾川伺服读取并建立机械绝对坐标框架，禁止运动")
                .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"));
        }
        return false;
    }

    if (!m_rotaryServoEnabled[axis].load()) {
        if (err) {
            *err = QStringLiteral("%1轴尚未人工使能，禁止运动")
                .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"));
        }
        return false;
    }
    return true;
}

bool ZmcController::resetRotarySafetyState(QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "resetRotarySafetyState", [&] { return resetRotarySafetyState(err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    m_rotaryAbsReady[AxisB].store(false);
    m_rotaryAbsReady[AxisC].store(false);
    m_rotaryServoEnabled[AxisB].store(false);
    m_rotaryServoEnabled[AxisC].store(false);

    if (!m_handle) return true;

    // 不再使用控制器 Basic/TABLE 握手。上位机直接关闭两个物理使能输出。
    const int32 retB = ZAux_Direct_SetOp(m_handle, Constants::kBServoEnableOutput, 0);
    const int32 retC = ZAux_Direct_SetOp(m_handle, Constants::kCServoEnableOutput, 0);

    if (retB != ERR_OK) {
        if (err) *err = errorTextFromCode(retB, "SetOp(B servo disable)");
        return false;
    }
    if (retC != ERR_OK) {
        if (err) *err = errorTextFromCode(retC, "SetOp(C servo disable)");
        return false;
    }
    return true;
}

bool ZmcController::setRotaryServoEnable(int axis, bool enable, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(enable ? IoPriority::Motion : IoPriority::Emergency, "setRotaryServoEnable",
            [&] { return setRotaryServoEnable(axis, enable, err); }, err,
            enable ? eci::EciIoScheduler::Motion : eci::EciIoScheduler::None, nullptr,
            enable ? eci::EciIoScheduler::None : eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    if (axis != AxisB && axis != AxisC) {
        if (err) *err = QStringLiteral("轴 %1 不是 B/C 旋转轴").arg(axis);
        return false;
    }

    if (!checkNoBufferedTrajectory5D(err)) return false;

    // 人工使能的唯一前置条件：本次连接后已经成功读取绝对值并建立该轴软件机械坐标框架。
    if (enable && !m_rotaryAbsReady[axis].load()) {
        if (err) {
            *err = QStringLiteral("%1轴绝对位置未读取/机械坐标框架未建立，禁止使能")
                .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"));
        }
        return false;
    }

    // 运动中不允许直接掉伺服使能。
    if (!enable) {
        int idle = 0;
        const int32 retIdle = ZAux_Direct_GetIfIdle(m_handle, axis, &idle);
        if (retIdle != ERR_OK) {
            if (err) *err = errorTextFromCode(retIdle, "GetIfIdle(before servo disable)");
            return false;
        }
        if (idle == 0) {
            if (err) {
                *err = QStringLiteral("%1轴仍在运动，必须先停止并确认停止后才能断使能")
                    .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"));
            }
            return false;
        }
    }

    const int output = rotaryEnableOutputForAxis(axis);
    const uint32 requested = enable ? 1U : 0U;
    const int32 ret = ZAux_Direct_SetOp(m_handle, output, requested);
    if (ret != ERR_OK) {
        if (enable) m_rotaryServoEnabled[axis].store(false);
        if (err) *err = errorTextFromCode(ret, "SetOp(rotary servo enable)");
        return false;
    }

    uint32 actual = 0;
    const int32 retRead = ZAux_Direct_GetOp(m_handle, output, &actual);
    if (retRead != ERR_OK) {
        m_rotaryServoEnabled[axis].store(false);
        if (err) *err = errorTextFromCode(retRead, "GetOp(rotary servo enable readback)");
        return false;
    }
    if ((actual ? 1U : 0U) != requested) {
        m_rotaryServoEnabled[axis].store(false);
        if (err) {
            *err = QStringLiteral("%1轴使能输出 OP(%2) 回读不一致: 请求=%3, 回读=%4")
                .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                .arg(output)
                .arg(requested)
                .arg(actual);
        }
        return false;
    }

    m_rotaryServoEnabled[axis].store(enable);
    LOG_INFO(LogCategory::Controller,
             QStringLiteral("%1 servo enable output OP(%2)=%3 readback OK")
                 .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                 .arg(output)
                 .arg(actual));
    return true;
}

bool ZmcController::initializeRotaryAxesFromAbsolute(double& cAxisDeg, double& bAxisDeg,
                                                      QString* err)
{
    if (m_io.isWorkerThread()) { if (err) *err = QStringLiteral("RS485 不能占用 ECI I/O 线程"); return false; }
    const auto operationToken = m_io.token();
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    if (!runIo(IoPriority::Control, "absolute.prepare", [&]() -> bool {
        p0::IoLock ioLock(m_ioMutex);

    if (m_activeBlockingCalls.load() > 1) {
        if (err) *err = QStringLiteral("已有阻塞任务执行中，请完成/停止后再读取绝对位置");
        return false;
    }
    cAxisDeg = 0.0;
    bAxisDeg = 0.0;

    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器，无法读取/建立 B/C 机械绝对坐标");
        return false;
    }

    // 重读绝对位置之前必须保证 B/C 已停稳；随后由上位机直接关闭 OP24/OP26。
    for (const int axis : { AxisC, AxisB }) {
        int idle = 0;
        const int32 retIdle = ZAux_Direct_GetIfIdle(m_handle, axis, &idle);
        if (retIdle != ERR_OK) {
            if (err) *err = errorTextFromCode(retIdle, "GetIfIdle(before absolute frame init)");
            return false;
        }
        if (idle == 0) {
            if (err) {
                *err = QStringLiteral("%1轴仍在运动，必须先停止后才能重读绝对位置")
                    .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"));
            }
            return false;
        }
    }

    QString safetyErr;
    if (!resetRotarySafetyState(&safetyErr)) {
        if (err) *err = QStringLiteral("无法关闭 B/C 伺服使能输出: %1").arg(safetyErr);
        return false;
    }


        return true;
    }, err)) return false;
#if defined(Q_OS_WIN) && !defined(MLS_PORTABLE_TRANSPORT_TEST)
    try {
        AbsServoPosition cPos{};
        AbsServoPosition bPos{};
        {
        // COM3 read may take seconds. Do not prevent an ECI stop/status request.
        // V19: standalone RS485 stays on the caller thread, not the I/O worker.
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("打开 B/C 绝对值串口: %1, 9600, 8N2")
                     .arg(QString::fromLatin1(kAbsServoPortName)));
        AbsServoSerialPort sp(kAbsServoPortName, kAbsServoBaudRate, kAbsServoDataFormat);

        // 只读取绝对位置本身：P21.32(多圈) + P21.34(单圈)。
        // 不再读取 P00.08/P00.14/P00.16/P00.18，不再与控制器 Basic/TABLE 握手。
        // 控制器 ENCODER_RATIO(1,-1) 仍由 Basic 固定；上位机只在 RS485 绝对值换算时
        // 应用同样的 -1 方向，不再动态判断或下发 ENCODER_RATIO。
        cPos = readAbsServoPosition(
            sp, kCServoSlaveId, kZeroCountsC, kReducerRatioC, kAbsDirectionC,
            kEncoderBitsC, kEncoderCprC);
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("C absolute read OK: slave=%1, turns=%2, single=%3, totalCounts=%4, zeroCounts=%5, deltaCounts=%6, direction=%7, axisDeg=%8")
                     .arg(kCServoSlaveId).arg(cPos.turns).arg(cPos.singleTurn)
                     .arg(cPos.totalCounts).arg(kZeroCountsC)
                     .arg(cPos.totalCounts - kZeroCountsC)
                     .arg(kAbsDirectionC)
                     .arg(cPos.axisDeg, 0, 'f', 6));

        // The standalone file supplied by the user only exercises C/slave 2 in main().
        // Read and log B separately so a missing/wrong B slave address is distinguishable
        // from a COM3/C-axis failure.
        bPos = readAbsServoPosition(
            sp, kBServoSlaveId, kZeroCountsB, kReducerRatioB, kAbsDirectionB,
            kEncoderBitsB, kEncoderCprB);
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("B absolute read OK: slave=%1, turns=%2, single=%3, totalCounts=%4, zeroCounts=%5, deltaCounts=%6, direction=%7, axisDeg=%8")
                     .arg(kBServoSlaveId).arg(bPos.turns).arg(bPos.singleTurn)
                     .arg(bPos.totalCounts).arg(kZeroCountsB)
                     .arg(bPos.totalCounts - kZeroCountsB)
                     .arg(kAbsDirectionB)
                     .arg(bPos.axisDeg, 0, 'f', 6));

        } // independent RS485 scope ends; the following commit is a separate I/O transaction

        return runIo(IoPriority::Control, "absolute.commit", [&]() -> bool {
        p0::IoLock ioLock(m_ioMutex);
        if (!m_handle) {
            if (err) *err = QStringLiteral("读取绝对位置期间控制器已断开");
            return false;
        }

        // V14: 不再向控制器写 MPOS / DPOS / OFFPOS。
        // 实机已证明，在 ZMotion 控制器不断电、仅上位机断开/重连时，
        // SetMpos/SetDpos 会通过坐标偏移机制叠加，导致“回 0°”位置逐次漂移。
        // 现在把禾川绝对值只用于建立上位机机械坐标框架：
        //     machine = controller + offset
        // 其中 offset = RS485_mechanical_absolute - controller_MPOS。
        // 单轴绝对运动时再把 machine target 转回 controller target；相对运动量不变。
        // 控制器自身的 B/C 软限位也按同一个 offset 变换，因此仍保留控制器侧限位保护。
        //
        // 如果旧版已经把 DPOS/MPOS 坐标污染成很大的 FE，则这里绝不尝试“自动修复”。
        // 必须先重新上电 ZMotion 控制器一次，使其自身 DPOS/MPOS 回到干净一致状态。
        // 这是一次性的旧版本清理要求；V14 后续反复断开/重连不会再写坐标，因此不会累加。
        // V22 never reads B/C DPOS for the upper-level machine pose.  The
        // absolute software frame is defined exclusively from the external
        // absolute angle and encoder MPOS: machine = rawMPOS + offset.
        struct RotaryFrameCandidate {
            int axis = 0;
            QString name;
            double machineAbsDeg = 0.0;
            float controllerMpos = 0.0f;
            double offset = 0.0;
        };

        auto prepareRotaryFrame = [this, err](RotaryFrameCandidate& frame) -> bool {
            const float machineFs = static_cast<float>(m_rotaryMachineSoftLimitPos[frame.axis].load());
            const float machineRs = static_cast<float>(m_rotaryMachineSoftLimitNeg[frame.axis].load());
            if (frame.machineAbsDeg > machineFs || frame.machineAbsDeg < machineRs) {
                if (err) {
                    *err = QStringLiteral("%1轴 RS485 机械绝对位置 %2° 超出机械软限位 [%3°, %4°]，禁止使能")
                        .arg(frame.name).arg(frame.machineAbsDeg, 0, 'f', 4)
                        .arg(machineRs, 0, 'f', 4).arg(machineFs, 0, 'f', 4);
                }
                return false;
            }

            float legacyAlias = 0.0f; bool idle = false;
            if (!getAxisPositionPairRaw(frame.axis, legacyAlias, frame.controllerMpos, idle, err)) return false;
            if (!idle) {
                if (err) *err = QStringLiteral("%1轴仍在运动，禁止建立绝对机械坐标").arg(frame.name);
                return false;
            }
            frame.offset = frame.machineAbsDeg - static_cast<double>(frame.controllerMpos);
            const double machineMpos = static_cast<double>(frame.controllerMpos) + frame.offset;
            LOG_INFO(LogCategory::Controller,
                     QStringLiteral("%1 absolute software-frame V22: RS485=%2, rawMPOS=%3, offset=%4, machineMPOS=%5 (B/C DPOS not read)")
                         .arg(frame.name).arg(frame.machineAbsDeg, 0, 'f', 6)
                         .arg(frame.controllerMpos, 0, 'f', 6).arg(frame.offset, 0, 'f', 6)
                         .arg(machineMpos, 0, 'f', 6));
            return true;
        };

        RotaryFrameCandidate cFrame{ AxisC, QStringLiteral("C"), cPos.axisDeg };
        RotaryFrameCandidate bFrame{ AxisB, QStringLiteral("B"), bPos.axisDeg };
        if (!prepareRotaryFrame(cFrame) || !prepareRotaryFrame(bFrame)) {
            resetRotarySafetyState(nullptr);
            return false;
        }

        // 两个轴都验证通过后才提交软件坐标框架，避免只初始化一半。
        m_rotaryMachineOffset[AxisC].store(cFrame.offset);
        m_rotaryMachineOffset[AxisB].store(bFrame.offset);

        // 将控制器原来的 ±90°“机械软限位”变换到它自己的原始坐标系。
        // 这是绝对值赋值，不是坐标偏移，重复连接时写入完全相同的值，不会累加。
        QString limitErr;
        if (!applyRotaryControllerSoftLimits(AxisC, &limitErr)
            || !applyRotaryControllerSoftLimits(AxisB, &limitErr)) {
            m_rotaryMachineOffset[AxisC].store(0.0);
            m_rotaryMachineOffset[AxisB].store(0.0);
            resetRotarySafetyState(nullptr);
            if (err) *err = QStringLiteral("B/C 机械坐标已读取，但变换控制器软限位失败: %1").arg(limitErr);
            return false;
        }

        // 两个轴的软件机械坐标框架和变换软限位均建立成功后，才记为 ready。
        // 仍然保持 OP24/OP26=OFF，必须由操作员点击“人工使能”按钮后才会打开。
        m_rotaryAbsReady[AxisC].store(true);
        m_rotaryAbsReady[AxisB].store(true);
        m_rotaryServoEnabled[AxisC].store(false);
        m_rotaryServoEnabled[AxisB].store(false);

        cAxisDeg = cPos.axisDeg;
        bAxisDeg = bPos.axisDeg;

        qInfo().noquote()
            << QStringLiteral("Rotary absolute software-frame ready: C=%1 deg, B=%2 deg; no MPOS/DPOS writes; OP24/OP26 remain OFF")
                   .arg(cAxisDeg, 0, 'f', 6)
                   .arg(bAxisDeg, 0, 'f', 6);
        return true;
        }, err, eci::EciIoScheduler::Motion, &operationToken);
    }
    catch (const std::exception& e) {
        // OP24/OP26 were already forced OFF before opening COM3. Do not issue another
        // synchronous controller I/O here: if Ethernet also disappeared during an
        // RS485 timeout, a redundant SetOp could delay error reporting/exit.
        m_rotaryAbsReady[AxisB].store(false);
        m_rotaryAbsReady[AxisC].store(false);
        m_rotaryServoEnabled[AxisB].store(false);
        m_rotaryServoEnabled[AxisC].store(false);
        if (err) *err = QStringLiteral("读取 B/C 禾川绝对位置失败: %1")
            .arg(QString::fromUtf8(e.what()));
        return false;
    }
#else
    m_rotaryAbsReady[AxisB].store(false);
    m_rotaryAbsReady[AxisC].store(false);
    m_rotaryServoEnabled[AxisB].store(false);
    m_rotaryServoEnabled[AxisC].store(false);
    if (err) *err = QStringLiteral("B/C 绝对位置读取当前仅实现 Windows COM 串口版本");
    return false;
#endif
}

// ==================== 连接管理 ====================

bool ZmcController::connectTcp(const QString& ip, int port, QString* err)
{
    Q_UNUSED(port);

    // The controller QObject is owned by the GUI thread.  Do not run the vendor
    // connection DLL from arbitrary QtConcurrent workers.  V5/V6 did that and the
    // field test showed a process-level crash when the controller/network was absent.
    if (QThread::currentThread() != thread()) {
        if (err) {
            *err = QStringLiteral("控制器连接必须在 ZmcController 所属线程执行；已拒绝跨线程 ZAux_OpenEth 调用");
        }
        return false;
    }

    std::lock_guard<std::mutex> lifecycleLock(m_lifecycleMutex);
    if (m_activeBlockingCalls.load() != 0) {
        if (err) *err = QStringLiteral("运动/绝对值读取/采样测试尚未结束，拒绝重连控制卡");
        return false;
    }
    m_lifecycleChanging.store(true);
    auto lifecycleDone = eci::scopeExit([&] { m_lifecycleChanging.store(false); });
    if (!runLifecycle("connect.closeOldSession", [&] {
            if (m_activeBlockingCalls.load() != 0) {
                if (err) *err = QStringLiteral("任务尚未结束，拒绝重连");
                return false;
            }
            disconnect();
            return true;
        }, err)) return false;

    QByteArray ipBytes = ip.toUtf8();
    ipBytes.append('\0');
    char* ipStr = ipBytes.data();

    ZMC_HANDLE pendingHandle = nullptr;
    int32 ret = -1;

#if defined(Q_OS_WIN) && defined(_MSC_VER) && !defined(MLS_PORTABLE_TRANSPORT_TEST)
    DWORD sehCode = 0;
    if (!safeZAuxOpenEth(ipStr, &pendingHandle, &ret, &sehCode)) {
        const QString errMsg = QStringLiteral(
            "ZAux_OpenEth 发生 Windows 异常 0x%1；控制器句柄未建立。请检查 PC 网卡/IP 后再手动连接。")
            .arg(static_cast<qulonglong>(sehCode), 8, 16, QLatin1Char('0'))
            .toUpper();
        if (err) *err = errMsg;
        pendingHandle = nullptr;
        emit errorOccurred(errMsg);
        return false;
    }
#else
    ret = ZAux_OpenEth(ipStr, &pendingHandle);
#endif

    if (ret != ERR_SUCCESS && ret != ERR_OK) {
        const QString errMsg = errorTextFromCode(ret, "ZAux_OpenEth");
        if (err) *err = errMsg;
        pendingHandle = nullptr;
        emit errorOccurred(errMsg);
        return false;
    }

    // OpenEth remains on the verified QObject/GUI thread, while admission is
    // closed and the old queue fenced. Transfer the new handle to the single I/O worker.
    const bool adopted = runLifecycle("connect.adoptHandle", [&] {
        p0::IoLock ioLock(m_ioMutex);
        m_handle = pendingHandle;
        QString safetyErr;
        if (!resetRotarySafetyState(&safetyErr)) {
            if (err) *err = QStringLiteral("控制卡已连接，但无法建立 B/C 安全状态: %1").arg(safetyErr);
            ZAux_Close(m_handle);
            m_handle = nullptr;
            return false;
        }
        m_lastIp = ip;
        m_connected.store(true);

        // V32.13: the five-axis Home-session mask lives in controller RAM for
        // the whole controller power session. XYZ bits are re-proved immediately;
        // C/B bits become effective only after RS485 absolute-ready is rebuilt. Restore it while we are already on
        // the lifecycle/I-O worker: connectTcp keeps m_lifecycleChanging=true until
        // this block returns, so dispatching a normal runIo() after adopt would be
        // correctly rejected by the admission gate.  IoMutex is recursive, making
        // the helper safe inside this transaction.
        QString homeStateErr;
        if (!restoreMachineHomeStateFromController(&homeStateErr)) {
            LOG_WARNING(LogCategory::Controller,
                        QStringLiteral("未恢复XYZBC回零状态，按未回零处理: %1").arg(homeStateErr));
        }
        return true;
    }, err);
    return adopted;
}

void ZmcController::disconnect()
{
    if (!m_io.isWorkerThread()) {
        std::lock_guard<std::mutex> lifecycleLock(m_lifecycleMutex);
        if (m_activeBlockingCalls.load() != 0) {
            emit errorOccurred(QStringLiteral("阻塞任务尚未退出，已拒绝断开；请先停止并等待任务结束"));
            return;
        }
        m_lifecycleChanging.store(true);
        auto lifecycleDone = eci::scopeExit([&] { m_lifecycleChanging.store(false); });
        QString error;
        if (!runLifecycle("disconnect", [&] { disconnect(); return true; }, &error))
            emit errorOccurred(error);
        return;
    }

    p0::IoLock ioLock(m_ioMutex);
    if (m_activeBlockingCalls.load() != 0) {
        // The caller must stop and join its workers first. Never close a handle
        // still used by a sleeping motion worker, or recycle it into a new session.
        emit errorOccurred(QStringLiteral("阻塞任务尚未退出，已拒绝断开；请先停止并等待任务结束"));
        return;
    }
    m_io.newSession(); // reject all queued requests carrying the old handle generation
    m_connected.store(false);
    if (m_handle) {
        // 不再使用 ZAux_SearchEth 判断是否可达，因为该探测存在假阴性。
        // 有有效句柄时直接尝试关闭 B/C 使能；失败也继续关闭控制器句柄。
        // 本地状态无论通信结果如何都由 resetRotarySafetyState 清为禁止状态。
        (void)resetRotarySafetyState(nullptr);
        ZAux_Close(m_handle);
        m_handle = nullptr;
    }
    m_lastIp.clear();
    ++m_mergeRevision;
    m_axisCompEnabled.store(false);
    // Compensation tables belong to the verified connection/session.  Do not
    // allow a new controller session to re-enable stale tables without an
    // explicit re-installation and calibration validation.
    m_axisCompLoaded = false;
    invalidateLinearTarget();
    m_mergePath.clear();
    m_mergeIndex = 0;
    m_hwSampling = false;
    m_hwReadCursorOwned = false;
    
    for (auto& t : m_axisTask) {
        t.store(AxisTaskNone);
    }
    for (auto& s : m_stopRequested) {
        s.store(false);
    }
    for (auto& referenced : m_linearMposReferenced) {
        referenced.store(false);
    }
    for (auto& generation : m_linearMposSetGeneration) {
        generation.store(0);
    }
    for (auto& ready : m_rotaryAbsReady) {
        ready.store(false);
    }
    for (auto& enabled : m_rotaryServoEnabled) {
        enabled.store(false);
    }
    for (auto& homed : m_rotaryHomeCompleted) {
        homed.store(false);
    }
}

// ==================== 轴状态查询 ====================

bool ZmcController::isAxisHoming(int axis) const
{
    if (axis < 0 || axis >= kMaxAxisCount) return false;
    return m_axisTask[axis].load() == AxisTaskHome;
}

bool ZmcController::isAxisBusy(int axis) const
{
    if (axis < 0 || axis >= kMaxAxisCount) return false;
    return m_axisTask[axis].load() != AxisTaskNone;
}

// ==================== 任务辅助函数 ====================

bool ZmcController::checkNoJointTask5D(QString* err) const
{
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (m_axisTask[axis].load() == AxisTaskJointMove5D) {
            if (err) *err = QStringLiteral("五轴同步任务尚未退出，禁止插入其它运动/参数/采样命令");
            return false;
        }
    }
    return true;
}

bool ZmcController::checkNoBufferedTrajectory5D(QString* err)
{
    if (!m_io.isWorkerThread()) return runIo(IoPriority::Control, "trajectory.leaseCheck",
        [&] { return checkNoBufferedTrajectory5D(err); }, err, eci::EciIoScheduler::None);
    p0::IoLock lock(m_ioMutex);
    if (m_mergePath.empty()) return true;
    if (m_mergeIndex < m_mergePath.size() || m_mergePaused) {
        if (err) *err = QStringLiteral("五轴轨迹尚有待执行段，禁止插入单轴运动/改参数/改使能");
        return false;
    }
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        int idle = 0;
        if (!m_handle || ZAux_Direct_GetIfIdle(m_handle, axis, &idle) != ERR_OK || idle == 0) {
            if (err) *err = QStringLiteral("五轴控制器缓冲尚未执行完毕，禁止插入其它操作");
            return false;
        }
    }
    m_mergePath.clear(); m_mergeIndex = 0;
    return true;
}

bool ZmcController::tryStartAxisTask(int axis, AxisTask task, QString* err)
{
    if (!checkNoBufferedTrajectory5D(err)) return false;
    if (axis < 0 || axis >= kMaxAxisCount) {
        if (err) *err = QStringLiteral("轴号 %1 超出范围").arg(axis);
        return false;
    }

    int expected = AxisTaskNone;
    if (!m_axisTask[axis].compare_exchange_strong(expected, static_cast<int>(task))) {
        if (err) *err = QStringLiteral("轴 %1 正在执行其它运动，请先停止").arg(axis);
        return false;
    }
    m_stopRequested[axis].store(false);
    return true;
}

void ZmcController::finishAxisTask(int axis)
{
    if (axis < 0 || axis >= kMaxAxisCount) return;
    m_axisTask[axis].store(AxisTaskNone);
}

bool ZmcController::tryStartJointTask5D(QString* err)
{
    int acquired[5] = {0, 0, 0, 0, 0};
    int acquiredCount = 0;
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (!tryStartAxisTask(axis, AxisTaskJointMove5D, err)) {
            for (int i = 0; i < acquiredCount; ++i) {
                finishAxisTask(acquired[i]);
            }
            return false;
        }
        acquired[acquiredCount++] = axis;
        m_stopRequested[axis].store(false);
    }
    return true;
}

void ZmcController::finishJointTask5D()
{
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        m_stopRequested[axis].store(false);
        finishAxisTask(axis);
    }
}

bool ZmcController::waitJointMoveIdle5D(int baseAxis, int timeoutMs, QString* err)
{
    if (m_io.isWorkerThread()) return false;
    QElapsedTimer timer; timer.start();
    bool userStopping = false;
    while (true) {
        bool idle = false;
        const bool ok = runIo(IoPriority::Control, "joint.poll", [&] {
            p0::IoLock lock(m_ioMutex);
            if (!m_handle) { if (err) *err = QStringLiteral("尚未连接控制卡"); return false; }
            for (int axis = AxisX; axis <= AxisB; ++axis)
                if (m_stopRequested[axis].load()) userStopping = true;
            int value = 0;
            const int32 ret = ZAux_Direct_GetIfIdle(m_handle, baseAxis, &value);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetIfIdle(joint base)"); return false; }
            idle = value != 0 && areFiveJointAxesIdle(nullptr); return true;
        }, err);
        if (!ok) return false;
        if (idle) {
            if (userStopping) { if (err) *err = QStringLiteral("五轴运动已按用户请求停止并停稳"); return false; }
            return true;
        }
        if (timeoutMs > 0 && timer.elapsed() > timeoutMs) {
            if (err) *err = userStopping ? QStringLiteral("五轴停止请求已发送，但等待各轴停稳超时")
                                        : QStringLiteral("五轴运动超时");
            return false;
        }
        QThread::msleep(20); // caller only
    }
}

bool ZmcController::waitAxisIdle(int axis, int timeoutMs, QString* err,
                                  double expectedMpos, double effectiveSpeed)
{
    if (m_io.isWorkerThread()) {
        if(err)*err=QStringLiteral("运动等待不能在控制器I/O线程执行");
        return false;
    }
    QElapsedTimer timer; timer.start();
    qint64 lastReportMs=-1200;
    float latestDpos=std::numeric_limits<float>::quiet_NaN();
    float latestMpos=std::numeric_limits<float>::quiet_NaN();
    int latestIdle=0;
    while (true) {
        const bool takeSnapshot=timer.elapsed()-lastReportMs >= 1000;
        bool idle=false;
        const bool ok=runIo(IoPriority::Control,"axis.poll", [&] {
            p0::IoLock lock(m_ioMutex);
            if(!m_handle){if(err)*err=QStringLiteral("控制卡断开");return false;}
            if(axis>=0&&axis<kMaxAxisCount&&m_stopRequested[axis].load()){
                if(err)*err=QStringLiteral("轴 %1 收到用户停止请求").arg(axis);
                return false;
            }
            int value=0;
            const int32 ret=ZAux_Direct_GetIfIdle(m_handle,axis,&value);
            if(ret!=ERR_OK){if(err)*err=errorTextFromCode(ret,"GetIfIdle");return false;}
            latestIdle=value; idle=value!=0;
            if(takeSnapshot||idle){
                // B/C must only use encoder MPOS; never read their controller DPOS
                // into the upper-level mechanical frame diagnostics.
                const int32 rd=(axis==AxisB||axis==AxisC)?ERR_OK:
                    ZAux_Direct_GetDpos(m_handle,axis,&latestDpos);
                const int32 rm=ZAux_Direct_GetMpos(m_handle,axis,&latestMpos);
                if((axis==AxisB||axis==AxisC)&&rm==ERR_OK)latestDpos=latestMpos;
                if(rd!=ERR_OK||rm!=ERR_OK){
                    if(err)*err=QStringLiteral("轴 %1 运动反馈DPOS/MPOS回读失败: SDK=%2/%3")
                        .arg(axis).arg(rd).arg(rm);
                    return false;
                }
            }
            return true;
        },err);
        if(!ok)return false;
        if(takeSnapshot||idle){
            lastReportMs=timer.elapsed();
            const QString report=QStringLiteral("[单轴运动][进度] axis=%1 elapsed=%2ms budget=%3ms "
               "IDLE=%4 MPOS=%5mm DPOS=%6mm targetMPOS=%7mm requestedEffectiveSpeed=%8mm/s")
               .arg(axis).arg(timer.elapsed()).arg(timeoutMs).arg(latestIdle)
               .arg(latestMpos,0,'f',5).arg(latestDpos,0,'f',5)
               .arg(expectedMpos,0,'f',5).arg(effectiveSpeed,0,'f',3);
            LOG_INFO(LogCategory::Controller,report);
        }
        if(idle)return true;
        if(timeoutMs>0&&timer.elapsed()>timeoutMs){
            if(err)*err=QStringLiteral("轴 %1 等待IDLE超时: elapsed=%2ms limit=%3ms "
                "MPOS=%4mm DPOS=%5mm targetMPOS=%6mm effectiveV=%7mm/s (已请求停止轴运动)")
                .arg(axis).arg(timer.elapsed()).arg(timeoutMs)
                .arg(latestMpos,0,'f',5).arg(latestDpos,0,'f',5)
                .arg(expectedMpos,0,'f',5).arg(effectiveSpeed,0,'f',3);
            return false;
        }
        QThread::msleep(20);
    }
}

// ==================== 轴参数读写 ====================

bool ZmcController::readAxisParams(int axis, AxisParams& out, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Diagnostic, "readAxisParams", [&] { return readAxisParams(axis, out, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    float v = 0.0f;
    int32 ret = ERR_OK;

    ret = ZAux_Direct_GetUnits(m_handle, axis, &v);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetUnits"); return false; }
    out.units = v;

    ret = ZAux_Direct_GetLspeed(m_handle, axis, &v);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetLspeed"); return false; }
    out.lspeed = v;

    ret = ZAux_Direct_GetSpeed(m_handle, axis, &v);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetSpeed"); return false; }
    out.speed = v;

    ret = ZAux_Direct_GetAccel(m_handle, axis, &v);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetAccel"); return false; }
    out.accel = v;

    ret = ZAux_Direct_GetDecel(m_handle, axis, &v);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetDecel"); return false; }
    out.decel = v;

    // SRAMP 可能不支持，失败时保持默认值
    ret = ZAux_Direct_GetSramp(m_handle, axis, &v);
    if (ret == ERR_OK) {
        out.sramp = v;
    }

    ret = ZAux_Direct_GetCreep(m_handle, axis, &v);
    if (ret == ERR_OK) {
        out.creep = v;
    }

    if (axis == AxisB || axis == AxisC) {
        // B/C 参数表始终显示机械坐标软限位。控制器内部实际限位可能已按
        // software-frame offset 平移，不能直接暴露给 UI，否则每次重连会看起来变化。
        out.softLimitPos = static_cast<float>(m_rotaryMachineSoftLimitPos[axis].load());
        out.softLimitNeg = static_cast<float>(m_rotaryMachineSoftLimitNeg[axis].load());
    } else {
        ret = ZAux_Direct_GetFsLimit(m_handle, axis, &v);
        if (ret == ERR_OK) {
            out.softLimitPos = v;
        }

        ret = ZAux_Direct_GetRsLimit(m_handle, axis, &v);
        if (ret == ERR_OK) {
            out.softLimitNeg = v;
        }
    }

    return true;
}

bool ZmcController::writeAxisParams(int axis, const AxisParams& in, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Motion, "writeAxisParams", [&] { return writeAxisParams(axis, in, err); }, err, eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!checkNoJointTask5D(err) || !checkNoBufferedTrajectory5D(err)) return false;
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    if (axis < AxisX || axis > AxisB || !std::isfinite(in.speed) || in.speed <= 0 ||
        !std::isfinite(in.accel) || in.accel <= 0 || !std::isfinite(in.decel) || in.decel <= 0 ||
        !std::isfinite(in.lspeed) || in.lspeed < 0 || !std::isfinite(in.sramp) || in.sramp < 0 ||
        !std::isfinite(in.creep) || in.creep < 0) {
        if (err) *err = QStringLiteral("无效的轴号/速度/加减速度参数"); return false;
    }
    if (m_hwSampling || !std::isfinite(in.units) || in.units <= 0 ||
        !std::isfinite(in.softLimitNeg) || !std::isfinite(in.softLimitPos) ||
        in.softLimitNeg >= in.softLimitPos) {
        if (err) *err = QStringLiteral("采集中不能修改轴配置，或轴配置/软限位无效"); return false;
    }
    if (!areAllAxesIdle(err)) return false;

    // Distinguish coordinate-defining parameters from ordinary motion-profile
    // parameters.  V27.6.37 revoked XYZ DATUM and B/C absolute-ready even when
    // the operator only changed SPEED/ACCEL/DECEL, which made Stage-2 speed
    // tuning unnecessarily require another homing cycle.
    float currentUnits = 0.0f;
    int32 ret = ZAux_Direct_GetUnits(m_handle, axis, &currentUnits);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetUnits(before writeAxisParams)"); return false; }
    const bool unitsChanged = std::abs(static_cast<double>(currentUnits) - static_cast<double>(in.units)) > 1.0e-6;

    bool rotaryLimitChanged = false;
    if (axis == AxisB || axis == AxisC) {
        rotaryLimitChanged = std::abs(m_rotaryMachineSoftLimitPos[axis].load() - static_cast<double>(in.softLimitPos)) > 1.0e-6
            || std::abs(m_rotaryMachineSoftLimitNeg[axis].load() - static_cast<double>(in.softLimitNeg)) > 1.0e-6;
    }
    const bool rotaryFrameChanged = (axis == AxisB || axis == AxisC) && (unitsChanged || rotaryLimitChanged);

    if(axis>=AxisX && axis<=AxisZ && unitsChanged){
        // UNITS changes redefine the XYZ coordinate scale. Revoke both the host-side
        // reference flag and the V32.13 controller power-session Home bit before
        // changing the coordinate domain, otherwise a later PC restart could restore
        // a stale "homed" state from TABLE[357].
        m_linearMposReferenced[axis].store(false);
        QString stateErr;
        if (!updateControllerHomeMaskBit(axis, false, &stateErr)) {
            if (err) *err = QStringLiteral("XYZ UNITS修改前无法可靠撤销控制器Home位，已拒绝更改UNITS: %1")
                .arg(stateErr);
            return false;
        }
        m_axisCompEnabled.store(false); invalidateLinearTarget();
        LOG_INFO(LogCategory::Controller,QStringLiteral("XYZ UNITS changed: compensation OFF; controller/host Home reference revoked; DATUM and explicit re-enable required"));
    }

    // A B/C UNITS change redefines the controller coordinate scale, so a previously
    // completed mechanical-0deg Home cannot be trusted across a later PC restart.
    // Clear only that axis' persisted Home bit; changing soft limits alone does not
    // invalidate the already established zero-reference history.
    if ((axis == AxisB || axis == AxisC) && unitsChanged) {
        m_rotaryHomeCompleted[axis].store(false);
        QString stateErr;
        if (!updateControllerHomeMaskBit(axis, false, &stateErr)) {
            if (err) *err = QStringLiteral("%1轴UNITS修改前无法撤销控制器Home状态: %2")
                .arg(jointAxisName(axis), stateErr);
            return false;
        }
    }

    // B/C speed-profile changes are safe while stationary and do not alter the
    // absolute-coordinate mapping.  Only UNITS / machine-limit-frame changes
    // revoke ready/servo and force an absolute-position resynchronization.
    if (rotaryFrameChanged) {
        for (const int rotaryAxis : { AxisC, AxisB }) {
            int idle = 0;
            const int32 retIdle = ZAux_Direct_GetIfIdle(m_handle, rotaryAxis, &idle);
            if (retIdle != ERR_OK) {
                if (err) *err = errorTextFromCode(retIdle, "GetIfIdle(before rotary parameter write)");
                return false;
            }
            if (idle == 0) {
                if (err) {
                    *err = QStringLiteral("%1轴仍在运动，禁止修改 B/C 坐标参数")
                        .arg(rotaryAxis == AxisB ? QStringLiteral("B") : QStringLiteral("C"));
                }
                return false;
            }
        }

        QString safetyErr;
        if (!resetRotarySafetyState(&safetyErr)) {
            if (err) *err = QStringLiteral("写 B/C 坐标参数前无法建立断使能状态: %1").arg(safetyErr);
            return false;
        }
    }

    ret = ERR_OK;

    // Do not rewrite UNITS when only the motion profile is being tuned.  Some
    // controllers treat SetUnits as a coordinate-domain operation even when the
    // numeric value is unchanged; Stage-2 speed tuning must never invalidate a
    // previously established machine reference.
    if (unitsChanged) {
        ret = ZAux_Direct_SetUnits(m_handle, axis, in.units);
        if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetUnits"); return false; }
    }

    ret = ZAux_Direct_SetLspeed(m_handle, axis, in.lspeed);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetLspeed"); return false; }

    ret = ZAux_Direct_SetSpeed(m_handle, axis, in.speed);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSpeed"); return false; }

    ret = ZAux_Direct_SetAccel(m_handle, axis, in.accel);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetAccel"); return false; }

    ret = ZAux_Direct_SetDecel(m_handle, axis, in.decel);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetDecel"); return false; }

    ret = ZAux_Direct_SetSramp(m_handle, axis, in.sramp);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSramp"); return false; }

    ret = ZAux_Direct_SetCreep(m_handle, axis, in.creep);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetCreep"); return false; }

    if (axis == AxisB || axis == AxisC) {
        if (in.softLimitPos <= in.softLimitNeg) {
            if (err) *err = QStringLiteral("B/C 正软限位必须大于负软限位");
            return false;
        }
        if (rotaryFrameChanged) {
            m_rotaryMachineSoftLimitPos[axis].store(in.softLimitPos);
            m_rotaryMachineSoftLimitNeg[axis].store(in.softLimitNeg);

            // The ready state was revoked above, so temporarily write the
            // unshifted machine limits. Absolute-position initialization will
            // reapply the software-frame offset after it succeeds.
            ret = ZAux_Direct_SetFsLimit(m_handle, axis, in.softLimitPos);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetFsLimit(rotary machine)"); return false; }
            ret = ZAux_Direct_SetRsLimit(m_handle, axis, in.softLimitNeg);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetRsLimit(rotary machine)"); return false; }
        }
        // Motion-only B/C changes deliberately leave the already-correct
        // controller soft limits and absolute-coordinate state untouched.
    } else {
        ret = ZAux_Direct_SetFsLimit(m_handle, axis, in.softLimitPos);
        if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetFsLimit"); return false; }

        ret = ZAux_Direct_SetRsLimit(m_handle, axis, in.softLimitNeg);
        if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetRsLimit"); return false; }
    }

    return true;
}

bool ZmcController::writeMotionParams(int axis, const AxisParams& in, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Motion, "writeMotionParams", [&] { return writeMotionParams(axis, in, err); }, err, eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!checkNoJointTask5D(err) || !checkNoBufferedTrajectory5D(err)) return false;
    // 只写入运动参数，不包括软限位
    // 用于运动操作，避免覆盖控制器中已配置的软限位
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    if (axis < AxisX || axis > AxisB || !std::isfinite(in.speed) || in.speed <= 0 ||
        !std::isfinite(in.accel) || in.accel <= 0 || !std::isfinite(in.decel) || in.decel <= 0 ||
        !std::isfinite(in.lspeed) || in.lspeed < 0 || !std::isfinite(in.sramp) || in.sramp < 0 ||
        !std::isfinite(in.creep) || in.creep < 0) {
        if (err) *err = QStringLiteral("无效的轴号/速度/加减速度参数"); return false;
    }

    int32 ret = ERR_OK;

    // UNITS/gearing belongs to commissioned axis configuration, never to a jog or move.

    ret = ZAux_Direct_SetLspeed(m_handle, axis, in.lspeed);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetLspeed"); return false; }

    ret = ZAux_Direct_SetSpeed(m_handle, axis, in.speed);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSpeed"); return false; }

    ret = ZAux_Direct_SetAccel(m_handle, axis, in.accel);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetAccel"); return false; }

    ret = ZAux_Direct_SetDecel(m_handle, axis, in.decel);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetDecel"); return false; }

    ret = ZAux_Direct_SetSramp(m_handle, axis, in.sramp);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSramp"); return false; }

    ret = ZAux_Direct_SetCreep(m_handle, axis, in.creep);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetCreep"); return false; }

    // 注意：不设置软限位(FsLimit/RsLimit)，保持控制器中的原有值

    return true;
}

bool ZmcController::readAllAxisParams(AxisParams (&params)[Constants::kAxisParamArraySize], QString* err)
{
    const auto operationToken = m_io.token();
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (!runIo(IoPriority::Diagnostic, "readAllAxisParams.axis", [&] { return readAxisParams(axis, params[axis], err); }, err, eci::EciIoScheduler::None, &operationToken)) {
            return false;
        }
    }
    return true;
}

bool ZmcController::writeAllAxisParams(const AxisParams (&params)[Constants::kAxisParamArraySize], QString* err)
{
    const auto operationToken = m_io.token();
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (!runIo(IoPriority::Motion, "writeAllAxisParams.axis", [&] { return writeAxisParams(axis, params[axis], err); }, err, eci::EciIoScheduler::Motion, &operationToken)) {
            return false;
        }
    }
    return true;
}

// ==================== 位置读取 ====================

bool ZmcController::readAxisPositions(double& x, double& y, double& z, QString* errMsg) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Measurement, "readAxisPositions", [&] { return readAxisPositions(x, y, z, errMsg); }, errMsg, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (m_handle == nullptr) {
        if (errMsg) *errMsg = QStringLiteral("控制器未连接");
        return false;
    }
    if (!areLinearMposReferenced()) {
        if (errMsg) *errMsg = QStringLiteral("XYZ增量光栅尚未全部完成正规回零并建立MPOS机床零点");
        return false;
    }

    float fx = 0.0f, fy = 0.0f, fz = 0.0f;

    int32 ret = ZAux_Direct_GetMpos(m_handle, AxisX, &fx);
    if (ret != ERR_OK) {
        if (errMsg) *errMsg = errorTextFromCode(ret, "GetMpos X");
        return false;
    }
    
    ret = ZAux_Direct_GetMpos(m_handle, AxisY, &fy);
    if (ret != ERR_OK) {
        if (errMsg) *errMsg = errorTextFromCode(ret, "GetMpos Y");
        return false;
    }
    
    ret = ZAux_Direct_GetMpos(m_handle, AxisZ, &fz);
    if (ret != ERR_OK) {
        if (errMsg) *errMsg = errorTextFromCode(ret, "GetMpos Z");
        return false;
    }

    x = fx;
    y = fy;
    z = fz;
    return true;
}

bool ZmcController::readJointPose5D(JointPose5D& pose, QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "readJointPose5D", [&] { return readJointPose5D(pose, errMsg); }, errMsg, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    pose = JointPose5D{};
    if (!m_handle) {
        if (errMsg) *errMsg = QStringLiteral("控制器未连接");
        return false;
    }
    if (!m_rotaryAbsReady[AxisC].load() || !m_rotaryAbsReady[AxisB].load()) {
        if (errMsg) *errMsg = QStringLiteral("B/C绝对机械坐标尚未建立，不能读取可靠五轴运动学姿态");
        return false;
    }

    if (!readAxisPositions(pose.x, pose.y, pose.z, errMsg)) {
        return false;
    }

    float dpos = 0.0f, mpos = 0.0f;
    bool idle = false;
    if (!getAxisPositionPair(AxisC, dpos, mpos, idle, errMsg)) {
        return false;
    }
    pose.c = static_cast<double>(mpos); // ATYPE=4：运动学姿态优先使用真实编码器反馈

    if (!getAxisPositionPair(AxisB, dpos, mpos, idle, errMsg)) {
        return false;
    }
    pose.b = static_cast<double>(mpos);

    if (!pose.isFinite()) {
        if (errMsg) *errMsg = QStringLiteral("读取到的五轴姿态包含非法数值");
        return false;
    }
    return true;
}

bool ZmcController::readJointState5D(JointState5D& state, QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "readJointState5D", [&] { return readJointState5D(state, errMsg); }, errMsg, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    state = JointState5D{};
    if (!m_handle) {
        if (errMsg) *errMsg = QStringLiteral("控制器未连接");
        return false;
    }
    if (!areLinearMposReferenced()) {
        if (errMsg) *errMsg = QStringLiteral("XYZ尚未全部回零并建立光栅MPOS机床基准，不能读取可靠五轴状态");
        return false;
    }
    if (!m_rotaryAbsReady[AxisC].load() || !m_rotaryAbsReady[AxisB].load()) {
        if (errMsg) *errMsg = QStringLiteral("B/C绝对机械坐标尚未建立，不能读取可靠五轴状态");
        return false;
    }

    state.allIdle = true;
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        float dpos = 0.0f;
        float mpos = 0.0f;
        bool idle = false;
        if (!getAxisPositionPair(axis, dpos, mpos, idle, errMsg)) {
            return false;
        }
        setJointValue(state.dpos, axis, static_cast<double>(dpos));
        setJointValue(state.mpos, axis, static_cast<double>(mpos));
        if (!idle) state.allIdle = false;
    }

    if (!state.dpos.isFinite() || !state.mpos.isFinite()) {
        if (errMsg) *errMsg = QStringLiteral("读取到的五轴 DPOS/MPOS 包含非法数值");
        return false;
    }
    return true;
}

bool ZmcController::readAxisUiPosition(int axis, float& dpos, float& mpos, bool& idle, QString* err)
{
    return runIo(IoPriority::Diagnostic, "UI.position", [&] {
        return getAxisPositionPair(axis, dpos, mpos, idle, err);
    }, err);
}

bool ZmcController::readAxisType(int axis, int& atype, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Diagnostic, "readAxisType", [&] {
            return readAxisType(axis, atype, err);
        }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("控制器未连接");
        return false;
    }
    if (axis < AxisX || axis > AxisB) {
        if (err) *err = QStringLiteral("轴号超出五轴范围: %1").arg(axis);
        return false;
    }

    int value = 0;
    const int ret = ZAux_Direct_GetAtype(m_handle, axis, &value);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "GetAtype");
        return false;
    }
    atype = value;
    return true;
}

bool ZmcController::getAxisPositionPair(int axis, float& dpos, float& mpos,
                                        bool& idle, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "getAxisPositionPair", [&] { return getAxisPositionPair(axis, dpos, mpos, idle, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!getAxisPositionPairRaw(axis, dpos, mpos, idle, err)) {
        return false;
    }

    // B/C ready 后，对外统一返回机械坐标；控制器内部 DPOS/MPOS 保持原始坐标，
    // 这样 PC 断开/重连不会再修改或累计控制器坐标偏移。
    if ((axis == AxisB || axis == AxisC) && m_rotaryAbsReady[axis].load()) {
        const float offset = static_cast<float>(m_rotaryMachineOffset[axis].load());
        dpos += offset;
        mpos += offset;
    }
    return true;
}


bool ZmcController::readAxisDatumInputState(int axis, int& inputNumber, int& rawValue, QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Diagnostic, "readAxisDatumInputState", [&] {
            return readAxisDatumInputState(axis, inputNumber, rawValue, err);
        }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    inputNumber = -1;
    rawValue = -1;
    if (!m_handle) {
        if (err) *err = QStringLiteral("控制器未连接");
        return false;
    }
    if (axis < AxisX || axis > AxisB) {
        if (err) *err = QStringLiteral("轴号超出五轴范围: %1").arg(axis);
        return false;
    }

    int datumIn = -1;
    int32 ret = ZAux_Direct_GetDatumIn(m_handle, axis, &datumIn);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "GetDatumIn");
        return false;
    }
    inputNumber = datumIn;
    if (datumIn < 0) {
        if (err) *err = QStringLiteral("%1轴未配置DATUM_IN").arg(jointAxisName(axis));
        return false;
    }

    uint32 value = 0;
    ret = ZAux_Direct_GetIn(m_handle, datumIn, &value);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "GetIn(DATUM_IN)");
        return false;
    }
    rawValue = value ? 1 : 0;
    return true;
}


bool ZmcController::getAxisStatusWord(int axis, int& statusWord, QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "getAxisStatusWord", [&] { return getAxisStatusWord(axis, statusWord, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    statusWord = 0;
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    int value = 0;
    const int32 ret = ZAux_Direct_GetAxisStatus(m_handle, axis, &value);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "GetAxisStatus");
        return false;
    }

    statusWord = value;
    return true;
}

QString ZmcController::describeAxisStatusWord(int statusWord)
{
    if (statusWord == 0) {
        return QStringLiteral("正常");
    }

    struct StatusBitDesc {
        int mask;
        const char* text;
    };

    // 对照 ZBasic/PC 函数库 AXISSTATUS 定义。
    static const StatusBitDesc kStatusBits[] = {
        { 2,       "随动误差超限告警" },
        { 4,       "与远程轴通讯出错" },
        { 8,       "远程驱动器报错" },
        { 16,      "正向硬限位" },
        { 32,      "反向硬限位" },
        { 64,      "找原点中" },
        { 128,     "HOLD速度保持" },
        { 256,     "随动误差超限出错" },
        { 512,     "超过正向软限位" },
        { 1024,    "超过负向软限位" },
        { 2048,    "CANCEL执行中" },
        { 4096,    "脉冲频率超过MAX_SPEED" },
        { 16384,   "机械手指令坐标错误" },
        { 262144,  "电源异常" },
        { 524288,  "精准输出缓冲溢出" },
        { 2097152, "运动中触发特殊运动指令失败" },
        { 4194304, "伺服/告警信号输入" },
        { 8388608, "轴暂停" },
    };

    QStringList parts;
    int knownMask = 0;
    for (const auto& item : kStatusBits) {
        knownMask |= item.mask;
        if ((statusWord & item.mask) != 0) {
            parts << QString::fromUtf8(item.text);
        }
    }

    const int unknown = statusWord & ~knownMask;
    if (unknown != 0) {
        parts << QStringLiteral("未知状态位 0x%1")
                     .arg(static_cast<quint32>(unknown), 0, 16)
                     .toUpper();
    }

    return parts.isEmpty() ? QStringLiteral("状态字非零") : parts.join(QStringLiteral(" | "));
}

bool ZmcController::axisStatusHasAlarm(int statusWord)
{
    // 不把“找原点中/HOLD/CANCEL/暂停”当成故障报警；
    // 其余已知错误、硬限位、软限位均需要在日志中告警。
    constexpr int kAlarmMask =
          2
        | 4
        | 8
        | 16
        | 32
        | 256
        | 512
        | 1024
        | 4096
        | 16384
        | 262144
        | 524288
        | 2097152
        | 4194304;
    return (statusWord & kAlarmMask) != 0;
}

bool ZmcController::getAxisStatus(int axis, float& pos, bool& idle, QString* err)
{
    float dpos = 0.0f;
    float mpos = 0.0f;
    if (!getAxisPositionPair(axis, dpos, mpos, idle, err)) {
        return false;
    }

    // “当前位置”统一表示测量位置：XYZ 光栅 MPOS，B/C 机械 MPOS。
    // DPOS 仍由 readAxisUiPosition/getAxisPositionPair 单独保留用于命令/跟随误差诊断。
    pos = mpos;
    return true;
}

// ==================== 单轴运动 ====================

bool ZmcController::homeAxis(int axis, const HomeParams& hp,
                             const AxisParams& motion, QString* err)
{
    if (m_io.isWorkerThread()) { if (err) *err = QStringLiteral("回零等待不能占用 I/O 线程"); return false; }
    const auto operationToken = m_io.token();
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    bool taskStarted = false, backedUp = false;
    // Re-homing invalidates the previous incremental-grating reference immediately.
    // A failed/aborted DATUM must never leave an old MPOS origin marked as valid.
    if (axis >= AxisX && axis <= AxisZ) {
        m_linearMposReferenced[axis].store(false);
        m_axisCompEnabled.store(false); // re-home requires explicit operator re-enable
        // Clear the controller-resident bit BEFORE motion starts.  If the host
        // crashes or DATUM is aborted, the next application instance must not
        // resurrect the previous reference as valid.
        QString stateErr;
        if (!updateControllerHomeMaskBit(axis, false, &stateErr)) {
            if (err) *err = QStringLiteral("无法在控制器中撤销旧回零状态，已拒绝开始回零: %1").arg(stateErr);
            return false;
        }
    }
    float oldFs = 0, oldRs = 0;
    auto finish = [&]() -> bool {
        if (!taskStarted) return true;
        const bool result = runIo(IoPriority::Emergency, "home.restoreAndFinish", [&] {
            p0::IoLock lock(m_ioMutex);
            bool ok = true;
            if (backedUp && m_handle) {
                const auto a = ZAux_Direct_SetFsLimit(m_handle, axis, oldFs);
                const auto b = ZAux_Direct_SetRsLimit(m_handle, axis, oldRs);
                ok = a == ERR_OK && b == ERR_OK;
                if (!ok) emit errorOccurred(QStringLiteral("回零收尾恢复软限位失败，请检查控制卡状态"));
            }
            finishAxisTask(axis); return ok;
        }, nullptr);
        taskStarted = false;
        return result;
    };
    auto cleanup = eci::scopeExit([&] { (void)finish(); });
    if (!runIo(IoPriority::Motion, "home.start", [&]() -> bool {
        p0::IoLock lock(m_ioMutex);
        if (!m_handle) { if (err) *err = QStringLiteral("尚未连接控制卡"); return false; }
        if (axis == AxisC || axis == AxisB) { if (err) *err = QStringLiteral("B/C 旋转轴没有原点开关，禁止执行回零"); return false; }
        if (!tryStartAxisTask(axis, AxisTaskHome, err)) return false;
        taskStarted = true;
        auto check = [&](int32 r, const char* fn) {
            if (r == ERR_OK) return true; if (err) *err = errorTextFromCode(r, fn); return false;
        };
        if (!check(ZAux_Direct_GetFsLimit(m_handle, axis, &oldFs), "GetFsLimit(home)")) return false;
        if (!check(ZAux_Direct_GetRsLimit(m_handle, axis, &oldRs), "GetRsLimit(home)")) return false;
        backedUp = true;
        if (!check(ZAux_Direct_SetFsLimit(m_handle, axis, 1e9f), "SetFsLimit(home)")) return false;
        if (!check(ZAux_Direct_SetRsLimit(m_handle, axis, -1e9f), "SetRsLimit(home)")) return false;
        if (!check(ZAux_Direct_SetLspeed(m_handle, axis, motion.lspeed), "SetLspeed(home)")) return false;
        if (!check(ZAux_Direct_SetSpeed(m_handle, axis, motion.speed), "SetSpeed(home)")) return false;
        if (!check(ZAux_Direct_SetAccel(m_handle, axis, motion.accel), "SetAccel(home)")) return false;
        if (!check(ZAux_Direct_SetDecel(m_handle, axis, motion.decel), "SetDecel(home)")) return false;
        if (!check(ZAux_Direct_SetCreep(m_handle, axis, motion.creep), "SetCreep(home)")) return false;
        return hp.mode == 13
            ? check(ZAux_Direct_Single_Vmove(m_handle, axis, 1), "Single_Vmove(home approach)")
            : check(ZAux_Direct_Single_Datum(m_handle, axis, hp.mode), "Single_Datum");
    }, err, eci::EciIoScheduler::Motion, &operationToken)) return false;
    if (!waitAxisIdle(axis, Constants::kHomeTimeoutMs, err)) return false;
    if (hp.mode == 13) {
        if (!runIo(IoPriority::Motion, "home.datum", [&] {
            p0::IoLock lock(m_ioMutex);
            if (m_stopRequested[axis].load()) { if (err) *err = QStringLiteral("回零已被手动停止"); return false; }
            const auto ret = ZAux_Direct_Single_Datum(m_handle, axis, hp.mode);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "Single_Datum"); return false; }
            return true;
        }, err, eci::EciIoScheduler::Motion, &operationToken)) return false;
        if (!waitAxisIdle(axis, Constants::kHomeTimeoutMs, err)) return false;
    }
    const bool stopped = m_stopRequested[axis].load();
    if (stopped) {
        if (!finish()) { if (err) *err = QStringLiteral("回零被停止且软限位恢复失败"); return false; }
        if (err && err->isEmpty()) *err = QStringLiteral("回零已被手动停止；未建立XYZ光栅MPOS零点");
        return false;
    }

    // XYZ uses incremental linear gratings. DATUM establishes the machine reference
    // for DPOS, but the grating MPOS also needs an explicit coordinate origin after
    // every successful home. B/C use a separate absolute-mechanical-coordinate path
    // and must never be zeroed here. Never execute SetMpos on an idle/aborted datum:
    // GetHomeStatus must report success first.
    if (axis > AxisZ) {
        if (!finish()) { if (err) *err = QStringLiteral("回零收尾恢复软限位失败，不能确认安全完成"); return false; }
        return true;
    }
    if (!runIo(IoPriority::Motion, "home.alignLinearMpos", [&]() -> bool {
        p0::IoLock lock(m_ioMutex);
        uint32 homeStatus = 0;
        int32 ret = ZAux_Direct_GetHomeStatus(m_handle, static_cast<uint32>(axis), &homeStatus);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetHomeStatus");
            return false;
        }
        if (homeStatus != 1u) {
            if (err) *err = QStringLiteral("%1轴DATUM已停止但HOME_STATUS未成功，禁止把当前位置定义为光栅零点")
                .arg(jointAxisName(axis));
            return false;
        }

        float rawMposBeforeZero = 0.0f;
        ret = ZAux_Direct_GetMpos(m_handle, axis, &rawMposBeforeZero);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetMpos(pre-home-zero)");
            return false;
        }
        // A stopped DPOS does not guarantee the encoder has completely settled.
        // Require three short consecutive grating reads to remain within 0.02 mm.
        constexpr float kHomeMposSettleToleranceMm = 0.02f;
        float previousMpos = rawMposBeforeZero;
        for (int sample = 0; sample < 3; ++sample) {
            QThread::msleep(10);
            float currentMpos = 0.0f;
            ret = ZAux_Direct_GetMpos(m_handle, axis, &currentMpos);
            if (ret != ERR_OK) {
                if (err) *err = errorTextFromCode(ret, "GetMpos(home-settle)");
                return false;
            }
            if (!std::isfinite(currentMpos) || std::abs(currentMpos - previousMpos) > kHomeMposSettleToleranceMm) {
                if (err) *err = QStringLiteral("%1轴DATUM后光栅MPOS尚未稳定：相邻读数变化%2 mm，禁止建立零点")
                    .arg(jointAxisName(axis)).arg(std::abs(currentMpos - previousMpos),0,'f',6);
                return false;
            }
            previousMpos = currentMpos;
        }
        rawMposBeforeZero = previousMpos;
        ret = ZAux_Direct_SetMpos(m_handle, axis, 0.0f);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "SetMpos(home-zero)");
            return false;
        }
        const std::uint64_t setGeneration = m_linearMposSetGeneration[axis].fetch_add(1) + 1;
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("%1 host SetMpos(home-zero) committed; generation=%2")
                     .arg(jointAxisName(axis)).arg(static_cast<qulonglong>(setGeneration)));

        float dposAfter = 0.0f, mposAfter = 0.0f;
        ret = ZAux_Direct_GetDpos(m_handle, axis, &dposAfter);
        if (ret == ERR_OK) ret = ZAux_Direct_GetMpos(m_handle, axis, &mposAfter);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetDpos/GetMpos(home verify)");
            return false;
        }
        constexpr float kHomeCoordinateToleranceMm = 0.10f;
        if (!std::isfinite(dposAfter) || !std::isfinite(mposAfter)
            || std::abs(dposAfter) > kHomeCoordinateToleranceMm
            || std::abs(mposAfter) > kHomeCoordinateToleranceMm) {
            if (err) {
                *err = QStringLiteral("%1轴回零后坐标基准校验失败：DPOS=%2 mm，MPOS=%3 mm；禁止继续把该点作为机床零点")
                    .arg(jointAxisName(axis)).arg(dposAfter,0,'f',6).arg(mposAfter,0,'f',6);
            }
            return false;
        }
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("%1 home coordinate zero prepared: grating MPOS %2 -> 0, verified DPOS=%3 MPOS=%4 mm")
                     .arg(jointAxisName(axis)).arg(rawMposBeforeZero,0,'f',6)
                     .arg(dposAfter,0,'f',6).arg(mposAfter,0,'f',6));
        return true;
    }, err, eci::EciIoScheduler::Motion, &operationToken)) {
        (void)finish();
        return false;
    }

    if (!finish()) {
        if (axis >= AxisX && axis <= AxisZ) {
            m_linearMposReferenced[axis].store(false);
            m_axisCompEnabled.store(false);
        }
        if (err) *err = QStringLiteral("回零收尾恢复软限位失败，不能确认安全完成");
        return false;
    }
    if (axis >= AxisX && axis <= AxisZ) {
        // Only a fully successful DATUM + MPOS-zero + soft-limit restoration makes
        // the incremental grating machine coordinate authoritative.  Commit the
        // controller-side RAM bit last, so an abnormal host exit can be recovered
        // safely while the controller remains powered.
        QString stateErr;
        if (!updateControllerHomeMaskBit(axis, true, &stateErr)) {
            m_linearMposReferenced[axis].store(false);
            if (err) *err = QStringLiteral("%1轴物理回零已完成，但控制器回零状态提交失败：%2；为安全起见本次不标记为已回零")
                .arg(jointAxisName(axis), stateErr);
            return false;
        }
        m_linearMposReferenced[axis].store(true);
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("%1 home reference committed: grating MPOS machine zero valid; controller power-session Home bit set")
                     .arg(jointAxisName(axis)));
    }
    return true;
}

bool ZmcController::homeRotaryAxisToZero(int axis, const AxisParams& motion, QString* err)
{
    if (axis != AxisC && axis != AxisB) {
        if (err) *err = QStringLiteral("仅B/C旋转轴使用homeRotaryAxisToZero");
        return false;
    }

    // Fail closed BEFORE any motion.  If the upper-PC crashes or the move is
    // aborted, the next application instance must not resurrect an old C/B Home bit.
    m_rotaryHomeCompleted[axis].store(false);
    QString stateErr;
    if (!updateControllerHomeMaskBit(axis, false, &stateErr)) {
        if (err) *err = QStringLiteral("%1轴无法在控制器中撤销旧Home状态，已拒绝开始回零: %2")
            .arg(jointAxisName(axis), stateErr);
        return false;
    }

    if (!moveAbs(axis, 0.0f, &motion, err)) return false;

    // Re-read the machine-coordinate feedback after the move.  This is not a
    // substitute for the external absolute encoder; checkRotaryMotionAllowed()
    // already required the RS485 absolute frame before motion was admitted.
    float dpos = 0.0f, mpos = 0.0f;
    bool idle = false;
    QString verifyErr;
    if (!getAxisPositionPair(axis, dpos, mpos, idle, &verifyErr)) {
        if (err) *err = QStringLiteral("%1轴已执行0°定位，但回读失败，Home状态不提交: %2")
            .arg(jointAxisName(axis), verifyErr);
        return false;
    }
    constexpr float kRotaryHomeToleranceDeg = 0.20f;
    if (!idle || !std::isfinite(dpos) || !std::isfinite(mpos)
        || std::abs(dpos) > kRotaryHomeToleranceDeg
        || std::abs(mpos) > kRotaryHomeToleranceDeg) {
        if (err) {
            *err = QStringLiteral("%1轴0°回零后位置校验失败: DPOS=%2°, MPOS=%3°, IDLE=%4；Home状态不提交")
                .arg(jointAxisName(axis)).arg(dpos,0,'f',6).arg(mpos,0,'f',6).arg(idle?1:0);
        }
        return false;
    }

    if (!updateControllerHomeMaskBit(axis, true, &stateErr)) {
        if (err) *err = QStringLiteral("%1轴物理回到0°，但控制器Home状态提交失败: %2；本次按未回零处理")
            .arg(jointAxisName(axis), stateErr);
        return false;
    }
    m_rotaryHomeCompleted[axis].store(true);
    LOG_INFO(LogCategory::Controller,
             QStringLiteral("%1 rotary home committed: mechanical 0deg verified; controller power-session Home bit set")
                 .arg(jointAxisName(axis)));
    return true;
}

bool ZmcController::moveAbs(int axis, float pos, const AxisParams* motion, QString* err)
{ return moveAbsImpl(axis, pos, motion, err, false); }
bool ZmcController::moveAbsRawDpos(int axis, float pos, const AxisParams* motion, QString* err)
{ return moveAbsImpl(axis, pos, motion, err, true); }
bool ZmcController::moveLinearMposAbs(int axis, float machineMpos, const AxisParams* motion, QString* err)
{
    if (axis < AxisX || axis > AxisZ || !std::isfinite(machineMpos)) {
        if (err) *err = QStringLiteral("单轴MPOS定位只支持有限的XYZ机械坐标");
        return false;
    }
    if (!isLinearMposReferenced(axis)) {
        if (err) *err = QStringLiteral("XYZ光栅MPOS机床基准无效，禁止单轴扫描");
        return false;
    }
    return moveAbsImpl(axis, machineMpos, motion, err, false, true);
}
bool ZmcController::moveAbsImpl(int axis, float pos, const AxisParams* motion, QString* err,
                                bool rawDpos, bool targetIsMpos)
{
    if (m_io.isWorkerThread()) {
        if (err) *err = QStringLiteral("阻塞运动门面不能在 ECI I/O 线程调用");
        return false;
    }
    const auto operationToken = m_io.token();
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    bool taskStarted = false;
    int settleMs = 0;
    double moveLengthMm = 0.0;
    double effectiveSpeed = 0.0;
    double effectiveAccel = 0.0;
    double effectiveDecel = 0.0;
    double expectedMpos = std::numeric_limits<double>::quiet_NaN();
    auto taskDone = eci::scopeExit([&] {
        if (taskStarted) runIo(IoPriority::Emergency, "axis.finish", [&] {
            p0::IoLock lock(m_ioMutex); finishAxisTask(axis); return true;
        }, nullptr);
    });
    const bool started = runIo(IoPriority::Motion, "moveAbs.start", [&]() -> bool {
        p0::IoLock ioLock(m_ioMutex);

    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    // Preserve task exclusion before ANY SDK read, including compensation/limits.
    if (!checkNoJointTask5D(err) || !checkNoBufferedTrajectory5D(err)) return false;

    if (!checkRotaryMotionAllowed(axis, err)) {
        return false;
    }

    if (axis < AxisX || axis > AxisB || !std::isfinite(pos)) {
        if (err) *err = QStringLiteral("Invalid axis / non-finite absolute target"); return false;
    }
    float controllerTarget = pos;
    float startDpos=0.0f,startMpos=0.0f;
    bool startIdle=false;
    if (!getAxisPositionPairRaw(axis,startDpos,startMpos,startIdle,err)||!startIdle){
        if (err && err->isEmpty()) *err=QStringLiteral("定位前不能回读轴DPOS/MPOS或轴未空闲");
        return false;
    }
    expectedMpos=targetIsMpos?double(pos):(double(startMpos)+double(pos)-double(startDpos));
    const bool compensate = !rawDpos && m_axisCompEnabled.load() && axis <= AxisZ;
    if (compensate || targetIsMpos) {
        float d=0, m=0; bool idle=false;
        if (!getAxisPositionPairRaw(axis,d,m,idle,err)) return false;
        if (!idle) { if(err)*err=QStringLiteral("绝对MPOS定位需要轴已停稳"); return false; }
        double command=0;
        if (compensate) {
            if (!mapLinearTarget(axis,pos,d,m,command,err)) return false;
            settleMs=m_axisCompOptions.pointSettleMs;
        } else {
            // Preserve the instantaneous demand-vs-feedback offset, without
            // rewriting either DPOS or the referenced grating MPOS datum.
            command=static_cast<double>(d)+(static_cast<double>(pos)-static_cast<double>(m));
        }
        if (!std::isfinite(command) || !std::isfinite(static_cast<float>(command))) {
            if(err)*err=QStringLiteral("单轴MPOS目标转换为控制器DPOS时溢出");
            return false;
        }
        controllerTarget=static_cast<float>(command);
    }
    if (axis <= AxisZ && !validateLinearCommand(axis,pos,controllerTarget,err)) return false;
    if (axis == AxisB || axis == AxisC) {
        const float fs = static_cast<float>(m_rotaryMachineSoftLimitPos[axis].load());
        const float rs = static_cast<float>(m_rotaryMachineSoftLimitNeg[axis].load());
        if (pos > fs || pos < rs) {
            if (err) {
                *err = QStringLiteral("%1轴机械绝对目标 %2° 超出软限位 [%3°, %4°]")
                    .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                    .arg(pos, 0, 'f', 4).arg(rs, 0, 'f', 4).arg(fs, 0, 'f', 4);
            }
            return false;
        }
        controllerTarget = machineToControllerPosition(axis, pos);
        float curD = 0.0f, curM = 0.0f;
        bool curIdle = false;
        (void)getAxisPositionPair(axis, curD, curM, curIdle, nullptr);
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("%1 abs move request: machineTarget=%2, controllerTarget=%3, machineDPOS=%4, machineMPOS=%5, machineSoft=[%6,%7], offset=%8")
                     .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                     .arg(pos, 0, 'f', 4).arg(controllerTarget, 0, 'f', 4)
                     .arg(curD, 0, 'f', 4).arg(curM, 0, 'f', 4)
                     .arg(rs, 0, 'f', 4).arg(fs, 0, 'f', 4)
                     .arg(m_rotaryMachineOffset[axis].load(), 0, 'f', 6));
    }

    if (!tryStartAxisTask(axis, AxisTaskMoveAbs, err)) {
        return false;
    }

    taskStarted = true;

    AxisParams effective;
    if (motion) effective=*motion;
    else if (compensate && !readAxisParams(axis,effective,err)) return false;
    if (compensate && m_axisCompTables[axis-AxisX].precisionSpeedMmS>0) {
        const double qualified=m_axisCompTables[axis-AxisX].precisionSpeedMmS;
        if(motion && double(effective.speed)>qualified+1e-5){
            if(err)*err=QStringLiteral("轴%1请求速度%2mm/s高于XYZ补偿验证速度%3mm/s；不允许无提示降速。请选择合格速度或重新验证补偿表")
                .arg(axis).arg(effective.speed).arg(qualified);
            return false;
        }
        effective.speed=static_cast<float>((std::min)(double(effective.speed),qualified));
    }
    if ((motion || compensate) && !writeMotionParams(axis,effective,err)) return false;
    if (motion || compensate) {
        effectiveSpeed=effective.speed;
        effectiveAccel=effective.accel;
        effectiveDecel=effective.decel;
    }
    moveLengthMm=std::abs(double(controllerTarget)-double(startDpos));

    // 单轴控制面板直接使用单轴绝对定位 API，避免 BASE/轴列表状态对 B/C 单轴定位产生干扰。
    int32 ret = ZAux_Direct_Single_MoveAbs(m_handle, axis, controllerTarget);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "Single_MoveAbs");
        return false;
    }
    if (compensate || targetIsMpos) rememberLinearTarget(axis,pos,controllerTarget);
    else invalidateLinearTarget(axis);


        return true;
    }, err, eci::EciIoScheduler::Motion, &operationToken);
    if (!started) return false;
    const int budget=(effectiveSpeed>0&&effectiveAccel>0&&effectiveDecel>0)
        ? mls::calibration::small7::motionTimeoutMs(moveLengthMm,effectiveSpeed,
            effectiveAccel,effectiveDecel,Constants::kMoveTimeoutMs)
        : Constants::kMoveTimeoutMs;
    LOG_INFO(LogCategory::Controller,QStringLiteral("[单轴运动][START] axis=%1 distance=%2mm "
       "expectedMPOS=%3mm effectiveV=%4mm/s budget=%5ms")
       .arg(axis).arg(moveLengthMm,0,'f',4).arg(expectedMpos,0,'f',5)
       .arg(effectiveSpeed,0,'f',3).arg(budget));
    if(!waitAxisIdle(axis,budget,err,expectedMpos,effectiveSpeed)){
        requestStopAxis(axis,false); // a timeout must never leave motion running silently
        return false;
    }
    if (!waitCompensationSettle(settleMs,axis,err)) return false;
    return !m_stopRequested[axis].load();
}

bool ZmcController::moveRel(int axis, float delta, const AxisParams* motion, QString* err)
{ return moveRelImpl(axis, delta, motion, err, false); }
bool ZmcController::moveRelRawDpos(int axis, float delta, const AxisParams* motion, QString* err)
{ return moveRelImpl(axis, delta, motion, err, true); }
bool ZmcController::moveRelImpl(int axis, float delta, const AxisParams* motion, QString* err, bool rawDpos)
{
    if (m_io.isWorkerThread()) {
        if (err) *err = QStringLiteral("阻塞运动门面不能在 ECI I/O 线程调用");
        return false;
    }
    const auto operationToken = m_io.token();
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    bool taskStarted = false;
    int settleMs = 0;
    // runIo() is executed as a lambda: values used by the blocking timeout
    // must be copied to the enclosing scope before the lambda returns.
    double moveLength = std::abs(static_cast<double>(delta));
    double effectiveSpeed = 0.0;
    double effectiveAccel = 0.0;
    double effectiveDecel = 0.0;
    auto taskDone = eci::scopeExit([&] {
        if (taskStarted) runIo(IoPriority::Emergency, "axis.finish", [&] {
            p0::IoLock lock(m_ioMutex); finishAxisTask(axis); return true;
        }, nullptr);
    });
    const bool started = runIo(IoPriority::Motion, "moveRel.start", [&]() -> bool {
        p0::IoLock ioLock(m_ioMutex);

    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    // Preserve task exclusion before ANY SDK read, including compensation/limits.
    if (!checkNoJointTask5D(err) || !checkNoBufferedTrajectory5D(err)) return false;

    if (!checkRotaryMotionAllowed(axis, err)) {
        return false;
    }

    if (axis < AxisX || axis > AxisB || !std::isfinite(delta)) {
        if(err)*err=QStringLiteral("Invalid axis / non-finite relative target");return false;
    }
    const bool compensate = !rawDpos && m_axisCompEnabled.load() && axis <= AxisZ;
    double physicalTarget=0, commandTarget=0;
    if (axis <= AxisZ) {
        float d=0,m=0;bool idle=false;
        if (!getAxisPositionPairRaw(axis,d,m,idle,err)) return false;
        if (!idle) {if(err)*err=QStringLiteral("Relative positioning requires IDLE");return false;}
        physicalTarget=(compensate?double(m):double(d))+double(delta);
        commandTarget=physicalTarget;
        if (compensate && !mapLinearTarget(axis,physicalTarget,d,m,commandTarget,err,true)) return false;
        // Under compensation the controller travels in DPOS, which can differ
        // from the requested physical MPOS increment.
        if (compensate) moveLength = std::abs(commandTarget - static_cast<double>(d));
        if (!validateLinearCommand(axis,physicalTarget,static_cast<float>(commandTarget),err)) return false;
        if (compensate) settleMs=m_axisCompOptions.pointSettleMs;
    }

    if (axis == AxisB || axis == AxisC) {
        float currentDpos = 0.0f;
        float currentMpos = 0.0f;
        bool idle = false;
        if (!getAxisPositionPair(axis, currentDpos, currentMpos, idle, err)) {
            return false;
        }
        const float fs = static_cast<float>(m_rotaryMachineSoftLimitPos[axis].load());
        const float rs = static_cast<float>(m_rotaryMachineSoftLimitNeg[axis].load());
        const float target = currentDpos + delta;
        if (target > fs || target < rs) {
            if (err) {
                *err = QStringLiteral("%1轴相对运动后机械目标 %2° 超出软限位 [%3°, %4°]；当前机械 DPOS=%5°")
                    .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                    .arg(target, 0, 'f', 4).arg(rs, 0, 'f', 4).arg(fs, 0, 'f', 4)
                    .arg(currentDpos, 0, 'f', 4);
            }
            return false;
        }
        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("%1 rel move request: delta=%2, machineTarget=%3, machineDPOS=%4, machineMPOS=%5, machineSoft=[%6,%7]")
                     .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                     .arg(delta, 0, 'f', 4).arg(target, 0, 'f', 4)
                     .arg(currentDpos, 0, 'f', 4).arg(currentMpos, 0, 'f', 4)
                     .arg(rs, 0, 'f', 4).arg(fs, 0, 'f', 4));
    }

    if (!tryStartAxisTask(axis, AxisTaskMoveRel, err)) {
        return false;
    }

    taskStarted = true;

    AxisParams effective;
    if (motion) effective=*motion;
    else if (compensate && !readAxisParams(axis,effective,err)) return false;
    if (compensate && m_axisCompTables[axis-AxisX].precisionSpeedMmS>0) {
        const double qualified=m_axisCompTables[axis-AxisX].precisionSpeedMmS;
        if(motion && double(effective.speed)>qualified+1e-5){
            if(err)*err=QStringLiteral("轴%1相对运动请求速度%2mm/s超过已验证补偿速度%3mm/s")
                .arg(axis).arg(effective.speed).arg(qualified);
            return false;
        }
        effective.speed=static_cast<float>((std::min)(double(effective.speed),qualified));
    }
    if ((motion || compensate) && !writeMotionParams(axis,effective,err)) return false;
    if (motion || compensate) {
        effectiveSpeed = effective.speed;
        effectiveAccel = effective.accel;
        effectiveDecel = effective.decel;
    }

    // Relative physical displacement is MPOS + delta, then one absolute map.
    // Adding an absolute table correction to a DPOS delta would double-count it.
    int32 ret = compensate
        ? ZAux_Direct_Single_MoveAbs(m_handle,axis,static_cast<float>(commandTarget))
        : ZAux_Direct_Single_Move(m_handle,axis,delta);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "Single_Move");
        return false;
    }
    if (compensate) rememberLinearTarget(axis,physicalTarget,static_cast<float>(commandTarget));
    else invalidateLinearTarget(axis);


        return true;
    }, err, eci::EciIoScheduler::Motion, &operationToken);
    if (!started) return false;
    const int budget = (effectiveSpeed > 0.0 && effectiveAccel > 0.0 && effectiveDecel > 0.0)
        ? mls::calibration::small7::motionTimeoutMs(moveLength, effectiveSpeed,
            effectiveAccel, effectiveDecel, Constants::kMoveTimeoutMs)
        : Constants::kMoveTimeoutMs;
    if (!waitAxisIdle(axis,budget,err)) {
        requestStopAxis(axis,false);
        return false;
    }
    if (!waitCompensationSettle(settleMs,axis,err)) return false;
    return !m_stopRequested[axis].load();
}

bool ZmcController::jog(int axis, bool positive, const AxisParams& motion,
                        bool velocityMode, float step, QString* err)
{
    if(axis<AxisX || axis>AxisB){if(err)*err=QStringLiteral("Invalid jog axis");return false;}
    if (!velocityMode) {
        if (!std::isfinite(step) || step < 0) {if(err)*err=QStringLiteral("Invalid jog step");return false;}
        return moveRel(axis,step*(positive?1.0f:-1.0f),&motion,err);
    }
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Motion, "jog", [&] { return jog(axis, positive, motion, velocityMode, step, err); }, err, eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    if (!checkRotaryMotionAllowed(axis, err)) {
        return false;
    }

    if (isAxisBusy(axis)) {
        if (err) *err = QStringLiteral("轴 %1 正在执行定位/回零，不能点动").arg(axis);
        return false;
    }

    QString dummy;
    if (!writeMotionParams(axis, motion, &dummy)) {
        if (err) *err = dummy;
        return false;
    }

    int32 ret = ERR_OK;

    if (axis == AxisB || axis == AxisC) {
        float dpos = 0.0f, mpos = 0.0f;
        bool idle = false;
        if (!getAxisPositionPair(axis, dpos, mpos, idle, err)) return false;
        const float fs = static_cast<float>(m_rotaryMachineSoftLimitPos[axis].load());
        const float rs = static_cast<float>(m_rotaryMachineSoftLimitNeg[axis].load());



        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("%1 jog request: dir=%2, machineDPOS=%3, machineMPOS=%4, speed=%5, machineSoft=[%6,%7]")
                     .arg(axis == AxisB ? QStringLiteral("B") : QStringLiteral("C"))
                     .arg(positive ? QStringLiteral("+") : QStringLiteral("-"))
                     .arg(dpos, 0, 'f', 4).arg(mpos, 0, 'f', 4).arg(motion.speed, 0, 'f', 4)
                     .arg(rs, 0, 'f', 4).arg(fs, 0, 'f', 4));
    }

    // Velocity jog has no physical endpoint. No position-table correction is
    // applied, and stopping must never trigger an unrequested correction move.
    invalidateLinearTarget(axis);
    if (m_axisCompEnabled.load() && axis <= AxisZ)
        LOG_INFO(LogCategory::Controller,QStringLiteral("Velocity JOG remains RAW; use finite step/position for compensated endpoints"));
    // Finite steps already returned through compensated moveRel() above.
    ret = ZAux_Direct_Single_Vmove(m_handle, axis, positive ? 1 : -1);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "Single_Vmove");
        return false;
    }

    return true;
}

bool ZmcController::stopAxis(int axis, bool emergency, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Emergency, "stopAxis", [&] { return stopAxis(axis, emergency, err); }, err, eci::EciIoScheduler::None, nullptr, eci::EciIoScheduler::Motion | eci::EciIoScheduler::Acquisition);
    }

    p0::IoLock ioLock(m_ioMutex);
    invalidateLinearTarget();
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    if (axis >= AxisX && axis <= AxisB &&
        m_axisTask[axis].load() == AxisTaskJointMove5D) {
        return stopJointMove5D(emergency, err);
    }
    const bool bufferedFiveAxis = axis >= AxisX && axis <= AxisB && !m_mergePath.empty();
    if (axis >= AxisX && axis <= AxisB) {
        ++m_mergeRevision;
        m_mergePath.clear(); // never re-feed cancelled host segments on the next timer
        m_mergeIndex = 0;
    }

    if (axis >= 0 && axis < kMaxAxisCount) {
        if (m_axisTask[axis].load() != AxisTaskNone) {
            m_stopRequested[axis].store(true);
        }
    }

    Q_UNUSED(emergency)
    int mode = 2; // cancel current AND buffered; mode 1 is buffer-only
    int axes[5] = {AxisX,AxisY,AxisZ,AxisC,AxisB};
    int32 ret = bufferedFiveAxis ? ZAux_Direct_CancelAxisList(m_handle,5,axes,mode)
                                : ZAux_Direct_Single_Cancel(m_handle,axis,mode);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "Single_Cancel");
        return false;
    }

    return true;
}

bool ZmcController::zeroAllDpos(QString* err)
{
    // Redefining DPOS destroys the machine-coordinate basis of the saved calibration.
    // G92 is implemented in the NC interpreter as an offset, without touching DPOS.
    if (err) *err = QStringLiteral("五轴标定模式禁止直接清零DPOS；请使用G92程序坐标偏置，或正规回零后重新确认标定坐标基准");
    return false;
}

// ==================== P4 五轴关节空间同步定位 ====================

bool ZmcController::validateJointTargetAgainstControllerLimits5D(
    const JointPose5D& machineTarget,
    JointPose5D& controllerTarget,
    QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "validateJointTargetAgainstControllerLimits5D", [&] { return validateJointTargetAgainstControllerLimits5D(machineTarget, controllerTarget, err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    if (!machineTarget.isFinite()) {
        if (err) *err = QStringLiteral("五轴目标包含 NaN/Inf");
        return false;
    }

    controllerTarget = machineTarget;
    controllerTarget.c = static_cast<double>(machineToControllerPosition(
        AxisC, static_cast<float>(machineTarget.c)));
    controllerTarget.b = static_cast<double>(machineToControllerPosition(
        AxisB, static_cast<float>(machineTarget.b)));

    if (m_axisCompEnabled.load()) {
        for (int axis=AxisX;axis<=AxisZ;++axis) {
            float d=0,m=0;bool idle=false;
            if(!getAxisPositionPairRaw(axis,d,m,idle,err))return false;
            if(!idle){if(err)*err=QStringLiteral("Compensated joint target requires idle XYZ");return false;}
            double command=0;
            if(!mapLinearTarget(axis,jointValue(machineTarget,axis),d,m,command,err))return false;
            // Validate the actual float that will cross the SDK boundary as well.
            if(!validateLinearCommand(axis,jointValue(machineTarget,axis),static_cast<float>(command),err))return false;
            setJointValue(controllerTarget,axis,static_cast<float>(command));
        }
    }

    // 机械坐标层先复核 B/C 限位。
    for (const int axis : { AxisC, AxisB }) {
        const double machineValue = jointValue(machineTarget, axis);
        const double machineRs = m_rotaryMachineSoftLimitNeg[axis].load();
        const double machineFs = m_rotaryMachineSoftLimitPos[axis].load();
        if (machineValue < machineRs - 1e-6 || machineValue > machineFs + 1e-6) {
            if (err) {
                *err = QStringLiteral("%1轴机械目标 %2° 超出软限位 [%3°, %4°]")
                           .arg(jointAxisName(axis))
                           .arg(machineValue, 0, 'f', 6)
                           .arg(machineRs, 0, 'f', 6)
                           .arg(machineFs, 0, 'f', 6);
            }
            return false;
        }
    }

    // 控制器坐标层读取实际 FS_LIMIT / RS_LIMIT 再复核一次。
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        float fs = 0.0f;
        float rs = 0.0f;
        int32 ret = ZAux_Direct_GetFsLimit(m_handle, axis, &fs);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetFsLimit(joint5D)");
            return false;
        }
        ret = ZAux_Direct_GetRsLimit(m_handle, axis, &rs);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetRsLimit(joint5D)");
            return false;
        }

        const double ctl = jointValue(controllerTarget, axis);
        if (ctl < static_cast<double>(rs) - 1e-5 ||
            ctl > static_cast<double>(fs) + 1e-5) {
            if (err) {
                *err = QStringLiteral("%1轴控制器目标 %2 超出控制器软限位 [%3, %4]")
                           .arg(jointAxisName(axis))
                           .arg(ctl, 0, 'f', 6)
                           .arg(rs, 0, 'f', 6)
                           .arg(fs, 0, 'f', 6);
            }
            return false;
        }
    }
    return true;
}

bool ZmcController::areFiveJointAxesIdle(QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "areFiveJointAxesIdle", [&] { return areFiveJointAxesIdle(err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    for (int axis = AxisX; axis <= AxisB; ++axis) {
        int idle = 0;
        const int32 ret = ZAux_Direct_GetIfIdle(m_handle, axis, &idle);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetIfIdle(5D)");
            return false;
        }
        if (idle == 0) return false;
    }
    return true;
}

bool ZmcController::moveJointAbs5D(const JointMotionPlan5D& plan,
                                   JointMoveResult5D& result,
                                   QString* err)
{
    if (m_io.isWorkerThread()) {
        if (err) *err = QStringLiteral("五轴阻塞门面不能在 ECI I/O 线程调用");
        return false;
    }
    const auto operationToken = m_io.token();
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    bool taskStarted = false;
    int settleMs = 0;
    JointPose5D expectedDemand;

    auto taskDone = eci::scopeExit([&] {
        if (taskStarted) runIo(IoPriority::Emergency, "joint.finish", [&] {
            p0::IoLock lock(m_ioMutex); finishJointTask5D(); return true;
        }, nullptr);
    });
    const bool started = runIo(IoPriority::Motion, "joint.start", [&]() -> bool {
        p0::IoLock ioLock(m_ioMutex);

    result = JointMoveResult5D{};

    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    if (m_hwSampling || m_mergeIndex < m_mergePath.size()) {
        if (err) *err = QStringLiteral("扫描采样或未下发的三轴轨迹仍在使用控制卡，禁止五轴定位");
        return false;
    }
    if (!plan.valid || !plan.target.isFinite()) {
        if (err) *err = plan.error.isEmpty()
            ? QStringLiteral("五轴同步运动计划无效") : plan.error;
        return false;
    }
    if (plan.baseAxis < AxisX || plan.baseAxis > AxisB) {
        if (err) *err = QStringLiteral("五轴运动计划主轴号无效");
        return false;
    }

    // 五轴联动绝不能绕过 V15 已验证的 B/C absoluteReady + 人工使能链。
    if (!checkRotaryMotionAllowed(AxisC, err) ||
        !checkRotaryMotionAllowed(AxisB, err)) {
        return false;
    }

    if (!tryStartJointTask5D(err)) {
        return false;
    }

    taskStarted = true;

    // P4 只允许从静止状态启动，避免与三轴插补/GCode/其它异步运动混用。
    JointState5D before;
    if (!readJointState5D(before, err)) {
        return false;
    }
    if (!before.allIdle) {
        if (err) *err = QStringLiteral("至少一个轴仍在运动，禁止启动五轴同步定位");
        return false;
    }

    // 计划是从某一五轴起点计算出来的。若期间已被单轴操作改变，则必须重新规划。
    double startLinearDiff = 0.0;
    double startRotaryDiff = 0.0;
    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        updateMaximumValue(startLinearDiff,
                           std::abs(jointValue(before.mpos, axis) - jointValue(plan.start, axis)));
    }
    for (const int axis : { AxisC, AxisB }) {
        updateMaximumValue(startRotaryDiff,
                           std::abs(jointValue(before.mpos, axis) - jointValue(plan.start, axis)));
    }
    if (startLinearDiff > 0.50 || startRotaryDiff > 0.20) {
        if (err) {
            *err = QStringLiteral("五轴当前位置已在规划后发生变化，请重新规划。最大直线差=%1mm，旋转差=%2°")
                       .arg(startLinearDiff, 0, 'f', 6)
                       .arg(startRotaryDiff, 0, 'f', 6);
        }
        return false;
    }

    // 事件式读取一次 AXISSTATUS，并确认没有遗留运动缓冲。多轴联动遇到任何报警都拒绝启动；
    // 如处于软限位，请先使用单轴功能沿允许方向退出，再重新执行五轴定位。
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        int statusWord = 0;
        if (!getAxisStatusWord(axis, statusWord, err)) return false;
        if (axisStatusHasAlarm(statusWord)) {
            if (err) {
                *err = QStringLiteral("%1轴存在报警，禁止五轴同步定位: 0x%2 (%3)")
                           .arg(jointAxisName(axis))
                           .arg(static_cast<quint32>(statusWord), 0, 16)
                           .arg(describeAxisStatusWord(statusWord));
            }
            return false;
        }

        int movesBuffered = 0;
        const int32 retBuffered = ZAux_Direct_GetMovesBuffered(m_handle, axis, &movesBuffered);
        if (retBuffered != ERR_OK) {
            if (err) *err = errorTextFromCode(retBuffered, "GetMovesBuffered(joint5D)");
            return false;
        }
        if (movesBuffered != 0) {
            if (err) {
                *err = QStringLiteral("%1轴仍有 %2 条运动缓冲，禁止启动新的五轴同步定位")
                           .arg(jointAxisName(axis)).arg(movesBuffered);
            }
            return false;
        }
    }

    JointPose5D controllerTarget;
    if (!validateJointTargetAgainstControllerLimits5D(plan.target, controllerTarget, err)) {
        return false;
    }

    expectedDemand=controllerTarget;
    if(m_axisCompEnabled.load())settleMs=m_axisCompOptions.pointSettleMs;

    if (!plan.hasMotion) {
        if (!readJointState5D(result.finalState, err)) return false;
        result.completed = true;
        result.message = QStringLiteral("当前五轴已在目标位置，无需运动");
        return true;
    }

    if (!std::isfinite(plan.vectorSpeed) || plan.vectorSpeed <= 0.0 ||
        !std::isfinite(plan.vectorAccel) || plan.vectorAccel <= 0.0 ||
        !std::isfinite(plan.vectorDecel) || plan.vectorDecel <= 0.0) {
        if (err) *err = QStringLiteral("五轴计划的矢量速度/加速度无效");
        return false;
    }

    // 让限制时间最长的轴作为 BASE 列表第一轴；正运动以该轴保存插补矢量参数。
    int axisList[5] = {0, 0, 0, 0, 0};
    float dest[5] = {0, 0, 0, 0, 0};
    axisList[0] = plan.baseAxis;
    int outIndex = 1;
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (axis == plan.baseAxis) continue;
        axisList[outIndex++] = axis;
    }
    for (int i = 0; i < 5; ++i) {
        dest[i] = static_cast<float>(jointValue(controllerTarget, axisList[i]));
    }

    double vectorSpeed=plan.vectorSpeed, vectorAccel=plan.vectorAccel, vectorDecel=plan.vectorDecel;
    if(m_axisCompEnabled.load()) {
        double length=0;
        for(int axis=AxisX;axis<=AxisB;++axis) {
            const double start=axis<=AxisZ?jointValue(before.dpos,axis):
                machineToControllerPosition(axis,static_cast<float>(jointValue(before.mpos,axis)));
            length=std::hypot(length,jointValue(controllerTarget,axis)-start);
        }
        if(length>1e-12)for(int axis=AxisX;axis<=AxisB;++axis) {
            const double start=axis<=AxisZ?jointValue(before.dpos,axis):
                machineToControllerPosition(axis,static_cast<float>(jointValue(before.mpos,axis)));
            const double ratio=std::abs(jointValue(controllerTarget,axis)-start)/length;
            if(ratio<=1e-14)continue;
            double speedLimit=m_trajectoryLimits.speed[axis-AxisX];
            if(axis<=AxisZ && m_axisCompTables[axis-AxisX].precisionSpeedMmS>0)
                speedLimit=(std::min)(speedLimit,m_axisCompTables[axis-AxisX].precisionSpeedMmS);
            vectorSpeed=(std::min)(vectorSpeed,speedLimit/ratio);
            vectorAccel=(std::min)(vectorAccel,m_trajectoryLimits.accel[axis-AxisX]/ratio);
            vectorDecel=(std::min)(vectorDecel,m_trajectoryLimits.decel[axis-AxisX]/ratio);
        }
    }
    const int baseAxis = plan.baseAxis;
    int32 ret = ZAux_Direct_SetSpeed(m_handle, baseAxis, static_cast<float>(vectorSpeed));
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSpeed(joint5D)"); return false; }
    ret = ZAux_Direct_SetAccel(m_handle, baseAxis, static_cast<float>(vectorAccel));
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetAccel(joint5D)"); return false; }
    ret = ZAux_Direct_SetDecel(m_handle, baseAxis, static_cast<float>(vectorDecel));
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetDecel(joint5D)"); return false; }
    ret = ZAux_Direct_SetLspeed(m_handle, baseAxis, 0.0f);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetLspeed(joint5D)"); return false; }
    if (plan.srampMs >= 0.0 && std::isfinite(plan.srampMs)) {
        ret = ZAux_Direct_SetSramp(m_handle, baseAxis, static_cast<float>(plan.srampMs));
        if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSramp(joint5D)"); return false; }
    }
    // P4 是单段点到点定位，明确关闭连续 MERGE，避免继承旧三轴连续插补状态。
    ret = ZAux_Direct_SetMerge(m_handle, baseAxis, 0);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetMerge(joint5D=0)"); return false; }

    // 显式设置 BASE 便于控制器状态/调试与直接 MoveAbs 使用同一轴序。
    ret = ZAux_Direct_Base(m_handle, 5, axisList);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "Base(joint5D)"); return false; }

    LOG_INFO(LogCategory::Controller,
             QStringLiteral("P4 five-axis MOVEABS: base=%1, machineTarget=[X=%2 Y=%3 Z=%4 C=%5 B=%6], controllerC=%7, controllerB=%8, V=%9, A=%10, D=%11, scale=%12%")
                 .arg(jointAxisName(baseAxis))
                 .arg(plan.target.x, 0, 'f', 4).arg(plan.target.y, 0, 'f', 4)
                 .arg(plan.target.z, 0, 'f', 4).arg(plan.target.c, 0, 'f', 4)
                 .arg(plan.target.b, 0, 'f', 4)
                 .arg(controllerTarget.c, 0, 'f', 4).arg(controllerTarget.b, 0, 'f', 4)
                 .arg(plan.vectorSpeed, 0, 'f', 4).arg(plan.vectorAccel, 0, 'f', 4)
                 .arg(plan.vectorDecel, 0, 'f', 4).arg(plan.speedScale * 100.0, 0, 'f', 1));

    ret = ZAux_Direct_MoveAbs(m_handle, 5, axisList, dest);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "MoveAbs(5 axis)");
        return false;
    }
    if(m_axisCompEnabled.load())for(int axis=AxisX;axis<=AxisZ;++axis)
        rememberLinearTarget(axis,jointValue(plan.target,axis),jointValue(controllerTarget,axis));



        return true;
    }, err, eci::EciIoScheduler::Motion, &operationToken);
    if (!started) return false;
    if (result.completed) return true; // no-motion plan
    if (!waitJointMoveIdle5D(plan.baseAxis, Constants::kMoveTimeoutMs, err)) return false;
    if(!waitCompensationSettle(settleMs,-1,err))return false;
    if (!readJointState5D(result.finalState, err)) {
        return false;
    }

    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        updateMaximumValue(result.maxLinearDposErrorMm,
                           std::abs(jointValue(result.finalState.dpos, axis) - jointValue(expectedDemand, axis)));
        updateMaximumValue(result.maxLinearMposErrorMm,
                           std::abs(jointValue(result.finalState.mpos, axis) - jointValue(plan.target, axis)));
    }
    for (const int axis : { AxisC, AxisB }) {
        updateMaximumValue(result.maxRotaryMposErrorDeg,
                           std::abs(jointValue(result.finalState.mpos, axis) - jointValue(plan.target, axis)));
    }

    result.completed = true;
    result.message = QStringLiteral("五轴同步定位完成");
    return true;
}

bool ZmcController::moveJointAbs5DIndependent(
    const JointPose5D& target,
    const std::array<AxisParams, Constants::kAxisParamArraySize>& axisParams,
    double speedScale,
    JointMoveResult5D& result,
    QString* err)
{
    if (m_io.isWorkerThread()) {
        if (err) *err = QStringLiteral("五轴独立阻塞门面不能在 ECI I/O 线程调用");
        return false;
    }
    if (!target.isFinite() || !std::isfinite(speedScale) || speedScale <= 0.0 || speedScale > 1.0) {
        if (err) *err = QStringLiteral("五轴独立定位目标或速度倍率无效");
        return false;
    }

    const auto operationToken = m_io.token();
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    bool taskStarted = false;
    int settleMs = 0;
    JointPose5D expectedDemand;
    int waitAxis = AxisX;
    auto taskDone = eci::scopeExit([&] {
        if (taskStarted) runIo(IoPriority::Emergency, "jointIndependent.finish", [&] {
            p0::IoLock lock(m_ioMutex); finishJointTask5D(); return true;
        }, nullptr);
    });

    const bool started = runIo(IoPriority::Motion, "jointIndependent.start", [&]() -> bool {
        p0::IoLock ioLock(m_ioMutex);
        result = JointMoveResult5D{};

        if (!m_handle) {
            if (err) *err = QStringLiteral("尚未连接控制器");
            return false;
        }
        if (m_hwSampling || m_mergeIndex < m_mergePath.size()) {
            if (err) *err = QStringLiteral("扫描采样或未下发的轨迹仍在使用控制卡，禁止五轴独立定位");
            return false;
        }
        if (!checkRotaryMotionAllowed(AxisC, err) || !checkRotaryMotionAllowed(AxisB, err)) {
            return false;
        }
        if (!tryStartJointTask5D(err)) return false;
        taskStarted = true;

        JointState5D before;
        if (!readJointState5D(before, err)) return false;
        if (!before.allIdle) {
            if (err) *err = QStringLiteral("至少一个轴仍在运动，禁止启动五轴独立定位");
            return false;
        }

        for (int axis = AxisX; axis <= AxisB; ++axis) {
            const AxisParams& p = axisParams[axis];
            if (!std::isfinite(p.speed) || p.speed <= 0.0f ||
                !std::isfinite(p.accel) || p.accel <= 0.0f ||
                !std::isfinite(p.decel) || p.decel <= 0.0f) {
                if (err) *err = QStringLiteral("%1轴独立定位速度/加减速度参数无效").arg(jointAxisName(axis));
                return false;
            }

            int statusWord = 0;
            if (!getAxisStatusWord(axis, statusWord, err)) return false;
            if (axisStatusHasAlarm(statusWord)) {
                if (err) {
                    *err = QStringLiteral("%1轴存在报警，禁止五轴独立定位: 0x%2 (%3)")
                               .arg(jointAxisName(axis))
                               .arg(static_cast<quint32>(statusWord), 0, 16)
                               .arg(describeAxisStatusWord(statusWord));
                }
                return false;
            }
            int movesBuffered = 0;
            const int32 retBuffered = ZAux_Direct_GetMovesBuffered(m_handle, axis, &movesBuffered);
            if (retBuffered != ERR_OK) {
                if (err) *err = errorTextFromCode(retBuffered, "GetMovesBuffered(jointIndependent)");
                return false;
            }
            if (movesBuffered != 0) {
                if (err) *err = QStringLiteral("%1轴仍有 %2 条运动缓冲，禁止五轴独立定位")
                                    .arg(jointAxisName(axis)).arg(movesBuffered);
                return false;
            }
        }

        JointPose5D controllerTarget;
        if (!validateJointTargetAgainstControllerLimits5D(target, controllerTarget, err)) return false;
        expectedDemand=controllerTarget;
        if(m_axisCompEnabled.load())settleMs=m_axisCompOptions.pointSettleMs;


        bool anyMotion = false;
        for (int axis = AxisX; axis <= AxisB; ++axis) {
            const double delta = axis<=AxisZ && m_axisCompEnabled.load()
                ? std::abs(jointValue(before.dpos,axis)-jointValue(controllerTarget,axis))
                : std::abs(jointValue(before.mpos,axis)-jointValue(target,axis));
            const double tolerance = axis <= AxisZ ? 1.0e-6 : 1.0e-7;
            if (delta <= tolerance) continue;

            anyMotion = true;
            waitAxis = axis;
            const AxisParams& p = axisParams[axis];
            double effectiveSpeed=static_cast<double>(p.speed)*speedScale;
            if(m_axisCompEnabled.load() && axis<=AxisZ && m_axisCompTables[axis-AxisX].precisionSpeedMmS>0)
                effectiveSpeed=(std::min)(effectiveSpeed,m_axisCompTables[axis-AxisX].precisionSpeedMmS);
            const float speed = static_cast<float>(effectiveSpeed);
            const float accel = static_cast<float>(static_cast<double>(p.accel) * speedScale);
            const float decel = static_cast<float>(static_cast<double>(p.decel) * speedScale);

            // Independent calibration transit deliberately starts from rest with no
            // S-curve coupling.  This makes each axis motion profile deterministic
            // and prevents a short-travel axis from inheriting an interpolated
            // distance-ratio velocity from a long-travel axis.
            int32 ret = ZAux_Direct_SetLspeed(m_handle, axis, 0.0f);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetLspeed(jointIndependent)"); return false; }
            ret = ZAux_Direct_SetSpeed(m_handle, axis, speed);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSpeed(jointIndependent)"); return false; }
            ret = ZAux_Direct_SetAccel(m_handle, axis, accel);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetAccel(jointIndependent)"); return false; }
            ret = ZAux_Direct_SetDecel(m_handle, axis, decel);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetDecel(jointIndependent)"); return false; }
            ret = ZAux_Direct_SetSramp(m_handle, axis, 0.0f);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetSramp(jointIndependent)"); return false; }
            ret = ZAux_Direct_SetMerge(m_handle, axis, 0);
            if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "SetMerge(jointIndependent=0)"); return false; }
        }

        if (!anyMotion) {
            result.finalState = before;
            result.completed = true;
            result.message = QStringLiteral("当前五轴已在目标位置，无需独立运动");
            return true;
        }

        LOG_INFO(LogCategory::Controller,
                 QStringLiteral("Calibration independent five-axis move: target=[X=%1 Y=%2 Z=%3 C=%4 B=%5], speedScale=%6%; each axis uses own SPEED/ACCEL/DECEL; no multi-axis interpolation")
                     .arg(target.x, 0, 'f', 4).arg(target.y, 0, 'f', 4).arg(target.z, 0, 'f', 4)
                     .arg(target.c, 0, 'f', 4).arg(target.b, 0, 'f', 4).arg(speedScale * 100.0, 0, 'f', 1));

        // Configure every active axis first, then issue all absolute targets in the
        // same scheduler transaction.  Commands are independent Single_MoveAbs calls;
        // there is no BASE list and no MoveAbs(5 axis) interpolation.
        for (int axis = AxisX; axis <= AxisB; ++axis) {
            const double delta = axis<=AxisZ && m_axisCompEnabled.load()
                ? std::abs(jointValue(before.dpos,axis)-jointValue(controllerTarget,axis))
                : std::abs(jointValue(before.mpos,axis)-jointValue(target,axis));
            const double tolerance = axis <= AxisZ ? 1.0e-6 : 1.0e-7;
            if (delta <= tolerance) continue;
            const float destination = static_cast<float>(jointValue(controllerTarget, axis));
            const int32 ret = ZAux_Direct_Single_MoveAbs(m_handle, axis, destination);
            if (ret != ERR_OK) {
                // Some axes may already have started; cancel the complete five-axis
                // set before returning so a partial command cannot continue silently.
                int axisList[5] = { AxisX, AxisY, AxisZ, AxisC, AxisB };
                (void)ZAux_Direct_CancelAxisList(m_handle, 5, axisList, 2);
                if (err) *err = errorTextFromCode(ret, "Single_MoveAbs(jointIndependent)");
                return false;
            }
        }
        if(m_axisCompEnabled.load())for(int axis=AxisX;axis<=AxisZ;++axis)
            rememberLinearTarget(axis,jointValue(target,axis),jointValue(controllerTarget,axis));
        return true;
    }, err, eci::EciIoScheduler::Motion, &operationToken);

    if (!started) return false;
    if (result.completed) return true;
    if (!waitJointMoveIdle5D(waitAxis, Constants::kMoveTimeoutMs, err)) return false;
    if(!waitCompensationSettle(settleMs,-1,err))return false;
    if (!readJointState5D(result.finalState, err)) return false;

    for (int axis = AxisX; axis <= AxisZ; ++axis) {
        updateMaximumValue(result.maxLinearDposErrorMm,
                           std::abs(jointValue(result.finalState.dpos, axis) - jointValue(expectedDemand, axis)));
        updateMaximumValue(result.maxLinearMposErrorMm,
                           std::abs(jointValue(result.finalState.mpos, axis) - jointValue(target, axis)));
    }
    for (const int axis : { AxisC, AxisB }) {
        updateMaximumValue(result.maxRotaryMposErrorDeg,
                           std::abs(jointValue(result.finalState.mpos, axis) - jointValue(target, axis)));
    }
    result.completed = true;
    result.message = QStringLiteral("五轴独立绝对定位完成（各轴独立速度，非五轴插补）");
    return true;
}

bool ZmcController::stopJointMove5D(bool emergency, QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Emergency, "stopJointMove5D", [&] { return stopJointMove5D(emergency, err); }, err, eci::EciIoScheduler::None, nullptr, eci::EciIoScheduler::Motion | eci::EciIoScheduler::Acquisition);
    }

    p0::IoLock ioLock(m_ioMutex);
    invalidateLinearTarget();
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }

    int axisList[5] = { AxisX, AxisY, AxisZ, AxisC, AxisB };
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        if (m_axisTask[axis].load() == AxisTaskJointMove5D) {
            m_stopRequested[axis].store(true);
        }
    }

    // mode=2: 当前+缓冲运动按 FASTDEC 取消；mode=3: 立即中断脉冲。
    Q_UNUSED(emergency)
    const int mode = 2; // current + buffered; physical safety circuit remains mandatory
    const int32 ret = ZAux_Direct_CancelAxisList(m_handle, 5, axisList, mode);
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "CancelAxisList(joint5D)");
        return false;
    }
    return true;
}

// ==================== 原三轴插补：保留指令语义，V19将补缓冲分批调度 ====================

bool ZmcController::moveLineAbs(const PathPoint& target, const AxisParams& motion, QString* err)
{
    // Even a single G01 is the same checked, five-axis SP path.
    if (!prepareMergePath({target}, motion, err)) return false;
    bool sent=false;
    return pumpMergeBuffer(sent, err);
}


bool ZmcController::prepareMergePath(const std::vector<PathPoint>& path,
                                     const AxisParams& motion, QString* err, PathStartReference startReference)
{
    if (!m_io.isWorkerThread()) return runIo(IoPriority::Motion, "prepareMergePath5D", [&] {
        return prepareMergePath(path,motion,err,startReference);
    },err,eci::EciIoScheduler::Motion);
    p0::IoLock lock(m_ioMutex);
    if (!m_handle || !checkNoJointTask5D(err) || !checkRotaryMotionAllowed(AxisC,err) ||
        !checkRotaryMotionAllowed(AxisB,err)) return false;
    JointState5D state;
    if (!readJointState5D(state,err)) return false;
    if (!state.allIdle || m_mergeIndex < m_mergePath.size()) {
        if(err)*err=QStringLiteral("五轴或上位机队列尚未停止，不能覆盖轨迹"); return false;
    }
    using namespace mls::runtime;
    // The SP queue is expressed in controller demand coordinates. Keep DPOS as the
    // command-trajectory start here; measured/RTCP/calibration pose paths use MPOS.
    const bool compensate=m_axisCompEnabled.load();
    if(compensate && (!m_axisCompLoaded || !areLinearMposReferenced())){
        if(err)*err=QStringLiteral("XYZ compensation requires current-session DATUM and a complete table set");return false;
    }
    Joint commandStart{state.dpos.x,state.dpos.y,state.dpos.z,state.mpos.c,state.mpos.b};
    Joint start=commandStart;
    if(compensate){
        start[0]=compensatedNominalStart(AxisX,state.dpos.x,state.mpos.x);
        start[1]=compensatedNominalStart(AxisY,state.dpos.y,state.mpos.y);
        start[2]=compensatedNominalStart(AxisZ,state.dpos.z,state.mpos.z);
        // RTCP computes FK/IK from the measured joint pose. Match that same start,
        // rather than substituting an earlier nominal target with residual error.
        if(startReference==PathStartReference::MeasuredMpos)
            start={state.mpos.x,state.mpos.y,state.mpos.z,state.mpos.c,state.mpos.b};
    }
    Limits limits=m_trajectoryLimits;
    m_mergeOffsets={m_rotaryMachineOffset[AxisC].load(),m_rotaryMachineOffset[AxisB].load()};
    for(int j=0;j<5;++j) {
        float lo=0,hi=0; const int axis=j+1;
        int32 r=ZAux_Direct_GetRsLimit(m_handle,axis,&lo);
        if(r==ERR_OK)r=ZAux_Direct_GetFsLimit(m_handle,axis,&hi);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetLimits(path5D)");return false;}
        if(!std::isfinite(lo)||!std::isfinite(hi)||lo>hi){if(err)*err=QStringLiteral("控制器软限位无效");return false;}
        const double offset=j>=3?m_mergeOffsets[j-3]:0.0;
        limits.minimum[j]=(std::max)(limits.minimum[j],double(lo)+offset);
        limits.maximum[j]=(std::min)(limits.maximum[j],double(hi)+offset);
        if(j>=3){
            limits.minimum[j]=(std::max)(limits.minimum[j],m_rotaryMachineSoftLimitNeg[axis].load());
            limits.maximum[j]=(std::min)(limits.maximum[j],m_rotaryMachineSoftLimitPos[axis].load());
        }
        int status=0;r=ZAux_Direct_GetAxisStatus(m_handle,axis,&status);
        // Exclude normal HOLD/paused/homing bits; faults/limits are not ignored.
        constexpr int faults=2|4|8|16|32|256|512|1024|4096|16384|262144|524288|2097152|4194304;
        if(r!=ERR_OK || (status&faults)){if(err)*err=QStringLiteral("轴报警/限位或状态读取失败，拒绝轨迹");return false;}
    }
    std::vector<Waypoint> wp;wp.reserve(path.size());Joint previous=start;std::string detail;
    for(const auto& p:path){
        Joint values{p.x,p.y,p.z,p.c,p.b},q;
        if(!resolve(previous,values,p.axisMask,true,q,detail)){if(err)*err=QString::fromStdString(detail);return false;}
        wp.push_back({q,p.feed>0?p.feed:motion.speed,p.feedMode,p.continuous});previous=q;
    }
    Plan staged;
    // Original physical path must be valid before transforming demands.
    if(!plan(start,wp,limits,motion.accel,motion.decel,staged,detail)){
        if(err)*err=QString::fromStdString(detail);return false;
    }
    Joint finalNominal=wp.empty()?start:wp.back().q;
    if(compensate){
        std::vector<Waypoint> commandWaypoints;
        if(!mls::axiscomp::mapPath(m_axisCompTables,m_axisCompOptions,start,commandStart,wp,commandWaypoints,detail)){
            if(err)*err=QString::fromStdString(detail);return false;
        }
        if(commandWaypoints.empty()){staged=Plan{};staged.accel=motion.accel;staged.decel=motion.decel;}
        else {
            // SDK uses float coordinates; round now, then run the actual limits,
            // speed/acceleration and G93 feasibility checks on what will be sent.
            for(auto& w:commandWaypoints)for(int j=0;j<3;++j)w.q[j]=static_cast<float>(w.q[j]);
            if(!plan(commandStart,commandWaypoints,limits,motion.accel,motion.decel,staged,detail)){
                if(err)*err=QString::fromStdString(detail);return false;
            }
        }
    }
    // No SDK motion is emitted before EVERY target and the complete speed plan validate.
    auto checked=[&](int32 r,const char* name){if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,name);return false;}return true;};
    for(int axis=AxisX;axis<=AxisB;++axis)
        if(!checked(ZAux_Direct_SetInterpFactor(m_handle,axis,1),"SetInterpFactor5D"))return false;
    if(!checked(ZAux_Direct_SetAccel(m_handle,AxisX,float(staged.accel)),"SetAccel(path5D)") ||
       !checked(ZAux_Direct_SetDecel(m_handle,AxisX,float(staged.decel)),"SetDecel(path5D)") ||
       !checked(ZAux_Direct_SetSramp(m_handle,AxisX,0),"SetSramp(path5D)") ||
       !checked(ZAux_Direct_SetLspeed(m_handle,AxisX,0),"SetLspeed(path5D)") ||
       !checked(ZAux_Direct_SetMerge(m_handle,AxisX,1),"SetMerge(path5D)"))return false;
    const bool smooth=std::any_of(path.begin(),path.end(),[](const PathPoint& p){return p.continuous;});
    if(!checked(ZAux_Direct_SetCornerMode(m_handle,AxisX,smooth?2:0),"SetCornerMode(path5D)"))return false;
    if(smooth){
        constexpr float pi=3.14159265358979323846f;
        if(!checked(ZAux_Direct_SetDecelAngle(m_handle,AxisX,5.0f*pi/180.0f),"SetDecelAngle(path5D)") ||
           !checked(ZAux_Direct_SetStopAngle(m_handle,AxisX,30.0f*pi/180.0f),"SetStopAngle(path5D)"))return false;
    }
    m_mergeUsesAxisCompensation=compensate;m_mergeFinalNominal=finalNominal;
    m_mergePlan=std::move(staged);m_trajectorySeconds.store(m_mergePlan.seconds);
    m_mergePath.clear();m_mergePath.reserve(m_mergePlan.segments.size());
    for(const auto& seg:m_mergePlan.segments){PathPoint p;p.x=seg.target[0];p.y=seg.target[1];p.z=seg.target[2];p.c=seg.target[3];p.b=seg.target[4];p.axisMask=31;m_mergePath.push_back(p);}
    ++m_mergeRevision;m_mergeIndex=0;m_mergeRefilling=true;m_mergePaused=false;
    m_mergeToken=m_io.transactionToken();
    return true;
}


bool ZmcController::pumpMergeBuffer(bool& finished, QString* err)
{
    if (!m_io.isWorkerThread()) {
        const auto operationToken = m_io.token();
        for (;;) {
            int sent = 0;
            if (!runIo(IoPriority::Motion, "merge.refillBatch", [&] {
                    const bool ok = pumpMergeBuffer(finished, err);
                    sent = m_mergeLastBatchSent;
                    return ok;
                }, err, eci::EciIoScheduler::Motion, &operationToken)) return false;
            if (finished || sent == 0) return true;
            // No I/O lock held: due TABLE/STOP jobs can run before the next batch.
        }
    }
    p0::IoLock lock(m_ioMutex);
    finished = false;
    m_mergeLastBatchSent = 0;
    if (!m_handle) { if (err) *err = QStringLiteral("尚未连接控制卡"); return false; }
    if (m_mergeIndex >= m_mergePath.size()) { finished = true; return true; }
    if (!checkNoJointTask5D(err)) return false;
    const auto nowToken = m_io.token();
    if (nowToken.session != m_mergeToken.session || nowToken.motion != m_mergeToken.motion) {
        m_mergePath.clear(); m_mergeIndex = 0; finished = true;
        return true; // an old producer must never resurrect a stopped path
    }
    if (m_mergePaused) return true;
    int remain = 0, buffered = 0;
    const int baseAxis = AxisX;
    auto ret = ZAux_Direct_GetRemain_LineBuffer(m_handle, baseAxis, &remain);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetRemain_LineBuffer"); return false; }
    ret = ZAux_Direct_GetMovesBuffered(m_handle, baseAxis, &buffered);
    if (ret != ERR_OK) { if (err) *err = errorTextFromCode(ret, "GetMovesBuffered(refill)"); return false; }
    // Host thresholds, NOT claimed ECI3808 capacity. Remain_LineBuffer is the
    // authority. Small batches return to scheduler between refills for TABLE/STOP.
    constexpr int low = 16, high = 48, maxBatch = 8;
    if (buffered <= low) m_mergeRefilling = true;
    if (buffered >= high) m_mergeRefilling = false;
    if (!m_mergeRefilling) return true;
    int count = (std::min)({maxBatch, (std::max)(0, remain - 1), (std::max)(0, high - buffered)});
    int axes[5] = {AxisX, AxisY, AxisZ, AxisC, AxisB};
    for (int i=0;i<count && m_mergeIndex<m_mergePath.size();++i) {
        // GetRemain_Buffer is conservative for the complete SP command group;
        // never infer ECI capacity from a generic ZMC buffer size.
        int freeComplex=0;
        ret=ZAux_Direct_GetRemain_Buffer(m_handle,baseAxis,&freeComplex);
        if(ret!=ERR_OK){if(err)*err=errorTextFromCode(ret,"GetRemain_Buffer(SP5)");return false;}
        if(freeComplex<2)break;
        const auto& seg=m_mergePlan.segments[m_mergeIndex];
        auto checked=[&](int32 r,const char* name){
            if(r!=ERR_OK){ZAux_Direct_CancelAxisList(m_handle,5,axes,2);m_mergePath.clear();m_mergeIndex=0;
                if(err)*err=errorTextFromCode(r,name);return false;}return true;};
        // Per-segment SP parameters enter the queue; global SPEED is NOT changed here.
        if(!checked(ZAux_Direct_SetStartMoveSpeed(m_handle,baseAxis,float(seg.entry)),"STARTMOVE_SPEED(SP5)") ||
           !checked(ZAux_Direct_SetForceSpeed(m_handle,baseAxis,float(seg.speed)),"FORCE_SPEED(SP5)") ||
           !checked(ZAux_Direct_SetEndMoveSpeed(m_handle,baseAxis,float(seg.exit)),"ENDMOVE_SPEED(SP5)"))return false;
        float dest[5];for(int j=0;j<5;++j)dest[j]=float(seg.target[j]-(j>=3?m_mergeOffsets[j-3]:0.0));
        if(!checked(ZAux_Direct_MoveAbsSp(m_handle,5,axes,dest),"MoveAbsSp(5 axes)"))return false;
        ++m_mergeIndex;++m_mergeLastBatchSent;
    }
    finished = m_mergeIndex >= m_mergePath.size();
    if(finished && m_mergeUsesAxisCompensation && !m_mergePlan.segments.empty()){
        const auto& last=m_mergePlan.segments.back().target;
        for(int j=0;j<3;++j)rememberLinearTarget(j+AxisX,m_mergeFinalNominal[j],static_cast<float>(last[j]));
    }
    return true;
}


bool ZmcController::readTrajectoryLimits5D(mls::runtime::Limits& out,QString* err)
{
    if (!m_io.isWorkerThread()) return runIo(IoPriority::Control,
        "readTrajectoryLimits5D",[&]{return readTrajectoryLimits5D(out,err);},err);
    p0::IoLock lock(m_ioMutex);
    out=m_trajectoryLimits;
    return true;
}

bool ZmcController::setTrajectoryLimits5D(const mls::runtime::Limits& limits,QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"setTrajectoryLimits5D",[&]{return setTrajectoryLimits5D(limits,err);},err);
    p0::IoLock lock(m_ioMutex);
    if(m_handle && (!areAllAxesIdle(err) || m_mergeIndex<m_mergePath.size() || m_hwSampling)){
        if(err)*err=QStringLiteral("运行中不能修改五轴轨迹限制");return false;
    }
    for(int j=0;j<5;++j)if(!std::isfinite(limits.speed[j]) || limits.speed[j]<=0 ||
        !std::isfinite(limits.accel[j]) || limits.accel[j]<=0 || !std::isfinite(limits.decel[j]) || limits.decel[j]<=0 ||
        !std::isfinite(limits.minimum[j]) || !std::isfinite(limits.maximum[j]) || limits.minimum[j]>=limits.maximum[j]){
        if(err)*err=QStringLiteral("五轴限制必须为有限的正速度/加减速及合法范围");return false;
    }
    m_trajectoryLimits=limits;return true;
}
bool ZmcController::checkMotionHealth5D(QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"motionHealth5D",[&]{return checkMotionHealth5D(err);},err);
    p0::IoLock lock(m_ioMutex);
    if(!m_handle){if(err)*err=QStringLiteral("控制器已断开");return false;}
    constexpr int faults=2|4|8|16|32|256|512|1024|4096|16384|262144|524288|2097152|4194304;
    for(int a=AxisX;a<=AxisB;++a){int status=0;const int32 ret=ZAux_Direct_GetAxisStatus(m_handle,a,&status);
        if(ret!=ERR_OK){if(err)*err=errorTextFromCode(ret,"GetAxisStatus(running5D)");return false;}
        if(status&faults){if(err)*err=QStringLiteral("运动中轴 %1 报警/限位，AXISSTATUS=%2").arg(a).arg(status);return false;}
    }
    return true;
}
bool ZmcController::pauseTrajectory5D(QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"pauseTrajectory5D",[&]{return pauseTrajectory5D(err);},err);
    p0::IoLock lock(m_ioMutex);if(!m_handle)return false;
    const int32 r=ZAux_Direct_MovePause(m_handle,AxisX,0);
    if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"MovePause(5-axis group)");return false;}
    m_mergePaused=true;return true;
}
bool ZmcController::resumeTrajectory5D(QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Control,"resumeTrajectory5D",[&]{return resumeTrajectory5D(err);},err,eci::EciIoScheduler::Motion);
    p0::IoLock lock(m_ioMutex);if(!m_handle)return false;
    const int32 r=ZAux_Direct_MoveResume(m_handle,AxisX);
    if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"MoveResume(5-axis group)");return false;}
    m_mergePaused=false;return true;
}
bool ZmcController::readJointSample5D(SamplePoint& sample,QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Measurement,"readJointSample5D",[&]{return readJointSample5D(sample,err);},err);
    p0::IoLock lock(m_ioMutex);
    if(!m_handle || !areLinearMposReferenced()
       || !m_rotaryAbsReady[AxisC].load() || !m_rotaryAbsReady[AxisB].load()){
        if(err)*err=QStringLiteral("缺少连接、XYZ光栅回零基准或B/C机械绝对坐标框架");return false;
    }
    const qint64 before=GlobalTimeSync::instance().timestampUs();
    float v[mls::runtime::Table5D::dimension]={0};
    for(int j=0;j<3;++j){const int32 r=ZAux_Direct_GetMpos(m_handle,j+1,&v[j+1]);
        if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetMpos(XYZ sample5D)");return false;}}
    int32 r=ZAux_Direct_GetMpos(m_handle,AxisC,&v[4]);
    if(r==ERR_OK)r=ZAux_Direct_GetMpos(m_handle,AxisB,&v[5]);
    if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"GetMpos(CB sample5D)");return false;}
    mls::runtime::Frame5D f;std::string detail;
    if(!mls::runtime::decodeRecord(v,0,0,{m_rotaryMachineOffset[AxisC].load(),m_rotaryMachineOffset[AxisB].load()},f,detail)){
        if(err)*err=QString::fromStdString(detail);return false;
    }
    sample={};sample.tUs=(before+GlobalTimeSync::instance().timestampUs())/2;
    sample.x=f.joints[0];sample.y=f.joints[1];sample.z=f.joints[2];sample.c=f.joints[3];sample.b=f.joints[4];sample.hasRotary=true;
    f.hostEstimateUs=sample.tUs;sample.frame=f;return true;
}

// ==================== 软件位置采样 ====================

bool ZmcController::capturePositionStream(int periodUs, int sampleCount,
                                          QVector<SamplePoint>& out, QString* errMsg) const
{
    if (m_io.isWorkerThread()) return false;
    BlockingCallGuard activeCall(m_activeBlockingCalls, m_lifecycleMutex);
    out.clear();

    if (!isConnected()) {
        if (errMsg) *errMsg = QStringLiteral("尚未连接控制器");
        return false;
    }

    if (periodUs <= 0 || sampleCount <= 0) {
        if (errMsg) *errMsg = QStringLiteral("非法的采样周期或采样次数");
        return false;
    }

    out.reserve(sampleCount);
#if defined(Q_OS_WIN) && !defined(MLS_PORTABLE_TRANSPORT_TEST)
    timeBeginPeriod(1);
#endif

    QElapsedTimer timer;
    timer.start();

    qint64 nextDeadlineUs = 0;

    for (int i = 0; i < sampleCount; ++i) {
        // 忙等到下一个采样时刻
        while (true) {
            qint64 nowUs = timer.nsecsElapsed() / 1000;
            if (nowUs >= nextDeadlineUs) break;
        }

        qint64 tUs = timer.nsecsElapsed() / 1000;

        SamplePoint sp;
        if(!const_cast<ZmcController*>(this)->readJointSample5D(sp,errMsg))return false;
        out.append(sp);

        nextDeadlineUs += periodUs;
    }

#if defined(Q_OS_WIN) && !defined(MLS_PORTABLE_TRANSPORT_TEST)
    timeEndPeriod(1);
#endif

    return true;
}

// ==================== 硬件 TABLE 采样 ====================

bool ZmcController::hwTableCommissioningPreflight(QString& summary, QString* errMsg, bool* executionAuthorized)
{
    using T = mls::runtime::Table5D;
    summary.clear();
    if (executionAuthorized) *executionAuthorized = false;
    if (m_io.isWorkerThread()) {
        if (errMsg) *errMsg = QStringLiteral("硬件同步只读预检不能阻塞I/O工作线程");
        return false;
    }
    BlockingCallGuard active(m_activeBlockingCalls, m_lifecycleMutex);
    return runIo(IoPriority::Measurement, "table.preflightV3213", [&] {
        p0::IoLock lock(m_ioMutex);
        if (!m_handle) { if (errMsg) *errMsg = QStringLiteral("尚未连接控制器"); return false; }
        if (!checkNoJointTask5D(errMsg)) return false;
        if (!areLinearMposReferenced()) {
            if (errMsg) *errMsg = QStringLiteral("XYZ增量光栅尚未全部回零并建立MPOS零点，不能确认五轴坐标约定");
            return false;
        }
        if (!m_rotaryAbsReady[AxisC].load() || !m_rotaryAbsReady[AxisB].load()) {
            if (errMsg) *errMsg = QStringLiteral("B/C机械绝对坐标尚未就绪，不能确认五轴坐标约定");
            return false;
        }
        if (!areAllAxesHomed()) {
            if (errMsg) *errMsg = QStringLiteral("本次会话的XYZ/C/B五轴Home未经全部确认，拒绝预检授权");
            return false;
        }
        float h[T::headerSize] = {};
        const int32 r = ZAux_Direct_GetTable(m_handle, T::base, T::headerSize, h);
        if (r != ERR_OK) { if (errMsg) *errMsg = errorTextFromCode(r, "GetTable(V32.13 preflight)"); return false; }
        std::string detail;
        if (!T::validate(h, -1, detail)) {
            QStringList snapshot;
            snapshot.reserve(T::headerSize);
            for (int i = 0; i < T::headerSize; ++i)
                snapshot << QStringLiteral("%1=%2").arg(T::base + i).arg(h[i]);
            if (errMsg) *errMsg = QStringLiteral(
                "ECI3808 V32.16 H1/H3 FULL-MPOS BASIC协议头不匹配：%1\nTABLE[330..351]快照：%2")
                .arg(QString::fromStdString(detail), snapshot.join(QStringLiteral(", ")));
            return false;
        }
        if (h[3] != 0.0f) { if (errMsg) *errMsg = QStringLiteral("BASIC当前已有故障码 %1；只读预检拒绝继续").arg(int(h[3])); return false; }
        if (h[4] != 0.0f) { if (errMsg) *errMsg = QStringLiteral("BASIC采集enable=%1，不处于安全空闲状态").arg(int(h[4])); return false; }
        mls::runtime::TableHeaderProfile profile;
        if (!T::decodeProfile(h, profile, detail) || profile.variant != mls::runtime::TablePoseVariant::FullMposLatched) {
            if (errMsg) *errMsg = QStringLiteral("V32.16接受五轴MPOS：H1 REGIST(104) 或 H3 REGIST(114/115)");
            return false;
        }
        float runtime[T::runtimeStatusWords] = {};
        const int32 rr = ZAux_Direct_GetTable(m_handle, T::base + T::runtimeCapabilityOffset,
                                               T::runtimeStatusWords, runtime);
        if (rr != ERR_OK) { if (errMsg) *errMsg = errorTextFromCode(rr, "GetTable(V32.13 runtime config)"); return false; }
        if (!T::integer(runtime[0], T::runtimeCapabilityMagic, T::runtimeCapabilityMagic)) {
            if (errMsg) *errMsg = QStringLiteral("控制器BASIC不是V32.16 H1/H3协议(TABLE[354]=321058)；请下载controller/ECI3808_V32_16_H1_H3_ONLY.bas");
            return false;
        }
        if (!T::integer(runtime[1], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz) ||
            !T::integer(runtime[2], T::minTriggerFrequencyHz, T::maxTriggerFrequencyHz) ||
            !T::integer(runtime[3], 0, static_cast<int>(T::allAxesMask))) {
            if (errMsg) *errMsg = QStringLiteral("V32.13 PWM/Home状态不是有效整数或已越界，拒绝预检");
            return false;
        }
        const int requestedHz = static_cast<int>(runtime[1]);
        const int appliedHz = static_cast<int>(runtime[2]);
        const int homeMask = static_cast<int>(runtime[3]);
        if (requestedHz < T::minTriggerFrequencyHz || requestedHz > T::maxTriggerFrequencyHz ||
            appliedHz < T::minTriggerFrequencyHz || appliedHz > T::maxTriggerFrequencyHz) {
            if (errMsg) *errMsg = QStringLiteral("V32.13运行时PWM频率状态非法: requested=%1Hz applied=%2Hz").arg(requestedHz).arg(appliedHz);
            return false;
        }
        if (homeMask != 31) {
            if (errMsg) *errMsg = QStringLiteral("V32.13控制器XYZBC Home mask=%1，不是31；请确认五轴均已完成本次上电会话回零").arg(homeMask);
            return false;
        }
        float modes[4]={};
        const int32 modeR=ZAux_Direct_GetTable(m_handle,T::topologyRequestIndex,4,modes);
        if(modeR!=ERR_OK || !T::integer(modes[2],1,3) || !T::integer(modes[3],104,115) ||
           !T::validTopologyEdge(int(modes[2]),int(modes[3])) ||
           !T::modeReady(modes,int(modes[2]),int(modes[3])) || h[19]!=modes[3] ||
           h[17]!=(modes[2]==1?0:1)) {
            if(errMsg)*errMsg=QStringLiteral("V32.16 H1/H3拓扑/锁存通道 TABLE[365..368] 回读或头信息不一致");
            return false;
        }
        if (requestedHz!=appliedHz || runtime[4]!=T::IdleReady ||
            runtime[5]!=runtime[6] || runtime[7]!=0) {
            if(errMsg)*errMsg=QStringLiteral("BASIC未处于配置一致的真实IdleReady状态：req=%1 ackHz=%2 state=%3 seq=%4 ackSeq=%5 cfgErr=%6")
                .arg(requestedHz).arg(appliedHz).arg(runtime[4])
                .arg(runtime[5]).arg(runtime[6]).arg(runtime[7]);
            return false;
        }

        float stopTailMarker=0;
        const int32 stopMarkResult=ZAux_Direct_GetTable(
            m_handle,T::base+T::stopTailMarkerOffset,1,&stopTailMarker);
        if(stopMarkResult!=ERR_OK ||
           !T::integer(stopTailMarker,T::stopTailMarker,T::stopTailMarker)){
            if(errMsg)*errMsg=QStringLiteral("当前BASIC缺少STOP尾帧原始FIFO收敛标识TABLE[362]=1316；"
                "必须先更新STOP-DRAIN BASIC，否则本版上位机拒绝硬件采集");
            return false;
        }
        const bool authorized = T::executionAuthorized(h);
        if (executionAuthorized) *executionAuthorized = authorized;
        summary = QStringLiteral(
            "V32.16 H1/H3 TABLE协议只读预检通过\n"
            "variant=V32.16-FullMPOSLatched（XYZ/C/B均为REGIST(R0/R1)硬件锁存raw MPOS）\n"
            "magic=%1, legacyPeriodMarker=%2 ms, recordWords=%3, capacity=%4\n"
            "runtimePWM: requested=%8 Hz, applied=%9 Hz, period=%10 ms；XYZBC HomeMask=%11\n"
            "feedbackMask=31, commandMask=0, effectiveLatchMask=31, timing=HardwareLatched\n"
            "REGIST=%5（H1=R0/IN0, H3=R1/IN1），triggerOutput=%6，latchBridgeOutput=%7\n"
            "coldPrimeCapability=1：正式session前允许enable=3，使用当前运行时PWM频率触发scanCONTROL预热Profile/SDK/以太网数据链；预热帧丢弃，新session随后清空并重新Arm五轴REGIST FIFO。\n"
            "BASIC/TABLE严格按光栅反馈方案保存XYZ/C/B五轴MPOS；上位机普通实际位置、标定和软件同步也使用XYZ/C/B MPOS。DPOS仅保留为控制器需求位置和跟随误差诊断，不改写硬件同步TABLE的MPOS语义。\n"
            "B/C坐标规则：TABLE内C/B是控制器增量编码器raw MPOS；上位机继续按RS485机械绝对位置建立的offset恢复机械绝对C/B。")
            .arg(int(h[7])).arg(int(h[9])).arg(int(h[8])).arg(int(h[0]))
            .arg(int(h[19])).arg(int(h[18])).arg(int(h[20]))
            .arg(requestedHz).arg(appliedHz).arg(1000.0 / static_cast<double>(appliedHz), 0, 'f', 6).arg(homeMask)
            + QStringLiteral("\nhardwareExecutionAuthorization=%1 (%2)")
                  .arg(int(h[12]))
                  .arg(authorized ? QStringLiteral("已授权：可执行V32.16 H1/H3五轴MPOS硬件同步")
                                  : QStringLiteral("未授权：协议可读，但物理硬件同步保持锁定"))
            + QStringLiteral("\nB/C software-frame offset: C=%1°, B=%2° (machine=TABLE_raw_MPOS+offset)")
                  .arg(m_rotaryMachineOffset[AxisC].load(),0,'f',6)
                  .arg(m_rotaryMachineOffset[AxisB].load(),0,'f',6);
        return true;
    }, errMsg);
}

bool ZmcController::hwSensorColdPrimeStart(QString* errMsg)
{
    if (!m_io.isWorkerThread())
        return runIo(IoPriority::Measurement, "table.sensorPrimeStartV329", [&]{ return hwSensorColdPrimeStart(errMsg); }, errMsg, eci::EciIoScheduler::Acquisition);
    p0::IoLock lock(m_ioMutex); using T=mls::runtime::Table5D;
    if(!m_handle){if(errMsg)*errMsg=QStringLiteral("尚未连接控制器");return false;}
    if(m_hwSampling){if(errMsg)*errMsg=QStringLiteral("正式TABLE session已启动，禁止进入传感器冷启动预热");return false;}
    if(m_hwTopology.load()==3){if(errMsg)*errMsg=QStringLiteral("H3 Sensor-master不使用ECI PWM cold-prime");return false;}
    if(!areAllAxesHomed()){
        if(errMsg)*errMsg=QStringLiteral("五轴Home未全部确认，拒绝ColdPrime触发输出");return false;
    }
    float h[T::headerSize]={}; std::string detail;
    int32 r=ZAux_Direct_GetTable(m_handle,T::base,T::headerSize,h);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(V32.13 cold prime)");return false;}
    if(!T::validate(h,-1,detail) || !T::executionAuthorized(h) || h[4]!=0 || h[10]!=h[11]){
        if(errMsg)*errMsg=QStringLiteral("V32.13冷启动预热要求SINGLE-OP14 FULL-MPOS协议已授权、enable=0且REQ=ACK：%1").arg(QString::fromStdString(detail));
        return false;
    }
    float v[T::runtimeStatusWords]={};
    r=ZAux_Direct_GetTable(m_handle,T::base+T::runtimeCapabilityOffset,T::runtimeStatusWords,v);
    if(r!=ERR_OK || !T::runtimeReady(v,m_hwTriggerFrequencyHz.load(),T::IdleReady)){
        if(errMsg)*errMsg=QStringLiteral("ColdPrime必须匹配Host期望频率、配置序号ACK且BASIC真正IdleReady");
        return false;
    }
    float zero=0.0f;
    r=ZAux_Direct_SetTable(m_handle,T::base+T::primeCounterOffset,1,&zero);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"clear sensor prime counter");return false;}
    float mode=3.0f;
    r=ZAux_Direct_SetTable(m_handle,T::base+4,1,&mode);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"start sensor cold prime mode");return false;}
    return true;
}

bool ZmcController::hwSensorColdPrimeStop(QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        if (!runIo(IoPriority::Measurement,"table.sensorPrimeStopV3213",
                   [&]{return hwSensorColdPrimeStop(errMsg);},errMsg,eci::EciIoScheduler::Acquisition)) return false;
        // EN=0 only requests stop. Wait until BASIC has finished its old-frequency tail.
        return hwTableWaitIdleReady(1500,errMsg);
    }
    p0::IoLock lock(m_ioMutex); using T=mls::runtime::Table5D;
    if(!m_handle){if(errMsg)*errMsg=QStringLiteral("尚未连接控制器");return false;}
    float zero=0.0f;
    const int32 r=ZAux_Direct_SetTable(m_handle,T::base+4,1,&zero);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"stop sensor cold prime mode");return false;}
    return true;
}

bool ZmcController::hwSensorColdPrimeCount(qint64& pulseCount, QString* errMsg)
{
    if (!m_io.isWorkerThread())
        return runIo(IoPriority::Measurement, "table.sensorPrimeCountV329", [&]{ return hwSensorColdPrimeCount(pulseCount,errMsg); }, errMsg, eci::EciIoScheduler::None);
    p0::IoLock lock(m_ioMutex); using T=mls::runtime::Table5D; pulseCount=0;
    if(!m_handle){if(errMsg)*errMsg=QStringLiteral("尚未连接控制器");return false;}
    float v=0.0f; const int32 r=ZAux_Direct_GetTable(m_handle,T::base+T::primeCounterOffset,1,&v);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(sensor prime counter)");return false;}
    if(!std::isfinite(v)||v<0||std::floor(v)!=v){if(errMsg)*errMsg=QStringLiteral("sensor prime counter非法");return false;}
    pulseCount=static_cast<qint64>(v);return true;
}

bool ZmcController::hwTableWaitCmmArmed(int timeoutMs, QString* errMsg)
{
    if(m_io.isWorkerThread()){if(errMsg)*errMsg=QStringLiteral("Arm握手不能阻塞I/O工作线程");return false;}
    QElapsedTimer timer;timer.start();
    while(timer.elapsed()<timeoutMs){
        bool ready=false,failed=false;
        if(!runIo(IoPriority::Measurement,"table.waitCmmArmV3214",[&]{
            p0::IoLock lock(m_ioMutex);
            if(!m_handle){if(errMsg)*errMsg=QStringLiteral("Arm期间ECI断开");return false;}
            float v[3]={};
            int32 r=ZAux_Direct_GetTable(m_handle,367,3,v);
            if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(armed)");return false;}
            float q[2]={};
            r=ZAux_Direct_GetTable(m_handle,333,2,q);
            if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(armFault)");return false;}
            failed=(q[0]!=0 || q[1]!=2);
            ready=v[2]==1 && v[0]==m_hwTopology.load() && v[1]==m_hwCmmEdge.load();
            return true;
        },errMsg,eci::EciIoScheduler::Acquisition))return false;
        if(failed){if(errMsg)*errMsg=QStringLiteral("ECI进入Probe前产生故障或EN被取消");return false;}
        if(ready)return true;
        QThread::msleep(2);
    }
    if(errMsg)*errMsg=QStringLiteral("等待ECI TABLE[369] IN1硬件锁存ArmReady超时");
    return false;
}

bool ZmcController::hwTableSampleStart(QString* errMsg)
{
    using T=mls::runtime::Table5D;
    if(m_io.isWorkerThread()){if(errMsg)*errMsg=QStringLiteral("握手不能阻塞 I/O 工作线程");return false;}
    const auto token=m_io.token();BlockingCallGuard active(m_activeBlockingCalls,m_lifecycleMutex);
    bool success=false,touched=false;
    auto cleanup=eci::scopeExit([&]{if(!success && touched)hwTableSampleStop(nullptr);});

    if(!runIo(IoPriority::Measurement,"table.start.requestV32",[&]{
        p0::IoLock lock(m_ioMutex);
        if(!m_handle || !checkNoJointTask5D(errMsg))return false;
        if(m_hwSampling){if(errMsg)*errMsg=QStringLiteral("重复启动采样");return false;}
        m_hwReadCursorOwned=false;
        float stopMark=0;
        const int32 sm=ZAux_Direct_GetTable(m_handle,T::base+T::stopTailMarkerOffset,1,&stopMark);
        if(sm!=ERR_OK || !T::integer(stopMark,T::stopTailMarker,T::stopTailMarker)){
            if(errMsg)*errMsg=QStringLiteral("启动拒绝：BASIC不支持STOP-DRAIN尾帧闭环(TABLE[362]=1316)");
            return false;
        }
        if(!areLinearMposReferenced()){
            if(errMsg)*errMsg=QStringLiteral("先完成XYZ正规回零并建立光栅MPOS机床零点");return false;
        }
        if(!m_rotaryAbsReady[AxisC].load() || !m_rotaryAbsReady[AxisB].load()){
            if(errMsg)*errMsg=QStringLiteral("先建立B/C机械绝对坐标");return false;
        }
        if (!areAllAxesHomed()) {
            if (errMsg) *errMsg=QStringLiteral("XYZ/C/B五轴本地回零状态未全部确认，拒绝采集启动");
            return false;
        }
        float h[T::headerSize]={};std::string e;
        int32 r=ZAux_Direct_GetTable(m_handle,T::base,T::headerSize,h);
        if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(V32)");return false;}
        if(!T::validate(h,-1,e)){if(errMsg)*errMsg=QStringLiteral("ECI3808 V32.16 H1/H3 FULL-MPOS TABLE BASIC 协议未就绪：")+QString::fromStdString(e);return false;}
        if(!T::executionAuthorized(h)){
            if(errMsg)*errMsg=QStringLiteral("TABLE[342]=0：控制器BASIC仅允许只读预检，拒绝启动物理硬件同步；请下载controller/ECI3808_V32_16_H1_H3_ONLY.bas（仅H1/H3）");
            return false;
        }
        if(h[4]!=0 || h[3]!=0 || h[10]!=h[11]){
            if(errMsg)*errMsg=QStringLiteral("新硬件Session启动必须确认旧Session已Idle/无故障/已ACK");
            return false;
        }
        float runtime[T::runtimeStatusWords]={};
        r=ZAux_Direct_GetTable(m_handle,T::base+T::runtimeCapabilityOffset,
                               T::runtimeStatusWords,runtime);
        if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(V32.16 H1/H3 runtime before start)");return false;}
        if (!T::integer(runtime[3], 0, static_cast<int>(T::allAxesMask))) {
            if (errMsg) *errMsg=QStringLiteral("控制器Home mask非法，拒绝硬件采集启动");
            return false;
        }
        const int requestedHz=T::integer(runtime[1],T::minTriggerFrequencyHz,T::maxTriggerFrequencyHz)
            ? static_cast<int>(runtime[1]) : -1;
        const int appliedHz=T::integer(runtime[2],T::minTriggerFrequencyHz,T::maxTriggerFrequencyHz)
            ? static_cast<int>(runtime[2]) : -1;
        const int homeMask=static_cast<int>(runtime[3]);
        float modes[4]={};
        r=ZAux_Direct_GetTable(m_handle,T::topologyRequestIndex,4,modes);
        const bool modeOk=r==ERR_OK && T::modeReady(modes,m_hwTopology.load(),m_hwCmmEdge.load())
            && h[19]==m_hwCmmEdge.load() && h[17]==(m_hwTopology.load()==1?0:1);
        if(!modeOk || !T::runtimeReady(runtime,m_hwTriggerFrequencyHz.load(),T::IdleReady) ||
           homeMask!=31){
            if(errMsg)*errMsg=QStringLiteral("V32.16 H1/H3拒绝启动：req=%1Hz applied=%2Hz HostExpected=%3Hz homeMask=%4 state=%5 seq=%6 ackSeq=%7 cfgErr=%8")
                .arg(requestedHz).arg(appliedHz).arg(m_hwTriggerFrequencyHz.load()).arg(homeMask)
                .arg(runtime[4]).arg(runtime[5]).arg(runtime[6]).arg(runtime[7]);
            return false;
        }

        // Always force PWM/scan enable off before starting a new generation.
        float zero=0;touched=true;
        r=ZAux_Direct_SetTable(m_handle,T::base+4,1,&zero);
        if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"disable TABLE/PWM");return false;}

        m_hwSession=(int(h[11])+1)%T::maxOrdinal;if(m_hwSession==0)m_hwSession=1;
        float request=float(m_hwSession);
        r=ZAux_Direct_SetTable(m_handle,T::base+10,1,&request);
        if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"session request");return false;}
        m_hwSampling=true;
        return true;
    },errMsg,eci::EciIoScheduler::Acquisition,&token))return false;

    // BASIC accepts session changes only while disabled.  Its reset clears the
    // host-facing ring and the active variant's raw latch FIFO cursors.
    for(int attempt=0;attempt<120;++attempt){
        bool ack=false;
        if(!runIo(IoPriority::Measurement,"table.start.awaitAckV32",[&]{
            p0::IoLock lock(m_ioMutex);float h[T::headerSize]={};std::string e;
            const int32 r=ZAux_Direct_GetTable(m_handle,T::base,T::headerSize,h);
            if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"read session ack");return false;}
            if(!T::validate(h,-1,e)){if(errMsg)*errMsg=QString::fromStdString(e);return false;}
            ack=(h[10]==m_hwSession && h[11]==m_hwSession && h[1]==0 && h[2]==0 &&
                 h[3]==0 && h[4]==0 && h[5]==0 && h[6]==0);
            if(ack){
                mls::runtime::TableHeaderProfile profile;
                if(!T::decodeProfile(h,profile,e)){
                    if(errMsg)*errMsg=QString::fromStdString(e);
                    return false;
                }
                m_hwFeedbackMask=profile.feedbackMask;
                m_hwLatchMask=profile.latchMask;
                m_hwCommandMask=profile.commandMask;
                m_hwTiming=profile.timing;
                m_hwVariant=profile.variant;
            }
            return true;
        },errMsg,eci::EciIoScheduler::Acquisition,&token))return false;
        if(ack){
            success=runIo(IoPriority::Measurement,"table.start.probeV32",[&]{
                p0::IoLock lock(m_ioMutex);
                m_hwOffsets={m_rotaryMachineOffset[AxisC].load(),m_rotaryMachineOffset[AxisB].load()};
                LOG_INFO(LogCategory::Sync,
                    QStringLiteral("V32.13 hardware session freezes B/C software-frame offsets: C=%1°, B=%2°; "
                                   "TABLE raw XYZ/C/B are hardware-latched MPOS; C/B are converted as machine=rawMPOS+offset")
                        .arg(m_hwOffsets[0],0,'f',6).arg(m_hwOffsets[1],0,'f',6));
                m_hwBufSize=T::capacity;m_hwReadIndex=0;m_hwSampleIndex=0;
                m_hwReadFault=false;m_hwReadFaultReason.clear();
                // enable=2 asks V32.13 BASIC to clear/re-arm XYZ/C/B REGIST(104) FIFOs.
                // BASIC emits exactly one 1 ms OP14 probe pulse; scanCONTROL EdgeRising and all five REGIST(104)
                // channels correspond to the same OP14 OFF / IN0 falling event. After one record the host
                // sets enable=1 and PWM0 continues the same single-OP14 path at the host-selected runtime frequency.
                float mode=2;const int32 r=ZAux_Direct_SetTable(m_handle,T::base+4,1,&mode);
                if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"start V32.16 H1/H3 one-shot full-MPOS probe");return false;}
                m_hwReadCursorOwned=true;
                return true;
            },errMsg,eci::EciIoScheduler::Acquisition,&token);
            if(success && m_hwTopology.load()!=1) success=hwTableWaitCmmArmed(1200,errMsg);
            return success;
        }
        QThread::msleep(5);
    }
    if(errMsg)*errMsg=QStringLiteral("V32.16 BASIC会话握手超时；检查controller/ECI3808_V32_16_H1_H3_ONLY.bas是否运行，以及TABLE 330..369协议头");
    return false;
}

bool ZmcController::hwTableContinueAfterProbe(QString* err)
{
    if(!m_io.isWorkerThread())return runIo(IoPriority::Measurement,"table.continueProbeV32",[&]{return hwTableContinueAfterProbe(err);},err,eci::EciIoScheduler::Acquisition);
    p0::IoLock lock(m_ioMutex);using T=mls::runtime::Table5D;
    if(!m_handle || !m_hwSampling || m_hwReadFault.load())return false;
    float h[T::headerSize]={};std::string detail;
    int32 r=ZAux_Direct_GetTable(m_handle,T::base,T::headerSize,h);
    if(r!=ERR_OK || !T::validate(h,m_hwSession,detail) || !T::executionAuthorized(h) ||
       h[4]!=2 ||
       (m_hwTopology.load()==3 ? h[5]<8 : h[5]!=1) || h[6]!=h[5] || h[3]!=0){
        if(err)*err=QStringLiteral("V32.16 Probe未满足授权/有效五轴MPOS锁存数量（H1需要1个，H3至少8个）");return false;
    }
    float runtime[T::runtimeStatusWords]={};
    r=ZAux_Direct_GetTable(m_handle,T::base+T::runtimeCapabilityOffset,T::runtimeStatusWords,runtime);
    float modes[4]={};
    const int32 modeR=ZAux_Direct_GetTable(m_handle,T::topologyRequestIndex,4,modes);
    if(r!=ERR_OK || modeR!=ERR_OK ||
       !T::modeReady(modes,m_hwTopology.load(),m_hwCmmEdge.load()) ||
       h[19]!=m_hwCmmEdge.load() ||
       !T::runtimeReady(runtime,m_hwTriggerFrequencyHz.load(),T::Probe) ||
       runtime[3]!=31){
        if(err)*err=QStringLiteral("Probe→Run拒绝：BASIC实际状态不是Probe或Host频率/seq ACK/HomeMask发生变化");
        return false;
    }
    float one=1;r=ZAux_Direct_SetTable(m_handle,T::base+4,1,&one);
    if(r!=ERR_OK){if(err)*err=errorTextFromCode(r,"continue V32.16 H1/H3 configured hardware topology after probe");return false;}
    return true;
}

bool ZmcController::hwTableSampleStop(QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Emergency, "hwTableSampleStop", [&] { return hwTableSampleStop(errMsg); }, errMsg, eci::EciIoScheduler::None, nullptr, eci::EciIoScheduler::Acquisition);
    }
    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {if (errMsg) *errMsg = QStringLiteral("尚未连接控制器");return false;}
    float enable0 = 0.0f;
    const int32 ret = ZAux_Direct_SetTable(m_handle, mls::runtime::Table5D::base + 4, 1, &enable0);
    if (ret != ERR_OK) {if (errMsg) *errMsg = errorTextFromCode(ret, "SetTable(enable=0/PWM off)");return false;}
    // Reader remains allowed to drain already-published Buffer-B records.  The
    // BASIC first disables PWM, then drains raw Buffer-A into Buffer-B.
    m_hwSampling = false;
    return true;
}

bool ZmcController::hwTriggerOnlyStop(QString* errMsg)
{
    return hwTableSampleStop(errMsg);
}

bool ZmcController::hwGetTriggerCount(qint64& triggerCount, QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Measurement, "hwGetTriggerCount", [&] { return hwGetTriggerCount(triggerCount, errMsg); }, errMsg, eci::EciIoScheduler::None);
    }
    p0::IoLock ioLock(m_ioMutex);triggerCount=0;
    if (!m_handle) {if (errMsg) *errMsg = QStringLiteral("尚未连接控制器");return false;}
    float valueF=0.0f;
    const int32 ret=ZAux_Direct_GetTable(m_handle,mls::runtime::Table5D::base+6,1,&valueF);
    if(ret!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(ret,"GetTable(published latch ordinal count)");return false;}
    const double value=valueF;
    if(!std::isfinite(value)||value<0||std::floor(value)!=value||value>mls::runtime::Table5D::maxOrdinal){
        if(errMsg)*errMsg=QStringLiteral("已发布锁存ordinal计数不是有效的单精度整数");return false;
    }
    triggerCount=static_cast<qint64>(value);return true;
}

bool ZmcController::hwReadStopTailDiagnostics(HwStopTailDiag& diag, QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Measurement,"hwReadStopTailDiagnostics",
                     [&] { return hwReadStopTailDiagnostics(diag,errMsg); },
                     errMsg,eci::EciIoScheduler::None);
    }
    p0::IoLock lock(m_ioMutex);
    using T = mls::runtime::Table5D;
    diag = HwStopTailDiag{};
    if (!m_handle) { if(errMsg)*errMsg=QStringLiteral("未连接控制器");return false; }
    float e[T::stopTailWords]={}, h[12]={}, state=0;
    int32 r=ZAux_Direct_GetTable(m_handle,T::base+T::stopTailMarkerOffset,T::stopTailWords,e);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(STOP drain extension)");return false;}
    if(!T::integer(e[0],T::stopTailMarker,T::stopTailMarker)){
        if(errMsg)*errMsg=QStringLiteral("BASIC无可靠STOP Drain标识(TABLE[362]!=1316)，拒绝判定安全收尾");
        return false;
    }
    r=ZAux_Direct_GetTable(m_handle,T::base,12,h);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(STOP ring header)");return false;}
    r=ZAux_Direct_GetTable(m_handle,T::base+T::controllerStateOffset,1,&state);
    if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(STOP runtime state)");return false;}
    if(!T::integer(e[1],T::TailReset,T::TailFailure) ||
       !T::integer(e[2],0,128) ||
       !T::integer(state,T::IdleReady,T::Fault) ||
       !T::integer(h[1],0,T::capacity-1) || !T::integer(h[2],0,T::capacity-1) ||
       !T::integer(h[3],0,T::maxOrdinal) || !T::integer(h[4],0,3) ||
       !T::integer(h[5],0,T::maxOrdinal) || !T::integer(h[6],0,T::maxOrdinal) ||
       !T::integer(h[11],0,T::maxOrdinal)){
        if(errMsg)*errMsg=QStringLiteral("STOP诊断含非整数/越界字段，拒绝声称已收尾");return false;
    }
    diag.phase=int(e[1]);diag.pumpedHead=int(e[2]);
    diag.expectedSession=m_hwSession;
    diag.state=int(state);diag.enable=int(h[4]);diag.basicFault=int(h[3]);
    diag.sessionAck=int(h[11]);diag.ringWrite=int(h[1]);diag.ringRead=int(h[2]);
    diag.published=qint64(h[6]);
    float duty=0.0f;
    uint32 op=0;
    r=ZAux_Direct_GetPwmDuty(m_handle,0,&duty);
    if(r!=ERR_OK || !std::isfinite(duty) || duty<0.0f || duty>1.0f){
        if(errMsg)*errMsg=QStringLiteral("STOP时PWM0 Duty读回失败或无效；禁止声称已关闭PWM");
        return false;
    }
    r=ZAux_Direct_GetOp(m_handle,T::triggerOutput,&op);
    if(r!=ERR_OK || op>1){
        if(errMsg)*errMsg=QStringLiteral("STOP时OP14逻辑状态读回失败或无效");
        return false;
    }
    diag.pwmDuty=duty;diag.op14=int(op);
    if(h[5]!=h[6]){if(errMsg)*errMsg=QStringLiteral("STOP诊断已发布计数TABLE[335]/[336]不一致");return false;}
    const int ptrs[5]={950,1079,1208,1337,1466};
    for(int i=0;i<5;++i){
        float value=0;
        r=ZAux_Direct_GetTable(m_handle,ptrs[i],1,&value);
        if(r!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(r,"GetTable(REGIST FIFO head)");return false;}
        if(!T::integer(value,0,128)){
            if(errMsg)*errMsg=QStringLiteral("REGIST FIFO轴%1指针非法：%2").arg(i).arg(value);
            return false;
        }
        diag.rawHead[i]=int(value);
    }
    diag.stopRawHead=diag.rawHead[0]; // sampled after BASIC reports TailDrained
    return true;
}

bool ZmcController::hwResetTriggerCount(QString* errMsg)
{
    if(errMsg)*errMsg=QStringLiteral("V32禁止单独清零发布计数；新session统一复位Buffer-B/序号/计数");
    return false;
}

bool ZmcController::hwTableSampleReadOnce(QVector<SamplePoint>& outNewSamples,
                                          bool& overflow, QString* errMsg)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Measurement, "hwTableSampleReadOnce", [&] { return hwTableSampleReadOnce(outNewSamples, overflow, errMsg); }, errMsg, eci::EciIoScheduler::None);
    }
    p0::IoLock ioLock(m_ioMutex);
    outNewSamples.clear();overflow=false;
    if(!m_handle){if(errMsg)*errMsg=QStringLiteral("尚未连接控制器");return false;}
    if(m_hwReadFault.load()){if(errMsg)*errMsg=m_hwReadFaultReason;return false;}

    using T=mls::runtime::Table5D;
    float rawHeader[T::headerSize]={};
    int32 ret=ZAux_Direct_GetTable(m_handle,T::base,T::headerSize,rawHeader);
    if(ret!=ERR_OK){if(errMsg)*errMsg=errorTextFromCode(ret,"GetTable(V32 header)");return false;}
    std::string detail;
    if(!T::validate(rawHeader,m_hwSession,detail)){
        m_hwReadFaultReason=QString::fromStdString(detail);m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;return false;
    }
    if(!T::executionAuthorized(rawHeader)){
        m_hwReadFaultReason=QStringLiteral("扫描期间TABLE[342]硬件执行授权被撤销；立即判本次采集无效");m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;return false;
    }
    const int controllerFault=static_cast<int>(rawHeader[3]);
    if(controllerFault!=0){
        const QString reason=[&](){
            switch(controllerFault){
            case 1:return QStringLiteral("V32 Buffer-B接近满/溢出");
            case 2:return QStringLiteral("V32采集中session被修改");
            case 3:return QStringLiteral("兼容保留故障码3：旧版BASIC PC heartbeat超时");
            case 4:return QStringLiteral("V32.13 XYZ/C/B五轴REGIST FIFO计数持续不同步");
            case 5:return QStringLiteral("V32启动单脉冲Probe产生了多个记录");
            case 6:return QStringLiteral("V32 PC readIndex非法");
            case 7:return QStringLiteral("V32.13保留故障码7");
            case 8:return QStringLiteral("旧V27连续REGIST原始FIFO故障码8");
            case 9:return QStringLiteral("V32.13 BASIC检测到TABLE[342]硬件执行授权无效并安全停机");
            case 10:return QStringLiteral("V32.13 BASIC运行时PWM频率请求超出25~2000Hz保护范围");
            default:return QStringLiteral("V32 BASIC未知故障码 %1").arg(controllerFault);
            }
        }();
        m_hwReadFaultReason=reason;m_hwReadFault=true;
        if(errMsg)*errMsg=reason;
        if(controllerFault==1){overflow=true;return true;}
        return false;
    }
    mls::runtime::TableHeaderProfile profile;
    if(!T::decodeProfile(rawHeader,profile,detail)){
        m_hwReadFaultReason=QString::fromStdString(detail);m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;return false;
    }
    if(profile.feedbackMask!=m_hwFeedbackMask || profile.latchMask!=m_hwLatchMask ||
       profile.commandMask!=m_hwCommandMask || profile.timing!=m_hwTiming || profile.variant!=m_hwVariant){
        m_hwReadFaultReason=QStringLiteral("扫描期间V32轴来源/时序profile发生变化；禁止继续重建");m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;return false;
    }
    p0::TableRingHeader header;
    if(!p0::parseTableRingHeader(rawHeader,T::capacity,header)){
        if(errMsg)*errMsg=QStringLiteral("V32 Buffer-B header非法");return false;
    }
    if(m_hwBufSize!=0 && header.capacity!=m_hwBufSize){
        m_hwReadFaultReason=QStringLiteral("扫描期间Buffer-B容量发生变化");m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;return false;
    }
    // H_R is PC-owned during an acknowledged session. If it differs from the
    // last successfully acknowledged cursor, records could have been skipped or
    // the controller/session state could have been modified outside this reader.
    // Fail closed instead of silently accepting a discontinuous host ring.
    if(m_hwReadCursorOwned && header.read!=m_hwReadIndex){
        m_hwReadFaultReason=QStringLiteral("V32 Buffer-B readIndex与PC已确认游标不一致；禁止静默跳过记录");
        m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;
        return false;
    }

    const auto batch=p0::readTableRingBatch(header,T::dimension,
        [&](int start,int count,float* destination){
            const int32 r=ZAux_Direct_GetTable(m_handle,T::data+start*T::dimension,count*T::dimension,destination);
            if(r!=ERR_OK && errMsg)*errMsg=errorTextFromCode(r,"GetTable(V32 Buffer-B data)");
            return r==ERR_OK;
        },
        [&](int nextRead){
            float next=float(nextRead);
            const int32 r=ZAux_Direct_SetTable(m_handle,T::base+2,1,&next);
            if(r!=ERR_OK && errMsg)*errMsg=errorTextFromCode(r,"SetTable(V32 readIndex)");
            return r==ERR_OK;
        });
    if(batch.status==p0::TableReadStatus::Overflow){
        overflow=true;m_hwReadFaultReason=QStringLiteral("V32 Buffer-B溢出：本扫描段失效");m_hwReadFault=true;return true;
    }
    if(batch.status==p0::TableReadStatus::AckUncertain){
        const QString d=errMsg?*errMsg:QString();
        m_hwReadFaultReason=QStringLiteral("V32 readIndex回写应答不确定，禁止盲目重试：%1").arg(d);m_hwReadFault=true;
        if(errMsg)*errMsg=m_hwReadFaultReason;return false;
    }
    if(batch.status!=p0::TableReadStatus::Ok)return false;

    const qint64 nowUs=GlobalTimeSync::instance().timestampUs();
    const qint64 triggerPeriodUs=static_cast<qint64>(std::llround(
        1.0e6/static_cast<double>(std::max(1,m_hwTriggerFrequencyHz.load()))));
    QVector<SamplePoint> staged;staged.reserve(batch.records);
    for(int i=0;i<batch.records;++i){
        SamplePoint sp;mls::runtime::Frame5D frame;
        if(!mls::runtime::decodeRecord(batch.values.data()+i*T::dimension,
             static_cast<std::uint64_t>(m_hwSampleIndex.load()+i),m_hwSession,m_hwOffsets,
             m_hwFeedbackMask,m_hwLatchMask,m_hwCommandMask,m_hwTiming,frame,detail)){
            m_hwReadFaultReason=QString::fromStdString(detail);m_hwReadFault=true;
            if(errMsg)*errMsg=m_hwReadFaultReason;return false;
        }
        // This is only a host estimate.  Kinematics use the latched joint values.
        sp.tUs=nowUs-static_cast<qint64>(batch.records-1-i)*triggerPeriodUs;
        frame.hostEstimateUs=sp.tUs;sp.frame=frame;sp.hasRotary=true;
        sp.x=frame.joints[0];sp.y=frame.joints[1];sp.z=frame.joints[2];sp.c=frame.joints[3];sp.b=frame.joints[4];
        staged.append(sp);
    }
    outNewSamples.swap(staged);
    m_hwSampleIndex.fetch_add(batch.records);
    m_hwBufSize=header.capacity;m_hwReadIndex=header.write;
    return true;
}

// ==================== 圆弧插补 ====================

bool ZmcController::moveArc2Abs(float midX, float midY, 
                                 float endX, float endY,
                                 bool clockwise,
                                 const AxisParams& motion,
                                 QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Motion, "moveArc2Abs", [&] { return moveArc2Abs(midX, midY, endX, endY, clockwise, motion, err); }, err, eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!checkNoJointTask5D(err)) return false;
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    
    if(m_axisCompEnabled.load()){
        JointPose5D q;if(!readMotionStartPose5D(q,err))return false;
        mls::runtime::Joint start{q.x,q.y,q.z,q.c,q.b},mid{double(midX),double(midY),q.z,q.c,q.b},end{double(endX),double(endY),q.z,q.c,q.b};
        mls::runtime::Arc arc;std::string detail;
        const bool orderedClockwise=(mid[0]-start[0])*(end[1]-start[1])-(mid[1]-start[1])*(end[0]-start[0])<0;
        if(!mls::runtime::threePointArc(start,mid,end,orderedClockwise,arc,detail)){if(err)*err=QString::fromStdString(detail);return false;}
        arc.chordToleranceMm=0.001;
        return compensatedArc(end,arc,motion,err);
    }

    // 设置运动参数
    if (!writeMotionParams(AxisX, motion, err)) return false;
    
    // XY平面圆弧使用 V17 的物理轴映射 X=1, Y=2
    int axislist[2] = { AxisX, AxisY };
    
    // The ordered start/mid/end points determine orientation. Keep the legacy
    // parameter for source compatibility; do not reverse a valid midpoint.
    Q_UNUSED(clockwise);
    const int32 ret = ZAux_Direct_MoveCirc2Abs(m_handle, 2, axislist,
                                                    midX, midY, endX, endY);

    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "MoveCirc2Abs");
        return false;
    }
    
    return true;
}

bool ZmcController::moveArc2Rel(float midX, float midY,
                                 float endX, float endY,
                                 bool clockwise,
                                 const AxisParams& motion,
                                 QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Motion, "moveArc2Rel", [&] { return moveArc2Rel(midX, midY, endX, endY, clockwise, motion, err); }, err, eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!checkNoJointTask5D(err)) return false;
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    
    if(m_axisCompEnabled.load()){
        JointPose5D q;if(!readMotionStartPose5D(q,err))return false;
        mls::runtime::Joint start{q.x,q.y,q.z,q.c,q.b},mid{q.x+double(midX),q.y+double(midY),q.z,q.c,q.b},end{q.x+double(endX),q.y+double(endY),q.z,q.c,q.b};
        mls::runtime::Arc arc;std::string detail;
        const bool orderedClockwise=(mid[0]-start[0])*(end[1]-start[1])-(mid[1]-start[1])*(end[0]-start[0])<0;
        if(!mls::runtime::threePointArc(start,mid,end,orderedClockwise,arc,detail)){if(err)*err=QString::fromStdString(detail);return false;}
        arc.chordToleranceMm=0.001;
        return compensatedArc(end,arc,motion,err);
    }

    // 设置运动参数
    if (!writeMotionParams(AxisX, motion, err)) return false;
    
    int axislist[2] = { AxisX, AxisY };
    
    // The ordered start/mid/end points determine orientation. Keep the legacy
    // parameter for source compatibility; do not reverse a valid midpoint.
    Q_UNUSED(clockwise);
    const int32 ret = ZAux_Direct_MoveCirc2(m_handle, 2, axislist,
                                                    midX, midY, endX, endY);

    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "MoveCirc2");
        return false;
    }
    
    return true;
}

bool ZmcController::moveArcCenterAbs(float centerI, float centerJ,
                                      float endX, float endY,
                                      bool clockwise,
                                      const AxisParams& motion,
                                      QString* err)
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Motion, "moveArcCenterAbs", [&] { return moveArcCenterAbs(centerI, centerJ, endX, endY, clockwise, motion, err); }, err, eci::EciIoScheduler::Motion);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!checkNoJointTask5D(err)) return false;
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    
    if(m_axisCompEnabled.load()){
        JointPose5D q;if(!readMotionStartPose5D(q,err))return false;
        mls::runtime::Joint end{endX,endY,q.z,q.c,q.b};
        mls::runtime::Arc arc;arc.plane=mls::runtime::Plane::XY;arc.clockwise=clockwise;
        arc.centerOffset[0]=centerI;arc.centerOffset[1]=centerJ;arc.chordToleranceMm=0.001;
        return compensatedArc(end,arc,motion,err);
    }

    // 设置运动参数
    if (!writeMotionParams(AxisX, motion, err)) return false;
    
    int axislist[2] = { AxisX, AxisY };
    
    // 使用圆心偏移方式
    // 计算中间点（近似方法，实际应该用 MoveCircAbs）
    // 这里简化处理，使用三点圆弧近似
    
    // 获取当前位置
    double curX, curY, curZ;
    if (!readAxisPositions(curX, curY, curZ, err)) {
        return false;
    }
    
    // 计算圆心绝对位置
    float centerX = static_cast<float>(curX) + centerI;
    float centerY = static_cast<float>(curY) + centerJ;
    
    // 计算半径
    float radius = std::sqrt(centerI * centerI + centerJ * centerJ);
    
    // 计算中间点（在圆弧中间）
    float startAngle = std::atan2(-centerJ, -centerI);
    float endAngle = std::atan2(endY - centerY, endX - centerX);
    
    float midAngle;
    if (clockwise) {
        if (endAngle > startAngle) endAngle -= 2 * 3.14159265f;
        midAngle = (startAngle + endAngle) / 2;
    } else {
        if (endAngle < startAngle) endAngle += 2 * 3.14159265f;
        midAngle = (startAngle + endAngle) / 2;
    }
    
    float midX = centerX + radius * std::cos(midAngle);
    float midY = centerY + radius * std::sin(midAngle);
    
    // 使用三点圆弧
    int32 ret = ZAux_Direct_MoveCirc2Abs(m_handle, 2, axislist, 
                                          midX, midY, endX, endY);
    
    if (ret != ERR_OK) {
        if (err) *err = errorTextFromCode(ret, "MoveCirc2Abs (center mode)");
        return false;
    }
    
    return true;
}

// ==================== 轴状态检查 ====================

bool ZmcController::areAllAxesIdle(QString* err) const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "areAllAxesIdle", [&] { return areAllAxesIdle(err); }, err, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    if (!m_handle) {
        if (err) *err = QStringLiteral("尚未连接控制器");
        return false;
    }
    
    // All five physical joints, including pure C/B moves.
    for (int axis = AxisX; axis <= AxisB; ++axis) {
        int status = 0;
        int32 ret = ZAux_Direct_GetIfIdle(m_handle, axis, &status);
        if (ret != ERR_OK) {
            if (err) *err = errorTextFromCode(ret, "GetIfIdle");
            return false;
        }
        // status == -1 表示空闲（非零），status == 0 表示运动中
        if (status == 0) {
            return false;  // 还有轴在运动
        }
    }
    
    return true;  // 所有轴都空闲
}

bool ZmcController::isTrajectoryComplete() const
{
    if (!m_io.isWorkerThread()) {
        return runIo(IoPriority::Control, "isTrajectoryComplete", [&] { return isTrajectoryComplete(); }, nullptr, eci::EciIoScheduler::None);
    }

    p0::IoLock ioLock(m_ioMutex);
    // 1. 检查是否还有数据要发送
    if (m_mergeIndex < m_mergePath.size()) {
        return false;
    }
    
    // 2. 检查所有轴是否空闲
    return areAllAxesIdle(nullptr);
}
