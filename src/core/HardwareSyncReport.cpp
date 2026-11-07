#include "core/HardwareSyncReport.h"
#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>

namespace syncdiag {
namespace {
void distribution(std::ostream& out, const Distribution& d, const char* label) {
    if (!d.count) { out << "  " << label << ": 数据不足或时间戳无效\n"; return; }
    out << "  " << label << ": 均值=" << d.mean << " ms，最小=" << d.min
        << " ms，最大=" << d.max << " ms，标准差=" << d.stddev() << " ms\n";
}
void timing(std::ostream& out, const TimingSummary& t) {
    distribution(out, t.actual, "相邻接收帧时间差");
    distribution(out, t.residual, "相对 ΔIndex×周期 的残差");
    distribution(out, t.normalized, "每索引平均时间差 dt/ΔIndex（不是逐帧极值）");
    out << "  可分析间隔=" << t.actual.count << "，跨索引间隔=" << t.crossIndexIntervals
        << "，索引无效/重复/倒退跳过=" << t.skippedIndex << "，时间非正/非有限=" << t.invalidTime << "\n";
    out << "  时间残差超阈值=" << t.warningCount << "；时间残差不能单独判定丢包或漏触发\n";
    for (const auto& r : t.warnings)
        out << "    Index " << r.first << "→" << r.last << ": ΔIndex=" << r.deltaIndex
            << "，预期=" << r.expectedMs << " ms，实测=" << r.actualMs
            << " ms，残差=" << r.residualMs << " ms\n";
    if (t.invalidTime) out << "  时钟回绕/重置语义尚未证明，不自动展开为连续时间。\n";
}
}
std::string schedulerReport(const eci::EciIoScheduler::Stats& s) {
    std::ostringstream out; out.imbue(std::locale::classic()); out << std::fixed << std::setprecision(3);
    out << "【V19 上位机软实时通信调度器（本程序生命周期累计）】\n";
    out << "  队列待处理=" << s.pending << "，历史最高水位=" << s.highWater
        << "，会话代际=" << s.generation.session << "\n";
    out << "  I/O事务执行中=" << (s.inFlight ? "是" : "否");
    if (s.inFlight) out << "，名称=" << s.inFlightName << "，已持续=" << s.inFlightMs << " ms";
    out << "\n";
    const char* names[] = {"停止/安全收尾", "运动/补缓冲", "TABLE/测量", "控制/状态", "参数/诊断"};
    for (unsigned i = 0; i < static_cast<unsigned>(eci::EciIoScheduler::Priority::Count); ++i) {
        const auto& c = s.classes[i];
        out << "  " << names[i] << ": 提交=" << c.submitted << "，完成=" << c.completed
            << "，返回失败=" << c.failed << "，取消=" << c.cancelled << "，拒绝=" << c.rejected
            << "，排队过期=" << c.expired << "，软截止期超时=" << c.late << "\n"
            << "    最大排队=" << c.maxQueueMs << " ms，最大事务执行=" << c.maxExecutionMs << " ms\n";
    }
    out << "  此统计只读主机内存。截止期是调度预算，不是硬实时保证。\n"
        << "  STOP 优先于尚未开始的请求；不能抢占执行中的事务/DLL。\n"
        << "  软件停止不是硬件急停；DLL 阻塞时不能承诺最坏停止延迟。\n";
    return out.str();
}
std::string hardwareSyncReport(const HardwareReportInput& in) {
    std::ostringstream out; out.imbue(std::locale::classic()); out << std::fixed << std::setprecision(3);
    out << "═══════════════════════════════════════════════════════════════\n"
        << "              V19 硬件同步采集数据分析报告\n"
        << "═══════════════════════════════════════════════════════════════\n\n";
    out << "【本次会话拓扑】\n  ";
    if(in.topologyId==1) out << "H1：传感器外部EdgeRising，控制器IN0/R0 REGIST(104)\n";
    else if(in.topologyId==3) out << "H3：传感器内部周期，CMM RS422，控制器IN1/R1 REGIST(" << in.cmmR1Edge << ")\n";
    else out << "未知：不能假设为H1\n";
    out << "【完整性状态】\n  锁存故障: " << (in.fault ? "是，本段无效" : "未检测到") << "\n";
    if (in.fault) out << "  原因: " << in.faultReason << "\n  现有点云仅供诊断，不自动重新锚定 pose[0]。\n";
    if (in.running) out << "  当前仍在采集中：各模块不是同一时刻快照，请停止收尾后再作最终判断。\n";
    out << "  索引缺口/重复/倒退/非法: " << in.missing << " / " << in.duplicates << " / "
        << in.backward << " / " << in.invalidIndices << "\n"
        << "  TABLE读取失败=" << in.readErrors << "，溢出=" << in.overflows << "\n"
        << "  控制器已发布锁存计数仅在停止收尾时缓存；它与published ordinal同源，不是独立物理触发边沿计数。\n"
        << "  首Profile与首published-latch的对应由Probe建立；计数一致仍不是外部硬件触发身份的证明。\n\n";
    out << "【轮廓级统计：不从点云反推帧数】\n"
        << "  位置采样=" << in.poses << "，收到Profile=" << in.received << "\n"
        << "  已配对Profile=" << in.aligned << "，其中有有效点=" << in.validAligned
        << "，无有效点=" << in.emptyAligned << "，时域边界配对但不重建=" << in.temporalBoundarySkipped << "\n"
        << "  待配对=" << in.pending << "，已丢弃=" << in.dropped << "\n"
        << "  无有效点是通过 SensorUtils 有效性过滤后的结果，不等价于空行程或网络丢帧。\n";
    if (!in.frames.empty()) out << "  全部接收帧首末Index=" << in.frames.front().index << " → " << in.frames.back().index
                               << "，保存的逐帧时间记录=" << in.frames.size() << "（包括无有效点轮廓）\n";
    const bool cleanIndex = !in.missing && !in.duplicates && !in.backward && !in.invalidIndices;
    if (cleanIndex) out << "  接收索引未检测到缺口/重复/倒退/非法；仍需核对控制器published-latch计数。\n";
    out << "\n【ECI3808 已发布锁存计数（非独立触发计数）】\n";
    if (in.triggerCountValid) out << "  TABLE[336]=" << in.triggers << "，published-latch减接收=" << in.triggers - in.received
                                << "，位置减published-latch=" << in.poses - in.triggers << "\n";
    else out << "  尚无有效停止后快照，不按 0 次触发解释。\n";
    out << "\n【位置主机时间估计分析】\n"
        << "  来源：PC读取TABLE批次的主机时间减去批内名义周期。不是硬件锁存/曝光时间。\n";
    Distribution poseD; std::uint64_t poseWarnings = 0;
    for (std::size_t i = 1; i < in.poseEstimateUs.size(); ++i) {
        const double dt = (in.poseEstimateUs[i] - in.poseEstimateUs[i-1]) / 1000.0;
        poseD.add(dt); if (std::abs(dt - in.periodMs) > in.periodMs * 0.5) ++poseWarnings;
    }
    distribution(out, poseD, "主机估计间隔");
    out << "  主机估计超阈值间隔=" << poseWarnings << "；不据此认定控制卡漏采样/定时器抖动。\n";

    // Hardware mode deliberately inserts a Probe -> continuous handshake gap.
    // It is not a lost steady-state trigger period and must not contaminate steady-state jitter.
    std::vector<ProfileRecord> steadyFrames;
    if (in.frames.size() > 1) steadyFrames.assign(in.frames.begin() + 1, in.frames.end());
    const auto sensor = analyzeReceivedTiming(steadyFrames, in.periodMs, true);
    const auto pc = analyzeReceivedTiming(steadyFrames, in.periodMs, false);
    out << "\n【连续阶段Profile曝光时间戳分析】\n"
        << "  首个Probe→连续阶段的握手间隔已排除；其后预期 Δt=ΔIndex×" << in.periodMs
        << " ms，残差阈值 ±" << in.periodMs * 0.5 << " ms。\n";
    timing(out, sensor);
    out << "\n【连续阶段Profile的PC回调接收时间分析】\n"
        << "  同样排除Probe→连续切换间隔；来源为 LaserProfile.tPcUs（不是位置反推时间），含网络、驱动及主机调度影响。\n";
    timing(out, pc);

    out << "\n【Profile gap forensic diagnostics】\n";
    std::size_t forensicGaps = 0;
    for (std::size_t i = 1; i < in.frames.size(); ++i) {
        const auto& a = in.frames[i - 1];
        const auto& b = in.frames[i];
        if (a.index > p0::ProfileCounter24::kMask || b.index > p0::ProfileCounter24::kMask) continue;
        const std::uint32_t di = (b.index - a.index) & p0::ProfileCounter24::kMask;
        if (di <= 1 || di >= p0::ProfileCounter24::kModulus / 2) continue;
        ++forensicGaps;
        if (forensicGaps > 10) continue;
        const std::uint64_t dc = b.callbackOrdinal >= a.callbackOrdinal
            ? b.callbackOrdinal - a.callbackOrdinal : 0;
        out << "  GAP Index " << a.index << "→" << b.index << "，缺失=" << (di - 1)
            << "；callbackOrdinal " << a.callbackOrdinal << "→" << b.callbackOrdinal
            << "，ΔCallback=" << dc
            << "；callback PC Δ=" << (b.callbackPcUs - a.callbackPcUs) / 1000.0 << " ms"
            << "；sensor Δ=" << (b.shutterOpenSec - a.shutterOpenSec) * 1000.0 << " ms\n"
            << "    ingressDepth=" << a.ingressDepth << "→" << b.ingressDepth
            << "；callbackService=" << a.callbackServiceUs << "/" << b.callbackServiceUs << " us"
            << "；decoderQueue=" << a.decoderQueueUs << "/" << b.decoderQueueUs << " us"
            << "；decodeService=" << a.decodeServiceUs << "/" << b.decodeServiceUs << " us\n";
        if (dc == 1 && in.sensorIngressDrops == 0 && in.sensorConvertFailures == 0)
            out << "    判读：缺失索引未形成额外SDK callback，且本机raw-ring/转换无丢弃证据；优先排查传感器/以太网/LLT callback之前的数据链。\n";
        else if (dc > 1 && (in.sensorIngressDrops || in.sensorConvertFailures))
            out << "    判读：callback ordinal跨越且本机存在ingress/转换失败，主机接收/解码链可能参与该缺口。\n";
        else
            out << "    判读：证据不足以唯一定位；结合raw-ring、转换失败与系统调度日志继续判断。\n";

        const std::size_t begin = i > 2 ? i - 2 : 0;
        const std::size_t end = std::min(in.frames.size(), i + 3);
        out << "    邻域:";
        for (std::size_t j = begin; j < end; ++j)
            out << " [idx=" << in.frames[j].index << ",cb=" << in.frames[j].callbackOrdinal
                << ",depth=" << in.frames[j].ingressDepth << ",dq=" << in.frames[j].decoderQueueUs << "us]";
        out << "\n";
    }
    if (!forensicGaps) out << "  本段未发现前向Profile索引缺口。\n";
    else out << "  共检测到 " << forensicGaps << " 个前向索引缺口；最多打印前10个。\n";

    out << "\n【有效轮廓之间的间隔：不作为丢帧计数】\n";
    const auto gaps = effectiveGaps(in.frames, in.periodMs);
    out << "  跨索引有效轮廓间隔数=" << gaps.size() << "\n";
    for (std::size_t i = 0; i < gaps.size() && i < 10; ++i) {
        const auto& g = gaps[i]; const auto& r = g.timing;
        out << "  Index " << r.first << "→" << r.last << ": ΔIndex=" << r.deltaIndex;
        if (r.indexValid && r.timeValid) out << "，预期=" << r.expectedMs << " ms，实测=" << r.actualMs
            << " ms，残差=" << r.residualMs << " ms";
        else out << "，索引/时间不可用，不作时序结论";
        out << "，中间已配对无有效点=" << g.confirmedEmpty << "，其它接收帧=" << g.otherReceived << "\n";
    }
    out << "  即便跨帧残差很小，也不能反推中间每一帧的最大抖动或绝对同步精度。\n";
    out << "\n【V32.13传感器正式session触发/回调诊断】\n"
        << "  原始SDK回调=" << in.sensorRawCallbacks
        << "，转换成功=" << in.sensorConvertedProfiles
        << "，转换失败=" << in.sensorConvertFailures
        << "，raw-ring丢弃=" << in.sensorIngressDrops << "\n"
        << "  raw-ring当前/最高/容量=" << in.sensorRawRingDepth << "/"
        << in.sensorRawRingHighWater << "/" << in.sensorRawRingCapacity
        << "；lastCallbackOrdinal=" << in.sensorLastCallbackOrdinal << "\n"
        << "  callback最大服务时间=" << in.sensorMaxCallbackServiceUs
        << " us；decoder最大排队=" << in.sensorMaxDecoderQueueUs
        << " us；decode最大执行=" << in.sensorMaxDecodeServiceUs << " us\n"
        << "  lastConvertRet=" << in.sensorLastConvertResult
        << "，lastCallbackBytes=" << in.sensorLastCallbackBytes
        << "，lastResolution=" << in.sensorLastResolution << "\n"
        << "  TRIGGER requested=0x" << std::hex << in.sensorTriggerRequested
        << " readback=0x" << in.sensorTriggerReadback
        << "；DIGITAL_IO requested=0x" << in.sensorDigitalIoRequested
        << " readback=0x" << in.sensorDigitalIoReadback << std::dec
        << "；readbackValid=" << (in.sensorTriggerReadbackValid ? 1 : 0)
        << "；modeMatches=" << (in.sensorTriggerConfigMatches ? 1 : 0)
        << "；cmmPortVerified=" << (in.sensorCmmPortReadbackValid ? 1 : 0) << "\n";
    if(in.topologyId==3)
        out << "  H3只校验内部触发使能位关闭及RS422 CMM端口；不要按H1外部触发字节全等判断。\n";
    else
        out << "  判读：raw=0优先检查物理触发/接线/输入配置；raw>0但converted=0优先检查SDK Profile格式/转换参数。\n";

    out << "\n【V27.6.55 硬件同步时域标定/补偿】\n"
        << "  定义：tau = t_sensor,effective - t_ECI,latch；正值表示传感器有效测量晚于同ordinal REGIST(" <<
           (in.topologyId==3 ? in.cmmR1Edge : 104) << ")锁存。\n"
        << "  运行时补偿=" << (in.temporalCorrectionEnabled ? "启用" : "未启用");
    if(in.temporalCorrectionEnabled) {
        out << "，Calibration ID=" << in.temporalCalibrationId
            << "，tau=" << in.temporalDelayUs << " us，std=" << in.temporalDelayStdUs << " us\n";
        if(in.topologyId==3)
            out << "  H3方法：依据内部Profile与CMM R1同ordinal位姿和经验证的时间延迟补偿；不引用H1 OP14路径。\n";
        else
            out << "  方法：保持当前OP14->scanCONTROL + OP14->IN0->REGIST(104)不变；在上位机用同ordinal与下一条运行时周期五轴MPOS对XYZ/C/B插值，再进入Physical17。\n";
        out
            << "  当前运行时周期=" << in.periodMs << " ms；Probe和最终tail因没有可信相邻周期位姿而只计入Profile/Pose配对，不生成补偿点云；本段数量=" << in.temporalBoundarySkipped << "。\n";
    } else {
        out << "；当前点云使用原始同ordinal REGIST(" << (in.topologyId==3 ? in.cmmR1Edge : 104) << ")锁存位姿";
        if(!in.temporalCalibrationId.empty()) {
            out << "；活动标定已加载但未应用：Calibration ID=" << in.temporalCalibrationId
                << "，tau=" << in.temporalDelayUs << " us，std=" << in.temporalDelayStdUs << " us";
        }
        out << "。\n";
    }
    out << "  该动态几何标定给出等效固定时延；电气传播抖动/曝光相位的独立证据仍应由示波器/CMM Trigger测试提供。\n";

    out << "\n【传感器启动配置（上位机记录，非设备回读）】\n"
        << "  曝光=" << in.exposureMs << " ms，空闲=" << in.idleMs << " ms，名义触发周期=" << in.periodMs << " ms\n";
    if (in.exposureMs + in.idleMs > in.periodMs + 1e-4)
        out << "  配置周期超过名义触发周期，需要检查；不据配置单独判定已发生漏帧。\n";
    out << "\n【点级统计】\n  有效点云点数=" << in.cloudPoints
        << "；与轮廓级已配对数 " << in.aligned << " 的单位不同。\n";
    const bool metadataMatch = in.frames.size() == static_cast<std::size_t>(in.received)
        && in.validAligned + in.emptyAligned + in.temporalBoundarySkipped == in.aligned;
    const bool countMatch = in.received > 0 && in.poses == in.received && in.aligned == in.received
        && !in.pending && !in.dropped && in.triggerCountValid && in.triggers == in.received
        && metadataMatch && in.synchronized;
    out << "\n【诊断结论】\n";
    if (!metadataMatch) out << "  帧元数据数量/分类与统计不一致；不能套用旧点云的有效帧时间代替全部接收帧。\n";
    if (!in.running && !in.fault && cleanIndex && countMatch && !in.overflows)
        out << "  计数与接收索引检查通过，未检测到配对缺口。\n";
    else out << "  完整性存在异常或证据未齐，请在停止收尾后核对日志，不能判定有效测量。\n";
    out << "  曝光时间残差超阈值=" << sensor.warningCount << "，曝光时间不可用间隔=" << sensor.invalidTime
        << "，主机位置估计超阈值=" << poseWarnings << "\n"
        << "  本结论不是物理触发身份、曝光-位置时间偏差、同步精度或完整五轴测量能力的证明。\n\n";
    out << schedulerReport(in.io);
    if(in.topologyId==3) {
        out << "\n【V32.16 H3 INTERNAL + CMM IN1/R1 + FULL-MPOS-LATCH同步链】\n"
            << "  scanCONTROL内部周期产生Profile并通过RS422 CMM输出对ECI IN1产生R1位置锁存；不依赖H1 OP14外部触发。\n"
            << "  传感器触发模式读回=" << (in.sensorTriggerReadbackValid && in.sensorTriggerConfigMatches ? "通过" : "未验证/不匹配")
            << "，RS422 CMM端口读回=" << (in.sensorCmmPortReadbackValid ? "通过" : "未验证/不匹配") << "。\n"
            << "  控制器REGIST(" << in.cmmR1Edge << ")应来自实际TABLE只读预检；published-latch计数不构成独立电气触发边沿证明。\n"
            << "  五轴MPOS与B/C机械绝对角offset规则保持不变；时序精度、实际CMM信号和夹具安全须独立验证。\n";
    } else if(in.topologyId==1) {
    out << "\n【V32.13 ECI3808 SINGLE-OP14 + FULL-MPOS-LATCH + ColdPrime同步链】\n"
        << "  现场只使用OP14：该线同时驱动scanCONTROL外部触发输入和ECI IN0；scanCONTROL使用EdgeRising，ECI使用REGIST(104) R0逻辑下降沿，两者按已测极性对齐在OP14关闭/线路回高时刻。\n"
        << "  BASIC严格按最终XYZ光栅尺架构设计：XYZ/C/B五轴均为ATYPE=4，TABLE记录中的X/Y/Z/C/B全部来自REGIST(104)硬件锁存raw MPOS，协议语义为HardwareLatched。\n"
        << "  TABLE[357]为五轴上电会话Home mask：X/Y/Z成功DATUM+SetMpos(0)后提交bits0..2；C/B在RS485绝对坐标ready后成功回到机械0°再提交bits3..4。mask=31才表示XYZ/C/B都完成过本次控制器上电会话回零。仅重启上位机可恢复；控制器/BASIC重启会清零。\n"
        << "  上位机普通实际位置、软件同步、Physical17和传感标定位姿统一使用XYZ/C/B实际MPOS；XYZ DPOS继续用于命令轨迹与DPOS-MPOS跟随误差诊断。\n"
        << "  冷上电第一次硬件采集存在数据链未完全ready的现场现象，因此正式session之前执行enable=3预热：同样只用OP14/PWM0按当前上位机运行时频率产生触发，预热Profile全部抑制/丢弃。\n"
        << "  预热结束后建立新session，清空并重新Arm XYZ/C/B五轴REGIST FIFO，再执行正式1 Profile + 1 Pose Probe；这样预热边沿不会污染正式ordinal。\n"
        << "  上位机保持TransferProfiles在线，等待至少8个可转换Profile；这样避免冷启动时唯一Probe被scanCONTROL/SDK/以太网数据面丢失后造成死等。\n"
        << "  B/C为增量编码器：RS485绝对位置先按机械零点/方向/减速比换算，再与控制器raw MPOS建立offset；session开始时冻结offset，TABLE raw C/B按machine=rawMPOS+offset恢复机械绝对角。\n"
        << "  停止时BASIC先把PWM0切到100%保持超过一个当前运行时触发周期，再切到0%产生最终同步边沿；随后等待五轴REGIST FIFO、Buffer-B、上位机TABLE读取以及Profile raw-ring/DecodeThread计数收敛。\n"
        << "  V27.6.44可通过固定标准球正反多速度扫描标定等效动态时延；独立电气抖动/曝光相位仍建议用示波器验证。\n"
        << "═══════════════════════════════════════════════════════════════\n";
    } else {
        out << "\n【硬件同步拓扑未知】未得到H1/H3类型，禁止从本文推断具体物理信号路径。\n";
    }
    return out.str();
}
} // namespace syncdiag
