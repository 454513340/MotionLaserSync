#pragma once

#include "core/DataTypes.h"
#include "core/MachineCalibration.h"

#include <QMatrix4x4>
#include <QVector3D>
#include <QString>
#include <QtMath>

#include <cmath>

// Windows SDK/部分第三方头文件可能定义 min/max 宏。
// 这些宏会破坏 std::max(...) 并导致 MSVC/IntelliSense 出现大量级联语法错误。
#ifdef max
#  undef max
#endif
#ifdef min
#  undef min
#endif

/**
 * @brief XYZ-C-B 双旋转头五轴机床的名义运动学模型（P1/P2/P3）。
 *
 * 已确认机械拓扑：
 *   X -> Y -> Z -> C -> B
 *   C 绕机床 Z 轴；C=0 时 B 绕机床 Y 轴；B/C 轴线相交。
 *   末端参考坐标系原点暂定义在 B/C 两轴线交点，暂不包含线激光传感器外参。
 *
 * 控制器角度与数学右手角方向相反：
 *   thetaC = -C_controller
 *   thetaB = -B_controller
 *
 * 因而：
 *   T_ME = Trans(X,Y,Z) * Rz(thetaC) * Ry(thetaB)
 *
 * 说明：本类刻意采用保守的 C++11 写法，避免不同 VS2022/Qt 工程语言标准、
 * Windows min/max 宏和 IntelliSense 对嵌套默认成员初始化产生兼容性问题。
 */
class MachineKinematics5Axis
{
public:
    struct Config
    {
        double xSign;
        double ySign;
        double zSign;
        double cSign;
        double bSign;

        double cMinDeg;
        double cMaxDeg;
        double bMinDeg;
        double bMaxDeg;

        double rotationTolerance;
        double orthonormalTolerance;
        double jointLimitToleranceDeg;

        Config()
            : xSign(MachineCalibration::kKinematicSignX),
              ySign(MachineCalibration::kKinematicSignY),
              zSign(MachineCalibration::kKinematicSignZ),
              cSign(MachineCalibration::kKinematicSignC),
              bSign(MachineCalibration::kKinematicSignB),
              cMinDeg(MachineCalibration::kCMinDeg),
              cMaxDeg(MachineCalibration::kCMaxDeg),
              bMinDeg(MachineCalibration::kBMinDeg),
              bMaxDeg(MachineCalibration::kBMaxDeg),
              rotationTolerance(1e-5),
              orthonormalTolerance(1e-5),
              jointLimitToleranceDeg(1e-4)
        {
        }
    };

    MachineKinematics5Axis()
        : m_cfg()
    {
    }

    explicit MachineKinematics5Axis(const Config& cfg)
        : m_cfg(cfg)
    {
    }

    const Config& config() const { return m_cfg; }
    void setConfig(const Config& cfg) { m_cfg = cfg; }

    static double normalizeDeg(double deg)
    {
        while (deg >= 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    /**
     * @brief P2 正运动学：控制器五轴坐标 -> B/C 交点末端齐次变换矩阵。
     */
    QMatrix4x4 forward(const JointPose5D& q) const
    {
        const double x = m_cfg.xSign * q.x;
        const double y = m_cfg.ySign * q.y;
        const double z = m_cfg.zSign * q.z;

        const double c = qDegreesToRadians(m_cfg.cSign * q.c);
        const double b = qDegreesToRadians(m_cfg.bSign * q.b);

        const double cc = std::cos(c);
        const double sc = std::sin(c);
        const double cb = std::cos(b);
        const double sb = std::sin(b);

        // R = Rz(c) * Ry(b)
        QMatrix4x4 t;
        t.setToIdentity();
        t(0,0) = static_cast<float>( cc * cb);
        t(0,1) = static_cast<float>(-sc);
        t(0,2) = static_cast<float>( cc * sb);
        t(1,0) = static_cast<float>( sc * cb);
        t(1,1) = static_cast<float>( cc);
        t(1,2) = static_cast<float>( sc * sb);
        t(2,0) = static_cast<float>(-sb);
        t(2,1) = 0.0f;
        t(2,2) = static_cast<float>( cb);
        t(0,3) = static_cast<float>(x);
        t(1,3) = static_cast<float>(y);
        t(2,3) = static_cast<float>(z);
        return t;
    }

    EndPose5D forwardPose(const JointPose5D& q) const
    {
        EndPose5D out;
        out.joints = q;
        out.transform = forward(q);
        out.position = QVector3D(out.transform(0,3), out.transform(1,3), out.transform(2,3));
        out.mathCdeg = m_cfg.cSign * q.c;
        out.mathBdeg = m_cfg.bSign * q.b;
        return out;
    }

    /**
     * @brief P3 解析逆运动学：可达的末端齐次矩阵 -> 控制器 X/Y/Z/C/B。
     * @note 目标旋转必须属于本机床的二维姿态子空间 Rz(thetaC)*Ry(thetaB)。
     */
    InverseKinematics5DResult inverse(const QMatrix4x4& target,
                                      bool enforceRotaryLimits = true) const
    {
        InverseKinematics5DResult result;

        if (std::abs(m_cfg.xSign) < 1e-12 ||
            std::abs(m_cfg.ySign) < 1e-12 ||
            std::abs(m_cfg.zSign) < 1e-12 ||
            std::abs(m_cfg.cSign) < 1e-12 ||
            std::abs(m_cfg.bSign) < 1e-12) {
            result.error = QStringLiteral("运动学方向配置包含0，无法求逆");
            return result;
        }

        if (!matrixFinite(target)) {
            result.error = QStringLiteral("目标齐次矩阵包含 NaN/Inf");
            return result;
        }

        if (std::abs(static_cast<double>(target(3,0))) > 1e-6 ||
            std::abs(static_cast<double>(target(3,1))) > 1e-6 ||
            std::abs(static_cast<double>(target(3,2))) > 1e-6 ||
            std::abs(static_cast<double>(target(3,3)) - 1.0) > 1e-6) {
            result.error = QStringLiteral("目标矩阵最后一行不是 [0 0 0 1]");
            return result;
        }

        double orthoResidual = 0.0;
        if (!rotationIsOrthonormal(target, orthoResidual)) {
            result.rotationResidual = orthoResidual;
            result.error = QStringLiteral("目标旋转矩阵不是合法正交旋转，残差=%1")
                               .arg(orthoResidual, 0, 'g', 8);
            return result;
        }

        // R = Rz(c) * Ry(b)
        // 第二列 [-sin(c), cos(c), 0]^T 恢复 c；
        // r20=-sin(b), r22=cos(b) 恢复 b。
        const double thetaC = std::atan2(-static_cast<double>(target(0,1)),
                                          static_cast<double>(target(1,1)));
        const double thetaB = std::atan2(-static_cast<double>(target(2,0)),
                                          static_cast<double>(target(2,2)));

        JointPose5D q;
        q.x = static_cast<double>(target(0,3)) / m_cfg.xSign;
        q.y = static_cast<double>(target(1,3)) / m_cfg.ySign;
        q.z = static_cast<double>(target(2,3)) / m_cfg.zSign;
        q.c = normalizeDeg(qRadiansToDegrees(thetaC) / m_cfg.cSign);
        q.b = normalizeDeg(qRadiansToDegrees(thetaB) / m_cfg.bSign);

        if (enforceRotaryLimits) {
            if (q.c < m_cfg.cMinDeg - m_cfg.jointLimitToleranceDeg ||
                q.c > m_cfg.cMaxDeg + m_cfg.jointLimitToleranceDeg) {
                result.error = QStringLiteral("IK C轴解 %1° 超出 [%2°, %3°]")
                                   .arg(q.c, 0, 'f', 6)
                                   .arg(m_cfg.cMinDeg, 0, 'f', 3)
                                   .arg(m_cfg.cMaxDeg, 0, 'f', 3);
                return result;
            }

            if (q.b < m_cfg.bMinDeg - m_cfg.jointLimitToleranceDeg ||
                q.b > m_cfg.bMaxDeg + m_cfg.jointLimitToleranceDeg) {
                result.error = QStringLiteral("IK B轴解 %1° 超出 [%2°, %3°]")
                                   .arg(q.b, 0, 'f', 6)
                                   .arg(m_cfg.bMinDeg, 0, 'f', 3)
                                   .arg(m_cfg.bMaxDeg, 0, 'f', 3);
                return result;
            }
        }

        const QMatrix4x4 rebuilt = forward(q);
        result.rotationResidual = maxRotationDifference(target, rebuilt);
        result.positionResidual = maxTranslationDifference(target, rebuilt);

        if (result.rotationResidual > m_cfg.rotationTolerance) {
            result.error = QStringLiteral("目标姿态不属于当前五轴 Rz(C)*Ry(B) 可达姿态，旋转残差=%1")
                               .arg(result.rotationResidual, 0, 'g', 8);
            return result;
        }

        result.joints = q;
        result.success = true;
        return result;
    }

    bool roundTrip(const JointPose5D& input,
                   JointPose5D& recovered,
                   double& maxResidual,
                   QString* error = nullptr) const
    {
        const QMatrix4x4 t = forward(input);
        const InverseKinematics5DResult ik = inverse(t, true);
        if (!ik.success) {
            if (error) *error = ik.error;
            return false;
        }

        recovered = ik.joints;
        maxResidual = 0.0;
        updateMaximum(maxResidual, std::abs(input.x - recovered.x));
        updateMaximum(maxResidual, std::abs(input.y - recovered.y));
        updateMaximum(maxResidual, std::abs(input.z - recovered.z));
        updateMaximum(maxResidual, std::abs(normalizeDeg(input.c - recovered.c)));
        updateMaximum(maxResidual, std::abs(normalizeDeg(input.b - recovered.b)));
        return true;
    }

    QString describeModel() const
    {
        return QStringLiteral("XYZ-C-B: T=Trans(X,Y,Z)*Rz(-C)*Ry(-B), B/C轴线相交，末端原点=B/C交点");
    }

private:
    static void updateMaximum(double& current, double candidate)
    {
        if (candidate > current) current = candidate;
    }

    static bool matrixFinite(const QMatrix4x4& m)
    {
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!std::isfinite(static_cast<double>(m(r,c)))) return false;
            }
        }
        return true;
    }

    bool rotationIsOrthonormal(const QMatrix4x4& m, double& residual) const
    {
        double r[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r[i][j] = static_cast<double>(m(i,j));
            }
        }

        residual = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double dot = 0.0;
                for (int k = 0; k < 3; ++k) {
                    dot += r[k][i] * r[k][j];
                }
                const double expected = (i == j) ? 1.0 : 0.0;
                updateMaximum(residual, std::abs(dot - expected));
            }
        }

        const double det =
            r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
            r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
            r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);

        updateMaximum(residual, std::abs(det - 1.0));
        return residual <= m_cfg.orthonormalTolerance;
    }

    static double maxRotationDifference(const QMatrix4x4& a,
                                        const QMatrix4x4& b)
    {
        double value = 0.0;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                updateMaximum(value,
                              std::abs(static_cast<double>(a(r,c)) -
                                       static_cast<double>(b(r,c))));
            }
        }
        return value;
    }

    static double maxTranslationDifference(const QMatrix4x4& a,
                                           const QMatrix4x4& b)
    {
        double value = 0.0;
        for (int r = 0; r < 3; ++r) {
            updateMaximum(value,
                          std::abs(static_cast<double>(a(r,3)) -
                                   static_cast<double>(b(r,3))));
        }
        return value;
    }

private:
    Config m_cfg;
};
