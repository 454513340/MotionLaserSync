#pragma once

#include "core/DataTypes.h"

#include <array>
#include <cmath>

/**
 * @brief P4 五轴关节空间点到点规划器。
 *
 * 目标：在不引入 TCP/传感器几何的前提下，计算一个可安全交给 ZMotion 多轴 MOVEABS
 * 执行的五轴同步关节段。XYZ 使用 mm，C/B 使用 deg。规划器只保证每个物理轴的
 * 速度/加速度/减速度不超过当前轴参数的 speed/accel/decel * speedScale。
 *
 * controllerMetricLength 是控制器数值关节空间长度，并不表示 TCP 的 mm 距离；
 * 因此 P4 只能用于关节空间同步定位，不能作为最终五轴 TCP 恒速扫描规划器。
 */
class JointTrajectoryPlanner5Axis
{
public:
    static JointMotionPlan5D planPointToPoint(
        const JointPose5D& start,
        const JointPose5D& target,
        const std::array<AxisParams, Constants::kAxisParamArraySize>& axisParams,
        double speedScale)
    {
        JointMotionPlan5D plan;
        plan.start = start;
        plan.target = target;
        plan.speedScale = speedScale;

        if (!start.isFinite() || !target.isFinite()) {
            plan.error = QStringLiteral("五轴起点/目标包含 NaN 或 Inf");
            return plan;
        }

        if (!std::isfinite(speedScale) || speedScale <= 0.0 || speedScale > 1.0) {
            plan.error = QStringLiteral("五轴速度倍率必须在 (0, 1] 范围内");
            return plan;
        }

        // 先按上位机机械坐标软限位检查；控制器执行前还会再次读取控制器实际软限位复核。
        for (int axis = AxisX; axis <= AxisB; ++axis) {
            const double value = jointValue(target, axis);
            const AxisParams& p = axisParams[axis];
            if (value < static_cast<double>(p.softLimitNeg) - 1e-9 ||
                value > static_cast<double>(p.softLimitPos) + 1e-9) {
                plan.error = QStringLiteral("%1轴目标 %2 超出机械软限位 [%3, %4]")
                                 .arg(axisName(axis))
                                 .arg(value, 0, 'f', 6)
                                 .arg(p.softLimitNeg, 0, 'f', 6)
                                 .arg(p.softLimitPos, 0, 'f', 6);
                return plan;
            }
        }

        double delta[Constants::kAxisParamArraySize] = {0.0};
        double metricSquared = 0.0;
        int activeCount = 0;
        int limitingAxis = AxisX;
        double longestNormalizedTime = -1.0;

        for (int axis = AxisX; axis <= AxisB; ++axis) {
            delta[axis] = jointValue(target, axis) - jointValue(start, axis);
            metricSquared += delta[axis] * delta[axis];
            if (std::abs(delta[axis]) > 1e-9) {
                ++activeCount;
                const double vmax = static_cast<double>(axisParams[axis].speed) * speedScale;
                if (!std::isfinite(vmax) || vmax <= 0.0) {
                    plan.error = QStringLiteral("%1轴运行速度无效，无法规划五轴同步运动")
                                     .arg(axisName(axis));
                    return plan;
                }
                const double t = std::abs(delta[axis]) / vmax;
                if (t > longestNormalizedTime) {
                    longestNormalizedTime = t;
                    limitingAxis = axis;
                }
            }
        }

        plan.controllerMetricLength = std::sqrt(metricSquared);
        plan.baseAxis = limitingAxis;

        JointTrajectoryPoint5D p0;
        p0.position = start;
        p0.timeFromStartSec = 0.0;
        p0.scanEnabled = false;
        plan.trajectory.points.append(p0);

        if (activeCount == 0 || plan.controllerMetricLength <= 1e-9) {
            plan.valid = true;
            plan.hasMotion = false;
            plan.estimatedDurationSec = 0.0;
            JointTrajectoryPoint5D p1;
            p1.position = target;
            p1.timeFromStartSec = 0.0;
            p1.scanEnabled = false;
            plan.trajectory.points.append(p1);
            return plan;
        }

        // 对于一条关节直线，轴 i 的分速度 = vectorSpeed * |dq_i| / L。
        // 因此 vectorSpeed <= vmax_i * L / |dq_i| 可保证每轴均不超速。
        double vectorSpeed = 1.0e100;
        double vectorAccel = 1.0e100;
        double vectorDecel = 1.0e100;

        for (int axis = AxisX; axis <= AxisB; ++axis) {
            const double ad = std::abs(delta[axis]);
            if (ad <= 1e-9) continue;

            const AxisParams& p = axisParams[axis];
            const double vmax = static_cast<double>(p.speed) * speedScale;
            const double amax = static_cast<double>(p.accel) * speedScale;
            const double dmax = static_cast<double>(p.decel) * speedScale;

            if (!std::isfinite(vmax) || vmax <= 0.0 ||
                !std::isfinite(amax) || amax <= 0.0 ||
                !std::isfinite(dmax) || dmax <= 0.0) {
                plan.error = QStringLiteral("%1轴 speed/accel/decel 参数无效")
                                 .arg(axisName(axis));
                return plan;
            }

            updateMinimum(vectorSpeed, vmax * plan.controllerMetricLength / ad);
            updateMinimum(vectorAccel, amax * plan.controllerMetricLength / ad);
            updateMinimum(vectorDecel, dmax * plan.controllerMetricLength / ad);
        }

        if (!std::isfinite(vectorSpeed) || !std::isfinite(vectorAccel) || !std::isfinite(vectorDecel) ||
            vectorSpeed <= 0.0 || vectorAccel <= 0.0 || vectorDecel <= 0.0) {
            plan.error = QStringLiteral("无法得到有效的五轴矢量速度/加速度");
            return plan;
        }

        plan.vectorSpeed = vectorSpeed;
        plan.vectorAccel = vectorAccel;
        plan.vectorDecel = vectorDecel;
        plan.srampMs = static_cast<double>(axisParams[limitingAxis].sramp);
        plan.estimatedDurationSec = estimateTrapezoidDuration(
            plan.controllerMetricLength, vectorSpeed, vectorAccel, vectorDecel);

        JointTrajectoryPoint5D p1;
        p1.position = target;
        p1.timeFromStartSec = plan.estimatedDurationSec;
        p1.scanEnabled = false;
        plan.trajectory.points.append(p1);

        plan.hasMotion = true;
        plan.valid = true;
        return plan;
    }

    /**
     * @brief 将多个五轴关节点组织为统一轨迹数据结构，并给出保守累计时间。
     * @note P4 只建立数据/规划能力；V17 实机执行入口仍先采用单段 point-to-point。
     */
    static bool buildWaypointTrajectory(
        const QVector<JointPose5D>& waypoints,
        const std::array<AxisParams, Constants::kAxisParamArraySize>& axisParams,
        double speedScale,
        JointTrajectory5D& trajectory,
        QString* error = nullptr)
    {
        trajectory.points.clear();
        if (waypoints.isEmpty()) {
            if (error) *error = QStringLiteral("五轴轨迹点为空");
            return false;
        }
        if (!waypoints.first().isFinite()) {
            if (error) *error = QStringLiteral("五轴轨迹首点包含非法数值");
            return false;
        }

        JointTrajectoryPoint5D first;
        first.position = waypoints.first();
        first.timeFromStartSec = 0.0;
        trajectory.points.append(first);

        double accumulated = 0.0;
        for (int i = 1; i < waypoints.size(); ++i) {
            const JointMotionPlan5D seg = planPointToPoint(
                waypoints[i - 1], waypoints[i], axisParams, speedScale);
            if (!seg.valid) {
                if (error) {
                    *error = QStringLiteral("五轴轨迹第 %1 段规划失败: %2")
                                 .arg(i - 1).arg(seg.error);
                }
                trajectory.points.clear();
                return false;
            }
            accumulated += seg.estimatedDurationSec;
            JointTrajectoryPoint5D pt;
            pt.position = waypoints[i];
            pt.timeFromStartSec = accumulated;
            pt.scanEnabled = false;
            trajectory.points.append(pt);
        }
        return true;
    }

private:
    static QString axisName(int axis)
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

    static double jointValue(const JointPose5D& q, int axis)
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

    static void updateMinimum(double& current, double candidate)
    {
        if (candidate < current) current = candidate;
    }

    static double estimateTrapezoidDuration(double distance,
                                            double speed,
                                            double accel,
                                            double decel)
    {
        if (distance <= 1e-12) return 0.0;

        const double accelDistance = speed * speed / (2.0 * accel);
        const double decelDistance = speed * speed / (2.0 * decel);

        if (accelDistance + decelDistance <= distance) {
            const double cruiseDistance = distance - accelDistance - decelDistance;
            return speed / accel + cruiseDistance / speed + speed / decel;
        }

        // 三角速度曲线：vp^2/(2a) + vp^2/(2d) = distance
        const double peakSquared = 2.0 * distance * accel * decel / (accel + decel);
        const double peakSpeed = std::sqrt(peakSquared > 0.0 ? peakSquared : 0.0);
        return peakSpeed / accel + peakSpeed / decel;
    }
};
