#pragma once

#include <cstdint>

/**
 * @brief 五轴机床现场标定常量。
 *
 * 重新用千分表校准 B/C 机械零位后，通常只需要修改 kZeroCountsC / kZeroCountsB。
 * ZERO_COUNTS = P21.32 * 8388608 + P21.34
 * P21.32 为有符号多圈数，P21.34 为 23bit 单圈位置。
 */
namespace MachineCalibration {

constexpr int kEncoderBits = 23;
constexpr int64_t kEncoderCpr = 1LL << kEncoderBits; // 8,388,608 count/rev

// ===== 2026-09-04 千分表重新校准后的真正运动学零位 =====
// C轴机械 C=0°：P21.32=0，P21.34≈8,163,418
constexpr int64_t kZeroCountsC = 8163418;

// B轴机械 B=0°：P21.32=1，P21.34≈811,797
// 1 * 8,388,608 + 811,797 = 9,200,405
constexpr int64_t kZeroCountsB = 9200405;

// 机械减速比
constexpr double kReducerRatioC = 10.0;
constexpr double kReducerRatioB = 100.0;

// 禾川 P21.32/P21.34 原始计数 -> 上位机/控制器机械坐标的方向。
// 控制器 Axis4/5 已设置 ENCODER_RATIO(1,-1)，现场 V15 已验证该方向正确。
constexpr int kAbsoluteCountToControllerSignC = -1;
constexpr int kAbsoluteCountToControllerSignB = -1;

// ===== 五轴运动学坐标约定 =====
// XYZ 控制器正方向与机床数学 +X/+Y/+Z 一致。
constexpr double kKinematicSignX = +1.0;
constexpr double kKinematicSignY = +1.0;
constexpr double kKinematicSignZ = +1.0;

// 从 +Z/+Y 朝原点看，右手正转分别对应控制器 C- / B-。
// 因此运动学数学角 thetaC=-C_controller，thetaB=-B_controller。
constexpr double kKinematicSignC = -1.0;
constexpr double kKinematicSignB = -1.0;

// 当前控制器 Basic 中的机械软限位。
constexpr double kCMinDeg = -90.0;
constexpr double kCMaxDeg = +90.0;
constexpr double kBMinDeg = -90.0;
constexpr double kBMaxDeg = +90.0;

} // namespace MachineCalibration
