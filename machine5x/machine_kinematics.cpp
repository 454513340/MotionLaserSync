#include "machine_kinematics.h"

#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

// The physical positive directions of both rotary axes are opposite to the
// mathematical right-hand positive directions used by Eigen::AngleAxisd.
// Keep commanded/NC values positive in MachineState and apply the sign here.
constexpr double kBCommandToMathSign = -1.0;
constexpr double kCCommandToMathSign = -1.0;

double deg2rad(double deg) { return deg * kPi / 180.0; }

Eigen::Isometry3d translation(double x, double y, double z) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    return T;
}

Eigen::Isometry3d rotationY(double rad) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return T;
}

Eigen::Isometry3d rotationZ(double rad) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return T;
}
} // namespace

Eigen::Isometry3d linkDelta(const std::string& link, const MachineState& q) {
    // All zero-pose meshes were exported in the same MCS.
    // The MCS origin is the B/C-axis intersection, so no pivot translations
    // T(+O) * R * T(-O) are required.
    const Eigen::Isometry3d Tx = translation(q.x_mm, 0.0, 0.0);
    const Eigen::Isometry3d Ty = translation(0.0, q.y_mm, 0.0);
    const Eigen::Isometry3d Tz = translation(0.0, 0.0, q.z_mm);

    // Physical positive rotary directions are opposite to mathematical +RH.
    const Eigen::Isometry3d RC = rotationZ(
        deg2rad(kCCommandToMathSign * q.c_deg));
    const Eigen::Isometry3d RB = rotationY(
        deg2rad(kBCommandToMathSign * q.b_deg));

    // Correct serial rotary-head chain for this machine:
    // BASE -> X -> Y -> Z -> C -> B -> END_SENSOR
    //
    // C is the parent rotary axis. Therefore:
    //   - C command rotates C, B and END_SENSOR.
    //   - B command rotates only B and END_SENSOR relative to C.
    // B is about the local +Y axis after the C rotation, hence RC * RB.
    if (link == "01_BASE")       return Eigen::Isometry3d::Identity();
    if (link == "02_X_AXIS")     return Tx;
    if (link == "03_Y_AXIS")     return Tx * Ty;
    if (link == "04_Z_AXIS")     return Tx * Ty * Tz;
    if (link == "05_C_AXIS")     return Tx * Ty * Tz * RC;
    if (link == "06_B_AXIS")     return Tx * Ty * Tz * RC * RB;
    if (link == "07_END_SENSOR") return Tx * Ty * Tz * RC * RB;
    return Eigen::Isometry3d::Identity();
}
