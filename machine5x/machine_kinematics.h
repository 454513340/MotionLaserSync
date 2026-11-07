#pragma once

#include <Eigen/Geometry>
#include <string>

struct MachineState {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    double b_deg = 0.0;
    double c_deg = 0.0;
};

// Zero-pose-to-current-pose transform for a rigid link.
// Chain: BASE -> X -> Y -> Z -> B -> C -> END_SENSOR.
// MCS origin = B/C axis intersection.
// Physical B+ and C+ are implemented with negative mathematical rotations.
Eigen::Isometry3d linkDelta(const std::string& linkName, const MachineState& state);
