#pragma once

#include "calibration/SingleSphere14Calibration.h"
#include "core/DataTypes.h"

#include <QMatrix4x4>
#include <QVector4D>
#include <algorithm>
#include <cstdint>

namespace mls {
namespace calibration {

/**
 * @brief Thin Qt/V19 adapter for the portable single-sphere calibration core.
 *
 * This header contains no calibration logic. It only converts the project's
 * JointPose5D/LaserProfile types into the dependency-free calibration types.
 */
class SingleSphere14QtAdapter
{
public:
    static JointPose5d toPortable(const JointPose5D& pose)
    {
        JointPose5d out;
        out.x = pose.x;
        out.y = pose.y;
        out.z = pose.z;
        out.c = pose.c;
        out.b = pose.b;
        return out;
    }

    /**
     * Convert a segmented interval of one acquired profile.
     *
     * firstPoint is inclusive. lastPoint is exclusive; pass -1 to use the
     * common size of profile.x/profile.z. Sphere/background segmentation must
     * be completed before calibration; this helper intentionally does not
     * guess which points belong to the sphere.
     */
    static SphereProfileObservation fromLaserProfile(
        const JointPose5D& pose,
        const LaserProfile& profile,
        int firstPoint = 0,
        int lastPoint = -1,
        double pointWeight = 1.0)
    {
        SphereProfileObservation observation;
        observation.profileId = static_cast<std::int64_t>(profile.index);
        observation.joints = toPortable(pose);

        const int commonSize = static_cast<int>(std::min(profile.x.size(), profile.z.size()));
        const int begin = std::max(0, firstPoint);
        const int end = (lastPoint < 0)
                            ? commonSize
                            : std::min(commonSize, std::max(begin, lastPoint));
        observation.points.reserve(static_cast<std::size_t>(std::max(0, end - begin)));
        for (int i = begin; i < end; ++i) {
            SensorProfilePoint point;
            point.x = profile.x[i];
            point.z = profile.z[i];
            point.weight = std::max(0.0, pointWeight);
            observation.points.push_back(point);
        }
        return observation;
    }

    static QMatrix4x4 sensorToGaugeMachineQMatrix(
        const JointPose5D& pose,
        const Gauge14Parameters& parameters)
    {
        const auto t = SingleSphere14Model::sensorToGaugeMachineTransform(
            toPortable(pose), parameters);
        QMatrix4x4 matrix;
        matrix.setRow(0, QVector4D(static_cast<float>(t[0]),
                                  static_cast<float>(t[1]),
                                  static_cast<float>(t[2]),
                                  static_cast<float>(t[3])));
        matrix.setRow(1, QVector4D(static_cast<float>(t[4]),
                                  static_cast<float>(t[5]),
                                  static_cast<float>(t[6]),
                                  static_cast<float>(t[7])));
        matrix.setRow(2, QVector4D(static_cast<float>(t[8]),
                                  static_cast<float>(t[9]),
                                  static_cast<float>(t[10]),
                                  static_cast<float>(t[11])));
        matrix.setRow(3, QVector4D(0.0f, 0.0f, 0.0f, 1.0f));
        return matrix;
    }
};

} // namespace calibration
} // namespace mls
