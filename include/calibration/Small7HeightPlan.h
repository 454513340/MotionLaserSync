#pragma once
// Experiment 3: multi-measuring-field, fail-closed planning and timing policy.
// Optical coordinates below are the actual reported sensor X/Z coordinates, NOT machine Z.
#include "calibration/SingleSphere17Calibration.h"
#include <array>
#include <string>
#include <vector>

namespace mls::calibration::small7 {
struct OpticalWindow {
    bool verified = false;
    bool flipX = false;
    bool flipZ = true;
    int measuringFieldIndex = 7; // LLT supported 0..127, per-field verified ROI required
    double xMinMm = 0.0, xMaxMm = 0.0, zMinMm = 0.0, zMaxMm = 0.0;
    double marginMm = 0.15;
    std::string verificationId;  // traceable equipment/RAW verification record
};

struct Settings {
    int measuringFieldIndex = 7;
    double zBeginMm = 0.0, zEndMm = 0.0, zIntervalMm = 2.0;
    double positioningSpeedMmS = 3.0;
    // Scan speed is derived from the quantized sensor period and desired
    // adjacent-profile spacing, never entered as an arbitrary fixed feed.
    double scanSpeedMmS = 1.0;
    int profileRateHz = 200;
    double requestedLineSpacingMm = 0.005;
    double maximumFrameSpacingMm = 0.010;
    double knownSphereDiameterMm = 19.05;
    double scanMarginMm = 0.8;
    double sevenPointStepMm = 0.20;
};

struct ScanTiming {
    int period10us = 0;
    double effectiveProfileHz = 0;
    double scanSpeedMmS = 0;
    double nominalLineSpacingMm = 0;
};

// LLT exposure/idle use 10us integer units. This is the nominal period,
// NOT a guarantee of actual software-trigger callback spacing.
bool calculateScanTiming(int requestedHz, double desiredLineSpacingMm,
                         ScanTiming& out, std::string* error = nullptr);

// An ENTIRE 360-degree circumference is NOT required for diameter estimation:
// a wide enough contiguous observable segment can determine the circle.
// This is a conservative *geometric* precondition only; real scan + static
// diameter/arc-fitting gates are still required at every Z.
constexpr double kMinPlannedArcDeg = 50.0; // fitArc needs 40deg; leave margin
bool maxCircleFitArcInsideOptics(double centerX, double centerZ, double radiusMm,
                                 const OpticalWindow& window,
                                 double minimumArcDeg = kMinPlannedArcDeg);
// Optional additional diagnostic: can entire diameter/circumference fit?
bool fullCircleInsideOptics(double centerX, double centerZ, double radiusMm,
                            const OpticalWindow& window);

struct OpticalZInterval { double zStartMm = 0, zEndMm = 0; };
// Sampled optical-only Z assessment with bisected observed boundaries.
// MachineSafety, physical occlusion and measured diameters remain separate.
std::vector<OpticalZInterval> findOpticalZIntervals(
    const Settings&, const OpticalWindow&, const JointPose5d&,
    const Vec3d&, const Physical17Parameters&,
    double samplingStepMm = 0.1);

struct ScanPeakSample { double machineX = 0, radiusMm = 0; };
struct ScanPeak { double machineX = 0, fittedRadiusMm = 0; int usedSamples = 0; };
// Uses measured X-labelled scan profiles, NOT the maximum radius alone or
// equally spaced profile indices; a static multi-point refinement follows.
bool findScannedMaximumCircle(const std::vector<ScanPeakSample>& samples,
                              double scanStartX, double scanEndX,
                              double predictedPeakX, double sphereRadiusMm,
                              double diameterToleranceMm, ScanPeak& out,
                              std::string* error = nullptr);

struct HeightPlan {
    double machineZMm = 0.0;
    double centerXMachineMm = 0.0;
    double scanStartXMachineMm = 0.0;
    double scanEndXMachineMm = 0.0;
    double predictedSensorCenterX = 0.0;
    double predictedSensorCenterZ = 0.0;
    double visibleArcSpanDeg = 0.0;
    std::array<double,7> sevenPointX{};
};

// Conservative frequency limit from the project Quick Reference, not a live device capability.
int referenceMaxProfileHz(int fieldIndex);
bool validateSettings(const Settings&, const OpticalWindow&, std::string* error = nullptr);
// Returns the longest contiguous theoretical circle arc within a verified reported-X/Z ROI.
double visibleArcSpanDeg(double centerX, double centerZ, double radiusMm,
                         const OpticalWindow&);
bool planHeights(const Settings&, const OpticalWindow&,
                 const JointPose5d& currentMachineMpos,
                 const Vec3d& sphereCenterMachineMm,
                 const Physical17Parameters&, std::vector<HeightPlan>&,
                 std::string* error = nullptr);

// Per-move budget is based on length/speed/acceleration, never the fixed 10-minute
// timeout for every small step. This is an upper bound; an operator stop always wins.
int motionTimeoutMs(double distanceMm, double speedMmS, double accelMmS2,
                    double decelMmS2, int hardMaxMs = 600000);
}
