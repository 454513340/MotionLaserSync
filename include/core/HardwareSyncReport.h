#pragma once
#include "core/ProfileTimingAnalysis.h"
#include "core/EciIoScheduler.h"
#include "runtime/Acquisition5D.h"
#include <string>
#include <vector>

namespace syncdiag {
struct HardwareReportInput {
    int topologyId=0; // H1=1, H3=3; 0=unknown, never label unknown as H1
    int cmmR1Edge=0;
    bool running = false, fault = false, synchronized = false, triggerCountValid = false;
    std::string faultReason;
    std::int64_t poses = 0, received = 0, triggers = 0, aligned = 0, pending = 0, dropped = 0;
    std::int64_t validAligned = 0, emptyAligned = 0, temporalBoundarySkipped = 0, missing = 0, duplicates = 0;
    bool temporalCorrectionEnabled = false;
    double temporalDelayUs = 0.0, temporalDelayStdUs = 0.0;
    std::string temporalCalibrationId;
    std::int64_t backward = 0, invalidIndices = 0, readErrors = 0, overflows = 0;
    std::uint64_t cloudPoints = 0;
    std::uint64_t sensorRawCallbacks = 0, sensorConvertedProfiles = 0, sensorConvertFailures = 0;
    std::uint64_t sensorIngressDrops = 0, sensorRawRingDepth = 0, sensorRawRingHighWater = 0, sensorRawRingCapacity = 0;
    std::uint64_t sensorLastCallbackOrdinal = 0;
    std::int64_t sensorMaxCallbackServiceUs = 0, sensorMaxDecoderQueueUs = 0, sensorMaxDecodeServiceUs = 0;
    int sensorLastConvertResult = 0;
    std::uint32_t sensorLastCallbackBytes = 0, sensorLastResolution = 0;
    bool sensorTriggerReadbackValid = false;
    bool sensorTriggerConfigMatches = false;
    bool sensorCmmPortReadbackValid = false;
    std::uint32_t sensorTriggerRequested = 0, sensorTriggerReadback = 0;
    std::uint32_t sensorDigitalIoRequested = 0, sensorDigitalIoReadback = 0;
    double periodMs = 1000.0 / static_cast<double>(mls::runtime::Table5D::defaultTriggerFrequencyHz), exposureMs = 0, idleMs = 0;
    std::vector<std::int64_t> poseEstimateUs;
    std::vector<ProfileRecord> frames;
    eci::EciIoScheduler::Stats io;
};
// Pure, repeatable formatter. No Qt, no controller I/O, no point-cloud-to-frame inference.
std::string hardwareSyncReport(const HardwareReportInput& input);
std::string schedulerReport(const eci::EciIoScheduler::Stats& stats);
} // namespace syncdiag
