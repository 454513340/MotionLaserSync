#pragma once
// NC header semantics shared by the Qt parser, diagnosis and start paths.
// SyncMode::Hardware means hardware pose/profile synchronization. It does NOT
// imply that the sensor itself uses an external trigger in H3.
#include "gcode/GCodeTypes.h"

namespace mls::gcode {
inline QString sensorTriggerModeForTopology(int topology) {
    if (topology == 1) return QStringLiteral("HardwareExternal");
    if (topology == 3) return QStringLiteral("SoftwareInternal");
    return {};
}

inline bool validateSensorTriggerMetadata(const GCodeProgram& program,
                                          QString* error = nullptr) {
    const auto expected = sensorTriggerModeForTopology(program.hardwareTopologyId);
    if (expected.isEmpty()) {
        if (error) *error = QStringLiteral("HardwareTopology must be H1 or H3");
        return false;
    }
    if (!program.sensorTriggerMode.isEmpty() &&
        program.sensorTriggerMode != expected) {
        if (error) *error = program.hardwareTopologyId == 1
            ? QStringLiteral("H1 uses sensor HardwareExternal and OP14 EdgeRising; SensorTriggerMode contradicts H1")
            : QStringLiteral("H3 uses sensor SoftwareInternal and CMM/R1 latch; SensorTriggerMode contradicts H3");
        return false;
    }
    // Legacy HardwareTriggerType only describes the external scanCONTROL
    // edge in H1. Older H3 files may contain EdgeRising erroneously; it is
    // deprecated metadata and does not select external sensor triggering.
    if (!program.hardwareTriggerType.isEmpty() &&
        program.hardwareTriggerType != QStringLiteral("EdgeRising")) {
        if (error) *error = QStringLiteral("Legacy HardwareTriggerType only supports EdgeRising");
        return false;
    }
    return true;
}
} // namespace mls::gcode
