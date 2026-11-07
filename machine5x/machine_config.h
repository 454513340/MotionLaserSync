#pragma once

#include <filesystem>
#include <set>
#include <string>
#include <utility>
#include <vector>

struct LinkConfig {
    std::string name;
    std::filesystem::path visualPath;
    std::filesystem::path collisionPath;
};

struct MachinePlanningLimits {
    // Offline path planning must never fall back to AxisParams' placeholder
    // +/-999999 limits.  These bounds are the machine-coordinate travel limits
    // used by the field controller/BASIC and are stored with the digital twin so
    // disconnected planning has a finite, explicit search domain.
    bool configured = false;
    std::string source;
    double xMinMm = 0.0;
    double xMaxMm = 0.0;
    double yMinMm = 0.0;
    double yMaxMm = 0.0;
    double zMinMm = 0.0;
    double zMaxMm = 0.0;
    double cMinDeg = 0.0;
    double cMaxDeg = 0.0;
    double bMinDeg = 0.0;
    double bMaxDeg = 0.0;
};

struct MachineSafetyConfig {
    // Planner safety margins are deliberately separate from controller limits.
    double linearJointMarginMm = 3.0;
    double rotaryJointMarginDeg = 3.0;

    // Minimum non-neighbour mesh clearance checked at static study poses.
    // Continuous transitions are collision-sampled because full FCL distance
    // on every interpolation sample is substantially more expensive.
    double minimumStaticClearanceMm = 1.0;

    // Joint-space swept-path sampling resolution.
    double transitionLinearStepMm = 0.5;
    double transitionRotaryStepDeg = 0.5;
    int transitionMaxSamples = 4000;

    // Autonomous sphere-study arrival gate.
    double arrivalLinearToleranceMm = 0.05;
    double arrivalRotaryToleranceDeg = 0.05;

    // The RANSAC sphere is a dynamic fixed obstacle in MCS for the study.  It
    // is expanded by this amount and tested only against the listed moving links.
    double studySphereClearanceMm = 2.0;
    std::vector<std::string> studySphereCollisionLinks{
        "02_X_AXIS", "03_Y_AXIS", "04_Z_AXIS",
        "05_C_AXIS", "06_B_AXIS", "07_END_SENSOR"
    };
};

struct MachineConfig {
    std::string unit;
    std::string collisionProfile;
    std::string kinematicsNote;
    std::filesystem::path rootDir;
    std::vector<LinkConfig> links;
    std::set<std::pair<std::string, std::string>> ignoredPairs;
    MachinePlanningLimits planningLimits;
    MachineSafetyConfig safety;
};

MachineConfig loadMachineConfig(const std::filesystem::path& jsonPath);
std::pair<std::string, std::string> canonicalPair(std::string a, std::string b);
