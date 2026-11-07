#include "machine_config.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

std::pair<std::string, std::string> canonicalPair(std::string a, std::string b) {
    if (b < a) std::swap(a, b);
    return {std::move(a), std::move(b)};
}

MachineConfig loadMachineConfig(const std::filesystem::path& jsonPath) {
    std::ifstream ifs(jsonPath);
    if (!ifs) {
        throw std::runtime_error("Cannot open machine config: " + jsonPath.string());
    }

    json root;
    ifs >> root;

    MachineConfig cfg;
    cfg.rootDir = jsonPath.parent_path();
    cfg.unit = root.value("unit", "");
    cfg.collisionProfile = root.value("collision_profile", "");
    cfg.kinematicsNote = root.value("kinematics_note", "");

    if (cfg.unit != "mm") {
        throw std::runtime_error("This project expects millimetres. config unit=" + cfg.unit);
    }

    if (!root.contains("links") || !root.at("links").is_array()) {
        throw std::runtime_error("machine_config.json must contain a 'links' array.");
    }

    for (const auto& node : root.at("links")) {
        LinkConfig link;
        link.name = node.at("name").get<std::string>();
        link.visualPath = cfg.rootDir / node.at("visual").get<std::string>();
        link.collisionPath = cfg.rootDir / node.at("collision").get<std::string>();
        cfg.links.emplace_back(std::move(link));
    }

    if (root.contains("ignored_collision_pairs")) {
        for (const auto& p : root.at("ignored_collision_pairs")) {
            if (!p.is_array() || p.size() != 2) continue;
            cfg.ignoredPairs.insert(canonicalPair(
                p.at(0).get<std::string>(),
                p.at(1).get<std::string>()));
        }
    }

    if (root.contains("offline_planning_limits") && root.at("offline_planning_limits").is_object()) {
        const auto& p = root.at("offline_planning_limits");
        cfg.planningLimits.source = p.value("source", std::string{});
        cfg.planningLimits.xMinMm = p.value("x_min_mm", cfg.planningLimits.xMinMm);
        cfg.planningLimits.xMaxMm = p.value("x_max_mm", cfg.planningLimits.xMaxMm);
        cfg.planningLimits.yMinMm = p.value("y_min_mm", cfg.planningLimits.yMinMm);
        cfg.planningLimits.yMaxMm = p.value("y_max_mm", cfg.planningLimits.yMaxMm);
        cfg.planningLimits.zMinMm = p.value("z_min_mm", cfg.planningLimits.zMinMm);
        cfg.planningLimits.zMaxMm = p.value("z_max_mm", cfg.planningLimits.zMaxMm);
        cfg.planningLimits.cMinDeg = p.value("c_min_deg", cfg.planningLimits.cMinDeg);
        cfg.planningLimits.cMaxDeg = p.value("c_max_deg", cfg.planningLimits.cMaxDeg);
        cfg.planningLimits.bMinDeg = p.value("b_min_deg", cfg.planningLimits.bMinDeg);
        cfg.planningLimits.bMaxDeg = p.value("b_max_deg", cfg.planningLimits.bMaxDeg);
        cfg.planningLimits.configured =
            cfg.planningLimits.xMinMm < cfg.planningLimits.xMaxMm &&
            cfg.planningLimits.yMinMm < cfg.planningLimits.yMaxMm &&
            cfg.planningLimits.zMinMm < cfg.planningLimits.zMaxMm &&
            cfg.planningLimits.cMinDeg < cfg.planningLimits.cMaxDeg &&
            cfg.planningLimits.bMinDeg < cfg.planningLimits.bMaxDeg;
        if (!cfg.planningLimits.configured) {
            throw std::runtime_error("offline_planning_limits exists but contains an invalid/empty joint range.");
        }
    }

    if (root.contains("safety") && root.at("safety").is_object()) {
        const auto& s = root.at("safety");
        cfg.safety.linearJointMarginMm = s.value("linear_joint_margin_mm", cfg.safety.linearJointMarginMm);
        cfg.safety.rotaryJointMarginDeg = s.value("rotary_joint_margin_deg", cfg.safety.rotaryJointMarginDeg);
        cfg.safety.minimumStaticClearanceMm = s.value("minimum_static_clearance_mm", cfg.safety.minimumStaticClearanceMm);
        cfg.safety.transitionLinearStepMm = s.value("transition_linear_step_mm", cfg.safety.transitionLinearStepMm);
        cfg.safety.transitionRotaryStepDeg = s.value("transition_rotary_step_deg", cfg.safety.transitionRotaryStepDeg);
        cfg.safety.transitionMaxSamples = s.value("transition_max_samples", cfg.safety.transitionMaxSamples);
        cfg.safety.arrivalLinearToleranceMm = s.value("arrival_linear_tolerance_mm", cfg.safety.arrivalLinearToleranceMm);
        cfg.safety.arrivalRotaryToleranceDeg = s.value("arrival_rotary_tolerance_deg", cfg.safety.arrivalRotaryToleranceDeg);
        cfg.safety.studySphereClearanceMm = s.value("study_sphere_clearance_mm", cfg.safety.studySphereClearanceMm);
        if (s.contains("study_sphere_collision_links") && s.at("study_sphere_collision_links").is_array()) {
            cfg.safety.studySphereCollisionLinks.clear();
            for (const auto& v : s.at("study_sphere_collision_links")) {
                if (v.is_string()) cfg.safety.studySphereCollisionLinks.push_back(v.get<std::string>());
            }
        }
    }

    return cfg;
}
