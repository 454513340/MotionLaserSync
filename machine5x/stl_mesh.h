#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <vector>

struct Triangle3d {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;
};

std::vector<Triangle3d> loadStlTriangles(const std::filesystem::path& path);
