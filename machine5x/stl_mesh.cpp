#include "stl_mesh.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

float readFloatLE(std::istream& is) {
    std::array<unsigned char, 4> b{};
    is.read(reinterpret_cast<char*>(b.data()), 4);
    if (!is) throw std::runtime_error("Unexpected end of binary STL.");

    std::uint32_t u =
        (static_cast<std::uint32_t>(b[0])      ) |
        (static_cast<std::uint32_t>(b[1]) <<  8) |
        (static_cast<std::uint32_t>(b[2]) << 16) |
        (static_cast<std::uint32_t>(b[3]) << 24);

    float f = 0.0f;
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

std::uint32_t readU32LE(std::istream& is) {
    std::array<unsigned char, 4> b{};
    is.read(reinterpret_cast<char*>(b.data()), 4);
    if (!is) throw std::runtime_error("Unexpected end of binary STL header.");
    return
        (static_cast<std::uint32_t>(b[0])      ) |
        (static_cast<std::uint32_t>(b[1]) <<  8) |
        (static_cast<std::uint32_t>(b[2]) << 16) |
        (static_cast<std::uint32_t>(b[3]) << 24);
}

std::uint64_t fileSize(const std::filesystem::path& p) {
    std::error_code ec;
    const auto s = std::filesystem::file_size(p, ec);
    if (ec) throw std::runtime_error("Cannot stat STL: " + p.string());
    return static_cast<std::uint64_t>(s);
}

bool looksBinaryStl(const std::filesystem::path& path) {
    std::ifstream is(path, std::ios::binary);
    if (!is) throw std::runtime_error("Cannot open STL: " + path.string());
    char header[80];
    is.read(header, 80);
    if (!is) return false;
    const std::uint32_t count = readU32LE(is);
    const std::uint64_t expected = 84ull + static_cast<std::uint64_t>(count) * 50ull;
    return expected == fileSize(path);
}

std::vector<Triangle3d> loadBinary(const std::filesystem::path& path) {
    std::ifstream is(path, std::ios::binary);
    if (!is) throw std::runtime_error("Cannot open binary STL: " + path.string());

    char header[80];
    is.read(header, 80);
    const std::uint32_t count = readU32LE(is);

    std::vector<Triangle3d> tris;
    tris.reserve(count);

    for (std::uint32_t i = 0; i < count; ++i) {
        // normal (ignored)
        (void)readFloatLE(is); (void)readFloatLE(is); (void)readFloatLE(is);

        auto readVertex = [&]() {
            const double x = readFloatLE(is);
            const double y = readFloatLE(is);
            const double z = readFloatLE(is);
            return Eigen::Vector3d(x, y, z);
        };

        Triangle3d t;
        t.a = readVertex();
        t.b = readVertex();
        t.c = readVertex();

        char attr[2];
        is.read(attr, 2);
        if (!is) throw std::runtime_error("Truncated binary STL: " + path.string());
        tris.emplace_back(std::move(t));
    }
    return tris;
}

std::vector<Triangle3d> loadAscii(const std::filesystem::path& path) {
    std::ifstream is(path);
    if (!is) throw std::runtime_error("Cannot open ASCII STL: " + path.string());

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Triangle3d> tris;
    std::string word;

    while (is >> word) {
        if (word != "vertex") continue;
        double x = 0.0, y = 0.0, z = 0.0;
        is >> x >> y >> z;
        if (!is) throw std::runtime_error("Malformed ASCII STL: " + path.string());
        vertices.emplace_back(x, y, z);
        if (vertices.size() == 3) {
            tris.push_back({vertices[0], vertices[1], vertices[2]});
            vertices.clear();
        }
    }

    if (tris.empty()) {
        throw std::runtime_error("No triangles found in STL: " + path.string());
    }
    return tris;
}

} // namespace

std::vector<Triangle3d> loadStlTriangles(const std::filesystem::path& path) {
    return looksBinaryStl(path) ? loadBinary(path) : loadAscii(path);
}
