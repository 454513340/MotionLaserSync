#pragma once
#include "runtime/Acquisition5D.h"
#include <vector>
namespace mls::runtime {
struct ArchivePoint { std::uint32_t index=0; double x=0,z=0; };
struct ArchiveProfile {
    Frame5D frame;
    std::uint32_t profileIndex=0;
    std::int64_t timeUs=0;
    double sensorTimeSec=0;
    std::vector<ArchivePoint> points;
};
// V27.6.44 payload version 5: adds hardware-temporal correction provenance; decoder remains backward-compatible with v4.
constexpr std::size_t ArchiveMaxBytes = 512u*1024u*1024u;
std::uint32_t archiveCrc32(const void* data,std::size_t size);
bool encodeArchive(const std::vector<ArchiveProfile>& profiles,std::vector<std::uint8_t>& out,std::string& error);
bool decodeArchive(const std::uint8_t* bytes,std::size_t size,std::vector<ArchiveProfile>& out,std::string& error);
} // namespace mls::runtime
