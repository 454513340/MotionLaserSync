#pragma once
#include "runtime/FiveAxis.h"
#include <optional>
#include <map>
namespace mls::runtime {
struct NcState {
    bool absolute=true;
    FeedMode feedMode=FeedMode::LegacyPerSecond;
    Plane plane=Plane::XY;
    int modalMotion=-1;
    bool rtcp=false;
};
struct NcBlock {
    enum class Kind { Empty, Modal, Motion, Dwell, SetWorkPosition, MCode } kind=Kind::Empty;
    int motion=-1, mcode=-1, lineNumber=0;
    bool absolute=true, capture=false, rtcp=false;
    FeedMode feedMode=FeedMode::LegacyPerSecond;
    Plane plane=Plane::XY;
    Joint words{};std::uint8_t axisMask=0;
    std::optional<double> feed;
    enum class ArcDefinition { None, Center, Radius, ThreePoint } arcDefinition=ArcDefinition::None;
    Arc arc; Joint mid{};
    int dwellMs=0;
    std::map<char,std::string> strings;
};
// Supported dialect: G0/1/2/3/4(P milliseconds)/5(custom points),17/18/19/21,
// 90/91/92(work offsets only),93/94,G43.4(software RTCP on),G49(RTCP off);
// M0/2/30/50/51/60..63; S0/S1 capture. Unknown codes/words are fatal.
bool parseNcBlock(const std::string& line,NcState& state,NcBlock& block,std::string& error);
} // namespace mls::runtime
