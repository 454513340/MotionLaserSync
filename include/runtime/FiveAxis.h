#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace mls::runtime {
// Every array is X,Y,Z,C,B. Units: mm,mm,mm,deg,deg. Never B,C.
using Joint = std::array<double, 5>;
enum class FeedMode { LegacyPerSecond, PerMinute, InverseMinute };
struct Word { char letter = 0; double number = 0; bool text = false; std::string string; };
// Strict lexer, also used by the Qt parser. Invalid/trailing words are errors.
bool lex(const std::string& line, std::vector<Word>& words, std::string& error);
bool resolve(const Joint& current, const Joint& words, std::uint8_t mask,
             bool absolute, Joint& target, std::string& error);
struct Limits {
    Joint speed{5,5,5,2,2}, accel{25,25,25,20,20}, decel{25,25,25,20,20};
    Joint minimum{-150,-119,-35,-90,-90}, maximum{117,137,56,90,90};
};
struct Waypoint { Joint q{}; double feed = 5; FeedMode mode = FeedMode::LegacyPerSecond; bool continuous=false; };
struct Segment {
    Joint target{}, direction{};
    double length=0, speed=0, entry=0, exit=0, seconds=0;
};
struct Plan { std::vector<Segment> segments; double accel=0, decel=0, seconds=0; };
// Normal paths exact-stop at non-collinear joint corners.  Waypoints explicitly
// marked continuous permit bounded small-angle junction speed for dense RTCP polylines.
// This remains joint-space acceleration planning; TCP compensation is done upstream.
bool plan(const Joint& start, const std::vector<Waypoint>& path, const Limits& limits,
          double vectorAccelCap, double vectorDecelCap, Plan& result, std::string& error);
// Reconstruct an acceleration-aware G93 timeline for a continuous G05 group.
// Input contains the original XYZCB points and requested F=60/time. Returned
// times are attainable by the SAME inverse-minute SP planner and never shorter
// than the input intervals. Non-collinear corners remain exact-stop.
// speedReserve is fractional headroom below the configured per-axis maximum;
// this does NOT increase mechanical limits or grant motion permission.
bool retimeG93Polyline(const Joint& start,
                       const std::vector<Waypoint>& original,
                       const Limits& limits,double vectorAccelCap,
                       double vectorDecelCap,std::vector<double>& seconds,
                       std::string& error,double speedReserve=0.95);
enum class Plane { XY, ZX, YZ }; // G17,G18,G19 right-handed plane basis
struct Arc {
    Plane plane=Plane::XY; bool clockwise=false;
    bool useRadius=false; double radius=0;
    Joint centerOffset{}; // I,J,K only; incremental even under G90
    double chordToleranceMm=0.01, rotaryStepDeg=0.25;
};
bool arcWaypoints(const Joint& start, const Joint& end, const Arc& arc,
                  std::vector<Joint>& points, std::string& error);
bool threePointArc(const Joint& start, const Joint& mid, const Joint& end,
                   bool clockwise, Arc& arc, std::string& error);
} // namespace mls::runtime
