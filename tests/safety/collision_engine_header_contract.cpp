#include "../../machine5x/collision_engine.h"
#include <type_traits>
#include <string>

static_assert(CollisionEngine::kStepWorkpieceApiVersion == 2,
              "STEP/FCL CollisionEngine header is stale");
static_assert(std::is_same_v<decltype(&CollisionEngine::setWorkpiece),
    bool (CollisionEngine::*)(const mls::twin::TriangleMesh&,
                               const mls::twin::Transform&, std::string*)>,
    "CollisionEngine::setWorkpiece API not found");
static_assert(std::is_same_v<decltype(&CollisionEngine::clearWorkpiece),
    void (CollisionEngine::*)()>,
    "CollisionEngine::clearWorkpiece API not found");
static_assert(std::is_same_v<decltype(&CollisionEngine::minimumDistanceBelow),
    DistanceInfo (CollisionEngine::*)(double) const>,
    "CollisionEngine::minimumDistanceBelow API not found");
int main() { return 0; }
