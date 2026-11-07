#pragma once
#include "twin/TwinScene.h"
#include "twin/TwinTrajectory.h"
namespace mls::twin {
// M0/M1 session stays offline. Simulated profiles, NBV and fusion are later stages.
struct TwinSession {
    std::uint32_t schemaVersion=1;
    TwinScene scene;
    TwinTrajectory trajectory;
    bool readyForGeometry() const {return scene.hasWorkpiece();}
};
}
