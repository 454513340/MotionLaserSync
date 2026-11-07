#pragma once
#include "twin/TwinTypes.h"
#include <string>
namespace mls::twin {
struct CadImportOptions {
    double linearDeflectionMm=0.12;
    double angularDeflectionRad=0.25;
};
struct CadImportResult {
    bool ok=false;
    TriangleMesh mesh;
    std::size_t brepFaces=0;
    std::string message;
};
class CadGeometryAdapter {
public:
    static bool stepSupported();
    // STEP is a true B-Rep import via Open CASCADE, NOT a text/mesh filename shim.
    // The B-Rep is used for tessellation in M1; persistent analytic faces are scheduled for M5.
    static CadImportResult loadStep(const std::string& fileName,
                                   const CadImportOptions& options={});
};
} // namespace mls::twin
