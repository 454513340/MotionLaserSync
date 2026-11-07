#pragma once
// M3/M4 V2: portable inference for the RESEARCH conditional-return model.
// Models probability only after geometrical first-hit laser illumination,
// receiver visibility, and valid FOV are all satisfied. Never a physical
// radiometric model, nor approval for motion control/M6 use.
#include "twin/VirtualLaser2950.h"
#include <array>
#include <cstddef>
#include <string>
namespace mls::twin {

struct M34ReturnParams {
    // [intercept, laser grazing, receiver grazing, Z edge, X edge,
    //  transition vicinity, projected-hole penalty]
    std::array<double,7> weights{{4.,-3.,-3.,-.25,-.2,-.7,-1.}};
    Vec3 equivalentReceiverSensor{0,90,251.25};
    double fanXScale=1.0,fanXOffsetMm=0.0;
    bool independentPhysicalCalibration=false; // ALWAYS false until new experiments
    bool readyForM6=false;                      // NEVER inferred from a fit score
    bool validate(std::string* error=nullptr) const;
};

class TwinM34ReturnModel final {
public:
    TwinM34ReturnModel(const M34ReturnParams& parameters,
                       const Sensor2950Geometry& scannerGeometry)
        :parameters_(parameters),geometry_(scannerGeometry){}
    const M34ReturnParams& parameters() const {return parameters_;}
    // Caller supplies 'projectedHoleRoi' from the same indexed CAD / pose
    // used during calibration. Do not infer it from unknown physical return.
    double probability(const VirtualProfile& profile,std::size_t index,bool projectedHoleRoi) const;
    static bool readResearchParameters(const std::string& file,M34ReturnParams& out,std::string* error=nullptr);
private:
    M34ReturnParams parameters_;
    Sensor2950Geometry geometry_;
};
} // namespace mls::twin
