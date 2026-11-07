#pragma once
#include "twin/TwinTypes.h"
#include <array>
#include <vector>
namespace mls::twin {
using Joint5=std::array<double,5>; // X/Y/Z/C/B, angles deg
struct PoseSample { double tSec=0; Joint5 q{}; };
class TwinTrajectory {
public:
    bool setSamples(std::vector<PoseSample> samples, std::string* error=nullptr);
    const std::vector<PoseSample>& samples() const {return samples_;}
    bool interpolate(double t, Joint5* out) const;
private:
    std::vector<PoseSample> samples_;
};
}
