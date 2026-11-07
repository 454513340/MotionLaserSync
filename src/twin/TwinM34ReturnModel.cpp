#include "twin/TwinM34ReturnModel.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
namespace mls::twin {
namespace {
bool finite(const Vec3& v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
double clampRisk(double x,double upper){return std::clamp(std::isfinite(x)?x:0.,0.,upper);}
double sig(double x){if(x>=0){const double z=std::exp(-x);return 1/(1+z);}const double z=std::exp(x);return z/(1+z);}
}
bool M34ReturnParams::validate(std::string* error) const{
    const auto fail=[&](const char* s){if(error)*error=s;return false;};
    if(!finite(equivalentReceiverSensor))return fail("Nonfinite equivalent receiver");
    if(!std::isfinite(fanXScale)||!std::isfinite(fanXOffsetMm)||
        fanXScale<0.8||fanXScale>1.2||std::abs(fanXOffsetMm)>5.)return fail("Invalid fan index correction");
    if(independentPhysicalCalibration||readyForM6)return fail("Research fit has no physical certification");
    for(std::size_t i=0;i<weights.size();++i){
        if(!std::isfinite(weights[i])||std::abs(weights[i])>32)return fail("Bad coefficient");
        if(i>0&&weights[i]>0)return fail("Grazing/risk coefficients must not reward risk");
    }
    return true;
}
double TwinM34ReturnModel::probability(const VirtualProfile& profile,std::size_t i,bool projectedHoleRoi) const {
    if(!parameters_.validate()||!geometry_.validate()||i>=profile.samples.size())return 0.;
    if(std::abs(geometry_.sampleXScale-parameters_.fanXScale)>1e-6||
       std::abs(geometry_.sampleXOffsetMm-parameters_.fanXOffsetMm)>1e-6||
       length(geometry_.cameraOriginSensor-parameters_.equivalentReceiverSensor)>1e-5)return 0.;
    const auto& s=profile.samples[i];
    if(s.state!=VirtualSampleState::Valid||s.hitObject!=VirtualHitObject::Workpiece)return 0.;
    double edgeRisk=0.,depthRisk=0.;
    if(std::isfinite(s.rayHitSensorZ)){
        const double z=s.rayHitSensorZ;
        depthRisk=clampRisk(std::abs(z-geometry_.zCenter)/(geometry_.zFar-geometry_.zCenter),3.);
        if(std::isfinite(s.rayHitSensorX))edgeRisk=clampRisk(std::abs(s.rayHitSensorX)/(.5*geometry_.widthAtZ(z)),3.);
    }
    // Matches offline Python feature extraction EXACTLY (7-index neighbourhood).
    int nearest=99;
    for(int j=std::max(0,int(i)-7);j<=std::min(int(profile.samples.size())-1,int(i)+7);++j)
        if(profile.samples[std::size_t(j)].state!=VirtualSampleState::Valid)
            nearest=std::min(nearest,std::abs(int(i)-j));
    const double boundaryRisk=nearest<=7?std::exp(-nearest/2.):0.;
    const std::array<double,7> feature{{1.,
        1.-clampRisk(s.laserNormalCos,1.),1.-clampRisk(s.receiverNormalCos,1.),
        depthRisk,edgeRisk,boundaryRisk,projectedHoleRoi?1.:0.}};
    double logit=0.;for(std::size_t k=0;k<feature.size();++k)logit+=feature[k]*parameters_.weights[k];
    return sig(logit);
}
bool TwinM34ReturnModel::readResearchParameters(const std::string& file,M34ReturnParams& out,std::string* error){
    const auto fail=[&](const char* s){if(error)*error=s;return false;};
    std::ifstream in(file);std::string header;
    if(!(in>>header)||header!="M34P2")return fail("Not a V2 parameter file");
    M34ReturnParams candidate;
    if(!(in>>candidate.equivalentReceiverSensor.x>>candidate.equivalentReceiverSensor.y
           >>candidate.equivalentReceiverSensor.z))return fail("Missing equivalent receiver");
    for(auto& v:candidate.weights)if(!(in>>v))return fail("Missing coefficient");
    if(!(in>>candidate.fanXScale>>candidate.fanXOffsetMm))return fail("Missing fan projection calibration");
    std::string tail;if(in>>tail)return fail("Unexpected extra parameters");
    if(!candidate.validate(error))return false;
    out=candidate;return true;
}
} // namespace mls::twin
