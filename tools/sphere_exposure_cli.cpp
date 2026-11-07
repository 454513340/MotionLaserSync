#include "calibration/SphereExposureSweep.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <stdexcept>

using namespace mls::calibration;
static void require(bool v,const char* msg){if(!v)throw std::runtime_error(msg);}
static std::vector<SphereExposurePoint> frameArc(double cx,double cz,double r){
    std::vector<SphereExposurePoint> v;
    for(int i=0;i<250;++i){
        double a=(30.0+i*(140.0/249.0))*3.14159265358979323846/180.;
        // Sub-10-micron bounded deterministic perturbations and no background bias.
        double n=0.003*std::sin(i*1.723);
        v.push_back({cx+(r+n)*std::cos(a),cz+(r+n)*std::sin(a),i});
    }
    return v;
}
int main(){
    const auto arc=SphereExposureSweep::fitArc(frameArc(.31,90.2,9.525),.25,90.3,9.525);
    require(arc.valid,"true sphere arc should be valid");
    require(std::abs(arc.radius-9.525)<.02,"circle radius bias");
    require(arc.arcSpanDeg>90 && arc.rms<.010,"circle arc/RMS");
    require(!SphereExposureSweep::fitArc({},.25,90.3,9.525).valid,"empty arc must fail");
    std::vector<SphereExposurePass> passes;
    for(int exp : {60,150,240}){
        for(int repeat=0;repeat<2;++repeat){
            std::vector<SphereExposureFrame> frames;
            for(int i=0;i<180;++i){
                double h=(i-89.5)*.09;
                double R=9.525+(exp==150?0.0005:(exp==60?-0.025:0.025));
                double r=std::sqrt(std::max(0.0,R*R-h*h));
                auto fit=SphereExposureSweep::fitArc(frameArc(.31,90.2,r),.31,90.2,9.525);
                frames.push_back({static_cast<unsigned>(i),i*10000.0,fit});
            }
            auto p=SphereExposureSweep::summarize(frames,exp,100,repeat+1,repeat==0?1:-1,1.0);
            require(p.complete,"synthetic entire sphere pass must be eligible");
            passes.push_back(p);
        }
    }
    auto rank=SphereExposureSweep::rank(passes);
    require(rank.size()==3 && rank.front().eligible,"valid repetition ranking");
    require(rank.front().exposure10us==150,"best exposure should be chosen by diameter and arc stability");
    // Fail closed on sparse or overspaced profiles.
    std::vector<SphereExposureFrame> tiny(3);
    require(!SphereExposureSweep::summarize(tiny,150,25,1,1,5).complete,"undersampled data must fail");
    std::puts("sphere exposure portable tests PASS: circle, arc quality, peak radius, ranking, fail-closed");
    return 0;
}
