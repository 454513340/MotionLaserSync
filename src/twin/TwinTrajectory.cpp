#include "twin/TwinTrajectory.h"
#include <algorithm>
#include <cmath>
#include <utility>
namespace mls::twin {
bool TwinTrajectory::setSamples(std::vector<PoseSample> input,std::string* error){
    if(input.empty()){if(error)*error="Empty trajectory";return false;}
    for(std::size_t i=0;i<input.size();++i){
        if(!std::isfinite(input[i].tSec)||(i>0&&input[i].tSec<=input[i-1].tSec)){
            if(error) *error="Trajectory times must increase strictly";
            return false;
        }
        for(const auto q:input[i].q)if(!std::isfinite(q)){
            if(error) *error="Trajectory has non-finite axes";
            return false;
        }
    }
    samples_=std::move(input);return true;
}
bool TwinTrajectory::interpolate(double t, Joint5* out) const {
    if(!out||samples_.empty()||!std::isfinite(t))return false;
    if(t<=samples_.front().tSec){*out=samples_.front().q;return true;}
    if(t>=samples_.back().tSec){*out=samples_.back().q;return true;}
    auto it=std::lower_bound(samples_.begin(),samples_.end(),t,[](const PoseSample& p,double x){return p.tSec<x;});
    const auto& b=*it;const auto& a=*(it-1);
    const double alpha=(t-a.tSec)/(b.tSec-a.tSec);
    for(std::size_t i=0;i<5;++i)(*out)[i]=a.q[i]+alpha*(b.q[i]-a.q[i]);
    return true;
}
}
