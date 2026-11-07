#include "runtime/Acquisition5D.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <cstdlib>

namespace {
void require(bool ok,const char* text){ if(!ok){std::cerr<<"FAIL: "<<text<<'\n';std::exit(1);} }
}
int main(){
    using T=mls::runtime::Table5D;
    require(T::runtimeCapabilityMagic==321055,"V32.13 exact runtime capability magic");
    require(T::runtimeCapabilityOffset==24 && T::machineHomeMaskOffset==27 &&
            T::controllerStateOffset==28 && T::frequencyRequestSequenceOffset==29 &&
            T::frequencyAckSequenceOffset==30 && T::frequencyConfigErrorOffset==31,
            "V32.13 preserves wire protocol table layout");
    float v[T::runtimeStatusWords]={321055,333,333,31,0,5,5,0};
    require(T::runtimeReady(v,333,T::IdleReady),"good PWM ACK + all Home bits");
    v[3]=30.5f;
    require(!T::runtimeReady(v,333,T::IdleReady),"fractional Home mask must reject");
    v[3]=32.0f;
    require(!T::runtimeReady(v,333,T::IdleReady),"out-of-range Home mask must reject");
    v[3]=31.0f;
    v[4]=1.25f;
    require(!T::runtimeReady(v,333,T::IdleReady),"fractional BASIC state must reject");
    v[4]=0.0f;
    v[5]=5.25f;
    require(!T::runtimeReady(v,333,T::IdleReady),"fractional request seq must reject");
    v[5]=5.0f;v[6]=5.5f;
    require(!T::runtimeReady(v,333,T::IdleReady),"fractional ACK seq must reject");
    v[6]=5.0f;v[7]=0.1f;
    require(!T::runtimeReady(v,333,T::IdleReady),"fractional config error must reject");
    v[7]=0.0f;v[2]=333.25f;
    require(!T::runtimeReady(v,333,T::IdleReady),"fractional applied Hz must reject");
    v[2]=333.0f;v[0]=321054;
    require(!T::runtimeReady(v,333,T::IdleReady),"V32.12 BASIC must not match V32.13 host");
    v[0]=321055;
    const float nan=std::numeric_limits<float>::quiet_NaN();
    const float inf=std::numeric_limits<float>::infinity();
    v[3]=nan; require(!T::runtimeReady(v,333,T::IdleReady),"NaN Home mask must reject");
    v[3]=31;v[4]=inf; require(!T::runtimeReady(v,333,T::IdleReady),"Inf BASIC state must reject");
    v[4]=0;v[5]=inf; require(!T::runtimeReady(v,333,T::IdleReady),"Inf sequence must reject");
    v[5]=5;v[2]=nan; require(!T::runtimeReady(v,333,T::IdleReady),"NaN applied Hz must reject");
    v[2]=333;
    require(T::runtimeReady(v,333,T::IdleReady),"normal state accepted after fault cases");
    std::cout<<"PASS V27.6.55 V32.13 runtime protocol integer/fault guards\n";
    return 0;
}
