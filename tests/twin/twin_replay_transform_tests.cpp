// Portable guard for M2 exposure-time sensor -> MCS transform.
#include "runtime/Acquisition5D.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
    mls::runtime::Frame5D frame;
    // identity rotation and an explicit machine-space translation
    frame.sensorToMachine={1,0,0,10, 0,1,0,-20, 0,0,1,30, 0,0,0,1};
    assert(mls::runtime::validTransform(frame.sensorToMachine));
    auto a=mls::runtime::mapProfilePoint(frame,2.5,96.0);
    assert(std::abs(a[0]-12.5)<1.e-12);
    assert(std::abs(a[1]+20.0)<1.e-12);
    assert(std::abs(a[2]-126.0)<1.e-12);
    // A rotation with Sensor-X aimed along +Y in MCS (90deg about machine Z).
    frame.sensorToMachine={0,-1,0,10, 1,0,0,-20, 0,0,1,30, 0,0,0,1};
    assert(mls::runtime::validTransform(frame.sensorToMachine));
    a=mls::runtime::mapProfilePoint(frame,2.5,96.0);
    assert(std::abs(a[0]-10.0)<1.e-12);
    assert(std::abs(a[1]-(-17.5))<1.e-12);
    assert(std::abs(a[2]-126.0)<1.e-12);
    std::cout<<"M2 stored Frame5D.sensorToMachine mapping: PASS\n";
}
