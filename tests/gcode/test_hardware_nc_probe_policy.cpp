#include "gcode/HardwareNcProbeLogic.h"
#include <cassert>
using namespace mls::gcode::probe;
int main(){
 Evidence e;
 e.stopCompleted=e.synchronized=e.publishedTriggerCountValid=true;
 e.sensorTriggerReadbackValid=e.sensorTriggerConfigMatches=true;
 e.poses=e.profiles=e.publishedTriggers=e.aligned=10;
 e.validProfiles=9;e.emptyProfiles=1;e.rawCallbacks=10;
 e.configuredHz=200;e.measuredSteadyIntervals=8;e.measuredMedianPeriodMs=5.0;
 assert(assess(e).sessionValid);
 e.profiles=0;assert(!assess(e).sessionValid);e.profiles=10;
 e.pending=1;assert(!assess(e).sessionValid);e.pending=0;
 e.alignmentFault=true;assert(!assess(e).sessionValid);e.alignmentFault=false;
 e.sensorTriggerConfigMatches=false;assert(!assess(e).sessionValid);e.sensorTriggerConfigMatches=true;
 e.publishedTriggerCountValid=false;assert(!assess(e).sessionValid);e.publishedTriggerCountValid=true;
 e.rawCallbacks=0;assert(!assess(e).sessionValid);e.rawCallbacks=10;
 e.stopCompleted=false;assert(!assess(e).sessionValid);e.stopCompleted=true;
 e.measuredMedianPeriodMs=10.;assert(!assess(e).sessionValid);e.measuredMedianPeriodMs=5.;
 e.measuredSteadyIntervals=0;assert(!assess(e).sessionValid);
}
