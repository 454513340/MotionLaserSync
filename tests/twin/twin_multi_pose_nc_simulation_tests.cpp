#include "twin/MultiPoseNcSimulation.h"
#include "twin/TwinM6NcPreview.h"
#include <cmath>
#include <iostream>
#include <string>
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<": "<<#x<<'\n';return 1;}}while(0)
using namespace mls::twin;
int main(){
    M6Report p;p.options.scanRateHz=200;
    p.path={
      {0.0,{{0,0,0,0,0}},{},M6SegmentRole::Transfer},
      {1.0,{{1,0,0,0,0}},{},M6SegmentRole::Transfer},
      {1.0,{{1,0,0,0,0}},{},M6SegmentRole::Scan},
      {1.02,{{1.2,0,0,2,1}},{},M6SegmentRole::Scan},
      {1.04,{{1.4,0,0,4,2}},{},M6SegmentRole::Scan},
      {1.04,{{1.4,0,0,4,2}},{},M6SegmentRole::Transfer},
      {2.04,{{0,0,0,0,0}},{},M6SegmentRole::Transfer}
    };
    for(int topology:{1,3}){
        M6NcPreviewOptions settings;settings.proposedTopologyId=topology;settings.h3R1Edge=115;
        // Provenance describes imported source content; it is NOT certification.
        settings.sourcePartPoseId="POSE_CANDIDATE_SHA256_"+std::string(64,'a');
        settings.calibrationId="CAL17_IMPORTED_SHA256_"+std::string(64,'b');
        settings.physical17Sha256=std::string(64,'b');
        auto nc=makeM6NcPreview(p,settings);
        CHECK(nc.ok());
        CHECK(nc.text.find("; PartPoseId: POSE_CANDIDATE_SHA256_")!=std::string::npos);
        CHECK(nc.text.find("; CalibrationId: CAL17_IMPORTED_SHA256_")!=std::string::npos);
        CHECK(nc.text.find("; Physical17Sha256: "+std::string(64,'b'))!=std::string::npos);
        CHECK(nc.text.find("; OfflineOnly: true")!=std::string::npos);
        CHECK(nc.text.find("; RegistrationVerified: false")!=std::string::npos);
        auto sim=simulateMultiPoseNc(nc.text);
        if(!sim.ok())std::cerr<<sim.error<<'\n';
        CHECK(sim.ok());
        CHECK(sim.topology==topology);
        CHECK(sim.scanGroups==1&&sim.saveGroups==1);
        CHECK(sim.vertices.size()==5);
        CHECK(sim.profiles.size()>=8&&sim.profiles.size()<=10);
        CHECK(std::abs(sim.durationSeconds-2.04)<1e-6);
        CHECK(verifyMultiPoseNcAgainstPlan(sim,p).empty());
        CHECK(!sampleMultiPoseNc(sim,std::nan("")).valid);
        auto s=sampleMultiPoseNc(sim,1.01);
        CHECK(s.valid&&s.scanning);
        CHECK(std::abs(s.joints[0]-1.1)<1.e-7);
        CHECK(std::abs(s.joints[3]-1.0)<1.e-7);
        CHECK(s.passedProfiles>=2);
        s=sampleMultiPoseNc(sim,2.0);
        CHECK(s.valid&&!s.scanning);
        CHECK(!sim.warning.empty()&&sim.initialG00Unverified);
        auto changed=nc.text;
        const auto pos=changed.find("X1.2 ");
        CHECK(pos!=std::string::npos);
        changed.replace(pos,4,"X1.3");
        const auto modified=simulateMultiPoseNc(changed);
        CHECK(modified.ok());
        CHECK(!verifyMultiPoseNcAgainstPlan(modified,p).empty());
        changed=nc.text;
        auto target=changed.find("X1.2 ");CHECK(target!=std::string::npos);
        auto f=changed.find("F3000",target);CHECK(f!=std::string::npos);
        changed.replace(f,5,"F1500");
        const auto drift=simulateMultiPoseNc(changed);
        CHECK(drift.ok());
        CHECK(!verifyMultiPoseNcAgainstPlan(drift,p).empty());
        changed=nc.text;
        auto off=changed.find("G05 S1 ");CHECK(off!=std::string::npos);
        changed.replace(off+4,2,"S0");
        CHECK(!simulateMultiPoseNc(changed).ok());
        changed=nc.text;
        auto m=changed.find("M50\n");CHECK(m!=std::string::npos);
        changed.erase(m,4);
        CHECK(!simulateMultiPoseNc(changed).ok());
        changed=nc.text;
        m=changed.find("M51\n");CHECK(m!=std::string::npos);
        changed.erase(m,4);
        CHECK(!simulateMultiPoseNc(changed).ok());
        changed=nc.text;
        m=changed.find("; OfflineOnly: true");CHECK(m!=std::string::npos);
        changed.replace(m,19,"; OfflineOnly: false");
        // Regression contract: V30 offline interpretation must NOT mistake a
        // user-editable comment for a signed machine execution permit.
        // Both valid booleans remain simulatable; physical authorization is
        // enforced separately by the runtime on-site verifier.
        const auto markedFalse=simulateMultiPoseNc(changed);
        CHECK(markedFalse.ok());
        CHECK(markedFalse.initialG00Unverified);
        changed.replace(m,20,"; OfflineOnly: MAYBE");
        CHECK(!simulateMultiPoseNc(changed).ok());
        changed=nc.text;
        m=changed.find("G93\n");CHECK(m!=std::string::npos);
        changed.replace(m,4,"G94\n");
        CHECK(!simulateMultiPoseNc(changed).ok());
        changed=nc.text;
        m=changed.find("M63\n");CHECK(m!=std::string::npos);
        changed.erase(m,4);
        CHECK(!simulateMultiPoseNc(changed).ok());
    }
    std::cout<<"NC text-driven XYZCB/G93/M50/M51 scan and tamper guard PASS\n";
}
