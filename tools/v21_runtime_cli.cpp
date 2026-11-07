// Offline verification only. This program never opens a controller or sensor.
#include "runtime/ScanArchive5D.h"
#include "runtime/GCode5D.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace mls::runtime;
namespace {
using V3=std::array<double,3>;
void checked(bool ok,const std::string& error){if(!ok)throw std::runtime_error(error);}
double distance(const V3&a,const V3&b){return std::hypot(std::hypot(a[0]-b[0],a[1]-b[1]),a[2]-b[2]);}
V3 transformPoint(const std::array<double,16>&t,const V3&p){return {t[0]*p[0]+t[1]*p[1]+t[2]*p[2]+t[3],t[4]*p[0]+t[5]*p[1]+t[6]*p[2]+t[7],t[8]*p[0]+t[9]*p[1]+t[10]*p[2]+t[11]};}
V3 inverse(const std::array<double,16>&t,const V3&p){V3 v{p[0]-t[3],p[1]-t[7],p[2]-t[11]};return {t[0]*v[0]+t[4]*v[1]+t[8]*v[2],t[1]*v[0]+t[5]*v[1]+t[9]*v[2],t[2]*v[0]+t[6]*v[1]+t[10]*v[2]};}
V3 solve3(double a[3][4]) {
    for(int k=0;k<3;++k){int p=k;for(int j=k+1;j<3;++j)if(std::abs(a[j][k])>std::abs(a[p][k]))p=j;
        if(std::abs(a[p][k])<1e-12)throw std::runtime_error("singular translation basis");
        for(int j=k;j<4;++j)std::swap(a[k][j],a[p][j]);double d=a[k][k];for(int j=k;j<4;++j)a[k][j]/=d;
        for(int i=0;i<3;++i)if(i!=k){double f=a[i][k];for(int j=k;j<4;++j)a[i][j]-=f*a[k][j];}}
    return {a[0][3],a[1][3],a[2][3]};
}
void binaryWrite(const std::filesystem::path&p,const std::vector<std::uint8_t>&b){std::ofstream f(p,std::ios::binary);if(!f)throw std::runtime_error("cannot create "+p.string());f.write(reinterpret_cast<const char*>(b.data()),std::streamsize(b.size()));if(!f)throw std::runtime_error("write failed");}
std::vector<std::uint8_t> binaryRead(const std::filesystem::path&p){std::ifstream f(p,std::ios::binary|std::ios::ate);if(!f)throw std::runtime_error("cannot read file");auto n=f.tellg();if(n<0||std::uint64_t(n)>ArchiveMaxBytes)throw std::runtime_error("invalid file size");std::vector<std::uint8_t>b(static_cast<std::size_t>(n));f.seekg(0);f.read(reinterpret_cast<char*>(b.data()),std::streamsize(b.size()));if(!f)throw std::runtime_error("short read");return b;}
int demo(const std::filesystem::path&dir) {
    std::filesystem::create_directories(dir);
    std::array<double,17> theta{.0005,-.0003,.0004,.15,-.12,.0007,-.0005,.18,-.11,.0006,-.0004,.015,-.01,.02,2,-3,5};
    const V3 center{0,0,80};constexpr double radius=12;
    const std::array<double,2> offsets{10,-20};
    std::vector<ArchiveProfile> profiles;std::vector<V3> cloud;
    double inverseError=0,sphereError=0,replayError=0,legacyError=0;std::string error;
    std::ofstream csv(dir/"profiles.csv"),joints(dir/"joints.csv");csv<<std::setprecision(17);joints<<std::setprecision(17);
    csv<<"session,ordinal,profile_index,point_index,sensor_x_mm,sensor_z_mm,world_x_mm,world_y_mm,world_z_mm\n";
    joints<<"ordinal,X_mm,Y_mm,Z_mm,C_deg,B_deg,C_raw_deg,B_raw_deg,timing,absolute_gauge_known\n";
    for(int n=0;n<64;++n){
        Frame5D truth;truth.joints[3]=45*std::sin(n*.23);truth.joints[4]=20*std::cos(n*.31);
        checked(completeFrame(truth,theta,error),error);
        const V3 centerSensor{0,4*std::sin(n*.37),80};const auto initial=transformPoint(truth.sensorToMachine,centerSensor);
        double system[3][4]{};
        for(int k=0;k<3;++k){Frame5D unit=truth;unit.joints[k]=1;checked(completeFrame(unit,theta,error),error);auto moved=transformPoint(unit.sensorToMachine,centerSensor);for(int j=0;j<3;++j)system[j][k]=moved[j]-initial[j];}
        for(int j=0;j<3;++j)system[j][3]=center[j]-initial[j];auto translation=solve3(system);
        for(int j=0;j<3;++j)truth.joints[j]=translation[j];checked(completeFrame(truth,theta,error),error);
        // Controller TABLE is binary32; include that quantization instead of pretending doubles are transmitted.
        float record[Table5D::dimension]={float(n),float(truth.joints[0]),float(truth.joints[1]),float(truth.joints[2]),float(truth.joints[3]-offsets[0]),float(truth.joints[4]-offsets[1])};
        ArchiveProfile profile;checked(decodeRecord(record,n,22,offsets,profile.frame,error),error);
        profile.frame.timing=PoseTiming::Simulated;profile.profileIndex=1000+n;profile.timeUs=1000000+5000*n;profile.sensorTimeSec=1+.005*n;
        profile.frame.hostEstimateUs=profile.timeUs+1500;checked(completeFrame(profile.frame,theta,error),error);
        const auto&t=truth.sensorToMachine;const V3 direction{t[2],t[6],t[10]};
        for(int k=0;k<61;++k){const double sx=-14+28.0*k/60;const V3 origin=transformPoint(t,{sx,0,0});
            V3 oc{origin[0]-center[0],origin[1]-center[1],origin[2]-center[2]};
            double b=0,c=-radius*radius;for(int j=0;j<3;++j){b+=2*direction[j]*oc[j];c+=oc[j]*oc[j];}
            const double discriminant=b*b-4*c;if(discriminant<0)continue;
            double lambda=(-b-std::sqrt(discriminant))/2;if(lambda<=0)continue;
            V3 world{};for(int j=0;j<3;++j)world[j]=origin[j]+lambda*direction[j];
            const auto sensor=inverse(t,world);inverseError=std::max(inverseError,distance(world,transformPoint(t,sensor)));
            if(std::abs(sensor[1])>1e-10)throw std::runtime_error("inverse ray no longer in sensor XZ plane");
            const auto rebuilt=mapProfilePoint(profile.frame,sensor[0],sensor[2]);
            sphereError=std::max(sphereError,std::abs(distance(rebuilt,center)-radius));
            // Legacy scanner orientation mapped sensor x to machine y, ignoring CB.
            const auto&q=profile.frame.joints;legacyError=std::max(legacyError,distance(rebuilt,{q[0],q[1]+sensor[0],q[2]+sensor[2]}));
            profile.points.push_back({std::uint32_t(k),sensor[0],sensor[2]});cloud.push_back(rebuilt);
            csv<<22<<','<<n<<','<<profile.profileIndex<<','<<k<<','<<sensor[0]<<','<<sensor[2]<<','<<rebuilt[0]<<','<<rebuilt[1]<<','<<rebuilt[2]<<'\n';
        }
        joints<<n;for(double x:profile.frame.joints)joints<<','<<x;joints<<','<<profile.frame.raw[3]<<','<<profile.frame.raw[4]<<",3,0\n";
        profiles.push_back(std::move(profile));
    }
    ArchiveProfile empty=profiles.back();empty.frame.ordinal=64;empty.profileIndex=1064;empty.timeUs+=5000;empty.sensorTimeSec+=.005;empty.frame.hostEstimateUs+=5000;empty.points.clear();profiles.push_back(empty);
    joints<<64;for(double x:empty.frame.joints)joints<<','<<x;joints<<','<<empty.frame.raw[3]<<','<<empty.frame.raw[4]<<",3,0\n";
    std::vector<std::uint8_t> bytes;checked(encodeArchive(profiles,bytes,error),error);binaryWrite(dir/"capture.a5d",bytes);
    auto read=binaryRead(dir/"capture.a5d");std::vector<ArchiveProfile> restored;checked(decodeArchive(read.data(),read.size(),restored,error),error);
    checked(restored.size()==profiles.size(),"profile count mismatch");
    for(std::size_t i=0;i<restored.size();++i)for(std::size_t k=0;k<restored[i].points.size();++k){const auto&a=profiles[i].points[k];const auto&b=restored[i].points[k];replayError=std::max(replayError,distance(mapProfilePoint(profiles[i].frame,a.x,a.z),mapProfilePoint(restored[i].frame,b.x,b.z)));}
    std::ofstream ply(dir/"cloud.ply");ply<<"ply\nformat ascii 1.0\nelement vertex "<<cloud.size()<<"\nproperty double x\nproperty double y\nproperty double z\nend_header\n"<<std::setprecision(17);for(const auto&q:cloud)ply<<q[0]<<' '<<q[1]<<' '<<q[2]<<'\n';
    checked(csv.good()&&joints.good()&&ply.good(),"output write failure");
    checked(inverseError<1e-10&&sphereError<1e-4&&replayError==0,"end-to-end tolerance failure");
    std::ostringstream summary;summary<<std::setprecision(17)<<"{\n  \"test\": \"V22 parallel-rays / physical17 / 6-field hardware-latch record / binary64 archive\",\n  \"hardware_tested\": false,\n  \"qt_gui_tested\": false,\n  \"profiles\": "<<profiles.size()<<",\n  \"empty_profiles\": 1,\n  \"points\": "<<cloud.size()<<",\n  \"inverse_max_mm\": "<<inverseError<<",\n  \"table_float_sphere_max_mm\": "<<sphereError<<",\n  \"archive_replay_max_mm\": "<<replayError<<",\n  \"legacy_xyz_only_max_deviation_mm\": "<<legacyError<<",\n  \"archive_bytes\": "<<bytes.size()<<",\n  \"physical_parameters\": [";
    for(int k=0;k<17;++k)summary<<(k?", ":"")<<theta[k];summary<<"]\n}\n";
    std::ofstream json(dir/"summary.json");json<<summary.str();checked(json.good(),"summary write failed");std::cout<<summary.str();return 0;
}
int validateNc(const std::filesystem::path&file) {
    std::ifstream f(file);if(!f)throw std::runtime_error("cannot read NC");NcState state;Joint current{},offset{};std::string line,error;double feed=1,seconds=0;std::size_t blocks=0,segments=0;bool customPath=false;
    while(std::getline(f,line)){++blocks;NcBlock b;NcState parseState=state;
        std::vector<Word> words;checked(lex(line,words,error),"line "+std::to_string(blocks)+": "+error);
        bool command=false;for(auto&w:words)command=command||w.letter=='G'||w.letter=='M';
        if(customPath&&!command)parseState.modalMotion=1;else customPath=false;
        checked(parseNcBlock(line,parseState,b,error),"line "+std::to_string(blocks)+": "+error);state=parseState;
        if(b.kind==NcBlock::Kind::Empty)continue;
        if(b.feed)feed=*b.feed;
        if(b.kind==NcBlock::Kind::SetWorkPosition){for(int j=0;j<5;++j)if(b.axisMask&(1u<<j))offset[j]=current[j]-b.words[j];continue;}
        if(b.kind!=NcBlock::Kind::Motion)continue;
        if(b.motion==5){customPath=true;if(!b.axisMask)continue;}
        Joint values=b.words,target;for(int j=0;j<5;++j)if(b.absolute&&(b.axisMask&(1u<<j)))values[j]+=offset[j];
        checked(resolve(current,values,b.axisMask,b.absolute,target,error),error);
        std::vector<Joint> targets;
        if(b.motion==2||b.motion==3){Arc arc=b.arc;if(b.arcDefinition==NcBlock::ArcDefinition::ThreePoint){Joint mid=current;mid[0]=b.mid[0]+(b.absolute?offset[0]:current[0]);mid[1]=b.mid[1]+(b.absolute?offset[1]:current[1]);checked(threePointArc(current,mid,target,b.motion==2,arc,error),error);}checked(arcWaypoints(current,target,arc,targets,error),error);}else targets.push_back(target);
        std::vector<Waypoint> waypoints;for(const auto&q:targets)waypoints.push_back({q,b.motion==0?1:(b.feedMode==FeedMode::InverseMinute?feed*targets.size():feed),b.motion==0?FeedMode::LegacyPerSecond:b.feedMode});
        Plan p;checked(plan(current,waypoints,Limits{},25,25,p,error),"line "+std::to_string(blocks)+": "+error);seconds+=p.seconds;segments+=p.segments.size();current=target;
    }
    std::cout<<"Validated from assumed machine start XYZCB=(0,0,0,0,0); "<<blocks<<" lines, "<<segments<<" segments, "<<seconds<<" s conservative exact-block-stop estimate. No hardware commands sent.\n";return 0;
}
}
int main(int argc,char**argv){try{
    if(argc==3&&std::string(argv[1])=="demo")return demo(argv[2]);
    if(argc==3&&std::string(argv[1])=="validate")return validateNc(argv[2]);
    if(argc==3&&std::string(argv[1])=="inspect"){auto b=binaryRead(argv[2]);std::vector<ArchiveProfile>p;std::string e;checked(decodeArchive(b.data(),b.size(),p,e),e);std::size_t n=0;for(const auto&v:p)n+=v.points.size();std::cout<<p.size()<<" profiles, "<<n<<" points; full XYZCB/17D provenance and CRC valid.\n";return 0;}
    std::cerr<<"Usage: v21_runtime_cli (V22-compatible) demo OUTPUT_DIRECTORY | validate FILE.nc | inspect FILE.a5d\n";return 2;
}catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}}
