// Deterministic, hardware-free comparison harness.  The same source can be
// linked against the original or optimized SingleSphere17Calibration.cpp.
#define main shared_physical17_fixture_main
#include "../tests/v20_2_single_sphere17_tests.cpp"
#undef main
#include <chrono>
#include <random>
#include <iomanip>
#include <stdexcept>

std::vector<SphereProfileObservation> denseData(int copies,int pointCount)
{
    const auto truth=truthParameters();const Vec3d centerM{245,-105,165};
    const auto sparse=makeObservations(truth,centerM);
    std::minstd_rand gen(20260921);
    std::normal_distribution<double> noise(0.0,0.003);
    std::vector<SphereProfileObservation> out;
    for(std::size_t pose=0;pose<sparse.size();++pose) {
        const auto center=SingleSphere17Model::machinePointInSensor(sparse[pose].joints,centerM,truth);
        const double rho=std::sqrt(kRadius*kRadius-center.y*center.y);
        for(int k=0;k<copies;++k) {
            SphereProfileObservation o;o.profileId=static_cast<std::int64_t>(out.size()+1);o.joints=sparse[pose].joints;
            for(int i=0;i<pointCount;++i) {
                const double dx=(-0.88+1.76*double(i)/(pointCount-1))*rho;
                SensorProfilePoint p;p.x=center.x+dx+noise(gen);
                p.z=center.z-std::sqrt(rho*rho-dx*dx)+noise(gen);
                // Sparse deterministic outliers exercise the identical Huber objective.
                if((i+17*k+7*pose)%401==0) p.z+=0.06;
                p.weight=(i%11==0)?0.5:1.0;
                o.points.push_back(p);
            }
            out.push_back(std::move(o));
        }
    }
    return out;
}
std::vector<SphereProfileObservation> representatives(const std::vector<SphereProfileObservation>& full)
{
    std::vector<SphereProfileObservation> out;
    for(std::size_t i=0;i<full.size();i+=8)for(int sample:{0,4,7}) {
        auto o=full[i+sample];o.points.clear();
        for(int j=0;j<180;++j) {
            const auto index=static_cast<std::size_t>(std::lround(double(j)*(full[i+sample].points.size()-1)/179));
            o.points.push_back(full[i+sample].points[index]);
        }
        out.push_back(std::move(o));
    }
    return out;
}
double robustObjective(const std::vector<SphereProfileObservation>& obs,const Physical17Parameters& p,const Vec3d& center)
{
    double sum=0;
    for(const auto& o:obs) {
        const auto cs=SingleSphere17Model::machinePointInSensor(o.joints,center,p);
        for(const auto& point:o.points) {
            const double dx=point.x-cs.x,dy=-cs.y,dz=point.z-cs.z;
            const double r=std::sqrt(dx*dx+dy*dy+dz*dz)-kRadius,a=std::abs(r),delta=0.015;
            sum+=point.weight*(a<=delta?0.5*r*r:delta*(a-0.5*delta));
        }
    }
    return sum;
}
int main(int argc,char** argv)
{
    try {
        const std::string mode=argc>1?argv[1]:"solve";
        const auto full=denseData(8,512);const auto reduced=representatives(full);
        Physical17SolverOptions o;o.alternatingIterations=2;o.blockIterations=6;o.jointIterations=18;o.multiStartCount=2;
        if(mode=="ui_schedule") o.jointIterations=60;
        Physical17Parameters initial;
        const auto truth=truthParameters();
        for(int i=11;i<17;++i)initial[i]=truth[i];
        initial[11]+=0.001;initial[12]-=0.001;initial[13]+=0.0005;
        const Vec3d initialCenter{245.2,-105.15,165.1};
        std::cout<<std::setprecision(15);
        if(mode=="diagnostics") {
            SingleSphere17Calibrator cal(o);for(const auto& x:full)cal.addObservation(x);
            const Vec3d c{245,-105,165};
            const auto begin=std::chrono::steady_clock::now();
            const auto v=SingleSphere17Calibrator::validateModel(full,truth,c,19.05);
            const auto report=cal.analyzeObservability(truth,c);
            const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            std::cout<<"{\"mode\":\"diagnostics\",\"seconds\":"<<seconds<<",\"profiles\":"<<v.profileCount
                <<",\"points\":"<<v.pointCount<<",\"rms\":"<<v.residualRmsMm<<",\"rank\":"<<report.targetRank
                <<",\"full_rank\":"<<report.fullStateRank<<",\"condition\":"<<report.conditionNumber<<"}\n";
            return v.valid && report.targetRank>=14 && report.fullStateRank>=17 ? 0:1;
        }
        SingleSphere17Calibrator cal(o);for(const auto& x:reduced)cal.addObservation(x);
        const auto begin=std::chrono::steady_clock::now();
        auto result=cal.solve(initial,&initialCenter);
        const double reducedSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        if(!result.success) {std::cerr<<result.message<<'\n';return 1;}
        const double initialFullCost=robustObjective(full,result.parameters,result.sphereCenterMachineMm);
        double finalFullCost=initialFullCost;
        if(mode=="polish" || mode=="ui_schedule") {
            o.multiStartCount=1;o.alternatingIterations=0;o.jointIterations=40;
            SingleSphere17Calibrator fullCal(o);for(const auto& x:full)fullCal.addObservation(x);
            result=fullCal.solve(result.parameters,&result.sphereCenterMachineMm);
            finalFullCost=robustObjective(full,result.parameters,result.sphereCenterMachineMm);
            require(finalFullCost<=initialFullCost+1e-10,"full refinement must not increase full Huber objective");
#ifdef MLS_BENCH_FAST
            require(result.optimizationConverged,"full-data polish must meet a stopping tolerance");
#endif
        }
        const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        std::cout<<"{\"mode\":\""<<mode<<"\",\"seconds\":"<<seconds<<",\"solve_seconds\":"<<reducedSeconds
            <<",\"profiles\":"<<result.profileCount<<",\"points\":"<<result.pointCount<<",\"rms\":"<<result.residualRmsMm
            <<",\"rank\":"<<result.observability.targetRank<<",\"full_rank\":"<<result.observability.fullStateRank
            <<",\"condition\":"<<result.observability.conditionNumber
            <<",\"full_cost_before\":"<<initialFullCost<<",\"full_cost_after\":"<<finalFullCost;
#ifdef MLS_BENCH_FAST
        std::cout<<",\"converged\":"<<(result.optimizationConverged?"true":"false");
#endif
        std::cout<<",\"parameters\":[";
        for(int i=0;i<17;++i)std::cout<<(i?",":"")<<result.parameters[i];
        std::cout<<"],\"center\":["<<result.sphereCenterMachineMm.x<<","<<result.sphereCenterMachineMm.y<<","<<result.sphereCenterMachineMm.z<<"]}\n";
        return result.success?0:1;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
