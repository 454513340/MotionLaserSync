#include "calibration/SingleSphere17Calibration.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace mls::calibration;

namespace {

std::vector<std::string> split(const std::string& text, char delimiter)
{
    std::vector<std::string> values;
    std::string item;
    std::istringstream stream(text);
    while (std::getline(stream, item, delimiter)) {
        values.push_back(item);
    }
    return values;
}

bool parseDoubles(const std::string& text, std::vector<double>& values)
{
    values.clear();
    for (const std::string& item : split(text, ',')) {
        char* end = nullptr;
        const double value = std::strtod(item.c_str(), &end);
        if (end == item.c_str() || *end != '\0') {
            return false;
        }
        values.push_back(value);
    }
    return true;
}


bool samePose(const JointPose5d& a, const JointPose5d& b,
              double linearToleranceMm = 0.03,
              double angularToleranceDeg = 0.03)
{
    return std::abs(a.x-b.x)<=linearToleranceMm &&
           std::abs(a.y-b.y)<=linearToleranceMm &&
           std::abs(a.z-b.z)<=linearToleranceMm &&
           std::abs(a.c-b.c)<=angularToleranceDeg &&
           std::abs(a.b-b.b)<=angularToleranceDeg;
}

std::vector<SphereProfileObservation> retainScreenedPoses(
    const std::vector<SphereProfileObservation>& observations,
    const Physical17PoseScreeningReport& report,
    const Physical17PoseScreeningOptions& options)
{
    std::vector<SphereProfileObservation> kept;
    kept.reserve(observations.size());
    for (const auto& observation : observations) {
        bool rejected=false;
        for (const auto& summary : report.poses) {
            if (summary.rejected && samePose(observation.joints, summary.pose,
                    options.poseLinearToleranceMm, options.poseAngularToleranceDeg)) {
                rejected=true; break;
            }
        }
        if (!rejected) kept.push_back(observation);
    }
    return kept;
}

Physical17PoseScreeningReport limitRejectedPoseCount(
    Physical17PoseScreeningReport report, std::size_t keepRejectedCount)
{
    std::vector<std::size_t> rejected;
    for (std::size_t i=0; i<report.poses.size(); ++i)
        if (report.poses[i].rejected) rejected.push_back(i);
    std::sort(rejected.begin(), rejected.end(), [&](std::size_t a, std::size_t b) {
        return report.poses[a].rejectionScore > report.poses[b].rejectionScore;
    });
    keepRejectedCount=std::min(keepRejectedCount,rejected.size());
    for (auto& pose:report.poses) pose.rejected=false;
    for (std::size_t i=0;i<keepRejectedCount;++i) report.poses[rejected[i]].rejected=true;
    report.rejectedPoseCount=keepRejectedCount;
    report.retainedPoseCount=report.inputPoseCount>=keepRejectedCount
        ? report.inputPoseCount-keepRejectedCount : 0;
    return report;
}

void printUsage()
{
    std::cout
        << "Single-sphere physical-17 / data-rank-14 calibration CLI\n\n"
        << "Calibrate:\n"
        << "  single_sphere17_cli calibrate input.csv output.json\n"
        << "      [--diameter 19.05] [--handeye-translation tx,ty,tz]\n"
        << "      [--initial p0,p1,...,p16] [--sphere-center x,y,z]\n"
        << "      [--auto-handeye-bootstrap | --no-auto-handeye-bootstrap]\n"
        << "      [--pose-screening | --no-pose-screening]\n"
        << "      [--alternating 8] [--block 18] [--joint 90] [--multistart 3]\n\n"
        << "Generate D-optimal recorded-pose plan:\n"
        << "  single_sphere17_cli plan output.csv\n"
        << "      --reference x,y,z,c,b --sphere-center-s sx,sy,sz\n"
        << "      [--initial p0,p1,...,p16] [--handeye-translation tx,ty,tz] [--diameter 19.05] [--count 42]\n"
        << "      [--b-range min,max,step] [--c-range min,max,step]\n"
        << "      [--xyz-limits xmin,xmax,ymin,ymax,zmin,zmax]\n"
        << "      [--sensor-z-range min,max] [--sensor-x-half-width value]\n"
        << "      [--target-x v1,v2,...] [--target-z v1,v2,...]\n"
        << "      [--target-y-ratio value] [--max-per-bc count]\n\n"
        << "CSV input columns:\n"
        << "  profile_id,x_mm,y_mm,z_mm,c_deg,b_deg,sensor_x_mm,sensor_z_mm,weight\n\n"
        << "Important: all 17 physical parameters are retained, but one unknown fixed sphere\n"
        << "provides only rank 14/17 after eliminating its unknown center. By default, when no\n"
        << "manual seed is supplied, calibrate first solves a nominal-machine 9-state hand-eye\n"
        << "+ sphere bootstrap from the input CSV itself; its hand-eye translation is then fixed\n"
        << "as the explicit translation-gauge anchor for the rank-14 Physical17 solve. V27.6.41\n"
        << "then performs progressive rank-aware trimmed pose selection: small worst-tail batches\n"
        << "are re-bootstrapped/re-fit and accepted only when the penalized retained-data fit improves\n"
        << "while preserving XYZ/B/C excitation and the required numerical rank.\n";
}

bool parseInitial(const std::string& text, Physical17Parameters& parameters)
{
    std::vector<double> values;
    if (!parseDoubles(text, values) || values.size() != Physical17Parameters::Count) {
        return false;
    }
    for (std::size_t i = 0; i < values.size(); ++i) {
        parameters[i] = values[i];
    }
    return true;
}

int calibrateCommand(int argc, char** argv)
{
    if (argc < 4) {
        printUsage();
        return 2;
    }
    const std::string inputPath = argv[2];
    const std::string outputPath = argv[3];

    Physical17SolverOptions options;
    Physical17Parameters initial;
    Vec3d initialCenter;
    bool hasInitialCenter = false;
    bool autoHandEyeBootstrap = true;
    bool autoBootstrapFlagExplicit = false;
    bool manualSeedSpecified = false;
    bool poseScreening = true;

    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--diameter" && i + 1 < argc) {
            options.sphereDiameterMm = std::strtod(argv[++i], nullptr);
        } else if (arg == "--handeye-translation" && i + 1 < argc) {
            manualSeedSpecified = true;
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 3) {
                std::cerr << "Invalid --handeye-translation; use tx,ty,tz.\n";
                return 2;
            }
            initial[Physical17Parameters::HandEyeTx]=values[0];
            initial[Physical17Parameters::HandEyeTy]=values[1];
            initial[Physical17Parameters::HandEyeTz]=values[2];
        } else if (arg == "--initial" && i + 1 < argc) {
            manualSeedSpecified = true;
            if (!parseInitial(argv[++i], initial)) {
                std::cerr << "Invalid --initial list; exactly 17 comma-separated values are required.\n";
                return 2;
            }
        } else if (arg == "--sphere-center" && i + 1 < argc) {
            manualSeedSpecified = true;
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 3) {
                std::cerr << "Invalid --sphere-center; use x,y,z.\n";
                return 2;
            }
            initialCenter = Vec3d(values[0], values[1], values[2]);
            hasInitialCenter = true;
        } else if (arg == "--alternating" && i + 1 < argc) {
            options.alternatingIterations = std::atoi(argv[++i]);
        } else if (arg == "--block" && i + 1 < argc) {
            options.blockIterations = std::atoi(argv[++i]);
        } else if (arg == "--joint" && i + 1 < argc) {
            options.jointIterations = std::atoi(argv[++i]);
        } else if (arg == "--multistart" && i + 1 < argc) {
            options.multiStartCount = std::atoi(argv[++i]);
        } else if (arg == "--auto-handeye-bootstrap") {
            autoHandEyeBootstrap = true;
            autoBootstrapFlagExplicit = true;
        } else if (arg == "--no-auto-handeye-bootstrap") {
            autoHandEyeBootstrap = false;
            autoBootstrapFlagExplicit = true;
        } else if (arg == "--pose-screening") {
            poseScreening = true;
        } else if (arg == "--no-pose-screening") {
            poseScreening = false;
        } else {
            std::cerr << "Unknown or incomplete option: " << arg << "\n";
            return 2;
        }
    }

    if (manualSeedSpecified && autoBootstrapFlagExplicit && autoHandEyeBootstrap) {
        std::cerr << "Manual seed options cannot be combined with --auto-handeye-bootstrap. "
                     "Use either the data-driven bootstrap or an explicit manual seed.\n";
        return 2;
    }

    std::ifstream input(inputPath);
    if (!input) {
        std::cerr << "Cannot open input CSV: " << inputPath << "\n";
        return 3;
    }
    std::vector<SphereProfileObservation> observations;
    std::string error;
    if (!SingleSphere17Calibrator::readObservationCsv(input, observations, error)) {
        std::cerr << "CSV error: " << error << "\n";
        return 4;
    }

    SingleSphere17Calibrator calibrator(options);
    for (const auto& observation : observations) {
        calibrator.addObservation(observation);
    }

    NominalHandEyeSphereResult bootstrap;
    const bool useAutoBootstrap = autoHandEyeBootstrap && !manualSeedSpecified;
    if (useAutoBootstrap) {
        bootstrap = calibrator.solveNominalHandEyeBootstrap();
        if (!bootstrap.success || bootstrap.stateRank != bootstrap.expectedStateRank) {
            std::cerr << "Automatic hand-eye bootstrap failed: " << bootstrap.message
                      << " rank=" << bootstrap.stateRank << "/" << bootstrap.expectedStateRank
                      << " RMS=" << bootstrap.residualRmsMm
                      << " condition=" << bootstrap.conditionNumber << "\n";
            return 7;
        }
        initial = bootstrap.parameters;
        initialCenter = bootstrap.sphereCenterMachineMm;
        hasInitialCenter = true;
        std::cout << "Auto hand-eye bootstrap from input CSV: rank="
                  << bootstrap.stateRank << "/9, RMS=" << bootstrap.residualRmsMm
                  << " mm, t=["
                  << initial[Physical17Parameters::HandEyeTx] << ","
                  << initial[Physical17Parameters::HandEyeTy] << ","
                  << initial[Physical17Parameters::HandEyeTz] << "] mm\n";
    }

    SingleSphere17Result result = calibrator.solve(
        initial, hasInitialCenter ? &initialCenter : nullptr);

    std::vector<Physical17PoseResidualSummary> rejectedPoses;
    std::size_t inputPoseCount=0;
    int screeningRounds=0;
    double lastRmsThreshold=0.0,lastP95Threshold=0.0;
    if (useAutoBootstrap && bootstrap.success && poseScreening && result.success) {
        Physical17PoseScreeningOptions screeningOptions;
        screeningOptions.minRetainedPoseCount=18;
        screeningOptions.maxRejectedFraction=0.15;
        screeningOptions.madSigma=1.75;
        screeningOptions.relativeRmsFactor=1.30;
        screeningOptions.relativeP95Factor=1.30;
        screeningOptions.absoluteRmsFloorMm=0.050;
        screeningOptions.absoluteP95FloorMm=0.080;
        screeningOptions.targetTrainingRmsMm=0.045;
        screeningOptions.progressiveTrimFraction=0.12;
        screeningOptions.minProgressiveTrimCount=3;
        screeningOptions.maxProgressiveTrimCount=6;
        screeningOptions.minProgressiveSpreadMm=0.010;
        screeningOptions.progressiveTailFactor=1.02;
        constexpr int maxRounds=4;
        constexpr double maxCumulativeRejectedFraction=0.40;
        constexpr double penaltyMm=0.020;
        constexpr double minObjectiveImprovementMm=0.00025;
        double currentRms=result.residualRmsMm;
        double currentObjective=currentRms;

        for (int round=0; round<maxRounds; ++round) {
            auto report=SingleSphere17Calibrator::screenPoseOutliers(
                observations,result.parameters,result.sphereCenterMachineMm,
                options.sphereDiameterMm,screeningOptions);
            if (round==0) inputPoseCount=report.inputPoseCount;
            lastRmsThreshold=report.rmsThresholdMm;
            lastP95Threshold=report.p95ThresholdMm;
            report.refitRmsBeforeMm=currentRms;
            report.selectionObjectiveBefore=currentObjective;
            std::cout << report.message << "\n";
            if (!report.valid || report.rejectedPoseCount==0) break;

            const std::size_t cumulativeLimit=static_cast<std::size_t>(std::floor(
                maxCumulativeRejectedFraction*static_cast<double>(inputPoseCount)+1e-12));
            if (rejectedPoses.size()>=cumulativeLimit) break;
            const std::size_t remaining=cumulativeLimit-rejectedPoses.size();
            if (report.rejectedPoseCount>remaining) report=limitRejectedPoseCount(report,remaining);
            if (report.rejectedPoseCount==0) break;

            std::vector<std::size_t> trialCounts{report.rejectedPoseCount};
            if (report.rejectedPoseCount>3) trialCounts.push_back(std::max<std::size_t>(2,(2*report.rejectedPoseCount)/3));
            if (report.rejectedPoseCount>2) trialCounts.push_back(std::max<std::size_t>(2,report.rejectedPoseCount/2));
            if (report.rejectedPoseCount>1) trialCounts.push_back(1);
            std::sort(trialCounts.begin(),trialCounts.end(),std::greater<std::size_t>());
            trialCounts.erase(std::unique(trialCounts.begin(),trialCounts.end()),trialCounts.end());

            bool accepted=false;
            for (std::size_t trialCount:trialCounts) {
                auto trialReport=limitRejectedPoseCount(report,trialCount);
                auto proposed=retainScreenedPoses(observations,trialReport,screeningOptions);
                SingleSphere17Calibrator filtered(options);
                for (const auto& observation:proposed) filtered.addObservation(observation);
                const auto filteredBootstrap=filtered.solveNominalHandEyeBootstrap();
                if (!filteredBootstrap.success ||
                    filteredBootstrap.stateRank!=filteredBootstrap.expectedStateRank) continue;
                const auto filteredResult=filtered.solve(
                    filteredBootstrap.parameters,&filteredBootstrap.sphereCenterMachineMm);
                const double relativeConditionLimit=std::min(
                    1.0e9,std::max(1.0e6,5.0*std::max(1.0,result.observability.conditionNumber)));
                if (!filteredResult.success ||
                    filteredResult.observability.targetRank<filteredResult.observability.expectedTargetRank ||
                    filteredResult.observability.fullStateRank<filteredResult.observability.expectedFullStateRank ||
                    !std::isfinite(filteredResult.residualRmsMm) ||
                    !std::isfinite(filteredResult.observability.conditionNumber) ||
                    filteredResult.observability.conditionNumber>relativeConditionLimit) continue;

                const std::size_t cumulativeRejected=rejectedPoses.size()+trialCount;
                const double fraction=inputPoseCount>0
                    ? static_cast<double>(cumulativeRejected)/static_cast<double>(inputPoseCount) : 0.0;
                const double objective=filteredResult.residualRmsMm+penaltyMm*fraction;
                const bool improved=objective+minObjectiveImprovementMm<currentObjective;
                const bool hitTarget=filteredResult.residualRmsMm<=screeningOptions.targetTrainingRmsMm &&
                                     filteredResult.residualRmsMm<currentRms;
                if (!improved && !hitTarget) continue;

                trialReport.applied=true;
                trialReport.refitRmsBeforeMm=currentRms;
                trialReport.refitRmsAfterMm=filteredResult.residualRmsMm;
                trialReport.selectionObjectiveBefore=currentObjective;
                trialReport.selectionObjectiveAfter=objective;
                std::cout << "Accepted progressive trim: poses=" << trialCount
                          << ", RMS " << currentRms << " -> " << filteredResult.residualRmsMm
                          << " mm, objective " << currentObjective << " -> " << objective << "\n";
                for (const auto& summary:trialReport.poses)
                    if (summary.rejected) rejectedPoses.push_back(summary);
                observations=std::move(proposed);
                bootstrap=filteredBootstrap;
                result=filteredResult;
                currentRms=result.residualRmsMm;
                currentObjective=objective;
                ++screeningRounds;
                accepted=true;
                break;
            }
            if (!accepted) {
                std::cout << "Progressive screening stopped: no candidate batch improved the penalized fit.\n";
                break;
            }
            if (currentRms<=screeningOptions.targetTrainingRmsMm) break;
        }
    }

    if (useAutoBootstrap && bootstrap.success) {
        result.hasInitializationBootstrap = true;
        result.initializationSource = screeningRounds>0
            ? "stage2_pose_screened_training_data_nominal_machine_handeye"
            : "stage2_training_data_nominal_machine_handeye";
        result.initializationParameters = bootstrap.parameters;
        result.initializationSphereCenterMachineMm = bootstrap.sphereCenterMachineMm;
        result.initializationResidualRmsMm = bootstrap.residualRmsMm;
        result.initializationResidualMaxAbsMm = bootstrap.residualMaxAbsMm;
        result.initializationStateRank = bootstrap.stateRank;
        result.initializationConditionNumber = bootstrap.conditionNumber;
        result.initializationProfileCount = bootstrap.profileCount;
        result.initializationPointCount = bootstrap.pointCount;
        result.poseScreeningRounds=screeningRounds;
        result.trainingPoseCountBeforeScreening=inputPoseCount;
        result.trainingPoseCountAfterScreening=inputPoseCount>=rejectedPoses.size()
            ? inputPoseCount-rejectedPoses.size() : 0;
        result.poseScreeningRmsThresholdMm=lastRmsThreshold;
        result.poseScreeningP95ThresholdMm=lastP95Threshold;
        result.rejectedTrainingPoses=rejectedPoses;
        result.gaugeDescription =
            "V27.6.41 retained-pose nominal-machine hand-eye+sphere bootstrap supplies the "
            "translation gauge anchor after progressive rank-aware pose trimming. " + result.gaugeDescription;
    }

    std::ofstream output(outputPath);
    if (!output) {
        std::cerr << "Cannot open result path: " << outputPath << "\n";
        return 5;
    }
    if (!SingleSphere17Calibrator::writeResultJson(output, result, error)) {
        std::cerr << "Result write error: " << error << "\n";
        return 6;
    }

    std::cout << result.message << "\n"
              << "Profiles: " << result.profileCount
              << ", points: " << result.pointCount << "\n"
              << "RMS: " << result.residualRmsMm << " mm\n"
              << "Physical rank: " << result.observability.targetRank
              << "/" << result.observability.physicalParameterCount
              << " (expected " << result.observability.expectedTargetRank << ")\n"
              << "Full data-state rank: " << result.observability.fullStateRank
              << "/" << result.observability.fullStateCount
              << " (expected " << result.observability.expectedFullStateRank << ")\n"
              << "Condition number (nonzero physical subspace): " << result.observability.conditionNumber << "\n";
    return result.success ? 0 : 7;
}

int planCommand(int argc, char** argv)
{
    if (argc < 3) {
        printUsage();
        return 2;
    }
    const std::string outputPath = argv[2];
    PosePlannerConfig config;
    PosePlannerSeed seed;
    Physical17Parameters initial;
    bool hasReference = false;
    bool hasSensorCenter = false;

    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--reference" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 5) {
                std::cerr << "Invalid --reference; use x,y,z,c,b.\n";
                return 2;
            }
            seed.taughtJoint.x = values[0];
            seed.taughtJoint.y = values[1];
            seed.taughtJoint.z = values[2];
            seed.taughtJoint.c = values[3];
            seed.taughtJoint.b = values[4];
            hasReference = true;
        } else if (arg == "--sphere-center-s" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 3) {
                std::cerr << "Invalid --sphere-center-s; use sx,sy,sz.\n";
                return 2;
            }
            seed.taughtSphereCenterSensorMm = Vec3d(values[0], values[1], values[2]);
            hasSensorCenter = true;
        } else if (arg == "--diameter" && i + 1 < argc) {
            config.sphereDiameterMm = std::strtod(argv[++i], nullptr);
        } else if (arg == "--handeye-translation" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 3) {
                std::cerr << "Invalid --handeye-translation; use tx,ty,tz.\n";
                return 2;
            }
            initial[Physical17Parameters::HandEyeTx]=values[0];
            initial[Physical17Parameters::HandEyeTy]=values[1];
            initial[Physical17Parameters::HandEyeTz]=values[2];
        } else if (arg == "--count" && i + 1 < argc) {
            config.requestedRecordedPoses = std::atoi(argv[++i]);
        } else if (arg == "--initial" && i + 1 < argc) {
            if (!parseInitial(argv[++i], initial)) {
                std::cerr << "Invalid --initial list; exactly 17 comma-separated values are required.\n";
                return 2;
            }
        } else if (arg == "--b-range" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 3) {
                std::cerr << "Invalid --b-range; use min,max,step.\n";
                return 2;
            }
            config.bMinDeg = values[0]; config.bMaxDeg = values[1]; config.bStepDeg = values[2];
        } else if (arg == "--c-range" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 3) {
                std::cerr << "Invalid --c-range; use min,max,step.\n";
                return 2;
            }
            config.cMinDeg = values[0]; config.cMaxDeg = values[1]; config.cStepDeg = values[2];
        } else if (arg == "--xyz-limits" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 6) {
                std::cerr << "Invalid --xyz-limits; use xmin,xmax,ymin,ymax,zmin,zmax.\n";
                return 2;
            }
            config.xMinMm=values[0]; config.xMaxMm=values[1];
            config.yMinMm=values[2]; config.yMaxMm=values[3];
            config.zMinMm=values[4]; config.zMaxMm=values[5];
        } else if (arg == "--sensor-z-range" && i + 1 < argc) {
            std::vector<double> values;
            if (!parseDoubles(argv[++i], values) || values.size() != 2) {
                std::cerr << "Invalid --sensor-z-range; use min,max.\n";
                return 2;
            }
            config.sensorZMinMm=values[0]; config.sensorZMaxMm=values[1];
        } else if (arg == "--sensor-x-half-width" && i + 1 < argc) {
            config.sensorXHalfWidthMm=std::strtod(argv[++i],nullptr);
        } else if (arg == "--target-x" && i + 1 < argc) {
            if (!parseDoubles(argv[++i], config.targetCenterXmm) || config.targetCenterXmm.empty()) {
                std::cerr << "Invalid --target-x list.\n";
                return 2;
            }
        } else if (arg == "--target-z" && i + 1 < argc) {
            if (!parseDoubles(argv[++i], config.targetCenterZmm) || config.targetCenterZmm.empty()) {
                std::cerr << "Invalid --target-z list.\n";
                return 2;
            }
        } else if (arg == "--target-y-ratio" && i + 1 < argc) {
            config.targetCenterYRadiusRatio=std::strtod(argv[++i],nullptr);
        } else if (arg == "--max-per-bc" && i + 1 < argc) {
            config.maxRecordedPosesPerBCPair=std::atoi(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete option: " << arg << "\n";
            return 2;
        }
    }
    if (!hasReference || !hasSensorCenter) {
        std::cerr << "Both --reference and --sphere-center-s are required.\n";
        return 2;
    }

    SingleSphere17PosePlanner planner(config);
    std::string error;
    const auto poses = planner.generate(seed, initial, error);
    if (!error.empty()) {
        std::cerr << "Plan error: " << error << "\n";
        return 4;
    }
    std::ofstream output(outputPath);
    if (!output) {
        std::cerr << "Cannot open plan path: " << outputPath << "\n";
        return 5;
    }
    if (!SingleSphere17PosePlanner::writeCsv(output, poses, error)) {
        std::cerr << "Plan write error: " << error << "\n";
        return 6;
    }
    std::cout << "Generated " << poses.size() << " recorded poses.\n"
              << "Predicted XYZ values are seeds only; use small B/C transit steps and closed-loop sphere recentering.\n";
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        printUsage();
        return 2;
    }
    const std::string command = argv[1];
    if (command == "calibrate") {
        return calibrateCommand(argc, argv);
    }
    if (command == "plan") {
        return planCommand(argc, argv);
    }
    printUsage();
    return 2;
}
