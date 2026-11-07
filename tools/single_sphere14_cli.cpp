#include "calibration/SingleSphere14Calibration.h"

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

void printUsage()
{
    std::cout
        << "Single-sphere 14-D calibration CLI\n\n"
        << "Calibrate:\n"
        << "  single_sphere14_cli calibrate input.csv output.json\n"
        << "      [--diameter 19.05] [--center-y-sign 1|-1]\n"
        << "      [--initial p0,p1,...,p13] [--sphere-center x,y,z]\n"
        << "      [--alternating 8] [--block 18] [--joint 90] [--multistart 3]\n\n"
        << "Generate D-optimal recorded-pose plan:\n"
        << "  single_sphere14_cli plan output.csv\n"
        << "      --reference x,y,z,c,b --sphere-center-s sx,sy,sz\n"
        << "      [--initial p0,p1,...,p13] [--diameter 19.05] [--count 42]\n"
        << "      [--b-range min,max,step] [--c-range min,max,step]\n"
        << "      [--xyz-limits xmin,xmax,ymin,ymax,zmin,zmax]\n"
        << "      [--sensor-z-range min,max] [--sensor-x-half-width value]\n"
        << "      [--target-x v1,v2,...] [--target-z v1,v2,...]\n"
        << "      [--target-y-ratio value] [--max-per-bc count]\n\n"
        << "CSV input columns:\n"
        << "  profile_id,x_mm,y_mm,z_mm,c_deg,b_deg,sensor_x_mm,sensor_z_mm,weight\n\n"
        << "Important: this tool identifies the 14 data-observable gauge parameters.\n"
        << "It does not claim a unique physical hand-eye translation or four traditional\n"
        << "rotary location PIGEs without an independent 3-D translation datum.\n";
}

bool parseInitial(const std::string& text, Gauge14Parameters& parameters)
{
    std::vector<double> values;
    if (!parseDoubles(text, values) || values.size() != Gauge14Parameters::Count) {
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

    SolverOptions options;
    Gauge14Parameters initial;
    Vec3d initialCenter;
    bool hasInitialCenter = false;

    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--diameter" && i + 1 < argc) {
            options.sphereDiameterMm = std::strtod(argv[++i], nullptr);
        } else if (arg == "--center-y-sign" && i + 1 < argc) {
            options.sphereCenterYSign = std::atoi(argv[++i]);
        } else if (arg == "--initial" && i + 1 < argc) {
            if (!parseInitial(argv[++i], initial)) {
                std::cerr << "Invalid --initial list; exactly 14 comma-separated values are required.\n";
                return 2;
            }
        } else if (arg == "--sphere-center" && i + 1 < argc) {
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
        } else {
            std::cerr << "Unknown or incomplete option: " << arg << "\n";
            return 2;
        }
    }

    std::ifstream input(inputPath);
    if (!input) {
        std::cerr << "Cannot open input CSV: " << inputPath << "\n";
        return 3;
    }
    std::vector<SphereProfileObservation> observations;
    std::string error;
    if (!SingleSphere14Calibrator::readObservationCsv(input, observations, error)) {
        std::cerr << "CSV error: " << error << "\n";
        return 4;
    }

    SingleSphere14Calibrator calibrator(options);
    for (const auto& observation : observations) {
        calibrator.addObservation(observation);
    }
    const SingleSphere14Result result = calibrator.solve(
        initial, hasInitialCenter ? &initialCenter : nullptr);

    std::ofstream output(outputPath);
    if (!output) {
        std::cerr << "Cannot open result path: " << outputPath << "\n";
        return 5;
    }
    if (!SingleSphere14Calibrator::writeResultJson(output, result, error)) {
        std::cerr << "Result write error: " << error << "\n";
        return 6;
    }

    std::cout << result.message << "\n"
              << "Profiles: " << result.profileCount
              << ", points: " << result.pointCount << "\n"
              << "RMS: " << result.residualRmsMm << " mm\n"
              << "Effective rank: " << result.observability.targetRank
              << "/" << result.observability.expectedTargetRank << "\n"
              << "Condition number: " << result.observability.conditionNumber << "\n";
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
    Gauge14Parameters initial;
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
        } else if (arg == "--count" && i + 1 < argc) {
            config.requestedRecordedPoses = std::atoi(argv[++i]);
        } else if (arg == "--initial" && i + 1 < argc) {
            if (!parseInitial(argv[++i], initial)) {
                std::cerr << "Invalid --initial list; exactly 14 comma-separated values are required.\n";
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

    SingleSphere14PosePlanner planner(config);
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
    if (!SingleSphere14PosePlanner::writeCsv(output, poses, error)) {
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
