cmake_minimum_required(VERSION 3.21)

# 工程名
project(MotionLaserSync_V30 LANGUAGES CXX)

# Standalone regression for imported laser-profile triangle strips; independent
# of Windows/Qt/VTK/SDK and therefore runnable on Linux CI as well.
option(ORGANIZED_CLOUD_MESH_TESTS_ONLY "Build only imported scan mesh topology tests" OFF)
if(ORGANIZED_CLOUD_MESH_TESTS_ONLY)
    enable_testing()
    add_executable(organized_cloud_mesh_tests tests/organized_cloud_mesh_tests.cpp)
    target_compile_features(organized_cloud_mesh_tests PRIVATE cxx_std_17)
    target_include_directories(organized_cloud_mesh_tests PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(organized_cloud_mesh_tests PRIVATE /utf-8 /Zc:__cplusplus)
    endif()
    add_test(NAME organized_cloud_mesh_topology COMMAND organized_cloud_mesh_tests)
    return()
endif()

# V28 M0+M1: independent offline STEP/CAD workpiece scene.
option(DIGITAL_TWIN_CLI_ONLY "Build portable V28 digital twin CLI/tests only (no Qt/VTK/vendor SDK)" OFF)
option(TWIN_ENABLE_OCCT "Enable true STEP import when Open CASCADE is installed" ON)
# User's confirmed Windows x64 Release OCCT prefix (override in CMakeSettings if needed).
set(TWIN_OCCT_PREFIX "D:/dev/vcpkg/installed/x64-windows-release" CACHE PATH "Optional OCCT installed prefix")

set(TWIN_CORE_SOURCES
    src/twin/TwinTypes.cpp
    src/twin/TwinScene.cpp
    src/twin/TwinTrajectory.cpp
    src/twin/CadGeometryAdapter.cpp
    src/twin/VirtualLaser2950.cpp
    src/twin/TwinM34ReturnModel.cpp
    src/twin/TwinM5Coverage.cpp
    src/twin/TwinM51Chamfer.cpp
    src/twin/TwinM52Geometry.cpp
    src/twin/TwinM6Continuous.cpp
    src/twin/TwinM6NcPreview.cpp
    src/twin/MultiPoseExperiment.cpp
    src/twin/MultiPoseOpticalCoverage.cpp
    src/twin/MultiPosePlayback.cpp
    src/twin/MultiPoseNcSimulation.cpp
    src/twin/NcG93MotionAudit.cpp
    src/runtime/FiveAxis.cpp
    src/runtime/GCode5D.cpp
    src/twin/MultiPoseCloudComparison.cpp
    src/twin/MultiPoseRegistrationAssist.cpp
    src/twin/TwinM7Fusion.cpp
    src/twin/TwinM8Workflow.cpp
)
# M4's portable algorithm sources are compiled into the offline twin library
# in DIGITAL_TWIN_CLI_ONLY mode; for the Windows GUI they are compiled directly
# into MotionLaserSync.exe to avoid the unresolved M4 symbols seen in VS2022.
set(TWIN_M4_SOURCES
    src/twin/TwinM4Comparison.cpp
    src/twin/TwinM4Calibration.cpp
)
set(TWIN_M4_HEADERS
    include/twin/TwinM4Comparison.h
    include/twin/TwinM4Calibration.h
)
set(TWIN_CORE_HEADERS
    include/twin/TwinTypes.h
    include/twin/TwinScene.h
    include/twin/TwinTrajectory.h
    include/twin/TwinSession.h
    include/twin/CadGeometryAdapter.h
    include/twin/VirtualLaser2950.h
    include/twin/TwinM34ReturnModel.h
    include/twin/TwinM5Coverage.h
    include/twin/TwinH1Geometry.h
    include/twin/TwinM51Chamfer.h
    include/twin/TwinM52Geometry.h
    include/twin/TwinM6Continuous.h
    include/twin/TwinM6NcPreview.h
    include/twin/MultiPoseExperiment.h
    include/twin/MultiPoseOpticalCoverage.h
    include/twin/MultiPosePlayback.h
    include/twin/MultiPoseNcSimulation.h
    include/twin/NcG93MotionAudit.h
    include/twin/MultiPoseCloudComparison.h
    include/twin/MultiPoseRegistrationAssist.h
    include/twin/TwinM7Fusion.h
    include/twin/TwinM8Workflow.h
)

function(mls_add_twin_core)
    add_library(digital_twin_core STATIC ${TWIN_CORE_SOURCES} ${TWIN_CORE_HEADERS})
    if(DIGITAL_TWIN_CLI_ONLY)
        # No Qt/VTK in CLI builds: portable M4 tests link against this library.
        target_sources(digital_twin_core PRIVATE ${TWIN_M4_SOURCES} ${TWIN_M4_HEADERS})
    endif()
    target_compile_features(digital_twin_core PUBLIC cxx_std_17)
    target_include_directories(digital_twin_core PUBLIC "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(digital_twin_core PRIVATE /utf-8 /Zc:__cplusplus)
    else()
        target_compile_options(digital_twin_core PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(TWIN_ENABLE_OCCT)
        # OCCT needs to be built for the same compiler/architecture/configuration.
        # A missing optional package does not break existing hardware GUI builds;
        # the STEP button reports that OCCT is required. OBJ preview remains available.
        find_package(OpenCASCADE CONFIG QUIET)
        if(NOT OpenCASCADE_FOUND)
            find_package(OpenCASCADE QUIET)
        endif()
        if(OpenCASCADE_FOUND)
            target_compile_definitions(digital_twin_core PUBLIC MLS_TWIN_HAVE_OCCT=1)
            if(OpenCASCADE_INCLUDE_DIR)
                target_include_directories(digital_twin_core PRIVATE "${OpenCASCADE_INCLUDE_DIR}")
            endif()
            if(OpenCASCADE_INCLUDE_DIRS)
                target_include_directories(digital_twin_core PRIVATE ${OpenCASCADE_INCLUDE_DIRS})
            endif()
            # Most OCCT distributions expose the list below; some export imported
            # targets, others expose library names. CMake handles both.
            if(OpenCASCADE_LIBRARIES)
                target_link_libraries(digital_twin_core PUBLIC ${OpenCASCADE_LIBRARIES})
            else()
                target_link_libraries(digital_twin_core PUBLIC
                    TKernel TKMath TKG2d TKG3d TKBRep TKGeomBase TKGeomAlgo
                    TKTopAlgo TKMesh TKSTEP TKSTEPBase TKSTEPAttr TKXSBase)
            endif()
            message(STATUS "V28 STEP import: Open CASCADE enabled")
        else()
            message(WARNING "V28 STEP import unavailable: Open CASCADE not found. Install opencascade:x64-windows and set OpenCASCADE_DIR / CMAKE_PREFIX_PATH. OBJ preview remains usable.")
        endif()
    endif()
endfunction()

if(DIGITAL_TWIN_CLI_ONLY)
    if(TWIN_OCCT_PREFIX)
        list(PREPEND CMAKE_PREFIX_PATH "${TWIN_OCCT_PREFIX}")
    endif()
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    mls_add_twin_core()
    # Regressions seen on Windows: MachineSafetyModel.cpp compiled against an
    # outdated/wrong collision_engine.h and lost the three STEP/FCL members.
    # This is a compile-time API contract test and uses minimal type-only
    # shims. The real GUI build continues to require genuine Eigen and FCL.
    add_executable(machine5x_collision_api_contract_test
        tests/safety/collision_engine_header_contract.cpp)
    target_include_directories(machine5x_collision_api_contract_test PRIVATE
        "${CMAKE_SOURCE_DIR}/tests/shims/collision_api_contract"
        "${CMAKE_SOURCE_DIR}/include")
    target_compile_features(machine5x_collision_api_contract_test PRIVATE cxx_std_17)
    add_test(NAME MACHINE5X_STEP_FCL_HEADER_API_CONTRACT
             COMMAND machine5x_collision_api_contract_test)
    # Standalone, Qt-free regression: Settings UI H3/100 must not be mistaken
    # for a stale H1/200 SyncManager cache; first G00 is a live motion command.
    add_executable(gcode_preflight_logic_tests tests/gcode/gcode_preflight_logic_tests.cpp)
    target_include_directories(gcode_preflight_logic_tests PRIVATE "${CMAKE_SOURCE_DIR}/include")
    target_compile_features(gcode_preflight_logic_tests PRIVATE cxx_std_17)
    add_test(NAME GCODE_PREPARED_SETTINGS_AND_LIVE_ENTRY_MOTION
             COMMAND gcode_preflight_logic_tests)
    add_executable(twin_g93_continuous_timing_tests tests/twin/twin_g93_continuous_timing_tests.cpp)
    target_compile_features(twin_g93_continuous_timing_tests PRIVATE cxx_std_17)
    target_link_libraries(twin_g93_continuous_timing_tests PRIVATE digital_twin_core)
    add_test(NAME NC_G93_CONTINUOUS_ACCELERATION_AWARE_PLANNER
             COMMAND twin_g93_continuous_timing_tests)
    add_executable(twin_g93_physical_audit_tests tests/twin/twin_g93_physical_audit_tests.cpp)
    target_compile_features(twin_g93_physical_audit_tests PRIVATE cxx_std_17)
    target_link_libraries(twin_g93_physical_audit_tests PRIVATE digital_twin_core)
    add_test(NAME NC_G93_PHYSICAL_DYNAMICS_BEFORE_HARDWARE
             COMMAND twin_g93_physical_audit_tests)
    add_executable(site_acceptance_policy_tests tests/gcode/site_acceptance_policy_tests.cpp)
    target_link_libraries(site_acceptance_policy_tests PRIVATE digital_twin_core)
    target_include_directories(site_acceptance_policy_tests PRIVATE "${CMAKE_SOURCE_DIR}/include")
    target_compile_features(site_acceptance_policy_tests PRIVATE cxx_std_17)
    add_test(NAME NC_SITE_ACCEPTANCE_REAL_PASS_AND_NEGATIVE_CASES COMMAND site_acceptance_policy_tests)
    add_executable(gcode_motion_completion_policy_tests
        tests/gcode/test_motion_completion_policy.cpp)
    target_include_directories(gcode_motion_completion_policy_tests PRIVATE "${CMAKE_SOURCE_DIR}/include")
    target_compile_features(gcode_motion_completion_policy_tests PRIVATE cxx_std_17)
    add_test(NAME GCODE_FIVE_AXIS_MPOS_COMPLETION_AND_WATCHDOG_POLICY
             COMMAND gcode_motion_completion_policy_tests)
    # Simulated Archive v5 round-trip and strict real-record provenance
    # are testable without Qt or Open CASCADE.
    # The uploaded V29 archive does not contain this historical test source.
    # Do not fail the entire offline build or report a fabricated pass.
    if(EXISTS "${CMAKE_SOURCE_DIR}/tests/twin/sensor_calibration_timing_tests.cpp")
        add_executable(sensor_calibration_timing_tests tests/twin/sensor_calibration_timing_tests.cpp)
        target_include_directories(sensor_calibration_timing_tests PRIVATE "${CMAKE_SOURCE_DIR}/include")
        target_compile_features(sensor_calibration_timing_tests PRIVATE cxx_std_17)
        add_test(NAME SENSOR_CALIBRATION_TIMING_UNITS_AND_RATE_CONSTRAINTS COMMAND sensor_calibration_timing_tests)
    else()
        message(WARNING "Uploaded V29 omits tests/twin/sensor_calibration_timing_tests.cpp: test skipped, NOT passed")
    endif()
    add_executable(twin_m6_simulated_archive_tests
        tests/twin/twin_m6_simulated_archive_tests.cpp
        src/runtime/ScanArchive5D.cpp
        src/runtime/Acquisition5D.cpp
        src/calibration/SingleSphere17Calibration.cpp)
    target_include_directories(twin_m6_simulated_archive_tests PRIVATE include)
    target_compile_features(twin_m6_simulated_archive_tests PRIVATE cxx_std_17)
    if(MSVC)
        target_compile_options(twin_m6_simulated_archive_tests PRIVATE /UNDEBUG)
    else()
        target_compile_options(twin_m6_simulated_archive_tests PRIVATE -UNDEBUG)
    endif()
    add_test(NAME V28_M6_SIMULATED_ARCHIVE_ROUNDTRIP COMMAND twin_m6_simulated_archive_tests)
    add_executable(twin_m34_v2_tests tests/twin/twin_m34_v2_tests.cpp)
    target_link_libraries(twin_m34_v2_tests PRIVATE digital_twin_core)
    if(MSVC)
        target_compile_options(twin_m34_v2_tests PRIVATE /UNDEBUG)
    else()
        target_compile_options(twin_m34_v2_tests PRIVATE -UNDEBUG)
    endif()
    add_test(NAME V28_M34_V2_PROBABILISTIC_MODEL COMMAND twin_m34_v2_tests)
    add_executable(twin_m34_v2_features_cli tools/twin_m34_v2_features_cli.cpp)
    target_link_libraries(twin_m34_v2_features_cli PRIVATE digital_twin_core)
    add_executable(twin_m0m5_optical_cli tools/twin_m0m5_optical_cli.cpp)
    target_link_libraries(twin_m0m5_optical_cli PRIVATE digital_twin_core)
    add_executable(twin_m0m5_multiview_cli tools/twin_m0m5_multiview_cli.cpp)
    target_link_libraries(twin_m0m5_multiview_cli PRIVATE digital_twin_core)
    add_executable(digital_twin_m3_cli tools/digital_twin_m3_cli.cpp)
    target_link_libraries(digital_twin_m3_cli PRIVATE digital_twin_core)
    add_executable(digital_twin_cli tools/digital_twin_cli.cpp)
    target_link_libraries(digital_twin_cli PRIVATE digital_twin_core)
    add_executable(twin_core_tests tests/twin/twin_core_tests.cpp)
    target_link_libraries(twin_core_tests PRIVATE digital_twin_core)
    add_executable(twin_reference_tests tests/twin/twin_reference_tests.cpp)
    target_link_libraries(twin_reference_tests PRIVATE digital_twin_core)
    add_executable(twin_m7_fusion_tests tests/twin/twin_m7_fusion_tests.cpp)
    target_link_libraries(twin_m7_fusion_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M7_COVERAGE_AND_PROVENANCE_FUSION COMMAND twin_m7_fusion_tests)
    add_executable(twin_multi_pose_registration_tests
        tests/twin/twin_multi_pose_registration_tests.cpp)
    target_link_libraries(twin_multi_pose_registration_tests PRIVATE digital_twin_core)
    add_test(NAME MULTIPOSE_REGISTRATION_PROPOSAL_OFFLINE_ONLY
             COMMAND twin_multi_pose_registration_tests)
    add_executable(twin_multi_pose_experiment_tests
        tests/twin/twin_multi_pose_experiment_tests.cpp)
    target_link_libraries(twin_multi_pose_experiment_tests PRIVATE digital_twin_core)
    target_include_directories(twin_multi_pose_experiment_tests PRIVATE include)
    add_test(NAME MULTIPOSE_EXPERIMENT_OFFLINE_GEOMETRY_OPTICS_H3_NC
             COMMAND twin_multi_pose_experiment_tests)
    # Reproduce the uploaded H3 SCAN first-frame Physical17 model and the
    # STEP-matched CAD reference mesh against the configured safe axis bounds.
    # Does NOT replace validation with real controller positions or FCL.
    add_executable(twin_real_h3_axis_bounds_regression
        tools/diagnose_real_h3_bounds.cpp
        src/calibration/SingleSphere17Calibration.cpp)
    target_link_libraries(twin_real_h3_axis_bounds_regression PRIVATE digital_twin_core)
    target_include_directories(twin_real_h3_axis_bounds_regression PRIVATE include)
    add_test(NAME MULTIPOSE_HISTORICAL_H3_BC45_AXIS_BOUND_REGRESSION
             COMMAND twin_real_h3_axis_bounds_regression)
    set_tests_properties(MULTIPOSE_HISTORICAL_H3_BC45_AXIS_BOUND_REGRESSION
                         PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    # New user experiment: 37 cases with independent 0/0 reference.
    # Uses the first profile's stored calibration snapshot from the later
    # 6061_Alumina_Scan_H2_New_01.scan (filename alone is not sync topology).
    add_executable(twin_newscan_bc37_axis_bounds
        tools/diagnose_newscan_bc37_bounds.cpp
        src/calibration/SingleSphere17Calibration.cpp)
    target_link_libraries(twin_newscan_bc37_axis_bounds PRIVATE digital_twin_core)
    target_include_directories(twin_newscan_bc37_axis_bounds PRIVATE include)
    add_test(NAME MULTIPOSE_NEW_SCAN_BC37_AXIS_BOUNDS_NO_FCL
             COMMAND twin_newscan_bc37_axis_bounds)
    set_tests_properties(MULTIPOSE_NEW_SCAN_BC37_AXIS_BOUNDS_NO_FCL
                         PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    add_executable(diagnose_nc_offline tools/diagnose_nc_offline.cpp)
    target_link_libraries(diagnose_nc_offline PRIVATE digital_twin_core)
    add_executable(twin_multi_pose_nc_simulation_tests tests/twin/twin_multi_pose_nc_simulation_tests.cpp)
    target_link_libraries(twin_multi_pose_nc_simulation_tests PRIVATE digital_twin_core)
    add_test(NAME MULTIPOSE_GCODE_NC_INTERPRETER_TIMING_TRIGGER_AND_TAMPER_REJECTION
             COMMAND twin_multi_pose_nc_simulation_tests)
    add_executable(twin_multi_pose_playback_compare_tests
        tests/twin/twin_multi_pose_playback_compare_tests.cpp)
    target_link_libraries(twin_multi_pose_playback_compare_tests PRIVATE digital_twin_core)
    target_include_directories(twin_multi_pose_playback_compare_tests PRIVATE include)
    add_test(NAME MULTIPOSE_OFFLINE_XYZCB_PLAYBACK_AND_REAL_SIM_GEOMETRY
             COMMAND twin_multi_pose_playback_compare_tests)
    add_executable(nc_g93_continuous_candidate_cli src/twin/NcG93ContinuousCandidateCli.cpp)
    target_compile_features(nc_g93_continuous_candidate_cli PRIVATE cxx_std_17)
    target_link_libraries(nc_g93_continuous_candidate_cli PRIVATE digital_twin_core)
    add_executable(nc_g93_retime_candidate_cli src/twin/NcG93RetimeCandidateCli.cpp)
    target_compile_features(nc_g93_retime_candidate_cli PRIVATE cxx_std_17)
    target_link_libraries(nc_g93_retime_candidate_cli PRIVATE digital_twin_core)
    add_executable(twin_m6_nc_preview_tests
        tests/twin/twin_m6_nc_preview_tests.cpp)
    target_link_libraries(twin_m6_nc_preview_tests PRIVATE digital_twin_core)
    add_test(NAME V29_M6_OFFLINE_NC_H1_H3_PREVIEW COMMAND twin_m6_nc_preview_tests)
    add_executable(twin_m8_workflow_tests tests/twin/twin_m8_workflow_tests.cpp)
    target_link_libraries(twin_m8_workflow_tests PRIVATE digital_twin_core)
    if(MSVC)
        target_compile_options(twin_m8_workflow_tests PRIVATE /UNDEBUG)
    else()
        target_compile_options(twin_m8_workflow_tests PRIVATE -UNDEBUG)
    endif()
    add_test(NAME V28_M8_WORKFLOW_AND_ACCEPTANCE COMMAND twin_m8_workflow_tests)
    add_executable(twin_m6_continuous_tests tests/twin/twin_m6_continuous_tests.cpp)
    target_link_libraries(twin_m6_continuous_tests PRIVATE digital_twin_core)
    if(MSVC)
        target_compile_options(twin_m6_continuous_tests PRIVATE /UNDEBUG)
    else()
        target_compile_options(twin_m6_continuous_tests PRIVATE -UNDEBUG)
    endif()
    add_test(NAME V28_M6_CONTINUOUS_5AXIS_OFFLINE COMMAND twin_m6_continuous_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj")
    enable_testing()
    add_executable(twin_replay_transform_tests tests/twin/twin_replay_transform_tests.cpp)
    target_link_libraries(twin_replay_transform_tests PRIVATE digital_twin_core)
    add_test(NAME V28_TWIN_REPLAY_TRANSFORM COMMAND twin_replay_transform_tests)
    add_executable(twin_virtual_laser_tests tests/twin/twin_virtual_laser_tests.cpp)
    target_link_libraries(twin_virtual_laser_tests PRIVATE digital_twin_core)
    add_executable(twin_m3_direction_tests tests/twin/twin_m3_direction_tests.cpp)
    add_executable(twin_m4_tests tests/twin/twin_m4_tests.cpp)
    add_executable(twin_m5_h1_real_tests tests/twin/twin_m5_h1_real_tests.cpp)
    target_link_libraries(twin_m5_h1_real_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M5_REAL_H1_100_PROFILE_REGRESSION COMMAND twin_m5_h1_real_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_real_100_profiles.bin")
    add_executable(twin_m51_h1_real_tests tests/twin/twin_m51_h1_real_tests.cpp)
    target_link_libraries(twin_m51_h1_real_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M51_REAL_H1_PROFILE_REGRESSION COMMAND twin_m51_h1_real_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_real_100_profiles.bin")
    add_executable(twin_m52_h1_real_tests tests/twin/twin_m52_h1_real_tests.cpp)
    target_link_libraries(twin_m52_h1_real_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M52_H1_REAL_100_PROFILE_REGRESSION COMMAND twin_m52_h1_real_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_real_100_profiles.bin")
    add_executable(twin_m52_tests tests/twin/twin_m52_tests.cpp)
    target_link_libraries(twin_m52_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M52_GEOMETRY_AND_HOLDOUT COMMAND twin_m52_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj")
    add_executable(twin_m51_tests tests/twin/twin_m51_tests.cpp)
    target_link_libraries(twin_m51_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M51_CENTER_CHAMFER_TEST COMMAND twin_m51_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj")
    add_executable(twin_m0m5_new_h1_tests tests/twin/twin_m0m5_new_h1_tests.cpp)
    target_link_libraries(twin_m0m5_new_h1_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M0_M5_NEW_CAD_GEOMETRY_AND_EXPOSURE
       COMMAND twin_m0m5_new_h1_tests
       "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
       "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_sensor_connector_NEW_2026.obj")
        add_executable(twin_m5_tests tests/twin/twin_m5_tests.cpp)
    target_link_libraries(twin_m5_tests PRIVATE digital_twin_core)
    add_test(NAME V28_M5_SURFACE_OCCUPANCY COMMAND twin_m5_tests)
    add_executable(twin_m4_h1_real_tests tests/twin/twin_m4_h1_real_tests.cpp)
    target_link_libraries(twin_m4_h1_real_tests PRIVATE digital_twin_core)
    target_link_libraries(twin_m4_tests PRIVATE digital_twin_core)
    target_link_libraries(twin_m3_direction_tests PRIVATE digital_twin_core)
    # All current twin unit tests use assert(); force active checks also in Release.
    foreach(tgt IN ITEMS twin_m0m5_new_h1_tests twin_core_tests twin_reference_tests twin_replay_transform_tests twin_virtual_laser_tests twin_m3_direction_tests twin_m4_tests twin_m4_h1_real_tests twin_m5_tests twin_m5_h1_real_tests twin_m51_tests twin_m51_h1_real_tests twin_m52_tests twin_m52_h1_real_tests)
        if(MSVC)
            target_compile_options(${tgt} PRIVATE /UNDEBUG)
        else()
            target_compile_options(${tgt} PRIVATE -UNDEBUG)
        endif()
    endforeach()
    add_test(NAME V28_TWIN_VIRTUAL_LASER COMMAND twin_virtual_laser_tests)
    add_test(NAME V28_M4_H1_REAL_100_PROFILE_REGRESSION COMMAND twin_m4_h1_real_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_real_100_profiles.bin")
    add_test(NAME V28_M4_PROFILE_COMPARISON_CALIBRATION COMMAND twin_m4_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv")
    add_test(NAME V28_M3_H1_DIRECTION_REGRESSION COMMAND twin_m3_direction_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv")
    add_test(NAME V28_TWIN_CORE_TESTS COMMAND twin_core_tests)
    add_test(NAME V28_TWIN_REFERENCE_GEOMETRY COMMAND twin_reference_tests
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj")
    add_test(NAME V28_M3_REFERENCE_SCENE COMMAND digital_twin_m3_cli
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj")
    add_test(NAME V28_M3_RECORDED_H1_POSE_SUBSET COMMAND digital_twin_m3_cli
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj"
             "${CMAKE_SOURCE_DIR}/tests/twin/data/h1_pose_subset.csv")
    add_test(NAME V28_REFERENCE_OBJ_IMPORT COMMAND digital_twin_cli --obj
             "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_reference_mesh.obj")
    if(OpenCASCADE_FOUND)
        add_test(NAME V28_REFERENCE_STEP_IMPORT COMMAND digital_twin_cli --step
                 "${CMAKE_SOURCE_DIR}/examples/digital_twin/6061_sensor_connector.step")
    endif()
    return()
endif()

# ---- 默认构建类型：Release ----
# Visual Studio 的 CMakeSettings.json 也只提供 x64-Release。
# 如果从命令行使用 Ninja 且没有显式传 -DCMAKE_BUILD_TYPE，则默认 Release。
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# P0 regression tests can build without Qt/VTK/Windows/vendor DLLs.
option(P0_TESTS_ONLY "Build only the portable P0 regression tests" OFF)
option(V19_TESTS_ONLY "Build only the portable V18/V19 regression tests" OFF)
option(SINGLE_SPHERE14_CLI_ONLY
       "Build only the portable single-sphere calibration/pose-planning CLI" OFF)
option(SINGLE_SPHERE17_CLI_ONLY
       "Build only the portable physical-17/rank-14 single-sphere CLI" OFF)
option(SPHERE_TWO_STAGE_QUALITY_CLI_ONLY "Portable two-stage sphere quality contract regression" OFF)
option(SPHERE_EXPOSURE_CLI_ONLY "Build portable whole-sphere exposure geometry regression" OFF)
option(EXPOSURE_ANALYSIS_CLI_ONLY
       "Build only the portable V27.6.43 exposure-analysis CLI" OFF)
option(V21_TESTS_ONLY "Build V21 portable numerical/runtime/mock tests" OFF)
option(V21_RUNTIME_CLI_ONLY "Build the portable V21 offline verifier" OFF)
if(V21_RUNTIME_CLI_ONLY)
    include(cmake/V21Runtime.cmake)
    return()
endif()
if(P0_TESTS_ONLY OR V19_TESTS_ONLY OR V21_TESTS_ONLY)
    include(cmake/V21Runtime.cmake)
    enable_testing()
    add_subdirectory(tests)
    return()
endif()

if(SINGLE_SPHERE14_CLI_ONLY)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_executable(single_sphere14_cli
        tools/single_sphere14_cli.cpp
        src/calibration/SingleSphere14Calibration.cpp
        include/calibration/SingleSphere14Calibration.h)
    target_include_directories(single_sphere14_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(single_sphere14_cli PRIVATE /utf-8 /Zc:__cplusplus)
    else()
        target_compile_options(single_sphere14_cli PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    return()
endif()

if(SINGLE_SPHERE17_CLI_ONLY)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_executable(single_sphere17_cli
        tools/single_sphere17_cli.cpp
        src/calibration/SingleSphere17Calibration.cpp
        include/calibration/SingleSphere17Calibration.h)
    target_include_directories(single_sphere17_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(single_sphere17_cli PRIVATE /utf-8 /Zc:__cplusplus)
    else()
        target_compile_options(single_sphere17_cli PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    return()
endif()

if(SPHERE_TWO_STAGE_QUALITY_CLI_ONLY)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_executable(sphere_two_stage_quality_cli
        tools/sphere_two_stage_quality_cli.cpp
        src/calibration/SphereExposureSweep.cpp
        src/calibration/SphereDistanceAngleStudy.cpp
        src/calibration/SingleSphere17Calibration.cpp)
    target_include_directories(sphere_two_stage_quality_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(sphere_two_stage_quality_cli PRIVATE /utf-8 /Zc:__cplusplus)
    endif()
    return()
endif()

if(SPHERE_EXPOSURE_CLI_ONLY)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_executable(sphere_exposure_cli
        tools/sphere_exposure_cli.cpp
        src/calibration/SphereExposureSweep.cpp)
    target_include_directories(sphere_exposure_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(sphere_exposure_cli PRIVATE /utf-8 /Zc:__cplusplus)
    else()
        target_compile_options(sphere_exposure_cli PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    return()
endif()

if(EXPOSURE_ANALYSIS_CLI_ONLY)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_executable(exposure_analysis_cli
        tools/exposure_analysis_cli.cpp
        src/calibration/ExposureAnalysisEngine.cpp
        include/calibration/ExposureAnalysisEngine.h)
    target_include_directories(exposure_analysis_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(exposure_analysis_cli PRIVATE /utf-8 /Zc:__cplusplus)
    else()
        target_compile_options(exposure_analysis_cli PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    return()
endif()

option(SMALL7_EXPERIMENT3_CLI_ONLY "Build the portable Small7 Experiment3 planner regression" OFF)
if(SMALL7_EXPERIMENT3_CLI_ONLY)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    enable_testing()
    add_executable(small7_experiment3_cli
        tools/small7_experiment3_cli.cpp
        src/calibration/Small7HeightPlan.cpp
        src/calibration/SingleSphere17Calibration.cpp)
    target_include_directories(small7_experiment3_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
    if(MSVC)
        target_compile_options(small7_experiment3_cli PRIVATE /utf-8 /Zc:__cplusplus)
    else()
        target_compile_options(small7_experiment3_cli PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME small7_experiment3_cli COMMAND small7_experiment3_cli)
    return()
endif()

# ---- C++ / Qt 自动处理 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

# ---- MSVC 特殊处理 ----
if(MSVC)
    # 修复 Qt6 与 MSVC 的 constexpr 兼容性问题
    add_compile_options(/Zc:__cplusplus)
    # 使用 UTF-8 源码编码
    add_compile_options(/utf-8)
    add_compile_definitions(NOMINMAX)
endif()

if(NOT WIN32 OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "The GUI uses bundled Windows x64 vendor SDKs. Use -DV21_TESTS_ONLY=ON for offline Linux tests.")
endif()

# ---- 找 Qt / VTK ----
# Keep the application's already-proven Qt/VTK installation untouched.
# The Machine5X package has a separate Release-only vcpkg tree and must not
# replace MotionLaserSync's Qt/VTK prefix (doing so can select a VTK build
# without GUISupportQt).
find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent OpenGLWidgets)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    FiltersCore
    FiltersSources
    FiltersGeneral
    IOPLY
    InteractionStyle
    RenderingCore
    RenderingOpenGL2
    RenderingAnnotation
    GUISupportQt
)
message(STATUS "VTK found: ${VTK_VERSION}")

# ---- Machine5X FCL digital-twin dependencies ----
# The user-provided collision project installs Eigen/FCL/nlohmann and their
# runtime dependencies in D:/m5x/vcpkg_installed/x64-windows-release.  Add
# that prefix only after Qt/VTK have been resolved, preserving the existing
# MotionLaserSync graphics stack.
set(M5X_VCPKG_INSTALLED_ROOT "D:/m5x/vcpkg_installed" CACHE PATH
    "Installed vcpkg root containing x64-windows-release for Machine5X/FCL")
set(M5X_VCPKG_PREFIX "${M5X_VCPKG_INSTALLED_ROOT}/x64-windows-release")
if(EXISTS "${M5X_VCPKG_PREFIX}")
    list(PREPEND CMAKE_PREFIX_PATH "${M5X_VCPKG_PREFIX}")
else()
    message(FATAL_ERROR "Machine5X dependency prefix not found: ${M5X_VCPKG_PREFIX}")
endif()
find_package(Eigen3 CONFIG REQUIRED)
find_package(fcl CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
if(TARGET fcl::fcl)
    set(M5X_FCL_TARGET fcl::fcl)
elseif(TARGET fcl)
    set(M5X_FCL_TARGET fcl)
else()
    message(FATAL_ERROR "FCL was found but no known CMake target exists.")
endif()

# V28 digital-twin geometry builds independently from controller and laser SDK.
# Optional OCCT prefix after Qt/VTK were found, so existing graphics selection remains stable.

if(TWIN_OCCT_PREFIX)
    list(PREPEND CMAKE_PREFIX_PATH "${TWIN_OCCT_PREFIX}")
endif()
mls_add_twin_core()

# ============ ZMC SDK ============
set(ZMC_ROOT "${CMAKE_SOURCE_DIR}/third_party/zmc")
set(ZMC_INC  "${ZMC_ROOT}/include")
set(ZMC_LIB  "${ZMC_ROOT}/lib/x64/zauxdll.lib")
set(ZMC_DLL1 "${ZMC_ROOT}/bin/x64/zauxdll.dll")
set(ZMC_DLL2 "${ZMC_ROOT}/bin/x64/zmotion.dll")

if(NOT EXISTS "${ZMC_INC}/zauxdll2.h")
  message(FATAL_ERROR "未找到 ZMC 头文件 zauxdll2.h: ${ZMC_INC}/zauxdll2.h")
endif()
if(NOT EXISTS "${ZMC_LIB}")
  message(FATAL_ERROR "未找到 ZMC 导入库: ${ZMC_LIB}（请确认是 x64 版本）")
endif()

# ============ scanCONTROL (LLT) SDK ============
set(SC_ROOT "${CMAKE_SOURCE_DIR}/third_party/scanCONTROL")
set(SC_INC  "${SC_ROOT}/include")
set(SC_LIB  "${SC_ROOT}/lib/x64/LLT.lib")
set(SC_DLL  "${SC_ROOT}/bin/x64/LLT.dll")

if(NOT EXISTS "${SC_INC}")
  message(FATAL_ERROR "未找到 scanCONTROL 头文件目录: ${SC_INC}")
endif()
if(NOT EXISTS "${SC_LIB}")
  message(FATAL_ERROR "未找到 LLT.lib (x64): ${SC_LIB}")
endif()

# ============ 源文件定义 ============

# Core 层 - 业务逻辑
set(CORE_SOURCES
    src/runtime/AxisCompensation.cpp
    src/core/ZmcAxisCompensation.cpp
    src/runtime/Rtcp17.cpp
    src/runtime/Acquisition5D.cpp
    src/runtime/HardwareTemporalCorrection.cpp
    src/runtime/ScanArchive5D.cpp
    src/core/HardwareSyncReport.cpp
    src/core/EciIoScheduler.cpp
    src/core/ZmcController.cpp
    src/core/LaserSensor.cpp
    src/core/SyncManager.cpp
    src/core/PointCloudBuilder.cpp
)

set(CORE_HEADERS
    include/runtime/AxisCompensation.h
    include/runtime/HardwareTemporalCorrection.h
    include/core/DataTypes.h
    include/core/ControllerIoGate.h
    include/core/ProfileCounter24.h
    include/core/ProfileSessionEpoch.h
    include/core/TableRingRead.h
    include/core/ProfileTimingAnalysis.h
    include/core/HardwareSyncReport.h
    include/core/EciIoScheduler.h
    include/core/ScopeExit.h
    include/core/ZmcController.h
    include/core/LaserSensor.h
    include/core/SyncManager.h
    include/core/PointCloudBuilder.h
)

# GCode 层 - G代码解析和执行
set(GCODE_SOURCES
    src/gcode/GCodeParser.cpp
    src/gcode/GCodeExecutor.cpp
)

set(GCODE_HEADERS
    include/gcode/GCodeTypes.h
    include/gcode/GCodeParser.h
    include/gcode/GCodeExecutor.h
)

# Algorithms 层 - 算法模块
set(ALGORITHM_SOURCES
    src/algorithms/CalibrationAlgorithms.cpp
    src/calibration/SingleSphere14Calibration.cpp
    src/calibration/SingleSphere17Calibration.cpp
    src/calibration/Small7HeightPlan.cpp
    src/calibration/SphereDistanceAngleStudy.cpp
    src/calibration/HardwareTemporalCalibration.cpp
    src/calibration/ExposureAnalysisEngine.cpp
    src/calibration/ExposureTimingPolicy.cpp
    src/calibration/MaterialExposureLibrary.cpp
    src/algorithms/PathPlanningAlgorithms.cpp
    src/algorithms/PointCloudProcessingAlgorithms.cpp
)

set(ALGORITHM_HEADERS
    include/algorithms/AlgorithmTypes.h
    include/calibration/SingleSphere14Calibration.h
    include/calibration/SingleSphere14QtAdapter.h
    include/calibration/SingleSphere17Calibration.h
    include/calibration/SphereDistanceAngleStudy.h
    include/calibration/Small7HeightPlan.h
    include/calibration/SphereTwoStageFit.h
    include/calibration/SphereArcQualityGate.h
    include/calibration/HardwareTemporalCalibration.h
    include/calibration/ExposureAnalysisEngine.h
    include/calibration/ExposureTimingPolicy.h
    include/calibration/MaterialExposureLibrary.h
    include/calibration/SingleSphere17QtAdapter.h
    include/algorithms/IAlgorithm.h
    include/algorithms/CalibrationAlgorithms.h
    include/algorithms/PathPlanningAlgorithms.h
    include/algorithms/PointCloudProcessingAlgorithms.h
)

# Widgets 层 - UI 组件
set(WIDGET_SOURCES
    src/widgets/Profile2DWidget.cpp
    src/widgets/ProfileImageWidget.cpp
    src/widgets/PointCloudWidget.cpp
    src/widgets/AxisControlWidget.cpp
    src/widgets/VideoWidget.cpp
    src/widgets/GCodeWidget.cpp
    src/widgets/LogWidget.cpp
    src/widgets/CalibrationWidget.cpp
    src/widgets/HardwareSyncCalibrationWidget.cpp
    src/widgets/SingleSphere14CalibrationWidget.cpp
    src/widgets/PathPlanningWidget.cpp
    src/widgets/DigitalTwinScanWidget.cpp
    src/widgets/MultiPoseExperimentWidget.cpp
    src/twin/MultiPoseExperimentQtAdapter.cpp
    src/twin/ScanReplayAdapter.cpp
    src/twin/TwinSimulationAdapter.cpp
    # M4 implementation object files belong to the final GUI target itself.
    # Keeps the GUI independent of which older digital_twin_core.lib VS cached.
    ${TWIN_M4_SOURCES}
    src/twin/TwinM4Adapter.cpp
    src/twin/TwinM5Adapter.cpp
    src/twin/TwinM51Adapter.cpp
    src/twin/TwinM52Adapter.cpp
    src/twin/TwinM6Adapter.cpp
    src/twin/TwinM6ScanArchive.cpp
    src/twin/TwinM7Adapter.cpp
    src/widgets/PointCloudProcessWidget.cpp
)

set(WIDGET_HEADERS
    include/widgets/Profile2DWidget.h
    include/widgets/ProfileImageWidget.h
    include/widgets/PointCloudWidget.h
    include/widgets/AxisControlWidget.h
    include/widgets/VideoWidget.h
    include/widgets/GCodeWidget.h
    include/widgets/LogWidget.h
    include/widgets/SaveFormatDialog.h
    include/widgets/CalibrationWidget.h
    include/widgets/HardwareSyncCalibrationWidget.h
    include/widgets/SingleSphere14CalibrationWidget.h
    include/widgets/PathPlanningWidget.h
    include/widgets/DigitalTwinScanWidget.h
    include/widgets/MultiPoseExperimentWidget.h
    include/twin/MultiPoseExperimentQtAdapter.h
    include/twin/ScanReplayAdapter.h
    include/twin/TwinSimulationAdapter.h
    ${TWIN_M4_HEADERS}
    include/twin/TwinM4Adapter.h
    include/twin/TwinM5Adapter.h
    include/twin/TwinM51Adapter.h
    include/twin/TwinM52Adapter.h
    include/twin/TwinM6Adapter.h
    include/twin/TwinM6ScanArchive.h
    include/twin/TwinM7Adapter.h
    include/widgets/PointCloudProcessWidget.h
)

# Utils 层 - 工具类
set(UTIL_SOURCES
    src/utils/GlobalTimeSync.cpp
    src/utils/CsvExporter.cpp
    src/utils/AsyncSaveManager.cpp
    src/utils/LogManager.cpp
    src/utils/CompactScanFormat.cpp
    src/utils/PointCloudLoader.cpp
)

set(UTIL_HEADERS
    include/utils/GlobalTimeSync.h
    include/utils/CsvExporter.h
    include/utils/AsyncSaveManager.h
    include/utils/LogManager.h
    include/utils/CompactScanFormat.h
    include/utils/PointCloudLoader.h
)

# Machine5X digital twin / collision safety (ported from the user-provided
# machine5x_vtk_fcl_outline_complete project; visual PLY and FCL collision STL
# share the same zero-pose MCS and exact XYZ-C-B link transforms).
set(MACHINE_SAFETY_SOURCES
    machine5x/machine_config.cpp
    machine5x/machine_kinematics.cpp
    machine5x/stl_mesh.cpp
    machine5x/collision_engine.cpp
    src/safety/MachineSafetyModel.cpp
)
set(MACHINE_SAFETY_HEADERS
    machine5x/machine_config.h
    machine5x/machine_kinematics.h
    machine5x/stl_mesh.h
    machine5x/collision_engine.h
    include/safety/MachineSafetyModel.h
)

# App 层 - 主程序
set(APP_SOURCES
    src/app/main.cpp
    src/app/MainWindow.cpp
    src/app/MainWindowSphereExposure.cpp
    src/app/MainWindowSmall7Experiment.cpp
    src/calibration/SphereExposureSweep.cpp
    src/app/MainWindowAxisCompensation.cpp
)

set(APP_HEADERS
    include/app/MainWindow.h
)

# M4 MSVC LNK2019 regression guard: the *GUI executable* must compile all
# three translation units, not only include declarations and Qt adapters.
foreach(_m4_required IN ITEMS
        src/twin/TwinM4Comparison.cpp
        src/twin/TwinM4Calibration.cpp
        src/twin/TwinM4Adapter.cpp)
    list(FIND WIDGET_SOURCES "${_m4_required}" _m4_found)
    if(_m4_found EQUAL -1 OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_m4_required}")
        message(FATAL_ERROR "M4 GUI missing implementation: ${_m4_required}")
    endif()
endforeach()
message(STATUS "V28.3.1 M4 GUI direct sources: Comparison + Calibration + Adapter [OK]")
# Verify that the portable M5 core and the Qt bridge reach the executable.
list(FIND WIDGET_SOURCES "src/twin/TwinM5Adapter.cpp" _m5_adapter_pos)
if(_m5_adapter_pos LESS 0)
    message(FATAL_ERROR "M5 GUI implementation missing: TwinM5Adapter.cpp")
endif()
list(FIND TWIN_CORE_SOURCES "src/twin/TwinM5Coverage.cpp" _m5_coverage_pos)
if(_m5_coverage_pos LESS 0)
    message(FATAL_ERROR "M5 portable CAD coverage source missing from digital_twin_core")
endif()
list(FIND TWIN_CORE_SOURCES "src/twin/TwinM8Workflow.cpp" _m8_core)
if(_m8_core EQUAL -1 OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/twin/twin_m8_workflow_tests.cpp")
    message(FATAL_ERROR "V28.9 M8 workflow and verification are missing")
endif()
message(STATUS "V28.9 M8 offline orchestration + M5/M6/M7 acceptance [registered]")
list(FIND TWIN_CORE_SOURCES "src/twin/TwinM7Fusion.cpp" _m7_core)
list(FIND WIDGET_SOURCES "src/twin/TwinM7Adapter.cpp" _m7_gui)
if(_m7_core LESS 0 OR _m7_gui LESS 0)
    message(FATAL_ERROR "V28.8 M7 CAD fusion core or Qt adapter is not registered")
endif()
message(STATUS "V28.8 M7 real/sim provenance fusion, CAD acceptance + GUI [OK]")
message(STATUS "V28.4 M5 CAD coverage: core + GUI + CSV report registered [OK]")
list(FIND WIDGET_SOURCES "src/twin/TwinM51Adapter.cpp" _m51_gui)
list(FIND TWIN_CORE_SOURCES "src/twin/TwinM51Chamfer.cpp" _m51_core)
if(_m51_gui LESS 0 OR _m51_core LESS 0)
    message(FATAL_ERROR "V28.5 M5.1 missing required implementation source")
endif()
message(STATUS "V28.5 M5.1 H1 center chamfer: core + GUI bridge registered [OK]")
list(FIND WIDGET_SOURCES "src/twin/TwinM52Adapter.cpp" _m52_gui)
list(FIND TWIN_CORE_SOURCES "src/twin/TwinM52Geometry.cpp" _m52_core)
if(_m52_gui LESS 0 OR _m52_core LESS 0)
    message(FATAL_ERROR "V28.6 M5.2 missing required core or Qt bridge source")
endif()
message(STATUS "V28.6.1 M5.2 geometric consistency + MSVC GUI syntax fix: core + GUI + JSON/CSV [OK]")


list(FIND TWIN_CORE_SOURCES "src/twin/TwinM6Continuous.cpp" _m6_core)
list(FIND WIDGET_SOURCES "src/twin/TwinM6Adapter.cpp" _m6_gui)
list(FIND WIDGET_SOURCES "src/twin/TwinM6ScanArchive.cpp" _m6_archive)
if(_m6_core LESS 0 OR _m6_gui LESS 0)
    message(FATAL_ERROR "V28.7 M6 continuous five-axis missing required implementation")
endif()
if(_m6_archive EQUAL -1)
    message(FATAL_ERROR "V28.7.3 M6 ScanArchive implementation not registered in GUI")
endif()
message(STATUS "V28.7.3 M6 streaming simulated SCAN v3 + segmented progress bars [OK]")

# ============ 可执行程序 ============
add_executable(MotionLaserSync WIN32
    ${APP_SOURCES}
    ${CORE_SOURCES}
    ${GCODE_SOURCES}
    ${ALGORITHM_SOURCES}
    ${WIDGET_SOURCES}
    ${UTIL_SOURCES}
    ${MACHINE_SAFETY_SOURCES}
    
    # 头文件（为了在 VS 里能看到）
    ${APP_HEADERS}
    ${CORE_HEADERS}
    ${GCODE_HEADERS}
    ${ALGORITHM_HEADERS}
    ${WIDGET_HEADERS}
    ${UTIL_HEADERS}
    ${MACHINE_SAFETY_HEADERS}
)
# V30 keeps the CMake target stable for existing linkage/deployment commands,
# but gives the Windows application a versioned executable filename.
set_target_properties(MotionLaserSync PROPERTIES OUTPUT_NAME "MotionLaserSync_V30")

# Portable offline calibration/pose-planning CLI. It intentionally has no Qt,
# VTK or vendor-SDK dependency and can be used to validate captured CSV data.
add_executable(single_sphere14_cli EXCLUDE_FROM_ALL
    tools/single_sphere14_cli.cpp
    src/calibration/SingleSphere14Calibration.cpp
    include/calibration/SingleSphere14Calibration.h
)
target_compile_features(single_sphere14_cli PRIVATE cxx_std_17)
target_include_directories(single_sphere14_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
if(MSVC)
    target_compile_options(single_sphere14_cli PRIVATE /utf-8 /Zc:__cplusplus)
endif()

add_executable(single_sphere17_cli EXCLUDE_FROM_ALL
    tools/single_sphere17_cli.cpp
    src/calibration/SingleSphere17Calibration.cpp
    include/calibration/SingleSphere17Calibration.h
)
target_compile_features(single_sphere17_cli PRIVATE cxx_std_17)
target_include_directories(single_sphere17_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
if(MSVC)
    target_compile_options(single_sphere17_cli PRIVATE /utf-8 /Zc:__cplusplus)
endif()

add_executable(exposure_analysis_cli EXCLUDE_FROM_ALL
    tools/exposure_analysis_cli.cpp
    src/calibration/ExposureAnalysisEngine.cpp
    include/calibration/ExposureAnalysisEngine.h
)
target_compile_features(exposure_analysis_cli PRIVATE cxx_std_17)
target_include_directories(exposure_analysis_cli PRIVATE "${CMAKE_SOURCE_DIR}/include")
if(MSVC)
    target_compile_options(exposure_analysis_cli PRIVATE /utf-8 /Zc:__cplusplus)
endif()

# Required out-of-line definitions for imported-sphere two-stage fitting.
# CMake must compile this file into MotionLaserSync, not merely include its .h.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/SphereTwoStageFit.cpp")
    message(FATAL_ERROR "SphereTwoStageFit.cpp is missing. Copy both .cpp and .h before building MotionLaserSync.")
endif()
target_sources(MotionLaserSync PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/SphereTwoStageFit.cpp"
)

# LLT SDK 自带的源码
target_sources(MotionLaserSync PRIVATE
    "${SC_ROOT}/source/InterfaceLLT_2.cpp"
    "${SC_ROOT}/source/DllLoader.cpp"
)

# 头文件路径：先加自己的 include，再加第三方
target_include_directories(MotionLaserSync PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/include/app"
    "${CMAKE_SOURCE_DIR}/include/core"
    "${CMAKE_SOURCE_DIR}/include/gcode"
    "${CMAKE_SOURCE_DIR}/include/algorithms"
    "${CMAKE_SOURCE_DIR}/include/widgets"
    "${CMAKE_SOURCE_DIR}/include/utils"
    "${CMAKE_SOURCE_DIR}/include/safety"
    "${CMAKE_SOURCE_DIR}/machine5x"
    "${ZMC_INC}"
    "${SC_INC}"
)

# ---- 链接 Qt + VTK + ZMC + scanCONTROL + Winsock + winmm ----
find_package(Threads REQUIRED)
target_link_libraries(MotionLaserSync PRIVATE
    Threads::Threads
    Qt6::Widgets
    Qt6::Concurrent
    Qt6::OpenGLWidgets
    ${VTK_LIBRARIES}
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    digital_twin_core
    ${M5X_FCL_TARGET}
    "${ZMC_LIB}"
    "${SC_LIB}"
    ws2_32
    winmm
)

# VTK 模块自动初始化
vtk_module_autoinit(
    TARGETS MotionLaserSync
    MODULES ${VTK_LIBRARIES}
)

# ---- 复制 ZMC / LLT 的 DLL 到输出目录 ----
if(EXISTS "${ZMC_DLL1}")
  add_custom_command(TARGET MotionLaserSync POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${ZMC_DLL1}"
              "$<TARGET_FILE_DIR:MotionLaserSync>/zauxdll.dll"
  )
else()
  message(WARNING "未发现 zauxdll.dll：${ZMC_DLL1}。运行时请确保 DLL 在 exe 同目录或 PATH 中")
endif()

if(EXISTS "${ZMC_DLL2}")
  add_custom_command(TARGET MotionLaserSync POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${ZMC_DLL2}"
              "$<TARGET_FILE_DIR:MotionLaserSync>/zmotion.dll"
  )
else()
  message(WARNING "未发现 zmotion.dll：${ZMC_DLL2}")
endif()

if(EXISTS "${SC_DLL}")
  add_custom_command(TARGET MotionLaserSync POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${SC_DLL}"
              "$<TARGET_FILE_DIR:MotionLaserSync>/LLT.dll"
  )
else()
  message(WARNING "未发现 LLT.dll：${SC_DLL}")
endif()

# ---- Qt Windows 平台插件：Release-only ----
# 本工程当前实机联调统一使用 Release，仅部署 Release 平台插件。
set(VCPKG_QT_ROOT "D:/dev/vcpkg/installed/x64-windows")
set(QT_QWINDOWS_PLUGIN
    "${VCPKG_QT_ROOT}/Qt6/plugins/platforms/qwindows.dll")

if(NOT EXISTS "${QT_QWINDOWS_PLUGIN}")
  message(FATAL_ERROR "未找到 Qt Release 平台插件 qwindows.dll: ${QT_QWINDOWS_PLUGIN}")
endif()

add_custom_command(TARGET MotionLaserSync POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:MotionLaserSync>/platforms"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${QT_QWINDOWS_PLUGIN}"
            "$<TARGET_FILE_DIR:MotionLaserSync>/platforms/"
)

# V27 运行时资源目录也继续复制到 exe 旁边。
add_custom_command(TARGET MotionLaserSync POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/controller"
            "$<TARGET_FILE_DIR:MotionLaserSync>/controller"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/examples"
            "$<TARGET_FILE_DIR:MotionLaserSync>/examples"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/config"
            "$<TARGET_FILE_DIR:MotionLaserSync>/config"
)

# Copy the exact Machine5X visual/collision model next to the application.
add_custom_command(TARGET MotionLaserSync POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:MotionLaserSync>/models/machine5x"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/models/machine5x"
            "$<TARGET_FILE_DIR:MotionLaserSync>/models/machine5x"
    COMMENT "Copying Machine5X PLY/FCL collision model"
)

# Copy runtime DLLs of FCL and its vcpkg dependencies.  Vendor SDK DLLs are
# still copied explicitly above.
add_custom_command(TARGET MotionLaserSync POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:MotionLaserSync>
            $<TARGET_FILE_DIR:MotionLaserSync>
    COMMAND_EXPAND_LISTS
)

# ---- 在 VS 中按文件夹分组 ----
source_group("Source Files\\app" FILES ${APP_SOURCES})
source_group("Source Files\\core" FILES ${CORE_SOURCES})
source_group("Source Files\\gcode" FILES ${GCODE_SOURCES})
source_group("Source Files\\algorithms" FILES ${ALGORITHM_SOURCES})
source_group("Source Files\\widgets" FILES ${WIDGET_SOURCES})
source_group("Source Files\\utils" FILES ${UTIL_SOURCES})
source_group("Source Files\\machine-safety" FILES ${MACHINE_SAFETY_SOURCES})
source_group("Source Files\\digital-twin" FILES ${TWIN_CORE_SOURCES})
source_group("Header Files\\digital-twin" FILES ${TWIN_CORE_HEADERS})
source_group("Header Files\\app" FILES ${APP_HEADERS})
source_group("Header Files\\core" FILES ${CORE_HEADERS})
source_group("Header Files\\gcode" FILES ${GCODE_HEADERS})
source_group("Header Files\\algorithms" FILES ${ALGORITHM_HEADERS})
source_group("Header Files\\widgets" FILES ${WIDGET_HEADERS})
source_group("Header Files\\utils" FILES ${UTIL_HEADERS})
source_group("Header Files\\machine-safety" FILES ${MACHINE_SAFETY_HEADERS})

# ---- 便于排查的提示 ----
message(STATUS "Qt6 Widgets     : OK")
message(STATUS "Qt6 Concurrent  : OK")
message(STATUS "Qt6 OpenGLWidgets: OK")
message(STATUS "VTK             : ${VTK_VERSION}")
message(STATUS "Machine5X deps  : ${M5X_VCPKG_PREFIX}")
message(STATUS "FCL target      : ${M5X_FCL_TARGET}")
message(STATUS "ZMC include     : ${ZMC_INC}")
message(STATUS "ZMC lib         : ${ZMC_LIB}")
message(STATUS "scanCONTROL inc : ${SC_INC}")
message(STATUS "scanCONTROL lib : ${SC_LIB}")
message(STATUS "Copy DLLs       : ZMC=${ZMC_DLL1};${ZMC_DLL2} | LLT=${SC_DLL}")
message(STATUS "Qt plugin       : ${QT_QWINDOWS_PLUGIN}")

# V27.6.36: ship the exact supplied calibration data beside the GUI executable.
add_custom_command(TARGET MotionLaserSync POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/calibration/runtime_xyz"
        "$<TARGET_FILE_DIR:MotionLaserSync>/calibration/runtime_xyz"
    COMMENT "Copying audited XYZ runtime compensation bundle (disabled by default)")

# V28.7.4: main/popup VTK overlay manager, per-feature M5 rescan diagnostics,
# balanced per-segment M6 preview and full XYZCB pose timeline.
message(STATUS "V28.7.4 M0-M6 independent layers + M5 rescan + M6 segment preview [registered]")
