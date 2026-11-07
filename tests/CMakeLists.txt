find_package(Threads REQUIRED)

# Lightweight ABI regression for the H1/H3-only hardware-sync protocol.
add_executable(h1_h3_only_contract_tests h1_h3_only_contract_tests.cpp)
target_compile_features(h1_h3_only_contract_tests PRIVATE cxx_std_17)
target_include_directories(h1_h3_only_contract_tests PRIVATE ../include)
add_test(NAME h1_h3_only_contract COMMAND h1_h3_only_contract_tests)
set_tests_properties(h1_h3_only_contract PROPERTIES TIMEOUT 20)
add_executable(p0_unit_tests p0_unit_tests.cpp)
target_compile_features(p0_unit_tests PRIVATE cxx_std_17)
target_include_directories(p0_unit_tests PRIVATE ../include)
target_link_libraries(p0_unit_tests PRIVATE Threads::Threads)
add_test(NAME p0_portable_unit COMMAND p0_unit_tests)
set_tests_properties(p0_portable_unit PROPERTIES TIMEOUT 20)

# Compile the real modified core against the bundled SDK declarations. Qt and
# the SDK transport are test doubles; never added to the normal app target.
add_executable(p0_core_mock_tests
    p0_core_mock_tests.cpp MockZaux.cpp
    ../src/core/EciIoScheduler.cpp ../src/core/ZmcController.cpp ../src/core/ZmcAxisCompensation.cpp ../src/runtime/AxisCompensation.cpp ../src/core/SyncManager.cpp
    ../src/utils/GlobalTimeSync.cpp
    ../src/runtime/FiveAxis.cpp ../src/runtime/Acquisition5D.cpp ../src/runtime/HardwareTemporalCorrection.cpp
    ../src/calibration/SingleSphere17Calibration.cpp ../src/calibration/Small7HeightPlan.cpp)
target_compile_features(p0_core_mock_tests PRIVATE cxx_std_17)
target_compile_definitions(p0_core_mock_tests PRIVATE MLS_P0_TESTING)
if(NOT MSVC)
    target_compile_definitions(p0_core_mock_tests PRIVATE "__int64=long long" "__stdcall=")
endif()
target_include_directories(p0_core_mock_tests PRIVATE
    shims ../include ../third_party/zmc/include)
target_link_libraries(p0_core_mock_tests PRIVATE Threads::Threads)
add_test(NAME p0_real_core_mock_transport COMMAND p0_core_mock_tests)
set_tests_properties(p0_real_core_mock_transport PROPERTIES TIMEOUT 30)

add_executable(v19_scheduler_tests v19_scheduler_tests.cpp ../src/core/EciIoScheduler.cpp)
target_compile_features(v19_scheduler_tests PRIVATE cxx_std_17)
target_include_directories(v19_scheduler_tests PRIVATE ../include)
target_link_libraries(v19_scheduler_tests PRIVATE Threads::Threads)
add_test(NAME v19_scheduler COMMAND v19_scheduler_tests)
set_tests_properties(v19_scheduler PROPERTIES TIMEOUT 30)


add_executable(v20_single_sphere14_tests
    v20_single_sphere14_tests.cpp
    ../src/calibration/SingleSphere14Calibration.cpp)
target_compile_features(v20_single_sphere14_tests PRIVATE cxx_std_17)
target_include_directories(v20_single_sphere14_tests PRIVATE ../include)
add_test(NAME v20_single_sphere14 COMMAND v20_single_sphere14_tests)
set_tests_properties(v20_single_sphere14 PROPERTIES TIMEOUT 60)

add_executable(v20_2_single_sphere17_tests
    v20_2_single_sphere17_tests.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v20_2_single_sphere17_tests PRIVATE cxx_std_17)
target_include_directories(v20_2_single_sphere17_tests PRIVATE ../include)
add_test(NAME v20_2_single_sphere17 COMMAND v20_2_single_sphere17_tests)
set_tests_properties(v20_2_single_sphere17 PROPERTIES TIMEOUT 90)


add_executable(v27616v_sphere_geometry_quality_tests
    v27616v_sphere_geometry_quality_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616v_sphere_geometry_quality_tests PRIVATE cxx_std_17)
target_include_directories(v27616v_sphere_geometry_quality_tests PRIVATE ../include)
add_test(NAME v27616v_sphere_geometry_quality COMMAND v27616v_sphere_geometry_quality_tests)

add_executable(v27616y_center_bearing_spacefill_tests
    v27616y_center_bearing_spacefill_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616y_center_bearing_spacefill_tests PRIVATE cxx_std_17)
target_include_directories(v27616y_center_bearing_spacefill_tests PRIVATE ../include)
add_test(NAME v27616y_center_bearing_spacefill COMMAND v27616y_center_bearing_spacefill_tests)

add_executable(v27616z_optical_intersection_probe_tests
    v27616z_optical_intersection_probe_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616z_optical_intersection_probe_tests PRIVATE cxx_std_17)
target_include_directories(v27616z_optical_intersection_probe_tests PRIVATE ../include)
add_test(NAME v27616z_optical_intersection_probe COMMAND v27616z_optical_intersection_probe_tests)
set_tests_properties(v27616z_optical_intersection_probe PROPERTIES TIMEOUT 60)

add_executable(v27616ab_scan_sim_arc_replan_tests
    v27616ab_scan_sim_arc_replan_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616ab_scan_sim_arc_replan_tests PRIVATE cxx_std_17)
target_include_directories(v27616ab_scan_sim_arc_replan_tests PRIVATE ../include)
add_test(NAME v27616ab_scan_sim_arc_replan COMMAND v27616ab_scan_sim_arc_replan_tests)
set_tests_properties(v27616ab_scan_sim_arc_replan PROPERTIES TIMEOUT 60)

add_executable(v27616ad_offline_planner_guard_tests
    v27616ad_offline_planner_guard_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616ad_offline_planner_guard_tests PRIVATE cxx_std_17)
target_include_directories(v27616ad_offline_planner_guard_tests PRIVATE ../include)
add_test(NAME v27616ad_offline_planner_guard COMMAND v27616ad_offline_planner_guard_tests)
set_tests_properties(v27616ad_offline_planner_guard PROPERTIES TIMEOUT 20)

add_executable(v27616ae_trapezoid_end_effector_arc_tests
    v27616ae_trapezoid_end_effector_arc_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616ae_trapezoid_end_effector_arc_tests PRIVATE cxx_std_17)
target_include_directories(v27616ae_trapezoid_end_effector_arc_tests PRIVATE ../include)
add_test(NAME v27616ae_trapezoid_end_effector_arc COMMAND v27616ae_trapezoid_end_effector_arc_tests)
set_tests_properties(v27616ae_trapezoid_end_effector_arc PROPERTIES TIMEOUT 20)

add_executable(v27616af_flipped_z_fov_orientation_tests
    v27616af_flipped_z_fov_orientation_tests.cpp
    ../src/calibration/SphereDistanceAngleStudy.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27616af_flipped_z_fov_orientation_tests PRIVATE cxx_std_17)
target_include_directories(v27616af_flipped_z_fov_orientation_tests PRIVATE ../include)
add_test(NAME v27616af_flipped_z_fov_orientation COMMAND v27616af_flipped_z_fov_orientation_tests)
set_tests_properties(v27616af_flipped_z_fov_orientation PROPERTIES TIMEOUT 20)

set_tests_properties(v27616v_sphere_geometry_quality PROPERTIES TIMEOUT 60)

add_executable(v19_report_tests v19_report_tests.cpp ../src/core/HardwareSyncReport.cpp)
target_compile_features(v19_report_tests PRIVATE cxx_std_17)
target_include_directories(v19_report_tests PRIVATE ../include)
add_test(NAME v19_sync_report COMMAND v19_report_tests)
set_tests_properties(v19_sync_report PROPERTIES TIMEOUT 20)

add_executable(v27653_runtime_pwm_tests
    v27653_runtime_pwm_tests.cpp
    ../src/calibration/ExposureTimingPolicy.cpp)
target_compile_features(v27653_runtime_pwm_tests PRIVATE cxx_std_17)
target_include_directories(v27653_runtime_pwm_tests PRIVATE ../include)
add_test(NAME v27653_runtime_pwm COMMAND v27653_runtime_pwm_tests)
set_tests_properties(v27653_runtime_pwm PROPERTIES TIMEOUT 20)

add_executable(v27644_hardware_temporal_tests
    v27644_hardware_temporal_tests.cpp
    ../src/calibration/HardwareTemporalCalibration.cpp
    ../src/calibration/SingleSphere17Calibration.cpp
    ../src/runtime/Acquisition5D.cpp
    ../src/runtime/HardwareTemporalCorrection.cpp)
target_compile_features(v27644_hardware_temporal_tests PRIVATE cxx_std_17)
target_include_directories(v27644_hardware_temporal_tests PRIVATE ../include)
add_test(NAME v27644_hardware_temporal COMMAND v27644_hardware_temporal_tests)
set_tests_properties(v27644_hardware_temporal PROPERTIES TIMEOUT 20)

add_executable(h3_signed_temporal_tests
    h3_signed_temporal_tests.cpp
    ../src/calibration/HardwareTemporalCalibration.cpp
    ../src/calibration/SingleSphere17Calibration.cpp
    ../src/runtime/Acquisition5D.cpp
    ../src/runtime/HardwareTemporalCorrection.cpp)
target_compile_features(h3_signed_temporal_tests PRIVATE cxx_std_17)
target_include_directories(h3_signed_temporal_tests PRIVATE ../include)
add_test(NAME h3_signed_temporal COMMAND h3_signed_temporal_tests)
set_tests_properties(h3_signed_temporal PROPERTIES TIMEOUT 30)

add_executable(v27647_multispeed_temporal_tests
    v27647_multispeed_temporal_tests.cpp
    ../src/calibration/HardwareTemporalCalibration.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27647_multispeed_temporal_tests PRIVATE cxx_std_17)
target_include_directories(v27647_multispeed_temporal_tests PRIVATE ../include)
add_test(NAME v27647_multispeed_temporal COMMAND v27647_multispeed_temporal_tests)
set_tests_properties(v27647_multispeed_temporal PROPERTIES TIMEOUT 20)

add_executable(v27650_drift_radius_temporal_tests
    v27650_drift_radius_temporal_tests.cpp
    ../src/calibration/HardwareTemporalCalibration.cpp
    ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27650_drift_radius_temporal_tests PRIVATE cxx_std_17)
target_include_directories(v27650_drift_radius_temporal_tests PRIVATE ../include)
add_test(NAME v27650_drift_radius_temporal COMMAND v27650_drift_radius_temporal_tests)
set_tests_properties(v27650_drift_radius_temporal PROPERTIES TIMEOUT 20)

if(MSVC)
    foreach(t h1_h3_only_contract_tests p0_unit_tests p0_core_mock_tests v19_scheduler_tests v19_report_tests v20_single_sphere14_tests v20_2_single_sphere17_tests v27616v_sphere_geometry_quality_tests v27616y_center_bearing_spacefill_tests v27616z_optical_intersection_probe_tests v27616ab_scan_sim_arc_replan_tests v27616ad_offline_planner_guard_tests v27616ae_trapezoid_end_effector_arc_tests v27616af_flipped_z_fov_orientation_tests v27653_runtime_pwm_tests v27644_hardware_temporal_tests h3_signed_temporal_tests v27647_multispeed_temporal_tests v27650_drift_radius_temporal_tests)
        target_compile_options(${t} PRIVATE /utf-8 /Zc:__cplusplus)
    endforeach()
    # The core test stubs have no Win32 COM implementation. Keep real Windows
    # RS485 code in the application; portable tests select its non-Windows stub.
    target_compile_definitions(p0_core_mock_tests PRIVATE MLS_PORTABLE_TRANSPORT_TEST)
endif()

find_package(Python3 3.9 COMPONENTS Interpreter QUIET)
# Preserve missing historical test names as explicit CTest SKIP, not a fake PASS.
# The uploaded V27.6.34 archive references 21 scripts that it does not contain.
function(mls_add_source_contract name script)
    if(EXISTS "${script}")
        add_test(NAME ${name} COMMAND ${Python3_EXECUTABLE} "${script}")
    else()
        message(WARNING "Historical contract unavailable in source archive: ${script}")
        add_test(NAME ${name} COMMAND ${Python3_EXECUTABLE} -c
            "import sys; print('SKIP: historical verifier was not included in the source archive: ${script}'); sys.exit(77)")
        set_tests_properties(${name} PROPERTIES SKIP_RETURN_CODE 77)
    endif()
endfunction()
if(Python3_Interpreter_FOUND)
    add_test(NAME h3_temporal_extension_contract
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_h3_temporal_extension.py)
    set_tests_properties(h3_temporal_extension_contract PROPERTIES TIMEOUT 20)
    # Historical v54 verifier checks old magic/filenames and is NOT a v55 contract.
    mls_add_source_contract(v27655_pwm_home_safety_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27655_pwm_home_safety.py)
    set_tests_properties(v27655_pwm_home_safety_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616u_hw_sync_unlock_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616u_hw_sync_unlock.py)
    set_tests_properties(v27616u_hw_sync_unlock_contract PROPERTIES TIMEOUT 20)

    # V/W/X are historical source contracts.  Y extends the active study with
    # center-bearing eta and constrained D/gamma/eta/plane space filling.
    mls_add_source_contract(v27616y_center_bearing_spacefill_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616y_center_bearing_spacefill.py)
    set_tests_properties(v27616y_center_bearing_spacefill_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616z_optical_intersection_probe_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616z_optical_intersection_probe.py)
    set_tests_properties(v27616z_optical_intersection_probe_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616aa_machine_safety_digital_twin_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616aa_machine_safety_digital_twin.py)
    set_tests_properties(v27616aa_machine_safety_digital_twin_contract PROPERTIES TIMEOUT 30)

    mls_add_source_contract(v27616ab_scan_sim_arc_replan_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616ab_scan_sim_arc_replan.py)
    set_tests_properties(v27616ab_scan_sim_arc_replan_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616ac_offline_scan_sim_capture_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616ac_offline_scan_sim_capture.py)
    set_tests_properties(v27616ac_offline_scan_sim_capture_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616ad_offline_planner_hang_fix_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616ad_offline_planner_hang_fix.py)
    set_tests_properties(v27616ad_offline_planner_hang_fix_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616ae_trapezoid_end_effector_arc_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616ae_trapezoid_end_effector_arc.py)
    set_tests_properties(v27616ae_trapezoid_end_effector_arc_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616af_flipped_z_fov_orientation_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616af_flipped_z_fov_orientation.py)
    set_tests_properties(v27616af_flipped_z_fov_orientation_contract PROPERTIES TIMEOUT 20)

    # The V19 baseline contract deliberately required unchanged 3D BASIC.
    # Preserve that historical verifier, but V21 tests the new coordinated ABI.
    mls_add_source_contract(v27_source_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27.py)
    set_tests_properties(v27_source_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v20_gui_source_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v20_single_sphere_gui_integration.py)
    set_tests_properties(v20_gui_source_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v2765_two_stage_calibration_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v2765_two_stage_calibration.py)
    set_tests_properties(v2765_two_stage_calibration_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v2766_independent_validation_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v2766_independent_validation.py)
    set_tests_properties(v2766_independent_validation_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27612_calibration_persistence_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27612_calibration_persistence.py)
    set_tests_properties(v27612_calibration_persistence_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27613_prevalidation_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27613_prevalidation.py)
    set_tests_properties(v27613_prevalidation_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27614_precommission_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27614_precommission.py)
    set_tests_properties(v27614_precommission_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27615_offline_holdout_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27615_offline_holdout.py)
    set_tests_properties(v27615_offline_holdout_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616_hardware_preflight_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616_hardware_preflight.py)
    set_tests_properties(v27616_hardware_preflight_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616b_table_diagnostics_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27616b_table_diagnostics.py)
    set_tests_properties(v27616b_table_diagnostics_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616d_v322_hw_exec_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v32_2_hardware_execution.py)
    set_tests_properties(v27616d_v322_hw_exec_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27616e_v323_sensor_trigger_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v32_3_sensor_trigger.py)
    set_tests_properties(v27616e_v323_sensor_trigger_contract PROPERTIES TIMEOUT 20)

    # V27.6.31's exact-title/private-inline-interpolator assertions are historical.
    # Its original script is preserved; V27.6.35 checks the shared runtime engine
    # and RAW validation isolation, with numerical behavior tested in C++.
    mls_add_source_contract(v27644_hardware_temporal_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27644_hardware_temporal.py)
    set_tests_properties(v27644_hardware_temporal_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27645_material_exposure_log_path_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27645_material_exposure_log_path.py)
    set_tests_properties(v27645_material_exposure_log_path_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27646_hardware_temporal_workflow_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27646_hardware_temporal_workflow.py)
    set_tests_properties(v27646_hardware_temporal_workflow_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27647_multispeed_temporal_experiment_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27647_multispeed_temporal_experiment.py)
    set_tests_properties(v27647_multispeed_temporal_experiment_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27648_memory_safe_temporal_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27648_memory_safe_temporal.py)
    set_tests_properties(v27648_memory_safe_temporal_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27649_resumable_temporal_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27649_resumable_temporal.py)
    set_tests_properties(v27649_resumable_temporal_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27650_drift_radius_temporal_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27650_drift_radius_temporal.py)
    set_tests_properties(v27650_drift_radius_temporal_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27651_temporal_runtime_control_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27651_temporal_runtime_control.py)
    set_tests_properties(v27651_temporal_runtime_control_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27635_xyz_compensation_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27635_xyz_compensation.py)
    set_tests_properties(v27635_xyz_compensation_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27636_compensation_safety_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27636_compensation_safety.py)
    set_tests_properties(v27636_compensation_safety_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27637_stage2_gate_profile_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27637_stage2_gate_profile.py)
    set_tests_properties(v27637_stage2_gate_profile_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27638_stage2_workflow_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27638_stage2_workflow_fix.py)
    set_tests_properties(v27638_stage2_workflow_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27639_stage2_data_self_bootstrap_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27639_stage2_data_self_bootstrap.py)
    set_tests_properties(v27639_stage2_data_self_bootstrap_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27640_stage2_robust_pose_screening_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27640_stage2_robust_pose_screening.py)
    set_tests_properties(v27640_stage2_robust_pose_screening_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27641_stage2_progressive_trim_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27641_stage2_progressive_trim.py)
    set_tests_properties(v27641_stage2_progressive_trim_contract PROPERTIES TIMEOUT 20)

    mls_add_source_contract(v27642b_validation_after_csv_contract ${CMAKE_CURRENT_SOURCE_DIR}/../tools/verify_v27642b_validation_after_csv.py)
    set_tests_properties(v27642b_validation_after_csv_contract PROPERTIES TIMEOUT 20)
endif()

add_executable(v21_runtime_tests v21_runtime_tests.cpp)
target_link_libraries(v21_runtime_tests PRIVATE mls_runtime Threads::Threads)
add_test(NAME v21_runtime_numerical_archive COMMAND v21_runtime_tests)
set_tests_properties(v21_runtime_numerical_archive PROPERTIES TIMEOUT 40)
add_test(NAME v21_ray_sphere_end_to_end COMMAND v21_runtime_cli demo "${CMAKE_BINARY_DIR}/v21_demo")
set_tests_properties(v21_ray_sphere_end_to_end PROPERTIES TIMEOUT 30)

# Validate the deployed examples, including the files actually used by GUI templates.
foreach(example
    v21_5axis_motion_only v21_5axis_capture v21_g05_and_arcs
    v21_template_basic v21_template_multiregion v21_template_raster v21_template_continuous)
    add_test(NAME ${example}_parse_plan COMMAND v21_runtime_cli validate
        "${CMAKE_CURRENT_SOURCE_DIR}/../examples/${example}.nc")
    set_tests_properties(${example}_parse_plan PROPERTIES TIMEOUT 10)
endforeach()

add_executable(v22_rtcp_tests v22_rtcp_tests.cpp)
target_link_libraries(v22_rtcp_tests PRIVATE mls_runtime Threads::Threads)
add_test(NAME v22_rtcp17_taskspace COMMAND v22_rtcp_tests)
set_tests_properties(v22_rtcp17_taskspace PROPERTIES TIMEOUT 40)

add_test(NAME v22_rtcp_latch_simulation COMMAND v22_rtcp_latch_sim "${CMAKE_BINARY_DIR}/v22_rtcp_latch_summary.json")
set_tests_properties(v22_rtcp_latch_simulation PROPERTIES TIMEOUT 40)
foreach(example v22_rtcp_line_scan v22_rtcp_orientation_hold)
    add_test(NAME ${example}_parse COMMAND v21_runtime_cli validate
        "${CMAKE_CURRENT_SOURCE_DIR}/../examples/${example}.nc")
    set_tests_properties(${example}_parse PROPERTIES TIMEOUT 10)
endforeach()

# White-box profile-factorization tests compile the implementation in one TU.
add_executable(v27618_profile_block_tests v27618_profile_block_tests.cpp)
target_compile_features(v27618_profile_block_tests PRIVATE cxx_std_17)
target_include_directories(v27618_profile_block_tests PRIVATE ../include)
add_test(NAME v27618_profile_block_equivalence COMMAND v27618_profile_block_tests)
set_tests_properties(v27618_profile_block_equivalence PROPERTIES TIMEOUT 60)
add_executable(v27618_calibration_benchmark EXCLUDE_FROM_ALL
    ../tools/v27618_calibration_benchmark.cpp ../src/calibration/SingleSphere17Calibration.cpp)
target_compile_features(v27618_calibration_benchmark PRIVATE cxx_std_17)
target_compile_definitions(v27618_calibration_benchmark PRIVATE MLS_BENCH_FAST=1)
target_include_directories(v27618_calibration_benchmark PRIVATE ../include)

# Portable real-core transport test Qt include wrappers, absent from input archive.
set(MLS_TEST_QT_WRAPPERS "${CMAKE_CURRENT_BINARY_DIR}/qt_mock_includes")
file(MAKE_DIRECTORY "${MLS_TEST_QT_WRAPPERS}")
foreach(qh QObject QString QVector QMutex QMetaType QVector3D QMatrix4x4 QDebug
           QThread QElapsedTimer QStringList QTimer QCoreApplication QEvent QEventLoop QMetaObject QByteArray)
    file(WRITE "${MLS_TEST_QT_WRAPPERS}/${qh}" "#pragma once\n#include \"${CMAKE_CURRENT_SOURCE_DIR}/shims/QtShim.h\"\n")
endforeach()
target_include_directories(p0_core_mock_tests BEFORE PRIVATE "${MLS_TEST_QT_WRAPPERS}")

add_executable(v27635_axis_compensation_tests v27635_axis_compensation_tests.cpp)
target_link_libraries(v27635_axis_compensation_tests PRIVATE mls_runtime)
target_compile_definitions(v27635_axis_compensation_tests PRIVATE
    MLS_COMPENSATION_DATA_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../calibration/runtime_xyz")
add_test(NAME v27635_axis_compensation COMMAND v27635_axis_compensation_tests)
set_tests_properties(v27635_axis_compensation PROPERTIES TIMEOUT 30)

# V27.6.54 / V32.12 frequency state machine ACK regression
add_executable(v27654_runtime_pwm_tests v27654_runtime_pwm_tests.cpp ../src/calibration/ExposureTimingPolicy.cpp)
target_compile_features(v27654_runtime_pwm_tests PRIVATE cxx_std_17)
target_include_directories(v27654_runtime_pwm_tests PRIVATE ../include)
add_test(NAME v27654_runtime_pwm COMMAND v27654_runtime_pwm_tests)
set_tests_properties(v27654_runtime_pwm PROPERTIES TIMEOUT 20)
if(MSVC)
  target_compile_options(v27654_runtime_pwm_tests PRIVATE /utf-8 /Zc:__cplusplus)
endif()

# V27.6.55 / V32.13 active protocol and strict input validation checks.
add_executable(v27655_runtime_pwm_tests v27655_runtime_pwm_tests.cpp)
target_compile_features(v27655_runtime_pwm_tests PRIVATE cxx_std_17)
target_include_directories(v27655_runtime_pwm_tests PRIVATE ../include)
add_test(NAME v27655_runtime_pwm COMMAND v27655_runtime_pwm_tests)
set_tests_properties(v27655_runtime_pwm PROPERTIES TIMEOUT 20)
if(MSVC)
    target_compile_options(v27655_runtime_pwm_tests PRIVATE /utf-8 /Zc:__cplusplus)
endif()

# Imported .scan/.ply/.csv topology test: portable geometry, no Qt/VTK required.
add_executable(organized_cloud_mesh_tests organized_cloud_mesh_tests.cpp)
target_compile_features(organized_cloud_mesh_tests PRIVATE cxx_std_17)
target_include_directories(organized_cloud_mesh_tests PRIVATE ../include)
if(MSVC)
    target_compile_options(organized_cloud_mesh_tests PRIVATE /utf-8 /Zc:__cplusplus)
endif()
add_test(NAME organized_cloud_mesh_topology COMMAND organized_cloud_mesh_tests)
set_tests_properties(organized_cloud_mesh_topology PROPERTIES TIMEOUT 20)
