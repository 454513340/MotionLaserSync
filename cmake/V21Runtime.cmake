# Portable code shared by the offline CLI and regression tests. No SDK transport.
add_library(mls_runtime STATIC
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/AxisCompensation.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/FiveAxis.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/Rtcp17.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/GCode5D.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/Acquisition5D.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/HardwareTemporalCorrection.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/runtime/ScanArchive5D.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/calibration/SingleSphere17Calibration.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/calibration/SphereDistanceAngleStudy.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../src/calibration/HardwareTemporalCalibration.cpp")
target_include_directories(mls_runtime PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../include")
target_compile_features(mls_runtime PUBLIC cxx_std_17)
if(MSVC)
    target_compile_options(mls_runtime PRIVATE /utf-8 /Zc:__cplusplus)
else()
    target_compile_options(mls_runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_executable(v21_runtime_cli "${CMAKE_CURRENT_LIST_DIR}/../tools/v21_runtime_cli.cpp")
target_link_libraries(v21_runtime_cli PRIVATE mls_runtime)

add_executable(v22_rtcp_latch_sim "${CMAKE_CURRENT_LIST_DIR}/../tools/v22_rtcp_latch_sim.cpp")
target_link_libraries(v22_rtcp_latch_sim PRIVATE mls_runtime)
