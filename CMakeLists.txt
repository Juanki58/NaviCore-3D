cmake_minimum_required(VERSION 3.15)
project(NaviCore3D CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# Optional: -DNAVICORE_ENABLE_SANITIZERS=ON / -DNAVICORE_ENABLE_COVERAGE=ON
include(${CMAKE_SOURCE_DIR}/cmake/NaviCoreAudit.cmake)

# Export compile_commands.json for clang-tidy when generator supports it
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(NAVICORE_CORE_DIR ${CMAKE_SOURCE_DIR}/src/core)
set(NAVICORE_PC_DIR ${CMAKE_SOURCE_DIR}/src/targets/generic_pc)
set(NAVICORE_SCENARIOS_DIR ${CMAKE_SOURCE_DIR}/src/scenarios)

include_directories(
    ${NAVICORE_CORE_DIR}
    ${NAVICORE_CORE_DIR}/interfaces
    ${NAVICORE_PC_DIR}
    ${NAVICORE_SCENARIOS_DIR})

set(NAVICORE_CORE_SOURCES
    ${NAVICORE_CORE_DIR}/NavState.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
    ${NAVICORE_CORE_DIR}/waypoint.cpp
    ${NAVICORE_CORE_DIR}/fusion.cpp
    ${NAVICORE_CORE_DIR}/guidance.cpp
    ${NAVICORE_CORE_DIR}/mission.cpp
    ${NAVICORE_CORE_DIR}/geodesy.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf_math.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf_15_state.cpp
    ${NAVICORE_CORE_DIR}/flight_recorder.cpp
    ${NAVICORE_CORE_DIR}/diagnostic.cpp
    ${NAVICORE_CORE_DIR}/api_ingest.cpp
    ${NAVICORE_CORE_DIR}/vehicle_bus_adapter.cpp
    ${NAVICORE_CORE_DIR}/command_ingestor.cpp
    ${NAVICORE_CORE_DIR}/time_guard.cpp
    ${NAVICORE_CORE_DIR}/slip_compensation.cpp
    ${NAVICORE_CORE_DIR}/geometry_guard.cpp
    ${NAVICORE_CORE_DIR}/divergence_guard.cpp
    ${NAVICORE_CORE_DIR}/recovery_guard.cpp
    ${NAVICORE_CORE_DIR}/navigation_cortex.cpp
    ${NAVICORE_CORE_DIR}/pid.cpp
    ${NAVICORE_CORE_DIR}/nmea_parser.cpp
    ${NAVICORE_CORE_DIR}/ubx_parser.cpp
    ${NAVICORE_CORE_DIR}/wt61c_parser.cpp
    ${NAVICORE_CORE_DIR}/health_policy.cpp
    ${NAVICORE_CORE_DIR}/nav_mode_policy.cpp
    ${NAVICORE_CORE_DIR}/imu_cross_check.cpp
)

add_executable(NaviCore3D_VehicleDemo
    ${NAVICORE_PC_DIR}/vehicle_bus_can_stub.cpp
    ${NAVICORE_CORE_SOURCES}
)
target_compile_definitions(NaviCore3D_VehicleDemo PRIVATE NAVICORE_VEHICLE_BUS_DEMO)

add_executable(NaviCore3D_Sim
    ${NAVICORE_PC_DIR}/main.cpp
    ${NAVICORE_CORE_DIR}/telemetry_interface.cpp
    ${NAVICORE_PC_DIR}/sensors_sim.cpp
    ${NAVICORE_PC_DIR}/inertial_replay.cpp
    ${NAVICORE_PC_DIR}/super_tunnel_benchmark.cpp
    ${NAVICORE_SCENARIOS_DIR}/tunnel_stress.cpp
    ${NAVICORE_SCENARIOS_DIR}/slalom_scenario.cpp
    ${NAVICORE_PC_DIR}/constant_slope_benchmark.cpp
    ${NAVICORE_PC_DIR}/slalom_benchmark.cpp
    ${NAVICORE_PC_DIR}/regression_suite.cpp
    ${NAVICORE_PC_DIR}/power_state_machine.cpp
    ${NAVICORE_PC_DIR}/telemetry_udp.cpp
    ${NAVICORE_PC_DIR}/telemetry_file_logger.cpp
    ${NAVICORE_CORE_SOURCES}
)
if(WIN32)
    target_link_libraries(NaviCore3D_Sim PRIVATE ws2_32)
endif()

set(NAVICORE_REPLAY_SOURCES
    ${NAVICORE_CORE_DIR}/NavState.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
    ${NAVICORE_CORE_DIR}/geodesy.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf_math.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf_15_state.cpp
    ${NAVICORE_CORE_DIR}/nav_mode_policy.cpp
)

add_executable(NaviCore3D_Replay
    ${NAVICORE_PC_DIR}/replay_main.cpp
    ${NAVICORE_PC_DIR}/real_run_replay.cpp
    ${NAVICORE_PC_DIR}/adaptive_nhc_controller.cpp
    ${NAVICORE_REPLAY_SOURCES}
)

add_executable(navicore_regression_test
    ${NAVICORE_PC_DIR}/regression_tests.cpp
    ${NAVICORE_PC_DIR}/regression_suite.cpp
    ${NAVICORE_PC_DIR}/super_tunnel_benchmark.cpp
    ${NAVICORE_PC_DIR}/constant_slope_benchmark.cpp
    ${NAVICORE_PC_DIR}/slalom_benchmark.cpp
    ${NAVICORE_PC_DIR}/sensors_sim.cpp
    ${NAVICORE_CORE_DIR}/NavState.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
    ${NAVICORE_CORE_DIR}/geodesy.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf_math.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf_15_state.cpp
    ${NAVICORE_CORE_DIR}/waypoint.cpp
    ${NAVICORE_CORE_DIR}/diagnostic.cpp
    ${NAVICORE_CORE_DIR}/time_guard.cpp
    ${NAVICORE_CORE_DIR}/geometry_guard.cpp
    ${NAVICORE_CORE_DIR}/command_ingestor.cpp
    ${NAVICORE_CORE_DIR}/fusion.cpp
    ${NAVICORE_CORE_DIR}/divergence_guard.cpp
    ${NAVICORE_CORE_DIR}/nmea_parser.cpp
    ${NAVICORE_CORE_DIR}/ubx_parser.cpp
    ${NAVICORE_CORE_DIR}/wt61c_parser.cpp
    ${NAVICORE_CORE_DIR}/health_policy.cpp
    ${NAVICORE_CORE_DIR}/nav_mode_policy.cpp
    ${NAVICORE_CORE_DIR}/imu_cross_check.cpp
)

# Host: test de estrés SPSC UART (campaña S7, ver docs/comarruga_lab_hardware.md)
set(NAVICORE_PICO2_HW_DIR ${CMAKE_SOURCE_DIR}/src/targets/pico2_hardware)

add_executable(ring_stress_test
    ${NAVICORE_PICO2_HW_DIR}/ring_stress_host_test.cpp
)
target_include_directories(ring_stress_test PRIVATE ${NAVICORE_PICO2_HW_DIR})
target_compile_options(ring_stress_test PRIVATE -O3)

navicore_apply_host_audit_flags(NaviCore3D_VehicleDemo)
navicore_apply_host_audit_flags(NaviCore3D_Sim)
navicore_apply_host_audit_flags(NaviCore3D_Replay)
navicore_apply_host_audit_flags(navicore_regression_test)
navicore_apply_host_audit_flags(ring_stress_test)

# --- Catch2 + RapidCheck unit / property tests ---
option(NAVICORE_BUILD_UNIT_TESTS "Build Catch2 + RapidCheck tests (navicore_unit_tests)" ON)
if(NAVICORE_BUILD_UNIT_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
    )
    FetchContent_MakeAvailable(Catch2)

    set(RC_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
    set(RC_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(RC_ENABLE_CATCH OFF CACHE BOOL "" FORCE)
    set(RC_ENABLE_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        rapidcheck
        GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
        GIT_TAG master
    )
    FetchContent_MakeAvailable(rapidcheck)

    add_executable(navicore_unit_tests
        ${CMAKE_SOURCE_DIR}/tests/unit/test_navstate_math.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_ins_ekf_math.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_fusion_isolated.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_properties_rapidcheck.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_sensor_wire_and_health.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_nav_mode_transitions.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_ins_ekf_edge.cpp
        ${CMAKE_SOURCE_DIR}/tests/unit/test_nhc_ops_policy.cpp
        ${NAVICORE_CORE_DIR}/NavState.cpp
        ${NAVICORE_CORE_DIR}/vector3d.cpp
        ${NAVICORE_CORE_DIR}/ins_ekf_math.cpp
        ${NAVICORE_CORE_DIR}/ins_ekf.cpp
        ${NAVICORE_CORE_DIR}/ins_ekf_15_state.cpp
        ${NAVICORE_CORE_DIR}/geodesy.cpp
        ${NAVICORE_CORE_DIR}/fusion.cpp
        ${NAVICORE_CORE_DIR}/divergence_guard.cpp
        ${NAVICORE_CORE_DIR}/diagnostic.cpp
        ${NAVICORE_CORE_DIR}/nmea_parser.cpp
        ${NAVICORE_CORE_DIR}/ubx_parser.cpp
        ${NAVICORE_CORE_DIR}/wt61c_parser.cpp
        ${NAVICORE_CORE_DIR}/health_policy.cpp
        ${NAVICORE_CORE_DIR}/nav_mode_policy.cpp
        ${NAVICORE_CORE_DIR}/imu_cross_check.cpp
    )
    target_link_libraries(navicore_unit_tests PRIVATE Catch2::Catch2WithMain rapidcheck)
    target_include_directories(navicore_unit_tests PRIVATE
        ${NAVICORE_CORE_DIR}
        ${NAVICORE_CORE_DIR}/interfaces)
    navicore_apply_host_audit_flags(navicore_unit_tests)

    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
    include(CTest)
    include(Catch)
    catch_discover_tests(navicore_unit_tests)
endif()

# --- Sensor wire fuzzers (libFuzzer on Clang; standalone corpus runner otherwise) ---
option(NAVICORE_BUILD_FUZZERS "Build NMEA/UBX/WT61C fuzz targets" OFF)
set(NAVICORE_FUZZ_SOURCES
    ${CMAKE_SOURCE_DIR}/tests/fuzz/sensor_wire_fuzz.cpp
    ${NAVICORE_CORE_DIR}/nmea_parser.cpp
    ${NAVICORE_CORE_DIR}/ubx_parser.cpp
    ${NAVICORE_CORE_DIR}/wt61c_parser.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
)
if(NAVICORE_BUILD_FUZZERS)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-fsanitize=fuzzer" NAVICORE_HAS_LIBFUZZER)
    if(NAVICORE_HAS_LIBFUZZER)
        add_executable(navicore_sensor_wire_fuzz ${NAVICORE_FUZZ_SOURCES})
        target_include_directories(navicore_sensor_wire_fuzz PRIVATE
            ${NAVICORE_CORE_DIR}
            ${NAVICORE_CORE_DIR}/interfaces)
        target_compile_options(navicore_sensor_wire_fuzz PRIVATE
            -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer -g)
        target_link_options(navicore_sensor_wire_fuzz PRIVATE
            -fsanitize=fuzzer,address,undefined)
    endif()

    # AFL / corpus smoke: same harness with a file/stdin main (no libFuzzer runtime)
    add_executable(navicore_sensor_wire_fuzz_standalone ${NAVICORE_FUZZ_SOURCES})
    target_include_directories(navicore_sensor_wire_fuzz_standalone PRIVATE
        ${NAVICORE_CORE_DIR}
        ${NAVICORE_CORE_DIR}/interfaces)
    target_compile_definitions(navicore_sensor_wire_fuzz_standalone PRIVATE
        NAVICORE_FUZZ_STANDALONE=1)
endif()

# Pico 2 W (laboratorio físico Comarruga): build standalone — PICO_BOARD debe
# fijarse antes de project(). Ver src/targets/pico2_hardware/CMakeLists.txt
#   cmake -S src/targets/pico2_hardware -B build_pico2 -G Ninja
#   cmake --build build_pico2
