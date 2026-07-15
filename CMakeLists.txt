cmake_minimum_required(VERSION 3.15)
project(NaviCore3D CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

set(NAVICORE_CORE_DIR ${CMAKE_SOURCE_DIR}/src/core)
set(NAVICORE_PC_DIR ${CMAKE_SOURCE_DIR}/src/targets/generic_pc)

include_directories(${NAVICORE_CORE_DIR} ${NAVICORE_PC_DIR})

set(NAVICORE_CORE_SOURCES
    ${NAVICORE_CORE_DIR}/NavState.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
    ${NAVICORE_CORE_DIR}/waypoint.cpp
    ${NAVICORE_CORE_DIR}/fusion.cpp
    ${NAVICORE_CORE_DIR}/guidance.cpp
    ${NAVICORE_CORE_DIR}/mission.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf.cpp
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
    ${NAVICORE_PC_DIR}/constant_slope_benchmark.cpp
    ${NAVICORE_PC_DIR}/regression_suite.cpp
    ${NAVICORE_PC_DIR}/power_state_machine.cpp
    ${NAVICORE_CORE_SOURCES}
)
if(WIN32)
    target_link_libraries(NaviCore3D_Sim PRIVATE ws2_32)
endif()

add_executable(navicore_regression_test
    ${NAVICORE_PC_DIR}/regression_tests.cpp
    ${NAVICORE_PC_DIR}/regression_suite.cpp
    ${NAVICORE_PC_DIR}/super_tunnel_benchmark.cpp
    ${NAVICORE_PC_DIR}/constant_slope_benchmark.cpp
    ${NAVICORE_PC_DIR}/sensors_sim.cpp
    ${NAVICORE_CORE_DIR}/NavState.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
    ${NAVICORE_CORE_DIR}/ins_ekf.cpp
)

# Host: test de estrés SPSC UART (campaña S7, ver docs/comarruga_lab_hardware.md)
set(NAVICORE_PICO2_HW_DIR ${CMAKE_SOURCE_DIR}/src/targets/pico2_hardware)

add_executable(ring_stress_test
    ${NAVICORE_PICO2_HW_DIR}/ring_stress_host_test.cpp
)
target_include_directories(ring_stress_test PRIVATE ${NAVICORE_PICO2_HW_DIR})
target_compile_options(ring_stress_test PRIVATE -O3)

# Pico 2 W (laboratorio físico Comarruga): build standalone — PICO_BOARD debe
# fijarse antes de project(). Ver src/targets/pico2_hardware/CMakeLists.txt
#   cmake -S src/targets/pico2_hardware -B build_pico2 -G Ninja
#   cmake --build build_pico2
