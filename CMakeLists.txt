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
)

add_executable(NaviCore3D_VehicleDemo
    ${NAVICORE_PC_DIR}/vehicle_bus_can_stub.cpp
    ${NAVICORE_CORE_SOURCES}
)
target_compile_definitions(NaviCore3D_VehicleDemo PRIVATE NAVICORE_VEHICLE_BUS_DEMO)

add_executable(NaviCore3D_Sim
    ${NAVICORE_PC_DIR}/main.cpp
    ${NAVICORE_PC_DIR}/telemetry_udp_sender.cpp
    ${NAVICORE_PC_DIR}/sensors_sim.cpp
    ${NAVICORE_PC_DIR}/power_state_machine.cpp
    ${NAVICORE_CORE_SOURCES}
)
if(WIN32)
    target_link_libraries(NaviCore3D_Sim PRIVATE ws2_32)
endif()

# Pico 2 W (laboratorio físico Comarruga): build standalone — PICO_BOARD debe
# fijarse antes de project(). Ver src/targets/pico2_hardware/CMakeLists.txt
#   cmake -S src/targets/pico2_hardware -B build_pico2 -G Ninja
#   cmake --build build_pico2
