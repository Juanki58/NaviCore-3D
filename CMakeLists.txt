cmake_minimum_required(VERSION 3.15)
project(NaviCore3D CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

set(NAVICORE_CORE_DIR ${CMAKE_SOURCE_DIR}/src/core)
set(NAVICORE_PC_DIR ${CMAKE_SOURCE_DIR}/src/targets/generic_pc)
set(NAVICORE_AMBIQ_DIR ${CMAKE_SOURCE_DIR}/src/targets/ambiq_apollo)

include_directories(${NAVICORE_CORE_DIR} ${NAVICORE_PC_DIR} ${NAVICORE_AMBIQ_DIR} ${NAVICORE_AMBIQ_DIR}/drivers)

set(NAVICORE_CORE_SOURCES
    ${NAVICORE_CORE_DIR}/NavState.cpp
    ${NAVICORE_CORE_DIR}/vector3d.cpp
    ${NAVICORE_CORE_DIR}/waypoint.cpp
    ${NAVICORE_CORE_DIR}/fusion.cpp
    ${NAVICORE_CORE_DIR}/guidance.cpp
    ${NAVICORE_CORE_DIR}/api_ingest.cpp
    ${NAVICORE_CORE_DIR}/vehicle_bus_adapter.cpp
)

add_executable(NaviCore3D_VehicleDemo
    ${NAVICORE_PC_DIR}/vehicle_bus_can_stub.cpp
    ${NAVICORE_CORE_SOURCES}
)
target_compile_definitions(NaviCore3D_VehicleDemo PRIVATE NAVICORE_VEHICLE_BUS_DEMO)

add_executable(NaviCore3D_Sim
    ${NAVICORE_PC_DIR}/main.cpp
    ${NAVICORE_PC_DIR}/sensors_sim.cpp
    ${NAVICORE_CORE_SOURCES}
)

set(NAVICORE_AMBIQ_DRIVER_SOURCES
    ${NAVICORE_AMBIQ_DIR}/drivers/ambiq_dma_stub.cpp
    ${NAVICORE_AMBIQ_DIR}/drivers/ambiq_spi_imu_stub.cpp
    ${NAVICORE_AMBIQ_DIR}/drivers/ambiq_gpio_gnss_stub.cpp
    ${NAVICORE_AMBIQ_DIR}/drivers/ambiq_uart_telemetry_stub.cpp
    ${NAVICORE_AMBIQ_DIR}/drivers/ambiq_power_monitor_stub.cpp
)

add_executable(NaviCore3D_Ambiq
    ${NAVICORE_AMBIQ_DIR}/main_ambiq.cpp
    ${NAVICORE_AMBIQ_DIR}/bsp_sensors.cpp
    ${NAVICORE_AMBIQ_DIR}/ambiq_system_stub.cpp
    ${NAVICORE_AMBIQ_DRIVER_SOURCES}
    ${NAVICORE_CORE_SOURCES}
)
