cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(NaviCore3D_PicoW CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

set(NAVICORE_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(NAVICORE_CORE_DIR ${NAVICORE_ROOT}/core)
set(NAVICORE_PC_DIR ${NAVICORE_ROOT}/targets/generic_pc)
set(NAVICORE_AMBIQ_DIR ${NAVICORE_ROOT}/targets/ambiq_apollo)
set(NAVICORE_PICO_DIR ${CMAKE_CURRENT_LIST_DIR})

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

add_executable(NaviCore3D_PicoW
    ${NAVICORE_PICO_DIR}/main.cpp
    ${NAVICORE_PICO_DIR}/pico_nav_tick.cpp
    ${NAVICORE_PICO_DIR}/bsp_sensors_stub.cpp
    ${NAVICORE_PICO_DIR}/telemetry_udp_lwip.cpp
    ${NAVICORE_PICO_DIR}/pico_system_stub.cpp
    ${NAVICORE_AMBIQ_DIR}/power_state_machine.cpp
    ${NAVICORE_CORE_SOURCES}
)

target_include_directories(NaviCore3D_PicoW PRIVATE
    ${NAVICORE_CORE_DIR}
    ${NAVICORE_PC_DIR}
    ${NAVICORE_AMBIQ_DIR}
    ${NAVICORE_AMBIQ_DIR}/drivers
    ${NAVICORE_PICO_DIR}
)

target_compile_definitions(NaviCore3D_PicoW PRIVATE
    NAVICORE_TARGET_PICO
    PICO_CYW43_ARCH_POLL=1
)

target_link_libraries(NaviCore3D_PicoW
    pico_stdlib
    pico_multicore
    pico_cyw43_arch_lwip_poll
    hardware_spi
    hardware_uart
)

pico_enable_stdio_usb(NaviCore3D_PicoW 1)
pico_add_extra_outputs(NaviCore3D_PicoW)
