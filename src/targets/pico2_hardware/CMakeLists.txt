cmake_minimum_required(VERSION 3.13)

# Configuración del Target para el laboratorio físico de Comarruga
set(PICO_BOARD pico2_w)
include(pico_sdk_import.cmake)
project(NaviCore3D C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Peor caso de reordenación del compilador para validar rings SPSC.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

pico_sdk_init()

set(NAVICORE_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(NAVICORE_CORE_DIR ${NAVICORE_ROOT}/core)
set(NAVICORE_PICO2_DIR ${CMAKE_CURRENT_LIST_DIR})

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
)

add_executable(NaviCore3D_Pico2
    ${NAVICORE_PICO2_DIR}/main.cpp
    ${NAVICORE_PICO2_DIR}/safe_log.cpp
    ${NAVICORE_PICO2_DIR}/loop_metrics.cpp
    ${NAVICORE_PICO2_DIR}/health_monitor.cpp
    ${NAVICORE_PICO2_DIR}/task_monitor.cpp
    ${NAVICORE_PICO2_DIR}/bsp_sensors.cpp
    ${NAVICORE_PICO2_DIR}/bsp_wt61c.cpp
    ${NAVICORE_PICO2_DIR}/bsp_gnss.cpp
    ${NAVICORE_PICO2_DIR}/bsp_power.cpp
    ${NAVICORE_CORE_SOURCES}
)

target_include_directories(NaviCore3D_Pico2 PRIVATE
    ${NAVICORE_ROOT}
    ${NAVICORE_PICO2_DIR}
)

target_compile_definitions(NaviCore3D_Pico2 PRIVATE
    NAVICORE_TARGET_PICO2
    PICO_CYW43_ARCH_POLL=1
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
)

target_compile_options(NaviCore3D_Pico2 PRIVATE -O3)

target_link_libraries(NaviCore3D_Pico2
    pico_stdlib
    pico_stdio_usb
    pico_cyw43_arch_lwip_poll
    hardware_uart
    hardware_i2c
    hardware_watchdog
)

pico_enable_stdio_usb(NaviCore3D_Pico2 1)
pico_enable_stdio_uart(NaviCore3D_Pico2 0)
pico_add_extra_outputs(NaviCore3D_Pico2)
