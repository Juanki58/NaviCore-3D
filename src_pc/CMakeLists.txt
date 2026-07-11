cmake_minimum_required(VERSION 3.15)
project(NaviCore3D_PC CXX)

# Forzar estándar C++17 (eficiente y moderno para Ambiq)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Evitar optimizaciones pesadas que hinchen el código en RAM
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# Incluir los directorios del proyecto
include_directories(core sensors fusion)

# Buscar archivos fuentes automáticamente
file(GLOB_RECURSE CORE_SOURCES "core/*.cpp" "core/*.h")
file(GLOB_RECURSE SENSOR_SOURCES "sensors/*.cpp" "sensors/*.h")
file(GLOB_RECURSE FUSION_SOURCES "fusion/*.cpp" "fusion/*.h")

# Ejecutable principal de simulación en PC
add_executable(NaviCore3D_Sim main.cpp ${CORE_SOURCES} ${SENSOR_SOURCES} ${FUSION_SOURCES})
