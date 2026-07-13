# Override PICO_SDK_PATH if not set
if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message(STATUS "Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR
        "PICO_SDK_PATH no definido. Clona el Pico SDK y exporta la variable, p. ej.:\n"
        "  git clone https://github.com/raspberrypi/pico-sdk.git\n"
        "  cd pico-sdk && git submodule update --init\n"
        "  $env:PICO_SDK_PATH = 'C:/pico/pico-sdk'")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
