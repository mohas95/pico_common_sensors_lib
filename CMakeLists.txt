# ====================================================================================
# CMake Pico project file for pico_common_sensors_lib (dual-mode: library + optional demo)
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fetch Pico SDK automatically if not already provided
set(PICO_SDK_FETCH_FROM_GIT ON)
set(PICOTOOL_FETCH_FROM_GIT_PATH "${CMAKE_SOURCE_DIR}/.pico-deps" CACHE PATH "Shared picotool path")

# Import Pico SDK
include(pico_sdk_import.cmake)

project(pico_common_sensors_lib C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ====================================================================================
# Library definition
# ====================================================================================

add_library(pico_common_sensors_lib
        # src/pico_common_sensors_lib.cpp
)

target_include_directories(pico_common_sensors_lib PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico_common_sensors_lib
        pico_stdlib
        hardware_i2c
        hardware_adc
        )

# ====================================================================================
# Standalone executable (only builds if this repo is the root project)
# ====================================================================================
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    add_executable(pico_common_sensors_lib_standalone
        src/main.cpp
    )

    target_link_libraries(pico_common_sensors_lib_standalone pico_common_sensors_lib)

    pico_set_program_name(pico_common_sensors_lib_standalone "pico_common_sensors_lib_standalone")
    pico_set_program_version(pico_common_sensors_lib_standalone "0.1")

    # Enable USB stdio, disable UART
    pico_enable_stdio_uart(pico_common_sensors_lib_standalone 0)
    pico_enable_stdio_usb(pico_common_sensors_lib_standalone 1)

    pico_add_extra_outputs(pico_common_sensors_lib_standalone)
endif()
