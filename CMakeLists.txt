# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/danieljin/Desktop/Pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MPUControl C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(MPUControl MPUControl.c )

pico_set_program_name(MPUControl "MPUControl")
pico_set_program_version(MPUControl "0.1")

pico_enable_stdio_uart(MPUControl 1)
pico_enable_stdio_usb(MPUControl 0)

# Add the standard library to the build
target_link_libraries(MPUControl pico_stdlib)

# Add any user requested libraries
target_link_libraries(MPUControl
        hardware_i2c
        )

pico_add_extra_outputs(MPUControl)

