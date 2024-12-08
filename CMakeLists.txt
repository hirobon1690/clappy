# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(tinyusb C CXX ASM)
# Set FAMILY variable for the build
set(FAMILY rp2040)

include(${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/hw/bsp/family_support.cmake)


# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(tinyusb main.cpp )

pico_set_program_name(tinyusb "tinyusb")
pico_set_program_version(tinyusb "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(tinyusb 1)
pico_enable_stdio_usb(tinyusb 0)

# Add the standard library to the build
target_link_libraries(tinyusb
        pico_stdlib
        tinyusb_device
        tinyusb_board
)
# Add the standard include files to the build
target_include_directories(tinyusb PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src
  ${CMAKE_CURRENT_LIST_DIR}/tinyusb/hw
  ${CMAKE_CURRENT_LIST_DIR}/tinyusb/hw/bsp
  ${CMAKE_CURRENT_LIST_DIR}/rpico-servo
  
  ${PICO_SDK_PATH}/src/common/pico_stdlib/include
  ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
)
# Add all .c files in tinyusb/src to the project
file(GLOB TINYUSB_SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src/*.c)
file(GLOB RPICO_SERVO_SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/rpico-servo/*.cpp)

target_sources(tinyusb PRIVATE ${TINYUSB_SRC_FILES})
target_sources(tinyusb PRIVATE ${RPICO_SERVO_SRC_FILES})

target_sources(tinyusb PRIVATE ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c)

# Ensure tusb_config.h is in the include path
target_include_directories(tinyusb PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src
)

pico_add_extra_outputs(tinyusb)

