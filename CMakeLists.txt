cmake_minimum_required(VERSION 3.22)

# setup compiler and paths
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(gcc-arm-none-eabi)

project(DroneFlightController)
enable_language(C ASM)

add_subdirectory(Drivers) # run the CMakeLists inside Drivers

# all of this project's source files and some system files
set(PROJECT_SOURCES
    src/main.c
    src/control.c
    src/filter.c
    src/pid.c
    src/i2c.c
    src/uart.c
    src/imu.c
    src/lidar.c
    src/radio.c
    src/motor.c
    src/debugger.c

    src/stm32f0xx_it.c
    src/stm32f0xx_hal_msp.c
)

add_executable(project ${PROJECT_SOURCES}) # create executable named "project"

# directory of all this project's header files
target_include_directories(project PRIVATE
    inc
)
target_compile_options(project PRIVATE -g3 -O0)

target_link_libraries(project PRIVATE
    STM32_Drivers
    STM32_Discovery
    VL53L1X_ULD
    m
    # uncomment line below to do semihosting
    #rdimon
    # it would go here, but we should probably just remove SEGGER from the project files
)

# flashing
include(flash_stm32)
flash_target(project)
