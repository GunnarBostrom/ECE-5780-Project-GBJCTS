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
    src/control.c
    src/filter.c
    src/i2c.c
    src/imu.c
    src/lidar.c
    src/main.c
    src/motor.c
    src/pwm.c
    src/radio.c
    src/uart.c
    src/uart_test_data.c

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




# ––––– OLD TOP LEVEL CMAKELISTS.txt BELOW –––––
# (keeping for now in case I broke something)




# collect all Source Files (includes drivers, core system files, and the startup assembly)
# ** this should now be unnecessary because it is handled by "STM32_Drivers" inside Drivers CMakeLists.txt
# set(HAL_SOURCES
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
#     Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c
#     Drivers/VL53L1X_ULD/Src/VL53L1X_api.c
#     Drivers/VL53L1X_ULD/Src/VL53L1X_calibration.c
#     Drivers/VL53L1X_ULD/Src/vl53l1_platform.c
# )


# set(PROJECT_SOURCES
# # Your project files
# Core/Src/system_stm32f0xx.c
# # Core/Src/syscalls.c # replaced with rdimon to allow semihosting
# Core/Src/sysmem.c
# src/control.c
# src/filter.c
# src/i2c.c
# src/imu.c
# src/lidar.c
# src/main.c
# src/motor.c
# src/pwm.c
# src/radio.c
# src/uart.c

# # Core system files (main.c is usually here)
# src/stm32f0xx_it.c
# src/stm32f0xx_hal_msp.c

# # Startup file (crucial for hardware boot)
# startup_stm32f072xb.s
# )

# # Add these to your main source list
# set(PROJECT_SOURCES
#     ${PROJECT_SOURCES}
#     ${HAL_SOURCES}
# )
# # 3. Define the Linker Script
# # Use the .ld file you listed in your root
# set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F072XX_FLASH.ld)

# # creates the executable named 'project'
# add_executable(project ${PROJECT_SOURCES})


# # 5. Include Directories
# target_include_directories(project PRIVATE
#     inc                                 # Your project headers
#     Core/Inc                            # Core system headers
#     Drivers/STM32F0xx_HAL_Driver/Inc    # HAL Driver
#     Drivers/CMSIS/Device/ST/STM32F0xx/Include
#     Drivers/CMSIS/Include
#     Drivers/BSP                         # Board Support Package

# )

# # 6. Add Project Symbols (Macros)
# # This tells the HAL which specific chip you are using
# target_compile_definitions(project PRIVATE
#     STM32F072xB
#     USE_HAL_DRIVER
# )

# # Linker flags for semihosting, not part of section 6.
# target_link_libraries(project PRIVATE m rdimon)

# # flashing logic
# include(flash_stm32)
# flash_target(project)