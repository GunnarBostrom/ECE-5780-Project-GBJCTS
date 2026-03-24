cmake_minimum_required(VERSION 3.22)

# 1. Setup compiler and paths
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Ensure the 'cmake' folder from your lab is copied here!
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(gcc-arm-none-eabi)

project(DroneFlightController)
enable_language(C ASM)

# 2. Collect all Source Files
# This includes your logic, core system files, and the startup assembly
set(HAL_SOURCES
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c
)


set(PROJECT_SOURCES
# Your project files
Core/Src/system_stm32f0xx.c
Core/Src/syscalls.c
Core/Src/sysmem.c
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

# Core system files (main.c is usually here)
src/stm32f0xx_it.c
src/stm32f0xx_hal_msp.c

# Startup file (crucial for hardware boot)
startup_stm32f072xb.s
)

# Add these to your main source list
set(PROJECT_SOURCES
    ${PROJECT_SOURCES}
    ${HAL_SOURCES}
)
# 3. Define the Linker Script
# Use the .ld file you listed in your root
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F072XX_FLASH.ld)

# 4. Create the executable named 'project'
add_executable(project ${PROJECT_SOURCES})


# 5. Include Directories
target_include_directories(project PRIVATE
    inc                                 # Your project headers
    Core/Inc                            # Core system headers
    Drivers/STM32F0xx_HAL_Driver/Inc    # HAL Driver
    Drivers/CMSIS/Device/ST/STM32F0xx/Include
    Drivers/CMSIS/Include
    Drivers/BSP                         # Board Support Package
    Drivers/SEGGER                      # For RTT/Debugging if used
)

# 6. Add Project Symbols (Macros)
# This tells the HAL which specific chip you are using
target_compile_definitions(project PRIVATE
    STM32F072xB
    USE_HAL_DRIVER
)

# 7. Flashing Logic
include(flash_stm32)
flash_target(project)