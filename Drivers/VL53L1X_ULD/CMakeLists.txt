cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

add_library(VL53L1X_ULD OBJECT)

target_include_directories(VL53L1X_ULD PUBLIC
    ${CMAKE_SOURCE_DIR}/Drivers/VL53L1X_ULD/Inc
)

target_sources(VL53L1X_ULD PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/VL53L1X_ULD/Src/VL53L1X_api.c
    ${CMAKE_SOURCE_DIR}/Drivers/VL53L1X_ULD/Src/VL53L1X_calibration.c
    ${CMAKE_SOURCE_DIR}/Drivers/VL53L1X_ULD/Src/vl53l1_platform.c
)

target_link_libraries(VL53L1X_ULD PRIVATE STM32_Drivers)