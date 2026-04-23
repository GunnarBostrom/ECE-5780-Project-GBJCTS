/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "i2c.h"

// i2c.h uses 7-bit addresses. VL53L1X default is 0x29 (7-bit).
// (The 0x52 used with HAL was the 8-bit form: 0x29 << 1)

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    i2c_write((uint8_t)dev, index, pdata, (uint8_t)count);
    return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    i2c_read((uint8_t)dev, index, pdata, (uint8_t)count);
    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    i2c_write((uint8_t)dev, index, &data, 1);
    return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8); 
    buf[1] = (uint8_t)(data & 0xFF);
    i2c_write((uint8_t)dev, index, buf, 2);
    return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)(data >> 16);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFF);
    i2c_write((uint8_t)dev, index, buf, 4);
    return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    i2c_read((uint8_t)dev, index, data, 1);
    return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t buf[2];
    i2c_read((uint8_t)dev, index, buf, 2);
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t buf[4];
    i2c_read((uint8_t)dev, index, buf, 4);
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
          | ((uint32_t)buf[2] << 8)  | buf[3];
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    for (volatile int32_t i = 0; i < wait_ms * 6000; i++); //FIXME: busy-wait... replace with systick or timer or something.
    return 0;
}