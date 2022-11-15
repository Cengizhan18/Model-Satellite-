/**
  ******************************************************************************

  EEPROM.h Using the HAL I2C Functions
  Author:   ControllersTech
  Updated:  Feb 16, 2021

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define EEPROM_OFFSET_PACKAGE 0
#define EEPROM_OFFSET_STATE	5
#define EEPROM_OFFSET_RESET 10
#define EEPROM_OFFSET_REF_ALT 15
#define EEPROM_OFFSET_PRESS_TOTAL_AVG 20
#define EEPROM_OFFSET_ACTUAL_PRESS_FAST 25
#define EEPROM_OFFSET_ACTUAL_PRESS_SLOW 35
#define EEPROM_OFFSET_ACTUAL_PRESSURE_DIFF 40
#define EEPROM_OFFSET_PRESS_MEM_LOC 45
#define EEPROM_OFFSET_ACTUAL_PRESS_LOC 50
#define EEPROM_OFFSET_BAROMETER_CNT 55

void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_PageErase (uint16_t page);

void EEPROM_Write_NUM (uint16_t page, uint16_t offset, float  fdata);
float EEPROM_Read_NUM (uint16_t page, uint16_t offset);

#endif /* INC_EEPROM_H_ */
