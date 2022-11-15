/*
 * BMX160.h
 *
 *  Created on: Aug 12, 2021
 *      Author: Batu
 */

#ifndef INC_BMX160_H_
#define INC_BMX160_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "i2c.h"

#define BMX160_I2C_ADDRESS 0x69
#define BMX160_CHIP_ID 0xD8


/** bmx160 Register map */
#define BMX160_CHIP_ID_ADDR                      0x00
#define BMX160_ERROR_REG_ADDR                    0x02
#define BMX160_MAG_DATA_ADDR                     0x04
#define BMX160_GYRO_DATA_ADDR                    0x0C
#define BMX160_ACCEL_DATA_ADDR                   0x12
#define BMX160_STATUS_ADDR                       0x1B
#define BMX160_INT_STATUS_ADDR                   0x1C
#define BMX160_FIFO_LENGTH_ADDR                  0x22
#define BMX160_FIFO_DATA_ADDR                    0x24
#define BMX160_ACCEL_CONFIG_ADDR                 0x40
#define BMX160_ACCEL_RANGE_ADDR                  0x41
#define BMX160_GYRO_CONFIG_ADDR                  0x42
#define BMX160_GYRO_RANGE_ADDR                   0x43
#define BMX160_MAGN_CONFIG_ADDR                  0x44
#define BMX160_FIFO_DOWN_ADDR                    0x45
#define BMX160_FIFO_CONFIG_0_ADDR                0x46
#define BMX160_FIFO_CONFIG_1_ADDR                0x47
#define BMX160_MAGN_RANGE_ADDR                   0x4B
#define BMX160_MAGN_IF_0_ADDR                    0x4C
#define BMX160_MAGN_IF_1_ADDR                    0x4D
#define BMX160_MAGN_IF_2_ADDR                    0x4E
#define BMX160_MAGN_IF_3_ADDR                    0x4F
#define BMX160_INT_ENABLE_0_ADDR                 0x50
#define BMX160_INT_ENABLE_1_ADDR                 0x51
#define BMX160_INT_ENABLE_2_ADDR                 0x52
#define BMX160_INT_OUT_CTRL_ADDR                 0x53
#define BMX160_INT_LATCH_ADDR                    0x54
#define BMX160_INT_MAP_0_ADDR                    0x55
#define BMX160_INT_MAP_1_ADDR                    0x56
#define BMX160_INT_MAP_2_ADDR                    0x57
#define BMX160_INT_DATA_0_ADDR                   0x58
#define BMX160_INT_DATA_1_ADDR                   0x59
#define BMX160_INT_LOWHIGH_0_ADDR                0x5A
#define BMX160_INT_LOWHIGH_1_ADDR                0x5B
#define BMX160_INT_LOWHIGH_2_ADDR                0x5C
#define BMX160_INT_LOWHIGH_3_ADDR                0x5D
#define BMX160_INT_LOWHIGH_4_ADDR                0x5E
#define BMX160_INT_MOTION_0_ADDR                 0x5F
#define BMX160_INT_MOTION_1_ADDR                 0x60
#define BMX160_INT_MOTION_2_ADDR                 0x61
#define BMX160_INT_MOTION_3_ADDR                 0x62
#define BMX160_INT_TAP_0_ADDR                    0x63
#define BMX160_INT_TAP_1_ADDR                    0x64
#define BMX160_INT_ORIENT_0_ADDR                 0x65
#define BMX160_INT_ORIENT_1_ADDR                 0x66
#define BMX160_INT_FLAT_0_ADDR                   0x67
#define BMX160_INT_FLAT_1_ADDR                   0x68
#define BMX160_FOC_CONF_ADDR                     0x69
#define BMX160_CONF_ADDR                         0x6A

#define BMX160_IF_CONF_ADDR                      0x6B
#define BMX160_SELF_TEST_ADDR                    0x6D
#define BMX160_OFFSET_ADDR                       0x71
#define BMX160_OFFSET_CONF_ADDR                  0x77
#define BMX160_INT_STEP_CNT_0_ADDR               0x78
#define BMX160_INT_STEP_CONFIG_0_ADDR            0x7A
#define BMX160_INT_STEP_CONFIG_1_ADDR            0x7B
#define BMX160_COMMAND_REG_ADDR                  0x7E
#define BMX160_SPI_COMM_TEST_ADDR                0x7F
#define BMX160_INTL_PULLUP_CONF_ADDR             0x85


#define BMX160_SOFT_RESET_CMD                    0xb6
#define BMX160_SOFT_RESET_DELAY_MS               15

/* Gyroscope sensitivity at 125dps */
#define BMX160_GYRO_SENSITIVITY_125DPS  0.0038110F // Table 1 of datasheet
/* Gyroscope sensitivity at 250dps */
#define BMX160_GYRO_SENSITIVITY_250DPS  0.0076220F // Table 1 of datasheet
/* Gyroscope sensitivity at 500dps */
#define BMX160_GYRO_SENSITIVITY_500DPS  0.0152439F
/* Gyroscope sensitivity at 1000dps */
#define BMX160_GYRO_SENSITIVITY_1000DPS 0.0304878F
/* Gyroscope sensitivity at 2000dps */
#define BMX160_GYRO_SENSITIVITY_2000DPS 0.0609756F

/* Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
#define BMX160_ACCEL_MG_LSB_2G      0.000061035F
/* Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
#define BMX160_ACCEL_MG_LSB_4G      0.000122070F
/* Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
#define BMX160_ACCEL_MG_LSB_8G      0.000244141F
/* Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */
#define BMX160_ACCEL_MG_LSB_16G     0.000488281F

/* Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define BMX160_MAGN_UT_LSB      (0.3F)

struct bmx160SensorData {
  /*! X-axis sensor data */
  double x;
  /*! Y-axis sensor data */
  double y;
  /*! Z-axis sensor data */
  double z;
  /*! sensor time */
  uint32_t sensortime;
};

void BMX160_writeByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * wdata,uint8_t size);
void BMX160_readByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * rdata,uint8_t size);
uint8_t BMX160_Read_ID(void);
void BMX160_SoftReset(void);
void BMX160_Begin(void);
void BMX160_readAllData(struct bmx160SensorData *magn, struct bmx160SensorData *gyro, struct bmx160SensorData *accel);
void BMX160_getEulerAngles(double * roll, double * pitch, double * yaw);


#endif /* INC_BMX160_H_ */
