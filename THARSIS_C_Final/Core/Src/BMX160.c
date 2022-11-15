/*
 * BMX160.c
 *
 *  Created on: Aug 12, 2021
 *      Author: Batu
 */

#include "BMX160.h"
#include "MahonyAHRS.h"
#include "math.h"

#define rad_to_deg 180/3.14

void BMX160_writeByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * wdata,uint8_t size)
{
	HAL_I2C_Mem_Write(&hi2cX,device_address << 1,register_address,1,wdata,size,10*size);
}
void BMX160_readByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * rdata,uint8_t size)
{
	HAL_I2C_Mem_Read(&hi2cX,device_address << 1,register_address,1,rdata,size,size*10);
}

uint8_t BMX160_Read_ID(void)
{
	uint8_t id;
	BMX160_readByte(hi2c2,BMX160_I2C_ADDRESS, BMX160_CHIP_ID_ADDR , &id, 1);
	return id;
}

void BMX160_SoftReset(void)
{
	uint8_t data = BMX160_SOFT_RESET_CMD;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_COMMAND_REG_ADDR,&data,1);
	HAL_Delay(15);
}

void BMX160_Begin(void)
{
	// Set accel to normal mode
	uint8_t data = 0x11;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_COMMAND_REG_ADDR,&data,1);

	// Set gyro to normal mode
	data = 0x15;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_COMMAND_REG_ADDR,&data,1);

	// Set magn to normal mode
	data = 0x19;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_COMMAND_REG_ADDR,&data,1);

	data = 0x80;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_0_ADDR,&data,1);

	// Sleep mode
	data = 0x01;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_3_ADDR,&data,1);
	data = 0x4B;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_2_ADDR,&data,1);

	// REPXY
	data = 0x04;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_3_ADDR,&data,1);
	data = 0x51;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_2_ADDR,&data,1);

	// REPZ
	data = 0x0E;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_3_ADDR,&data,1);
	data = 0x52;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_2_ADDR,&data,1);

	data = 0x02;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_3_ADDR,&data,1);
	data = 0x4C;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_2_ADDR,&data,1);
	data = 0x42;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_1_ADDR,&data,1);
	data = 0x08;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_CONFIG_ADDR,&data,1);

	data = 0x03;
	BMX160_writeByte(hi2c2,BMX160_I2C_ADDRESS,BMX160_MAGN_IF_0_ADDR,&data,1);

	HAL_Delay(50);
}

void BMX160_readAllData(struct bmx160SensorData *magn, struct bmx160SensorData *gyro, struct bmx160SensorData *accel)
{
	uint8_t data[23];
	BMX160_readByte(hi2c2, BMX160_I2C_ADDRESS, BMX160_MAG_DATA_ADDR, data, 23);

    magn->x = (int16_t) ((data[1] << 8) | data[0]);
    magn->y = (int16_t) ((data[3] << 8) | data[2]);
    magn->z = (int16_t) ((data[5] << 8) | data[4]);
    magn->x *= BMX160_MAGN_UT_LSB;
    magn->y *= BMX160_MAGN_UT_LSB;
    magn->z *= BMX160_MAGN_UT_LSB;

    gyro->x = (int16_t) ((data[9] << 8) | data[8]);
    gyro->y = (int16_t) ((data[11] << 8) | data[10]);
    gyro->z = (int16_t) ((data[13] << 8) | data[12]);
    gyro->x *= BMX160_GYRO_SENSITIVITY_125DPS;
    gyro->y *= BMX160_GYRO_SENSITIVITY_125DPS;
    gyro->z *= BMX160_GYRO_SENSITIVITY_125DPS;

    accel->x = (int16_t) ((data[15] << 8) | data[14]);
    accel->y = (int16_t) ((data[17] << 8) | data[16]);
    accel->z = (int16_t) ((data[19] << 8) | data[18]);
    accel->x *= BMX160_ACCEL_MG_LSB_2G;
    accel->y *= BMX160_ACCEL_MG_LSB_2G;
    accel->z *= BMX160_ACCEL_MG_LSB_2G;

}

void BMX160_getEulerAngles(double * roll, double * pitch, double * yaw)
{
	struct bmx160SensorData Magn,Accel,Gyro;
	BMX160_readAllData(&Magn, &Gyro, &Accel);

	for(int i = 0; i<16; i++)
	{
		MahonyAHRSupdate(Gyro.x,Gyro.y,Gyro.z,Accel.x,Accel.y,Accel.z,Magn.x,Magn.y,Magn.z);
		//MahonyAHRSupdate(Gyro.x,Gyro.y,Gyro.z,Accel.x,Accel.y,Accel.z);
	}

	*roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	*pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	*yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	*roll  *= rad_to_deg;
	*pitch *= rad_to_deg;
	*yaw   *= rad_to_deg;
}

