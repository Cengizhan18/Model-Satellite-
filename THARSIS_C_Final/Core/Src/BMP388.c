/*
 * BMP388.c
 *
 *  Created on: Aug 13, 2021
 *      Author: Batu
 */

#include "BMP388.h"


void BMP388_writeByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * wdata,uint8_t size)
{
	HAL_I2C_Mem_Write(&hi2cX,device_address << 1,register_address,1,wdata,size,10*size);
}
void BMP388_readByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * rdata,uint8_t size)
{
	HAL_I2C_Mem_Read(&hi2cX,device_address << 1,register_address,1,rdata,size,size*10);
}

// Compensation values
uint16_t NVM_PAR_T1, NVM_PAR_T2;
int8_t NVM_PAR_T3,NVM_PAR_P3,NVM_PAR_P4,NVM_PAR_P7,NVM_PAR_P8,NVM_PAR_P10,NVM_PAR_P11;
int16_t NVM_PAR_P1,NVM_PAR_P2,NVM_PAR_P9;
uint16_t NVM_PAR_P5, NVM_PAR_P6;

float par_t1, par_t2, par_t3;
float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;

void BMP388_Init(void)
{
	// Soft reset
	uint8_t data[2] = {0xB6,0};
	BMP388_writeByte(hi2c2,BMP3_I2C_ADDR_PRIM,BMP3_CMD_ADDR,&data[0],1);

	// Enable sensors and set power mode to normal
	BMP388_readByte(hi2c2,BMP3_I2C_ADDR_PRIM,BMP3_PWR_CTRL_ADDR,&data[0],1);
	data[0] |= 0b00110011;
	BMP388_writeByte(hi2c2,BMP3_I2C_ADDR_PRIM,BMP3_PWR_CTRL_ADDR,&data[0],1);



	uint8_t calib_data[BMP3_CALIB_DATA_LEN];
	BMP388_readByte(hi2c2,BMP3_I2C_ADDR_PRIM,BMP3_CALIB_DATA_ADDR,calib_data,BMP3_CALIB_DATA_LEN);

	NVM_PAR_T1 = (calib_data[1] << 8) | (calib_data[0]);
	NVM_PAR_T2 = (calib_data[3] << 8) | (calib_data[2]);
	NVM_PAR_T3 = (calib_data[4]);
	NVM_PAR_P1 = (calib_data[6] << 8) | (calib_data[5]);
	NVM_PAR_P2 = (calib_data[8] << 8) | (calib_data[7]);
	NVM_PAR_P3 = (calib_data[9]);
	NVM_PAR_P4 = (calib_data[10]);
	NVM_PAR_P5 = (calib_data[12] << 8) | (calib_data[11]);
	NVM_PAR_P6 = (calib_data[14] << 8) | (calib_data[13]);
	NVM_PAR_P7 = (calib_data[15]);
	NVM_PAR_P8 = (calib_data[16]);
	NVM_PAR_P9 = (calib_data[18] << 8) | (calib_data[17]);
	NVM_PAR_P10 = (calib_data[19]);
	NVM_PAR_P11 = (calib_data[20]);

	//float temp_var =
	par_t1 = NVM_PAR_T1/pow(2,-8);
	par_t2 = NVM_PAR_T2/pow(2,30);
	par_t3 = NVM_PAR_T3/pow(2,48);
	par_p1 = (NVM_PAR_P1-pow(2,14))/pow(2,20);
	par_p2 = (NVM_PAR_P2-pow(2,14))/pow(2,29);
	par_p3 = NVM_PAR_P3/pow(2,32);
	par_p4 = NVM_PAR_P4/pow(2,37);
	par_p5 = NVM_PAR_P5/pow(2,-3);
	par_p6 = NVM_PAR_P6/pow(2,6);
	par_p7 = NVM_PAR_P7/pow(2,8);
	par_p8 = NVM_PAR_P8/pow(2,15);
	par_p9 = NVM_PAR_P9/pow(2,48);
	par_p10 = NVM_PAR_P10/pow(2,48);
	par_p11= NVM_PAR_P11/pow(2,65);

}

float BMP388_readData(double * T, double * P)
{
	uint8_t data[6];
	BMP388_readByte(hi2c2,BMP3_I2C_ADDR_PRIM,BMP3_DATA_ADDR,data,6);

	uint32_t uncomp_temp, uncomp_press;
	uncomp_press = (data[2] << 16) | (data[1] << 8) | (data[0]);
	uncomp_temp = (data[5] << 16) | (data[4] << 8) | (data[3]);

	// Temperature compensation
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp-par_t1);
	partial_data2 = (float)(partial_data1 * par_t2);
	*T = partial_data2 + (partial_data1 * partial_data1) * par_t3;

	double temp = *T;
	// Pressure compensation
	partial_data1 = 0;
	partial_data2 = 0;
	float partial_data3, partial_data4, partial_out1, partial_out2;
	partial_data1 = par_p6 * temp;
	partial_data2 = par_p7 * temp*temp;
	partial_data3 = par_p8 * temp*temp*temp;
	partial_out1 = par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = par_p2 * temp;
	partial_data2 = par_p3 * temp*temp;
	partial_data3 = par_p4 * temp*temp*temp;
	partial_out2 = (float)uncomp_press * (par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = par_p9 + par_p10 * temp;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + (float)uncomp_press*(float)uncomp_press*(float)uncomp_press*par_p11;
	*P = partial_out1 + partial_out2 + partial_data4;
}

float BMP388_readAltitude (double pressure,float sealevelhPa)
{
	float altitude;
	pressure/=100;
	altitude= 44330*(1-pow(pressure/sealevelhPa,0.1903));
	return altitude;
}
