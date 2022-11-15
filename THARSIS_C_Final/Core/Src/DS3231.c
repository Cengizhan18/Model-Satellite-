#include "DS3231.h"

DS3231 rtc;
DS3231_days days;

HAL_StatusTypeDef ds3231_i2c_state;

void DS3231_writeByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * wdata,uint8_t size)
{
	ds3231_i2c_state = HAL_I2C_Mem_Write(&hi2cX,device_address << 1,register_address,1,wdata,size,10*size);
}
void DS3231_readByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * rdata,uint8_t size)
{
	ds3231_i2c_state = HAL_I2C_Mem_Read(&hi2cX,device_address << 1,register_address,1,rdata,size,size*10);
}

uint8_t BCD2DEC(uint8_t data)
{
	return (data >> 4)*10 + (data & 0x0F);
}

uint8_t DEC2BCD(uint8_t data)
{
	return ((data/10) << 4) | (data % 10);
}

void DS3231_getTime()
{
	uint8_t buffer[7];
	DS3231_readByte(hi2c1,DS3231_ADDRESS,DS3231_TIME_CAL,buffer,7);
	
	rtc.sec  = BCD2DEC(buffer[0]);
	rtc.min  = BCD2DEC(buffer[1]);
	rtc.hour = BCD2DEC(buffer[2]);
	rtc.dayofweek = BCD2DEC(buffer[3]);
	rtc.day   = BCD2DEC(buffer[4]);
	rtc.month = BCD2DEC(buffer[5]);
	rtc.year  = BCD2DEC(buffer[6]);
	
}

void DS3231_setTime(DS3231 dt)
{
	uint8_t buffer[7];
	
	buffer[0] = DEC2BCD(dt.sec);
	buffer[1] = DEC2BCD(dt.min);
	buffer[2] = DEC2BCD(dt.hour);
	buffer[3] = DEC2BCD(dt.dayofweek);
	buffer[4] = DEC2BCD(dt.day);
	buffer[5] = DEC2BCD(dt.month);
	buffer[6] = DEC2BCD(dt.year);
	
	DS3231_writeByte(hi2c1,DS3231_ADDRESS,DS3231_TIME_CAL,buffer,7);
		
}

