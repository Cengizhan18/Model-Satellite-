#include "MS5611.h"
#include "i2c.h"

void MS5611_Rest()
{
	uint8_t RESET = (uint8_t)MS5611_CMD_REST;
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &RESET, 1, 1000);
	HAL_Delay(4);
}

uint8_t MS5611_PROM_read(){

	uint8_t i;
	uint8_t data[2];
	uint8_t PROM[8] = {MS5611_PROM_READ_0,
			MS5611_PROM_READ_1,
			MS5611_PROM_READ_2,
			MS5611_PROM_READ_3,
			MS5611_PROM_READ_4,
			MS5611_PROM_READ_5,
			MS5611_PROM_READ_6,
			MS5611_PROM_READ_7
	};
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &PROM[0], 1, 100);
	//while(HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &PROM[0], 1, 100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 2, 100);
	//while(HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 2, 100) != HAL_OK);

	ms5611_t.reserve = (uint16_t)(data[0] << 8 | data[1]);

	for (i=1;i<=6;i++){
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &PROM[i], 1, 100);
		//while(HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &PROM[i], 1, 100) != HAL_OK);
		HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 2, 100);
		//while(HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 2, 100) != HAL_OK);

		ms5611_t.C[i-1] = (uint16_t )(data[0] << 8 | data[1]);
	}
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &PROM[7], 1, 100);
	//while(HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &PROM[7], 1, 100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 2, 100);
	//while(HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 2, 100) != HAL_OK);

	ms5611_t.crc = (uint16_t)(data[0] << 8 | data[1]);

	return MS5611_OK;
}
uint8_t MS5611_init()
{
	MS5611_Rest();
	ms5611_t.adress = MS5611_SLAVE_ADDR;
	MS5611_PROM_read();
	return MS5611_OK;
}

/*
 * reading raw temperature of the sensor
 */
uint8_t MS5611_read_temp()
{
	uint8_t reg = MS6511_ADC_READ;
	uint8_t data[5];
	uint8_t cmd;
	uint8_t conv_T;
	cmd = MS5611_CMD_CONVERT_D2_4096; conv_T = 9;


	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &cmd, 1, 100);
	//while(HAL_I2C_Master_Transmit(&hi2c1, address, &cmd, 1, 100) != HAL_OK);//asking adc to store data
	HAL_Delay(conv_T); 														//convertion time
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_SLAVE_ADDR, &reg, 1, 100);
	//while(HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, 100) != HAL_OK);//asking for the data
	HAL_I2C_Master_Receive(&hi2c1, MS5611_SLAVE_ADDR, data, 3, 100);
	//while(HAL_I2C_Master_Receive(&hi2c1, address, data, 3, 100) != HAL_OK);//receive the data

	ms5611_t.D[1] = (data[0] << 16 | data[1] << 8 | data[2]);
	return MS5611_OK;
}

/*
 * reading raw pressure of the sensor
 */
uint8_t MS5611_read_press ()
{

	uint8_t address = ms5611_t.adress;
	uint8_t reg = MS6511_ADC_READ;
	uint8_t data[3];
	uint8_t cmd;
	uint8_t conv_T;
	cmd = MS5611_CMD_CONVERT_D1_4096; conv_T = 9;
	HAL_I2C_Master_Transmit(&hi2c1, address, &cmd, 1, 100);


	//while(HAL_I2C_Master_Transmit(&hi2c1, address, &cmd, 1, 100) != HAL_OK);//asking adc to store data
	HAL_Delay(conv_T); 														//convertion time
	HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, 100);
	//while(HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, 100) != HAL_OK);//asking for the data
	HAL_I2C_Master_Receive(&hi2c1, address, data, 3, 100);
	//while(HAL_I2C_Master_Receive(&hi2c1, address, data, 3, 100) != HAL_OK);//receive the data

	ms5611_t.D[0] = (data[0] << 16 | data[1] << 8 | data[2]);
	return MS5611_OK;
}

/*
 * pressure and temperature calculation
 */
uint8_t MS5611_calculate()
{
	int64_t dT = 0,TEMP = 0,T2 = 0,OFF = 0,OFF2 = 0,SENS2 = 0,SENS = 0,PRES = 0;

	dT = ms5611_t.D[1] - ((int32_t) (ms5611_t.C[4])<<8);
	TEMP = 2000 + ((int32_t) (dT*(ms5611_t.C[5]))>>23);
	OFF = (((int64_t)(ms5611_t.C[1])) << 16) + (((ms5611_t.C[3]) * dT) >> 7);
	SENS = (((int64_t)(ms5611_t.C[0])) << 15) + (((ms5611_t.C[2]) * dT) >> 8);



	if(TEMP < 2000) {
		T2 = ( dT*dT )>>31;
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;


		if (TEMP < -1500) {
			OFF2 = OFF2 + (7 * (TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + (11 * (TEMP + 1500) * (TEMP + 1500) / 2);
		}
	}
	else {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	ms5611_t.dT = dT;
	ms5611_t.OFF = OFF - OFF2;
	ms5611_t.TEMP = TEMP - T2;
	ms5611_t.SENS = SENS - SENS2;

	PRES = ((((int32_t)(ms5611_t.D[0]) * (ms5611_t.SENS))>>21) - (ms5611_t.OFF))>>15;

	ms5611_t.P = PRES;
	return MS5611_OK;
}


uint8_t NB_MS5611_request_temp(){


	uint8_t address = ms5611_t.adress;
	uint8_t cmd;
	uint8_t timeout = 0;
	cmd = MS5611_CMD_CONVERT_D2_4096;

	while(HAL_I2C_Master_Transmit(&hi2c1, address, &cmd, 1, 100) != HAL_OK){
		HAL_Delay(1);
		timeout++;
		if(timeout >= 10){
			return MS5611_ERROR;
			break;
		}
	}
	return MS5611_OK;
}

uint8_t NB_MS5611_pull_temp(){

	uint8_t address = ms5611_t.adress;
	uint8_t reg = MS6511_ADC_READ;
	uint8_t data[3];
	uint8_t timeout = 0;

	while(HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, 100) != HAL_OK){
		HAL_Delay(1);
		timeout++;
		if(timeout >= 10){
			return MS5611_ERROR;
			break;
		}
	}
	while(HAL_I2C_Master_Receive(&hi2c1, address, data, 3, 100) != HAL_OK){
		HAL_Delay(1);
		timeout++;
		if(timeout >= 10){
			return MS5611_ERROR;
			break;
		}
	}
	ms5611_t.D[1] = (data[0] << 16 | data[1] << 8 | data[2]);
	return MS5611_OK;
}

uint8_t NB_MS5611_request_press(){


	uint8_t address = ms5611_t.adress;
	uint8_t cmd;
	uint8_t timeout = 0;
	cmd = MS5611_CMD_CONVERT_D1_4096;


	while(HAL_I2C_Master_Transmit(&hi2c1, address, &cmd, 1, 100) != HAL_OK){ //asking adc to store data
		HAL_Delay(1);
		timeout++;
		if(timeout >= 10){
			return MS5611_ERROR;
			break;
		}
	}
	return MS5611_OK;
}

uint8_t NB_MS5611_pull_press(){


	uint8_t address = ms5611_t.adress;
	uint8_t reg = MS6511_ADC_READ;
	uint8_t data[3];
	uint8_t timeout = 0;

	while(HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, 100) != HAL_OK){
		HAL_Delay(1);
		timeout++;
		if(timeout >= 10){
			return MS5611_ERROR;
			break;
		}
	}
	while(HAL_I2C_Master_Receive(&hi2c1, address, data, 3, 100) != HAL_OK){
		HAL_Delay(1);
		timeout++;
		if(timeout >= 10){
			return MS5611_ERROR;
			break;
		}
	}
	ms5611_t.D[0] = (data[0] << 16 | data[1] << 8 | data[2]);
	return MS5611_OK;
}

float MS5611_readAlt(float press)
{
	float altitude;
	altitude= 44330*(1-pow(press/100/1012,0.1903));
	return altitude;
}
