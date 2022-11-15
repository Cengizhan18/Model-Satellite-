/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMP388.h"
#include "MS5611.h"
#include "BMX160.h"
#include "DS3231.h"
#include "EEPROM.h"
#include "MahonyAHRS.h"
#include "Motor.h"
#include "MS5611.h"
#include "PID.h"
#include "string.h"
#include "GPS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Uydu statü tanımları
#define STATE_BEKLEME 0
#define STATE_BEKLEME_STRING "BEKLEME"
#define STATE_YUKSELME 1
#define STATE_YUKSELME_STRING "YUKSELME"
#define STATE_TASIYICI_INIS 2
#define STATE_TASIYICI_INIS_STRING "TASIYICI INIS"
#define STATE_AYRILMA 3
#define STATE_AYRILMA_STRING "AYRILMA"
#define STATE_AKTIF_INIS 4
#define STATE_AKTIF_INIS_STRING "AKTIF_INIS"
#define STATE_IRTIFA_SABITLEME 5
#define STATE_IRTIFA_SABITLEME_STRING "IRTIFA_SABITLEME"
#define STATE_AKTIF_INIS_2 6
#define STATE_AKTIF_INIS_2_STRING "AKTIF_INIS_2"
#define STATE_LANDING_1 7
#define STATE_LANDING_1_STRING "LANDING_1"
#define STATE_LANDING_2 8
#define STATE_LANDING_2_STRING "LANDING_2"
#define STATE_LANDING_3 9
#define STATE_LANDING_3_STRING "LANDING_3"
#define STATE_SALVAGE 10
#define STATE_SALVAGE_STRING "KURTARMA"

// Statü geçişleri
#define STATE_TRANSITION_BEKLEME 			(BMP388_ALT <= 10 || VELOCITY <= 3) && CURRENT_STATE == 255
#define STATE_TRANSITION_YUKSELME			(BMP388_ALT >= 10 && CURRENT_STATE == STATE_BEKLEME)
#define STATE_TRANSITION_TASIYICI_INIS 		(((BMP388_PREV_ALT - BMP388_ALT) > 0) && CURRENT_STATE == STATE_YUKSELME)
#define STATE_TRANSITION_AYRILMA 			(BMP388_ALT <= 400 + 50 && (BMP388_ALT >= 400 - 50) && CURRENT_STATE == STATE_TASIYICI_INIS)
#define STATE_TRANSITION_AKTIF_INIS			(CURRENT_STATE == STATE_AYRILMA && DETACH_CONTROL == 1)
#define	STATE_TRANSITION_IRTIFA_SABITLEME 	(CURRENT_STATE == STATE_AKTIF_INIS && BMP388_ALT <= 250)
#define STATE_TRANSITION_AKTIF_INIS_2 		(HAL_GetTick() - IRTIFA_SABITLEME_TIMER >= 15000 && ((CURRENT_STATE == STATE_IRTIFA_SABITLEME)))
#define STATE_TRANSITION_LANDING_1 			(BMP388_ALT <= 50 && ((CURRENT_STATE == STATE_AKTIF_INIS_2)))
#define STATE_TRANSITION_LANDING_2 			(BMP388_ALT <= 30 && ((CURRENT_STATE == STATE_LANDING_1)))
#define STATE_TRANSITION_LANDING_3 			(BMP388_ALT <= 15 && ((CURRENT_STATE == STATE_LANDING_2)))
#define STATE_TRANSITION_SALVAGE 			(VELOCITY <= 1 && VELOCITY >= -1 && ((CURRENT_STATE == STATE_LANDING_3)))


// Statülere göre PID Hedef değerleri
#define DESIRED_SPEED_AKTIF_INIS -7
#define DESIRED_SPEED_IRTIFA_SABITLEME 0
#define DESIRED_SPEED_LANDING_1 -5
#define DESIRED_SPEED_LANDING_2 -3
#define DESIRED_SPEED_LANDING_3 -1

HAL_StatusTypeDef status;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Genel algoritma değişkenleri
uint8_t CODE_RESET = 0; uint8_t CURRENT_STATE = 255;
char CURRENT_STATE_STRING[20];
uint16_t PACKAGE_COUNT = 0;
uint8_t DETACH_CONTROL = 0, MOTOR_CONTROL_1 = 0, MOTOR_CONTROL_2 = 0; uint32_t MOTOR_CONTROL_TIMER_1=0, MOTOR_CONTROL_TIMER_2=0;
uint32_t IRTIFA_SABITLEME_DURATION = 0; uint32_t IRTIFA_SABITLEME_TIMER=0;
uint32_t LOOP_TIMER_1HZ = 0, LOOP_TIMER_50HZ=0;
uint32_t TEST_TIME=0; uint8_t MOTOR_TEST_FLAG=0;
uint8_t SMOKE_CONTROL = 0;

// PID
PID_Object V_PID;
uint16_t CENTER_THROTTLE = 1500,THROTTLE=1000;
const uint16_t THROTTLE_MIN=1100, THROTTLE_MAX = 2000;

// BMP388 ve MS5611 verileri
double BMP388_T=0, BMP388_P=0, BMP388_ALT=0, BMP388_PREV_ALT = 0, BMP388_refAlt=0, BMP388_VELOCITY;
uint32_t RAW_TEMP[5], RAW_PRESSURE, RAW_TEMPERATURE, RAW_AVG_TEMP_TOTAL, TEMP;
uint8_t BAROMETER_COUNTER, TEMP_COUNTER, AVG_TEMP_MEM_LOCATION, PRESS_MEM_LOCATION;
int32_t PRESS_ROTATING_MEM[20], PRESS_TOTAL_AVG;
float MS5611_refAlt= 0, ACTUAL_PRESSURE, ACTUAL_PREV_PRESS, ACTUAL_PRESSURE_SLOW, ACTUAL_PRESSURE_FAST, ACTUAL_PRESSURE_DIFF, ACTUAL_ALTITUDE, PREV_ALTITUDE, VELOCITY;
float SLOW_FILTER_COEFFICIENT=0.985, FAST_FILTER_COEFFICIENT = 0.015;

// Motorlar
Motor mt1, mt2, servo; // mt1: Üstteki motor, mt2: Alttaki motor

// GPS Verileri
uint8_t RAW_GPS[512], GPS_CONTROL = 0;

// XBee değişkenleri
uint8_t GCS_Command = 0;
int xbee;



char XBee_Buffer[256];
//char XBee_Buffer2[256];
char VIDEO_STATE[16];
char Tasiyici_Buffer[256];
uint8_t xbee_rx_buffer[256];
uint8_t rxBuffer;
uint8_t rxBufferCounter = 0;
uint8_t dataArrived = 0;
// BMX160 değişkenleri
double ROLL=45, PITCH=30, YAW=127;
int RotationCount = 0;

//DS3231
extern DS3231 rtc;

//irtifa farkı
float subtraction;
// SD Card
FATFS fs;
FIL fil;
FRESULT fresult;
UINT br, bw;
FATFS *pfs;
DWORD fre_clust;

//ADC
uint16_t ADC_VAL; float VOLTAGE;


// I2C Scanner
uint8_t address[16];
int i=0;
extern HAL_StatusTypeDef i2c_status;

// Taşıyıcı data
float tPressure, tAltitude;
float tGPSAltitude,tLongitude, tLatitude;

uint8_t id;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SD_write(char * str);
void SD_Mount(void);
uint8_t CHECK_START_STATUS(void);
void WRITE_START_STATUS(uint8_t STATUS);
void SENSOR_INIT(void);
void MOTOR_INIT(void);
void SW_INIT(void);
void WAIT_START(void);
void EEPROM_WRITE_REF_ALT(double ref);
void CHECK_SATELLITE_STATE(uint8_t statu_ctrl);
void GET_DATA(void);
void CHECK_STATE(void);
void READ_BAROMETER(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc);
void DETACH(void);
void PID_LOOP(void);
void GET_CMD(void);
void TEST_MOTORS(void);
void EEPROM_1HZ(void);
void GET_GPS(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();

	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_FATFS_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  int k = 0;
  for(i=0; i<255; i++)
  {
	  i2c_status = HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 10);
	  if(!i2c_status)
	  {
		  address[k] = i;
		  k++;
		  i++;

	  }
  }

  id = BMX160_Read_ID();
  SW_INIT();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //sprintf(XBee_Buffer2,"13245, %.1f", 11.5);
	 // HAL_UART_Transmit_IT(&huart1, (uint8_t*)XBee_Buffer2, strlen(XBee_Buffer2));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 GET_DATA();
	 PID_LOOP();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void SENSOR_INIT(void)
{
	BMX160_Begin();
	BMP388_Init();
	BMP388_readData(&BMP388_T, &BMP388_P);
	BMP388_refAlt = BMP388_readAltitude(BMP388_P, 1013);
	BMX160_getEulerAngles(&ROLL, &PITCH, &YAW);
	//MS5611_init();
	/*rtc.sec = 0;
	rtc.min = 56;
	rtc.hour = 19;
	DS3231_setTime(rtc);*/
	DS3231_getTime();

	// GPS Start
	HAL_UART_Receive_DMA(&huart2, RAW_GPS, 512);


	// XBee Command Start
	HAL_UART_Receive_IT(&huart1, &rxBuffer, 1);

	// ADC init
	HAL_ADC_Start_IT(&hadc1);
}

void MOTOR_INIT(void)
{
	mt1 = setMotor(&htim1, TIM_CHANNEL_1);
	startMotor(mt1);
	setMotorThrottle(mt1, 1000);
	mt2 = setMotor(&htim1, TIM_CHANNEL_4);
	startMotor(mt2);
	setMotorThrottle(mt2, 1000);
	servo = setMotor(&htim3, TIM_CHANNEL_1);
	startMotor(servo);
	setMotorThrottle(servo, 1000);

}


uint8_t CHECK_START_STATUS(void)
{
	uint8_t rByte=0;
	EEPROM_Read(0, EEPROM_OFFSET_RESET, &rByte, 1);
	return rByte;
}

void WRITE_START_STATUS(uint8_t STATUS)
{
	EEPROM_Write(0, EEPROM_OFFSET_RESET, &STATUS, 1);
}

void EEPROM_WRITE_REF_ALT(double ref)
{
	EEPROM_Write_NUM(0, EEPROM_OFFSET_REF_ALT, (float)ref);
}

void WAIT_START(void)
{
	while(0)
	{
		if(GCS_Command == 83) // 83: 'S' in ASCII
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			GCS_Command = 0;
			break;
		}
	}
}

void SW_INIT(void)
{
	// Saat ayarlama
	/*rtc.hour = 23;
	rtc.min = 10;
	rtc.sec = 0;
	DS3231_setTime(rtc);

	while(1);*/


	SD_Mount();
	uint8_t _code_status = CHECK_START_STATUS();


	if(_code_status == 1)
	{
		// Yazılım uçuş sırasında resetlendiyse
		EEPROM_Read(0, EEPROM_OFFSET_STATE, &CURRENT_STATE, 1);
		CHECK_SATELLITE_STATE(CURRENT_STATE);
		CHECK_STATE();
		BMP388_refAlt = EEPROM_Read_NUM(0, EEPROM_OFFSET_REF_ALT);
		PACKAGE_COUNT = EEPROM_Read_NUM(0, EEPROM_OFFSET_PACKAGE);
		for(int i = 0; i<20; i++)
		{
			if(i>=10)
			{
				PRESS_ROTATING_MEM[i] = EEPROM_Read_NUM(2, (i-10)*4);
			}
			else
			{
				PRESS_ROTATING_MEM[i] = EEPROM_Read_NUM(1, i*4);
			}

		}

		sprintf(VIDEO_STATE,"HAYIR");
		SENSOR_INIT();
		MOTOR_INIT();
		PRESS_TOTAL_AVG = EEPROM_Read_NUM(0, EEPROM_OFFSET_PRESS_TOTAL_AVG);
		ACTUAL_PRESSURE_FAST = EEPROM_Read_NUM(0, EEPROM_OFFSET_ACTUAL_PRESS_FAST);
		ACTUAL_PRESSURE_SLOW = EEPROM_Read_NUM(0, EEPROM_OFFSET_ACTUAL_PRESS_SLOW);
		ACTUAL_PRESSURE_DIFF = EEPROM_Read_NUM(0, EEPROM_OFFSET_ACTUAL_PRESSURE_DIFF);
		PRESS_MEM_LOCATION = EEPROM_Read_NUM(0, EEPROM_OFFSET_PRESS_MEM_LOC);
		ACTUAL_PRESSURE = EEPROM_Read_NUM(0, EEPROM_OFFSET_ACTUAL_PRESS_LOC);
		BAROMETER_COUNTER = EEPROM_Read_NUM(0, EEPROM_OFFSET_BAROMETER_CNT);


	}
	else
	{

		// Yazılım ilk kez çalışıyorsa
		SENSOR_INIT();
		MOTOR_INIT();
		EEPROM_PageErase(0);
		EEPROM_PageErase(1);
		EEPROM_PageErase(2);

		sprintf(VIDEO_STATE,"HAYIR");
		/*for(int i=0; i<1200; i++)
		{
			READ_BAROMETER();
			HAL_Delay(4);
		}*/
		MS5611_refAlt = ACTUAL_ALTITUDE;
		READ_BAROMETER();

		for(int n = 0; n<20; n++)
		{
			if(n>=10)
			{
				EEPROM_Write_NUM(2, (n-10)*4, PRESS_ROTATING_MEM[n]);
			}
			else
			{
				EEPROM_Write_NUM(1, n*4, PRESS_ROTATING_MEM[n]);
			}
		}

		CODE_RESET = 1;
		CURRENT_STATE = 255;
		PACKAGE_COUNT = 0;
		WRITE_START_STATUS(CODE_RESET);
		BMP388_readData(&BMP388_T, &BMP388_P);
		BMP388_refAlt = BMP388_readAltitude(BMP388_P, 1012);
		EEPROM_WRITE_REF_ALT(MS5611_refAlt);

		WAIT_START();
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc)
{
	ADC_VAL = HAL_ADC_GetValue(hadc);
	VOLTAGE = (float)ADC_VAL*3.3/4096*(42.2 + 15)/15 + 0.25;
}

void CHECK_SATELLITE_STATE(uint8_t statu_ctrl)
{
	switch(statu_ctrl)
	{
		case STATE_BEKLEME:
			sprintf(CURRENT_STATE_STRING, STATE_BEKLEME_STRING);
			break;
		case STATE_YUKSELME:
			sprintf(CURRENT_STATE_STRING, STATE_YUKSELME_STRING);
			break;
		case STATE_TASIYICI_INIS:
			sprintf(CURRENT_STATE_STRING, STATE_TASIYICI_INIS_STRING);
			break;
		case STATE_AYRILMA:
			sprintf(CURRENT_STATE_STRING, STATE_AYRILMA_STRING);
			break;
		case STATE_AKTIF_INIS:
			sprintf(CURRENT_STATE_STRING, STATE_AKTIF_INIS_STRING);
			break;
		case STATE_IRTIFA_SABITLEME:
			sprintf(CURRENT_STATE_STRING, STATE_IRTIFA_SABITLEME_STRING);
			break;
		case STATE_AKTIF_INIS_2:
			sprintf(CURRENT_STATE_STRING, STATE_AKTIF_INIS_2_STRING);
			break;
		case STATE_LANDING_1:
			sprintf(CURRENT_STATE_STRING, STATE_LANDING_1_STRING);
			break;
		case STATE_LANDING_2:
			sprintf(CURRENT_STATE_STRING, STATE_LANDING_2_STRING);
			break;
		case STATE_LANDING_3:
			sprintf(CURRENT_STATE_STRING, STATE_LANDING_3_STRING);
			break;
		case STATE_SALVAGE:
			sprintf(CURRENT_STATE_STRING, STATE_SALVAGE_STRING);
			break;
	}

}

// Statü geçişlerini kontrol eder. Ayrılma ve aktif iniş kontrolleri için statü kontrollerini de gerçekleştirir.
void CHECK_STATE(void)
{
	// Statü geçişleri
	if(STATE_TRANSITION_BEKLEME)
	{
		CURRENT_STATE = STATE_BEKLEME;
		sprintf(CURRENT_STATE_STRING, STATE_BEKLEME_STRING);
		initPID(&V_PID, 5, 0.01, 0, -200, 200, DESIRED_SPEED_AKTIF_INIS, 20);
	}

	if(STATE_TRANSITION_YUKSELME)
	{
		CURRENT_STATE = STATE_YUKSELME;
		sprintf(CURRENT_STATE_STRING, STATE_YUKSELME_STRING);
	}

	if(STATE_TRANSITION_TASIYICI_INIS)
	{
		CURRENT_STATE = STATE_TASIYICI_INIS;
		sprintf(CURRENT_STATE_STRING, STATE_TASIYICI_INIS_STRING);

	}

	if(STATE_TRANSITION_AYRILMA)
	{
		CURRENT_STATE = STATE_AYRILMA;
		sprintf(CURRENT_STATE_STRING, STATE_AYRILMA_STRING);
	}

	if(STATE_TRANSITION_AKTIF_INIS)
	{
		CURRENT_STATE = STATE_AKTIF_INIS;
		sprintf(CURRENT_STATE_STRING, STATE_AKTIF_INIS_STRING);
		initPID(&V_PID, 20, 0.01, 0, -500, 500, DESIRED_SPEED_AKTIF_INIS, 20);
	}

	if(STATE_TRANSITION_IRTIFA_SABITLEME)
	{
		CURRENT_STATE = STATE_IRTIFA_SABITLEME;
		sprintf(CURRENT_STATE_STRING, STATE_IRTIFA_SABITLEME_STRING);
		initPID(&V_PID, 25, 0.01, 0, -500, 500, DESIRED_SPEED_IRTIFA_SABITLEME, 20);
		IRTIFA_SABITLEME_TIMER = HAL_GetTick();
	}

	if(STATE_TRANSITION_AKTIF_INIS_2)
	{
		CURRENT_STATE = STATE_AKTIF_INIS_2;
		sprintf(CURRENT_STATE_STRING, STATE_AKTIF_INIS_2_STRING);
		initPID(&V_PID, 20, 0.01, 0, -500, 500, DESIRED_SPEED_AKTIF_INIS, 20);
	}

	if(STATE_TRANSITION_LANDING_1)
	{
		CURRENT_STATE = STATE_LANDING_1;
		sprintf(CURRENT_STATE_STRING, STATE_LANDING_1_STRING);
		initPID(&V_PID, 20, 0.01, 0, -500, 500, DESIRED_SPEED_LANDING_1, 20);
	}

	if(STATE_TRANSITION_LANDING_2)
	{
		CURRENT_STATE = STATE_LANDING_2;
		sprintf(CURRENT_STATE_STRING, STATE_LANDING_2_STRING);
		initPID(&V_PID, 25, 0.01, 0, -500, 500, DESIRED_SPEED_LANDING_2, 20);
	}

	if(STATE_TRANSITION_LANDING_3)
	{
		CURRENT_STATE = STATE_LANDING_3;
		sprintf(CURRENT_STATE_STRING, STATE_LANDING_3_STRING);
		initPID(&V_PID, 30, 0.01, 0, -500, 500, DESIRED_SPEED_LANDING_3, 20);
	}

	if(STATE_TRANSITION_SALVAGE)
	{
		CURRENT_STATE = STATE_SALVAGE;
		sprintf(CURRENT_STATE_STRING, STATE_SALVAGE_STRING);
		initPID(&V_PID, 0, 0.00, 0, -200, 200, 0, 20);
		resetPID(&V_PID);
		setMotorThrottle(mt1, 1000);
		setMotorThrottle(mt2, 1000);
	}


	// Ayrılma ve motor kontrolleri
	if(CURRENT_STATE == STATE_AYRILMA)
	{
		DETACH();
	}

}

void DETACH(void)
{
	DETACH_CONTROL = 1;
	setMotorThrottle(servo, 2000);
}

void ATTACH(void)
{
	DETACH_CONTROL = 0;
	setMotorThrottle(servo, 1000);
}

char * str_xbee;
void GET_DATA(void)
{

	if(HAL_GetTick()-LOOP_TIMER_1HZ > 920)
	{
		GPS_Process((char)*RAW_GPS);
		GET_CMD();
		PACKAGE_COUNT++;
		BMX160_getEulerAngles(&ROLL, &PITCH, &YAW);
		BMP388_readData(&BMP388_T, &BMP388_P);
		BMP388_PREV_ALT = BMP388_ALT;
		BMP388_ALT = BMP388_readAltitude(BMP388_P, 1013) - BMP388_refAlt;
		BMP388_VELOCITY = BMP388_ALT - BMP388_PREV_ALT;
		HAL_ADC_Start_IT(&hadc1);
		EEPROM_1HZ();
		DS3231_getTime();

		if(dataArrived)
		{

			str_xbee = strstr(xbee_rx_buffer,"395669,");
			sscanf(str_xbee,"395669,%f,%f,%f,%f\r\n", &tPressure, &tLatitude, &tLongitude, &tGPSAltitude);
			memset(xbee_rx_buffer,1,256);
			tAltitude = BMP388_readAltitude(tPressure, 1013);
			tAltitude = tAltitude - BMP388_refAlt - 55.0;
			subtraction = tAltitude - BMP388_ALT;
			//sprintf
			/*snprintf(XBee_Buffer,256, "58479,%d, %d/%d/%d, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.2f, %f, %f, %.1f, %s, %.0f, %.0f, %.0f, 0, %d, %s\r\n",
			PACKAGE_COUNT,rtc.hour,rtc.min,rtc.sec,BMP388_P, tPressure, BMP388_ALT, tAltitude, VELOCITY,BMP388_T,VOLTAGE,hgps
			.GPGGA.LatitudeDecimal,hgps.GPGGA.LongitudeDecimal,hgps.GPGGA.MSL_Altitude,CURRENT_STATE_STRING,
			PITCH,ROLL,YAW,RotationCount,VIDEO_STATE);*/

			//snprintf(XBee_Buffer,256, "58479,%d,%d/%d/%d\r\n",PACKAGE_COUNT,rtc.hour,rtc.min,rtc.sec);



			snprintf(XBee_Buffer,256, "395669,%d,%d:%d:%d,%.1f, %.1f, %.1f,%.1f, %.1f, %.1f,%.1f,%.2f,%f,%f,%.1f,%.1f,%.1f,%.1f,%s,%.0f,%.0f,%.0f,%d,%s\r\n",
			PACKAGE_COUNT,rtc.hour,rtc.min,rtc.sec,BMP388_P,tPressure, BMP388_ALT,tAltitude, subtraction,VELOCITY,BMP388_T,VOLTAGE,hgps
			.GPGGA.LatitudeDecimal,hgps.GPGGA.LongitudeDecimal,hgps.GPGGA.MSL_Altitude,tLatitude, tLongitude, tGPSAltitude,CURRENT_STATE_STRING,
			PITCH,ROLL,YAW,RotationCount,VIDEO_STATE);




			//snprintf(XBee_Buffer,256 ,"58479, %d\r\n",10);
			dataArrived = 0;

			//memset(xbee_rx_buffer,0,sizeof(xbee_rx_buffer));
		}
		else
		{
			snprintf(XBee_Buffer,256, "395669,%d, %d:%d:%d, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.2f, %f, %f, %.1f, %s, %.0f, %.0f, %.0f, %d, %s\r\n",
						PACKAGE_COUNT,rtc.hour,rtc.min,rtc.sec,BMP388_P, tPressure, BMP388_ALT, tAltitude, subtraction, VELOCITY,BMP388_T,VOLTAGE,hgps
						.GPGGA.LatitudeDecimal,hgps.GPGGA.LongitudeDecimal,hgps.GPGGA.MSL_Altitude,CURRENT_STATE_STRING,
						PITCH,ROLL,YAW,RotationCount,VIDEO_STATE);
		}


		status = HAL_UART_Transmit_IT(&huart1, (uint8_t*)XBee_Buffer, strlen(XBee_Buffer));


		SD_write(XBee_Buffer);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if(CURRENT_STATE == STATE_SALVAGE)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		}
		LOOP_TIMER_1HZ = HAL_GetTick();
	}

	if(HAL_GetTick()-LOOP_TIMER_50HZ > 20)
	{
		CHECK_STATE();
		READ_BAROMETER();
		VELOCITY = (ACTUAL_ALTITUDE - PREV_ALTITUDE)*50; //(ACTUAL_PRESSURE - ACTUAL_PREV_PRESS)/11.7*50
		LOOP_TIMER_50HZ = HAL_GetTick();
		PREV_ALTITUDE = ACTUAL_ALTITUDE;
		ACTUAL_PREV_PRESS = ACTUAL_PRESSURE;

	}
}

void PID_LOOP(void)
{
	if(DETACH_CONTROL && CURRENT_STATE < STATE_SALVAGE && CURRENT_STATE > STATE_AYRILMA)
	{
		if(MOTOR_CONTROL_1 && !MOTOR_CONTROL_2)
		{
			setMotorThrottle(mt1, CENTER_THROTTLE);
		}

		if(MOTOR_CONTROL_1 && MOTOR_CONTROL_2)
		{
			computePID(&V_PID,VELOCITY);
			THROTTLE = CENTER_THROTTLE + V_PID.Output;
			if(THROTTLE >= THROTTLE_MAX) THROTTLE = THROTTLE_MAX;
			if(THROTTLE <= THROTTLE_MIN) THROTTLE = THROTTLE_MIN;
			setMotorThrottle(mt1, THROTTLE);
			setMotorThrottle(mt2, THROTTLE);
		}
	}
}

void READ_BAROMETER(void)
{
	BAROMETER_COUNTER++;
	MS5611_read_temp();
	MS5611_read_press();
	MS5611_calculate();
	PRESS_TOTAL_AVG -= PRESS_ROTATING_MEM[PRESS_MEM_LOCATION];
	PRESS_ROTATING_MEM[PRESS_MEM_LOCATION] = ms5611_t.P;
	PRESS_TOTAL_AVG += PRESS_ROTATING_MEM[PRESS_MEM_LOCATION];
	PRESS_MEM_LOCATION++;
	if(PRESS_MEM_LOCATION == 20) PRESS_MEM_LOCATION = 0;
	ACTUAL_PRESSURE_FAST = (float)PRESS_TOTAL_AVG/20.0;
	ACTUAL_PRESSURE_SLOW = ACTUAL_PRESSURE_SLOW*SLOW_FILTER_COEFFICIENT + ACTUAL_PRESSURE_FAST*FAST_FILTER_COEFFICIENT;
	ACTUAL_PRESSURE_DIFF = ACTUAL_PRESSURE_SLOW-ACTUAL_PRESSURE_FAST;
	if (ACTUAL_PRESSURE_DIFF > 8)ACTUAL_PRESSURE_DIFF = 8;
	if (ACTUAL_PRESSURE_DIFF < -8)ACTUAL_PRESSURE_DIFF = -8;
	if (ACTUAL_PRESSURE_DIFF > 1 || ACTUAL_PRESSURE_DIFF < -1) ACTUAL_PRESSURE_SLOW -= ACTUAL_PRESSURE_DIFF / 6.0;
	ACTUAL_PRESSURE = ACTUAL_PRESSURE_SLOW;
	ACTUAL_ALTITUDE = MS5611_readAlt(ACTUAL_PRESSURE) - MS5611_refAlt;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_DMA(&huart2, RAW_GPS, 512);
		GPS_CONTROL = 1;
	}

	if(huart->Instance == huart1.Instance)
	{
		if(rxBuffer != 0) xbee_rx_buffer[rxBufferCounter] = rxBuffer;
		rxBufferCounter++;
		HAL_UART_Receive_IT(&huart1, &rxBuffer, 1);
		if(rxBuffer == 10)
		{
			rxBufferCounter = 0;
			dataArrived = 1;
		}
		if(rxBuffer == 126)
		{
			GCS_Command = xbee_rx_buffer[rxBufferCounter-2];
			rxBufferCounter = 0;
		}
	}
}

void SD_write(char * str)
{
	fresult = f_open(&fil, "Telemetri.txt", FA_OPEN_ALWAYS|FA_READ|FA_WRITE);
	fresult = f_lseek(&fil,f_size(&fil));
	//fresult = f_puts(str,&fil);
	fresult = f_write(&fil,str,strlen(str),&bw);
	fresult = f_close(&fil);
}

void SD_Mount(void)
{
	fresult = f_mount(&fs,"",0);
}

void GET_CMD(void)
{
	switch(GCS_Command)
	{
		// Ayrılma komutu
		case 'A' :
			DETACH();
			break;

		// Birleşme komutu
		case 'B':
			ATTACH();
			break;

		// Motor tahrik
		case 'T':
			TEST_MOTORS();
			break;
		case 'D':
			setMotorThrottle(mt1, 1000);
			setMotorThrottle(mt2, 1000);
			MOTOR_TEST_FLAG = 0;
			break;
		// Sensör kalibrasyonu
		case 'K':
			break;

		case 'Z':
			EEPROM_PageErase(0);
			EEPROM_PageErase(1);
			EEPROM_PageErase(2);
			MOTOR_CONTROL_1 = 0;
			MOTOR_CONTROL_2 = 0;
			setMotorThrottle(mt1, 1000);
			setMotorThrottle(mt2, 1000);
			resetPID(&V_PID);
			while(1);
			break;
		// Video aktarımı
		case 'V':
			sprintf(VIDEO_STATE,"EVET");
			break;
		default: GCS_Command = 0;
	}
	GCS_Command = 0;
}

void TEST_MOTORS(void)
{
	if(1)
	{
		setMotorThrottle(mt1, 1200);
		TEST_TIME = HAL_GetTick();
		MOTOR_TEST_FLAG = 1;
	}
}

void EEPROM_1HZ(void)
{
	EEPROM_Write_NUM(0, EEPROM_OFFSET_PACKAGE, (float)PACKAGE_COUNT);
	EEPROM_Write(0, EEPROM_OFFSET_STATE, &CURRENT_STATE, 1);
	if(PRESS_MEM_LOCATION >= 10)
	{
		EEPROM_Write_NUM(2, (PRESS_MEM_LOCATION-10)*4, (float)PRESS_ROTATING_MEM[PRESS_MEM_LOCATION]);
	}
	else
	{
		EEPROM_Write_NUM(1, PRESS_MEM_LOCATION*4, (float)PRESS_ROTATING_MEM[PRESS_MEM_LOCATION]);
	}

	EEPROM_Write_NUM(0, EEPROM_OFFSET_PRESS_TOTAL_AVG, (float)PRESS_TOTAL_AVG);
	EEPROM_Write_NUM(0, EEPROM_OFFSET_ACTUAL_PRESS_FAST, (float)ACTUAL_PRESSURE_FAST);
	EEPROM_Write_NUM(0, EEPROM_OFFSET_ACTUAL_PRESS_SLOW, (float)ACTUAL_PRESSURE_SLOW);
	EEPROM_Write_NUM(0, EEPROM_OFFSET_ACTUAL_PRESSURE_DIFF, (float)ACTUAL_PRESSURE_DIFF);
	EEPROM_Write_NUM(0, EEPROM_OFFSET_PRESS_MEM_LOC, (float)PRESS_MEM_LOCATION);
	EEPROM_Write_NUM(0, EEPROM_OFFSET_ACTUAL_PRESS_LOC, (float)ACTUAL_PRESSURE);
	EEPROM_Write_NUM(0, EEPROM_OFFSET_BAROMETER_CNT, (float)BAROMETER_COUNTER);
}

void GET_GPS(void)
{
	if(GPS_CONTROL)
	{
		GPS_Process((char)*RAW_GPS);
		GPS_CONTROL = 0;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
