#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "i2c.h"

#define hi2cX hi2c1
#define DS3231_ADDRESS  0x68

// timekeeping registers
#define DS3231_TIME_CAL         0x00
#define DS3231_ALARM1           0x07
#define DS3231_ALARM2           0x0B
#define DS3231_CONTROL          0x0E
#define DS3231_STATUS           0x0F
#define DS3231_AGING_OFFSET     0x10
#define DS3231_TEMPERATURE      0x11

// control register bits
#define DS3231_CONTROL_A1IE     0x1		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_A2IE     0x2		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_INTCN    0x4		/* Interrupt Control */
#define DS3231_CONTROL_RS1	    0x8		/* square-wave rate select 2 */
#define DS3231_CONTROL_RS2    	0x10	/* square-wave rate select 2 */
#define DS3231_CONTROL_CONV    	0x20	/* Convert Temperature */
#define DS3231_CONTROL_BBSQW    0x40	/* Battery-Backed Square-Wave Enable */
#define DS3231_CONTROL_EOSC	    0x80	/* not Enable Oscillator, 0 equal on */


// status register bits
#define DS3231_STATUS_A1F       0x01		/* Alarm 1 Flag */
#define DS3231_STATUS_A2F       0x02		/* Alarm 2 Flag */
#define DS3231_STATUS_BUSY      0x04		/* device is busy executing TCXO */
#define DS3231_STATUS_EN32KHZ   0x08		/* Enable 32KHz Output  */
#define DS3231_STATUS_OSF       0x80		/* Oscillator Stop Flag */


typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t dayofweek;
}DS3231;

typedef enum{
	monday    = 1,
	tuesday   = 2,
	wednesday = 3,
	thursday  = 4,
	friday    = 5,
	saturday  = 6,
	sunday    = 7
}DS3231_days;

uint8_t DS3231_BCD2DEC(uint8_t data);
uint8_t DS3231_DEC2BCD(uint8_t data);
void DS3231_getTime();
void DS3231_setTime(DS3231 dt);
void DS3231_writeByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * wdata,uint8_t size);
void DS3231_readByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * rdata,uint8_t size);
