#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

typedef struct mot
{
	TIM_HandleTypeDef * htim;
	uint8_t channel;
	uint16_t throttle;

}Motor;

typedef struct dr
{
	Motor mt1,mt2,mt3,mt4;

}quad;


Motor setMotor(TIM_HandleTypeDef * htim, uint8_t channel);
void startMotor(Motor mt);
void stopMotor(Motor mt);
void setMotorThrottle(Motor mt, uint16_t thr);

quad setQuad(Motor mt1,Motor mt2,Motor mt3,Motor mt4);
void startQuad(quad qd);
void stopQuad(quad qd);
void setQuadThrottle(quad qd,uint16_t th1,uint16_t th2,uint16_t th3,uint16_t th4);
