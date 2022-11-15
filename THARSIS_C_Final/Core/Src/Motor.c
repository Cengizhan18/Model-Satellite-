#include "Motor.h"

Motor setMotor(TIM_HandleTypeDef * htim, uint8_t channel)
{
	Motor mot;

	mot.htim = htim;
	mot.channel = channel;

	return mot;
}

void startMotor(Motor mt)
{
	HAL_TIM_PWM_Start(mt.htim, mt.channel);
}

void stopMotor(Motor mt)
{
	HAL_TIM_PWM_Stop(mt.htim, mt.channel);
}

void setMotorThrottle(Motor mt, uint16_t thr)
{
	__HAL_TIM_SET_COMPARE(mt.htim,mt.channel,thr);
}

quad setQuad(Motor mt1,Motor mt2,Motor mt3,Motor mt4)
{
	quad qd;

	qd.mt1 = mt1;
	qd.mt2 = mt2;
	qd.mt3 = mt3;
	qd.mt4 = mt4;

	return qd;
}

void startQuad(quad qd)
{
	startMotor(qd.mt1);
	startMotor(qd.mt2);
	startMotor(qd.mt3);
	startMotor(qd.mt4);
}

void stopQuad(quad qd)
{
	stopMotor(qd.mt1);
	stopMotor(qd.mt2);
	stopMotor(qd.mt3);
	stopMotor(qd.mt4);
}

void setQuadThrottle(quad qd,uint16_t th1,uint16_t th2,uint16_t th3,uint16_t th4)
{
	setMotorThrottle(qd.mt1,th1);
	setMotorThrottle(qd.mt2,th2);
	setMotorThrottle(qd.mt3,th3);
	setMotorThrottle(qd.mt4,th4);
}

