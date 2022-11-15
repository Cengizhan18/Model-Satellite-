#include "PID.h"

/* Resets the PID controller
 * PID_Obj : Pointer of the PID object
 */
void resetPID(PID_Object * PID_Obj)
{
	PID_Obj->Kp = 0;
	PID_Obj->Ki = 0;
	PID_Obj->Kd = 0;

	PID_Obj->desired_value = 0;
	PID_Obj->real_value = 0;
	PID_Obj->error = 0;
	PID_Obj->previousError = 0;
	PID_Obj->lastUpdate = 0;
}


/* Sets the sample time of the PID controller
 * PID_Obj : Pointer of the PID object
 * time : Desired sample time in milliseconds
 */
void setPIDSampleTime(PID_Object * PID_Obj,uint16_t time)
{
	PID_Obj->sampleTime = time;
}


/* Sets the desired value of the PID controller
 * PID_Obj : Pointer of the PID object
 * dv : Desired value
 */
void setPIDDesiredValue(PID_Object * PID_Obj,double dv)
{
	PID_Obj->desired_value = dv;
}


/* Sets PID constants
 * PID_Obj : Pointer of the PID object
 */
void setPIDConstants(PID_Object * PID_Obj,double kp,double ki,double kd)
{
	PID_Obj->Kp = kp;
	PID_Obj->Ki = ki;
	PID_Obj->Kd = kd;
}

/* Sets PID limits
 * PID_Obj : Pointer of the PID object
 */
void setPIDLimits(PID_Object * PID_Obj,double max,double min)
{
	PID_Obj->outputMax = max;
	PID_Obj->outputMin = min;
}


/* Initializes the PID
 * Sets constants, min/max values, sample time
 */
void initPID(PID_Object * PID_Obj,double kp,double ki,double kd,double outMin,double outMax,double dv,uint16_t sampletime)
{
	PID_Obj->Kp = kp;
	PID_Obj->Ki = ki;
	PID_Obj->Kd = kd;

	PID_Obj->outputMax = outMax;
	PID_Obj->outputMin = outMin;
	PID_Obj->outSum = 0;

	PID_Obj->error = 0;
	PID_Obj->previousError= 0;
	PID_Obj->sampleTime = sampletime;
	PID_Obj->lastUpdate = 0;
	PID_Obj->desired_value = dv;
}

/* Computes the PID output
 * PID_Obj : Pointer of the PID object
 * input : Measured feedback value
 */
void computePID(PID_Object * PID_Obj,double real_value)
{
	PID_Obj->real_value = real_value;
	uint32_t now = HAL_GetTick();
	uint32_t dt = now - PID_Obj->lastUpdate;

	if(dt >= PID_Obj->sampleTime)
	{
		PID_Obj->error = PID_Obj->desired_value - PID_Obj->real_value;
		double dError = (PID_Obj->error - PID_Obj->previousError);
		PID_Obj->outSum += PID_Obj->Ki * PID_Obj->error;

		if(PID_Obj->outSum > PID_Obj->outputMax) PID_Obj->outSum = PID_Obj->outputMax;
		if(PID_Obj->outSum < PID_Obj->outputMin) PID_Obj->outSum = PID_Obj->outputMin;

		PID_Obj->Output = (PID_Obj->Kp * PID_Obj->error) + (PID_Obj->outSum) + (PID_Obj->Kd * dError / dt);

		if(PID_Obj->Output > PID_Obj->outputMax) PID_Obj->Output = PID_Obj->outputMax;
		if(PID_Obj->Output < PID_Obj->outputMin) PID_Obj->Output = PID_Obj->outputMin;


		PID_Obj->lastUpdate = now;
		PID_Obj->previousError = PID_Obj->error;

		return;
	}
	else
	{
		return;
	}
}
