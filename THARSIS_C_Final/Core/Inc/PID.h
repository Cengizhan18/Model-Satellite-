#include "math.h"
#include "stm32f4xx_hal.h"


typedef struct _PID {

	// PID constants
	double Kp;
	double Ki;
	double Kd;

	double error;
	double previousError;
	double real_value;
	double desired_value;
	double Output;
	double outSum;

	double lastUpdate;
	uint16_t sampleTime;

	double outputMin;
	double outputMax;

} PID_Object;

void resetPID(PID_Object * PID_Obj);
void setPIDSampleTime(PID_Object * PID_Obj,uint16_t time);
void setPIDDesiredValue(PID_Object * PID_Obj,double dv);
void setPIDConstants(PID_Object * PID_Obj,double kp,double ki,double kd);
void setPIDLimits(PID_Object * PID_Obj,double max,double min);
void initPID(PID_Object * PID_Obj,double kp,double ki,double kd,double outMin,double outMax,double dv,uint16_t sampletime);
void computePID(PID_Object * PID_Obj,double real_value);




