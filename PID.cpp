#include "PID.h"
#include <inttypes.h>
void PID::Compute()
{
		/*Compute all the working error variables*/
		float error = Setpoint - Input;
		ITerm+= (ki * error);
		if(ITerm > outMax) ITerm= outMax;
		else if(ITerm < outMin) ITerm= outMin;
		
		/*Compute PID Output*/
		Output = kp * error + ITerm + kd * dInput;
		if(Output > outMax) Output = outMax;
		else if(Output < outMin) Output = outMin;
}

void PID::SetTunings(float Kp, float Ki, float Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

void PID::SetOutputLimits(long Min, long Max)
{
	if(Min > Max) return;
	outMin = Min;
	outMax = Max;
}
void PID::Initialize()
{
	lastInput = Input;
	ITerm = Output;
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
}