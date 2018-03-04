#include <inttypes.h>
class PID{
	float lastTime;
	float ITerm, lastInput;
	float kp, ki, kd;
	float outMin, outMax;

	public:
	float Input;
	float dInput;
	float Output;
	float Setpoint;
	void Compute();
	void SetTunings(float Kp, float Ki, float Kd);
	void SetOutputLimits(long Min, long Max);
	void Initialize();

	};