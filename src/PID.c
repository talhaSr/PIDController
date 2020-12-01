#include "PID.h"

void PIDController_Init(PIDController *pid)
{
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differantiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    
    float proportional = pid->Kp * error;
    
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    if (pid->integrator > pid->limMaxInt) { pid->integrator = pid->limMaxInt; }
    else if (pid->integrator < pid->limMinInt) { pid->integrator = pid->limMinInt; }

    pid->differantiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)) + ((2.0f * pid->tau - pid->T) * pid->differantiator) / (2.0f * pid->tau + pid->T);

    if (pid->out > pid->limMaxInt) { pid->out = pid->limMaxInt; }
    else if (pid->out < pid->limMinInt) { pid->out = pid->limMinInt; }

    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return pid->out;
}