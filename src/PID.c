/**
 * @file PID.c
 * @author Talha Sarı (talha.sari@outlook.com.tr)
 * @brief PID Controller for Discrete Time Systems
 * @version v1.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "PID.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float Ts)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->tau = PID_TAU;
	pid->limMin = PID_LIM_MIN;
	pid->limMax = PID_LIM_MAX;
	pid->limMinInt = PID_LIM_MIN_INT;
	pid->limMaxInt = PID_LIM_MAX_INT;
	pid->T = Ts;

	pid->integrator = 0.0f;
	pid->prevError = 0.0f;
	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out = 0.0f;
}

float PID_Update(PID_t *pid, float setPoint, float measurement)
{
	//Hata
	float error = setPoint - measurement;

	//P
	float proportional = pid->Kp * error;

	//I
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	//Anti wind-up
	if (pid->integrator > pid->limMaxInt)
		pid->integrator = pid->limMaxInt;
	else if (pid->integrator < pid->limMinInt)
		pid->integrator = pid->limMinInt;

	//D
	pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator) / (2.0f * pid->tau + pid->T);


	//Çıkışı sinyalini hesapla
	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax)
		pid->out = pid->limMax;
	else if (pid->out < pid->limMin)
		pid->out = pid->limMin;

	//Ekstra
	pid->prevError = error;
	pid->prevMeasurement = measurement;

	return pid->out;
}

void PID_SetLimits(PID_t *pid, float outLimMin, float outLimMax, float intLimMin, float intLimMax)
{
	pid->limMin = outLimMin;
	pid->limMax = outLimMax;
	pid->limMinInt = intLimMin;
	pid->limMaxInt = intLimMax;
}

void PID_SetDerivativeFilter(PID_t *pid, float tau)
{
	pid->tau = tau;
}