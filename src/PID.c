/*
 * PID.c
 *
 *		Revision Date: 13/01/2021
 *		Author: Talha
 */

#include "PID.h"

void PIDController_Init(PIDController *pid, float kp, float ki, float kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->tau = PID_TAU;
	pid->limMin = PID_LIM_MIN;
	pid->limMax = PID_LIM_MAX;
	pid->limMinInt = PID_LIM_MIN_INT;
	pid->limMaxInt = PID_LIM_MAX_INT;
	pid->T = SAMPLE_TIME_S;

	pid->integrator = 0.0f;
	pid->prevError = 0.0f;
	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out = 0.0f;
}

int16_t PIDKontrolcu_Update(PIDKontrolcu_t *pid, uint16_t setPoint, uint16_t measurement);
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

	return (int16_t)pid->out;
}