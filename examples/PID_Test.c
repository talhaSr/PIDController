/*
 * PID_Test.c
 *
 *		Revision Date: 13/01/2021
 *		Author: Talha
 */

#include <stdio.h>
#include <stdlib.h>
#include <PID.h>

#define PID_KP 2.0f
#define PID_KD 0.5f
#define PID_KI 0.25f

float TestSim_Update(float inp);

int main()
{

    PIDController_Init(&pid, PID_KP, PID_KI, PID_KD);

    uint16_t setpoint = 21;

    printf("T (s)\tSystem Out\tPID Out\r\n");
    for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S)
    {
        float meas = TestSim_Update(pid.out);
        PIDController_Update(&pid, setpoint, meas);
        printf("%f\t%f\t%f\r\n", t, meas, pid.out);
    }
    
    return 0;
}

float TestSim_Update(float inp)
{
    static float out = 0.0f;
    static const float alpha = 0.02f;

    out = (SAMPLE_TIME_S * inp + out) / (1.0f + alpha * SAMPLE_TIME_S);
    return out;
}