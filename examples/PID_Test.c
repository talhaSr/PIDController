#include <stdio.h>
#include <stdlib.h>
#include <PID.h>

#define PID_KP 2.0f
#define PID_KD 0.5f
#define PID_KI 0.25f
#define PID_TAU 0.02f
#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f
#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f
#define SAMPLE_TIME_S 0.01f
#define SIMULATION_TIME_MAX 4.0f

float TestSim_Update(float inp);

int main()
{
    PIDController pid = { 
        PID_KP, PID_KD, PID_KI,
        PID_TAU,
        PID_LIM_MAX, PID_LIM_MIN,
        PID_LIM_MAX_INT, PID_LIM_MIN_INT,
        SAMPLE_TIME_S
     };

    PIDController_Init(&pid);

    float setpoint = 1.0f;

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