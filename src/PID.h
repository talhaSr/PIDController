#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float tau;
    float limMinInt;
    float limMaxInt;
    float T;
    float integrator;
    float prevError;
    float differantiator;
    float prevMeasurement;
    float out;
} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float set, float);

#endif