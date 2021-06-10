/**
 * @file PID.h
 * @author Talha Sarı (talha.sari@outlook.com.tr)
 * @brief PID Controller for Discrete Time Systems
 * @version v1.1
 * @date 2021-06-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_TAU 0.02f
#define PID_LIM_MIN -100.0f
#define PID_LIM_MAX 100.0f
#define PID_LIM_MIN_INT -100.0f
#define PID_LIM_MAX_INT 100.0f
#define SAMPLE_TIME_S 0.01f

typedef struct {

	//Kontrolcü katsayıları
	float Kp;
	float Ki;
	float Kd;

	//Derivatif low-pass filtre zaman sabiti
	float tau;

	//Çıkış limitleri
	float limMin;
	float limMax;

	//Integrator limitleri
	float limMinInt;
	float limMaxInt;

	//Örnek süresi (s)
	float T;

	//Kontrolcü Hafızası
	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;

	//Kontrolcü Çıkışı
	float out;

} PID_t;

/**
 * @brief PID Controller Initializer
 * 
 * @param pid PID Handler TypeDef
 * @param kp  Proportional
 * @param ki  Integral
 * @param kd  Derivative
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float Ts);

/**
 * @brief PID Controller Updater
 * 
 * @param pid		  PID Handler TypeDef
 * @param setPoint    Set (Reference) Point
 * @param measurement Measurement Value - x[n]
 * @return float 
 */
float PID_Update(PID_t *pid, float setPoint, float measurement);

/**
 * @brief PID Output and Integral Anti-windup Limiting Function
 * 
 * @param pid 		PID Handler TypeDef
 * @param outLimMin Minimum Signal Output
 * @param outLimMax Maximum Signal Output
 * @param intLimMin Minimum Integral Anti-windup Value
 * @param intLimMax Maximum Integral Anti-windup Value
 */
void PID_SetLimits(PID_t *pid, float outLimMin, float outLimMax, float intLimMin, float intLimMax);

/**
 * @brief PID Derivative's Filter Set for Noise
 * 
 * @param pid PID Handler TypeDef
 * @param tau Filter Coefficient
 */
void PID_SetDerivativeFilter(PID_t *pid, float tau);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
