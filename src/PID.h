/*
 * PID.h
 *
 *		Revision Date: 13/01/2021
 *		Author: Talha
 */

#ifndef INC_PID_H_
#define INC_PID_H_

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

} PIDKontrolcu_t;

void PIDKontrolcu_Init(PIDKontrolcu_t *pid);
int16_t PIDKontrolcu_Update(PIDKontrolcu_t *pid, uint16_t setPoint, uint16_t measurement);


#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
