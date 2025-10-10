#ifndef MYMOTOR_H_
#define MYMOTOR_H_

#include "main.h"


// ----------------------------------------------------------------------------------
// ----------- Motors control   ---------------------------------
// ----------------------------------------------------------------------------------
// Defaults; can be changed at runtime (per control tick, i.e., every 10 ms)
extern volatile float g_v_step_mmps;     // mm/s per tick
extern volatile float g_wz_step_radps;   // rad/s per tick
extern volatile int32_t g_ccr_applied[4];


void ctrlparams_set_steps(float v_step_mmps, float wz_step_radps);

void init_motors(void);
void Mecanum_Control(float vx_target, float vy_target, float w_target);
void Emergency_Stop(void);



// ----------------------------------------------------------------------------------
// ----------- PID Controller Structure ---------------------------------------------
// ----------------------------------------------------------------------------------
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement, float dt);



#endif /* MYMOTOR_H_ */
