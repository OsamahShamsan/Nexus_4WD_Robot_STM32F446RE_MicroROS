#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "main.h"


// ----------------------------------------------------------------------------------
// ----------- Motors control   ---------------------------------
// ----------------------------------------------------------------------------------
// Defaults; can be changed at runtime (per control tick, i.e., every 10 ms)
extern volatile float g_v_step;     // mm/s per tick
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

    float setpoint;
    float feedback;
    float output;

    float prev_error;
    float integral;
} PID_t;


void PID_Init(PID_t *pid, float Kp, float Ki, float Kd);
float PID_Update(PID_t *pid, float setpoint, float feedback, float dt);



#endif /* MOTOR_DRIVER_H_ */
