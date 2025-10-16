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
void Mecanum_Control(double vx_target, double vy_target, double w_target);
void Emergency_Stop(void);

// ----------------------------------------------------------------------------------
// ----------- PID Controller Structure ---------------------------------------------
// ----------------------------------------------------------------------------------
typedef struct {
    float Kc;               // Controller gain
    float tau_i;            // Integral time constant (s)
    float tau_d;            // Derivative time constant (s)

    float cof_A;            // Discrete PID coefficients
    float cof_B;
    float cof_C;

    float last_error;       // e(k-1)
    float prev_error;       // e(k-2)
    float prev_output;      // u(k-1)
    float output;           // Current output

    float output_max;		// Output limits
    float output_min;
} PID_t;

void PID_Init(float Kp, float Ki, float Kd);
float PID_Update(PID_t *pid, float setpoint, float input);



#endif /* MOTOR_DRIVER_H_ */
