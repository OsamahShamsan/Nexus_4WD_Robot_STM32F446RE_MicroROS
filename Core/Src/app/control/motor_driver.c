#include "motor_driver.h"
#include "robot_params.h"

#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>

/*
// ----------------------------------------------------------------------------------
// ----------- Motor control helpers  -----------------------------------------------
// ----------------------------------------------------------------------------------
inline float clamp(float v, float lo, float hi);
static inline float ramp(float target, float current, float step);
static inline float ramp_and_clamp_sym(float target, float current, float step, float limit);
static inline void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, float speed);

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

volatile float g_v_step   = 20.0f;
volatile float g_wz_step_radps = 0.10f;





static inline float ramp(float target, float current, float step)
{
    return (target > current) ? fminf(current + step, target)
         : (target < current) ? fmaxf(current - step, target)
         : current;
}

static inline float ramp_and_clamp_sym(float target, float current, float step, float limit)
{
    if (target > current)
        current = fminf(current + step, target);
    else if (target < current)
        current = fmaxf(current - step, target);

    // clamp symmetric
    if (current > limit)
        current = limit;
    else if (current < -limit)
        current = -limit;

    return current;
}


void ctrlparams_set_steps(float v_step, float wz_step_radps)
{
    if (v_step  > 0.0f) g_v_step   = v_step;
    if (wz_step_radps > 0.0f) g_wz_step_radps = wz_step_radps;
}



// ----------------------------------------------------------------------------------
// ----------- Motors control functions definitions  ---------------------------------
// ----------------------------------------------------------------------------------
*/

/*
void Mecanum_Control(float vx_twist, float vy_twist, float wz_twist)
{

    static float vx_twist_ramped = 0, vy_twist_ramped = 0, wz_twist_ramped = 0;
    vx_twist_ramped = ramp(vx_twist, vx_twist_ramped, g_v_step);
    vy_twist_ramped = ramp(vy_twist, vy_twist_ramped, g_v_step);
    wz_twist_ramped = ramp(wz_twist, wz_twist_ramped, g_wz_step_radps);

    // Desired wheel angular velocities
    float w_target[4];
    w_target[0] = (+vx_twist_ramped + vy_twist_ramped - (A_SUM * wz_twist_ramped)) / WHEEL_R;  // RL
    w_target[1] = (+vx_twist_ramped - vy_twist_ramped - (A_SUM * wz_twist_ramped)) / WHEEL_R;  // FL
    w_target[2] = (+vx_twist_ramped + vy_twist_ramped + (A_SUM * wz_twist_ramped)) / WHEEL_R;  // FR
    w_target[3] = (+vx_twist_ramped - vy_twist_ramped + (A_SUM * wz_twist_ramped)) / WHEEL_R;  // RR


    // Motor directions
    setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
                RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, w_target[0]);
    setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
                FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, w_target[1]);
    setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
                FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, w_target[2]);
    setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
                RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, w_target[3]);

    // Convert PID output to PWM
    uint32_t CCR[4];
    for (int i = 0; i < 4; i++) {
        CCR[i] = (uint32_t)((fabs(w_target[i]) / MAX_WHEEL_ANGULAR_V_RADPS) * CCR_MAX);
        CCR[i] = (uint32_t)clamp((float)CCR[i], 0.0f, 250.0f);
    }


    // Measured wheel speeds from encoder
    extern volatile int16_t deltaEncoder[4];
    float w_meas[4];

    for (int i = 0; i < 4; i++) {
        w_meas[i] = deltaEncoder[i] * RAD_PER_TICK_PER_SEC; // [rad/s]
    }

    // PID regulation
    float pid_output[4];
    for (int i = 0; i < 4; i++) {
        pid_output[i] = PID_Update(&pid_wheel[i], w_target[i], w_meas[i]);
    }

// vx_twist, vy_twist, wz_twist, vx_twist_ramped, vy_twist_ramped, wz_twist_ramped,  w_target[0],  w_target[1],  w_target[2], w_target[3], w_meas[0], w_meas[1], w_meas[2], w_meas[3]
//    ,pid_output[0], pid_output[1], pid_output[2], pid_output[3], CCR[0], CCR[1] CCR[2], CCR[3]

    // Motor directions
    setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
                RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, pid_output[0]);
    setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
                FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, pid_output[1]);
    setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
                FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, pid_output[2]);
    setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
                RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, pid_output[3]);


    // Convert PID output to PWM
    int CCR[4];
    for (int i = 0; i < 4; i++) {
        CCR[i] = (int)((fabsf(pid_output[i]) / MAX_WHEEL_ANGULAR_V_RADPS) * CCR_MAX);
        CCR[i] = (int)clamp((float)CCR[i], 0.0f, CCR_MAX);
    }

    // Apply PWM
	for (int i = 0; i < 4; i++) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNELS[i], CCR[i]);
		g_ccr_applied[i] = CCR[i];
	}
}




void Emergency_Stop(void) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	// RL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);	// FL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);	// FR Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);	// RR Motor
}

*/


// ----------------------------------------------------------------------------------
// ----------- PID functions definitions  -------------------------------------------
// ----------------------------------------------------------------------------------

