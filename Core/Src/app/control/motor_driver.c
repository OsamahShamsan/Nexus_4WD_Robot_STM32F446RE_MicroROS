#include "motor_driver.h"
#include "robot_params.h"

#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>


// ----------------------------------------------------------------------------------
// ----------- Motor control helpers  -----------------------------------------------
// ----------------------------------------------------------------------------------
static inline double clamp(double v, double lo, double hi);
static inline double ramp(double target, double current, double step);
static inline double ramp_and_clamp_sym(double target, double current, float step, float limit);
static inline void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, double speed);

extern TIM_HandleTypeDef htim2;

volatile float g_v_step   = 20.0f;
volatile float g_wz_step_radps = 0.10f;
volatile int32_t g_ccr_applied[4] = {0, 0, 0, 0};
float RAD_PER_TICK_PER_SEC  = 0.0f;

static const uint32_t TIM_CHANNELS[4] = {
	TIM_CHANNEL_1, // RL
	TIM_CHANNEL_2, // FL
	TIM_CHANNEL_3, // FR
	TIM_CHANNEL_4  // RR
};

PID_t pid_wheel[4];

rcl_subscription_t twist_sub;     // Subscription object → listens to /twist_nexus topic
geometry_msgs__msg__Twist twist_msg;  // Struct holding the received Twist message data


static inline double clamp(double v, double lo, double hi){
    if (v < lo) 		return lo;
    else if (v > hi) 	return hi;
    else 				return v;
}

static inline double ramp(double target, double current, double step)
{
    return (target > current) ? fmin(current + step, target)
         : (target < current) ? fmax(current - step, target)
         : current;
}

static inline double ramp_and_clamp_sym(double target, double current, float step, float limit)
{
    if (target > current)
        current = fmin(current + step, target);
    else if (target < current)
        current = fmax(current - step, target);

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

static inline void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, double speed) {

	if (speed > 0.0f) { // Forward
		HAL_GPIO_WritePin(INxB_Port, INxB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INxA_Port, INxA_Pin, GPIO_PIN_SET);

	} else if (speed < 0.0f) { // Reverse
		HAL_GPIO_WritePin(INxA_Port, INxA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INxB_Port, INxB_Pin, GPIO_PIN_SET);

	} else { // Stop / Brake
		HAL_GPIO_WritePin(INxA_Port, INxA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INxB_Port, INxB_Pin, GPIO_PIN_RESET);
	}
}

// ----------------------------------------------------------------------------------
// ----------- Motors control functions definitions  ---------------------------------
// ----------------------------------------------------------------------------------
void init_motors(void){

	// Set the direction to Forward (INA = 1) & (INB = 0)
	// Rear Left
	HAL_GPIO_WritePin(RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin, GPIO_PIN_SET);
	// Front Left
	HAL_GPIO_WritePin(FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin, GPIO_PIN_SET);
	// Rear Right
	HAL_GPIO_WritePin(RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin, GPIO_PIN_SET);
	// Front Right
	HAL_GPIO_WritePin(FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin, GPIO_PIN_SET);

	// Enable the full bridges of the motor drivers (VDD)
	HAL_GPIO_WritePin(RL_VDD_GPO_GPIO_Port, RL_VDD_GPO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FR_VDD_GPO_GPIO_Port, FR_VDD_GPO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RR_VDD_GPO_GPIO_Port, RR_VDD_GPO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FL_VDD_GPO_GPIO_Port, FL_VDD_GPO_Pin, GPIO_PIN_SET);

	// Start the PWM signals
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// RL Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	// FL Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// FR Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);	// RR Motor

	// Set RL, FL, FR and RR motors to initial speed. For example 0 PWM = 0 CCR = 0% Duty Cycle => t_on = 0 µs
	/*
	for(int i = 0; i <= 300; i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i+200);	// RL Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i+100);	// FL Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);		// FR Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);		// RR Motor
	  	HAL_Delay(15);
	  }
	*/
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	// RL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);	// FL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);	// FR Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);	// RR Motor

    //RAD_PER_TICK_PER_SEC = RAD_PER_TICK / DeltaT;

}


void Mecanum_Control(double vx_twist, double vy_twist, double wz_twist)
{

    static double vx_twist_ramped = 0, vy_twist_ramped = 0, wz_twist_ramped = 0;
    vx_twist_ramped = ramp(vx_twist, vx_twist_ramped, g_v_step);
    vy_twist_ramped = ramp(vy_twist, vy_twist_ramped, g_v_step);
    wz_twist_ramped = ramp(wz_twist, wz_twist_ramped, g_wz_step_radps);

    // Desired wheel angular velocities
    double w_target[4];
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
    int CCR[4];
    for (int i = 0; i < 4; i++) {
        CCR[i] = (int)((fabs(w_target[i]) / MAX_WHEEL_ANGULAR_V_RADPS) * CCR_MAX);
        CCR[i] = (int)clamp((float)CCR[i], 0.0f, CCR_MAX);
    }

  /*
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
*/


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

// ----------------------------------------------------------------------------------
// ----------- PID functions definitions  -------------------------------------------
// ----------------------------------------------------------------------------------
static void PID_WheelInit(PID_t *pid, float Kp, float Ki, float Kd, float dt);

void PID_Init(float Kc, float taui, float taud)
{
    if (Kc == 0.0f)   Kc   = KC;
    if (taui == 0.0f) taui = TAUI;
    if (taud == 0.0f) taud = TAUD;

    PID_WheelInit(&pid_wheel[0], Kc, taui, taud, DeltaT);   // RL
    PID_WheelInit(&pid_wheel[1], Kc, taui, taud, DeltaT);   // FL
    PID_WheelInit(&pid_wheel[2], Kc, taui, taud, DeltaT);   // FR
    PID_WheelInit(&pid_wheel[3], Kc, taui, taud, DeltaT);   // RR

}

static void PID_WheelInit(PID_t *pid, float Kc, float tau_i, float tau_d, float dt)
{
    pid->Kc = Kc;
    pid->tau_i = tau_i;
    pid->tau_d = tau_d;

    // Precompute coefficients (Tustin/bilinear transform)
    pid->cof_A = Kc * (1.0f + (dt / (2.0f * tau_i)) + (tau_d / dt));
    pid->cof_B = -Kc * (1.0f - (dt / (2.0f * tau_i)) + (2.0f * tau_d / dt));
    pid->cof_C = Kc * (tau_d / dt);

    // Initialize state
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->output = 0.0f;

    // Default limits (can be overridden)
    pid->output_max =  MAX_WHEEL_ANGULAR_V_RADPS;
    pid->output_min =  MIN_WHEEL_ANGULAR_V_RADPS;
}

float PID_Update(PID_t *pid, float setpoint, float input)
{
    float error = setpoint - input;

    float output = pid->prev_output
                 + pid->cof_A * error
                 + pid->cof_B * pid->last_error
                 + pid->cof_C * pid->prev_error;

    // Clamp to limits
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    // Shift states
    pid->prev_error = pid->last_error;
    pid->last_error = error;
    pid->prev_output = output;
    pid->output = output;

    return output;
}

