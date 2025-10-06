#include "myMotor.h"
#include "global_definitions.h"
#include "math.h"


extern TIM_HandleTypeDef htim2;


volatile float g_v_step_mmps   = 20.0f;  // start conservative
volatile float g_wz_step_radps = 0.10f;
volatile int32_t g_ccr_applied[4] = {0, 0, 0, 0};

void ctrlparams_set_steps(float v_step_mmps, float wz_step_radps)
{
    if (v_step_mmps  > 0.0f) g_v_step_mmps   = v_step_mmps;
    if (wz_step_radps > 0.0f) g_wz_step_radps = wz_step_radps;
}

static inline float clamp(float v, float lo, float hi);
static inline float ramp_step(float current, float target, float step);
static inline void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, float speed);

// ----------------------------------------------------------------------------------
// ----------- Motor control helpers  -----------------------------------------------
// ----------------------------------------------------------------------------------
static inline float clamp(float v, float lo, float hi){
    if (v < lo) 		return lo;
    else if (v > hi) 	return hi;
    else 				return v;
}

static inline float ramp_step(float target, float current, float step)
{
    if (target > current + step) 		 return current + step;
    else if (target < current - step) 	 return current - step;
    else 								 return target;
}


static inline void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, float speed) {

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
// ----------- Motor control functions definitions  ---------------------------------
// ----------------------------------------------------------------------------------
void init_car(void){

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
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	// RL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);	// FL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);	// FR Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);	// RR Motor

}

void Mecanum_Control(float vx_target, float vy_target, float w_target) {

	// Current velocities
	static float vx = 0, vy = 0, wz = 0;

	// Increment current velocities to the targets using fixed steps (prevent rapid changes for motor safety)
	vx = ramp_step(vx_target, vx, g_v_step_mmps);
	vy = ramp_step(vy_target, vy, g_v_step_mmps);
	wz = ramp_step(w_target,  wz, g_wz_step_radps);


	float V[4];
	V[0] = +vx + vy - (A_SUM * wz);  // RL
	V[1] = +vx - vy - (A_SUM * wz);  // FL
	V[2] = +vx + vy + (A_SUM * wz);  // FR
	V[3] = +vx - vy + (A_SUM * wz);  // RR

	// Clamp velocities to the allowed limits
	 for (int i = 0; i < 4; i++) {
		 V[i] = clamp(V[i], MIN_WHEEL_LINEAR_V_MMPS, MAX_WHEEL_LINEAR_V_MMPS);
	 }

	 // PWM Conversion and CCR clamping
	 int CCR[4];

	 for (int i = 0; i < 4; i++)
	 {
		 CCR[i] = (int)((fabsf(V[i]) / MAX_WHEEL_LINEAR_V_MMPS) * CCR_MAX);
		 CCR[i] = (int)clamp((float)CCR[i], 0.0f, CCR_MAX);
	 }

	setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
				RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, V[0]);

	setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
	            FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, V[1]);

	setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
				FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, V[2]);

	setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
				RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, V[3]);

	uint32_t TIM_CHANNELS[4] = {
	        TIM_CHANNEL_1, // RL
	        TIM_CHANNEL_2, // FL
	        TIM_CHANNEL_3, // FR
	        TIM_CHANNEL_4  // RR
	};

	for (int i = 0; i < 4; i++){
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
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measurement, float dt) {
    // Error
    float error = setpoint - measurement;

    // Integral term (with anti-windup)
    pid->integral += error * dt;
    if (pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
    if (pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;

    // PID Output
    float output = (pid->Kp * error)
                 + (pid->Ki * pid->integral)
                 + (pid->Kd * derivative);

    // Clamp output to safe range
    output = clamp(output, OUTPUT_MIN, OUTPUT_MAX);

    // Save error for next step
    pid->prev_error = error;

    return output;
}


