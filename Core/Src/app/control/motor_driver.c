#include "robot_params.h"

#include "motor_driver.h"


extern TIM_HandleTypeDef htim2;


volatile float g_v_step   = 20.0f;
volatile float g_wz_step_radps = 0.10f;
volatile int32_t g_ccr_applied[4] = {0, 0, 0, 0};

PID_t pid_wheel[4];

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


void ctrlparams_set_steps(float v_step, float wz_step_radps)
{
    if (v_step  > 0.0f) g_v_step   = v_step;
    if (wz_step_radps > 0.0f) g_wz_step_radps = wz_step_radps;
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
	for(int i = 0; i <= 300; i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i+200);	// RL Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i+100);	// FL Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);		// FR Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);		// RR Motor
	  	HAL_Delay(15);
	  }
}

void Emergency_Stop(void) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	// RL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);	// FL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);	// FR Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);	// RR Motor
}


void Mecanum_Control(float vx_target, float vy_target, float w_target)
{
    static float vx = 0, vy = 0, wz = 0;
    vx = ramp_step(vx_target, vx, g_v_step);
    vy = ramp_step(vy_target, vy, g_v_step);
    wz = ramp_step(w_target,  wz, g_wz_step_radps);

    // Desired wheel linear velocities
    float V_des[4];
    V_des[0] = +vx + vy - (A_SUM * wz);  // RL
    V_des[1] = +vx - vy - (A_SUM * wz);  // FL
    V_des[2] = +vx + vy + (A_SUM * wz);  // FR
    V_des[3] = +vx - vy + (A_SUM * wz);  // RR

    // Measured wheel speeds from encoder
    extern volatile int16_t deltaEncoder[4];
    const float DT = 0.001f; // 1 kHz
    float w_meas[4];
    float v_meas[4];

    for (int i = 0; i < 4; i++) {
        w_meas[i] = deltaEncoder[i] * RAD_PER_TICK / DT; // [rad/s]
        v_meas[i] = w_meas[i] * WHEEL_R;                  // [mm/s] or [m/s]
    }


    // PID regulation
    float pid_output[4];
    for (int i = 0; i < 4; i++) {
        pid_output[i] = PID_Update(&pid_wheel[i], V_des[i], v_meas[i], DT);
    }

    // Convert PID output to PWM
    int CCR[4];
    for (int i = 0; i < 4; i++) {
        CCR[i] = (int)((fabsf(pid_output[i]) / MAX_WHEEL_LINEAR_V) * CCR_MAX);
        CCR[i] = (int)clamp((float)CCR[i], 0.0f, CCR_MAX);
    }

    // Motor directions
    setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
                RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, pid_output[0]);
    setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
                FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, pid_output[1]);
    setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
                FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, pid_output[2]);
    setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
                RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, pid_output[3]);

    uint32_t TIM_CHANNELS[4] = {
    	TIM_CHANNEL_1, // RL
    	TIM_CHANNEL_2, // FL
    	TIM_CHANNEL_3, // FR
    	TIM_CHANNEL_4  // RR
    };


    // Apply PWM
    for (int i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNELS[i], CCR[i]);
        g_ccr_applied[i] = CCR[i];
    }
}




// ----------------------------------------------------------------------------------
// ----------- PID functions definitions  -------------------------------------------
// ----------------------------------------------------------------------------------

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}

void PID_All_Init(void)
{
    PID_Init(&pid_wheel[0], 0.35f, 0.02f, 0.0f);   // RL
    PID_Init(&pid_wheel[1], 0.35f, 0.02f, 0.0f);   // FL
    PID_Init(&pid_wheel[2], 0.35f, 0.02f, 0.0f);   // FR
    PID_Init(&pid_wheel[3], 0.35f, 0.02f, 0.0f);   // RR
}


float PID_Update(PID_t *pid, float setpoint, float feedback, float dt)
{
    float error = setpoint - feedback;

    pid->integral += error * dt;
    if (pid->integral > PID_INTEGRAL_LIMIT)
        pid->integral = PID_INTEGRAL_LIMIT;
    else if (pid->integral < -PID_INTEGRAL_LIMIT)
        pid->integral = -PID_INTEGRAL_LIMIT;

    float derivative = (error - pid->prev_error) / dt;

    float output = (pid->Kp * error)
                 + (pid->Ki * pid->integral)
                 + (pid->Kd * derivative);

    //output = clamp(output, OUTPUT_MIN, OUTPUT_MAX);
    if (output > MAX_WHEEL_LINEAR_V)
		output = MAX_WHEEL_LINEAR_V;
    else if (output < -MAX_WHEEL_LINEAR_V)
		output = -MAX_WHEEL_LINEAR_V;

    pid->prev_error = error;
    pid->output = output;

    return output;
}


