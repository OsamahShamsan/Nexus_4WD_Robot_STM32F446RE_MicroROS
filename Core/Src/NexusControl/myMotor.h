#ifndef MYMOTOR_H_
#define MYMOTOR_H_

#include "main.h"


void init_car(void);
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
