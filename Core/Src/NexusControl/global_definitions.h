/*
 * global_definitions.h
 *
 *  Created on: Oct 1, 2025
 *  Author: Osamah Shamsan
 *
 *  The documentation in this sketch and constants are based on:
 *  1- NEXUS ROBOT User Manual (https://openhacks.com/uploadsproductos/file-1342627200__1_.pdf)
 *  2- Faulhaber motor's datasheet (en_2342_cr_dff)
 *  3- Omni4WD and MotorWheel library (https://github.com/lupusorina/nexus-robots/tree/master/documentation-libraries/lib/MotorWheel)
 *
 *
 */

#ifndef GLOBAL_DEFINITIONS_H_
#define GLOBAL_DEFINITIONS_H_

/*
// ================= SYSTEM CONFIGURATION =================

All used units are in SI system.
------------------------------------------------------------------------------------------------

Top view of the nexus 4WD mecanum robot (See position of Power Switch for reference)

							Front side

							Sonar: 0x14
						------------------
					   |                  |
	   (2) wheel_FL // |		          | \\ wheel_FR (3)
		   	   	   	   |				  |
						-------------------
							   | |
						------------------
					   |                  |
		(Power Switch) |                  |
					   |     Top View     |
		 Sonar: 0x11   |                  | Sonar: 0x13
					   |  (STM32 board    |
	   (1) wheel_RL \\ |    not visible)  | // wheel_RR	(4)
					   |                  |
						------------------
						   Sonar: 0x12

							Back side


------------------------------------------------------------------------------------------------

		Bottom View (See position of Power Switch for reference)

						  Front side

						  Sonar: 0x14
					  ------------------
					 |                  |
	 (3) wheel_FR \\ |		            | // wheel_FL (2)
		 	 	 	 |					|
					  ------------------
							 | |
					  ------------------
					 |                  |
					 |                  | (Power Switch)
					 |   Bottom View    |
		 Sonar: 0x13 |                  | Sonar: 0x11
					 |  (STM32 board    |
	 (4) wheel_RR // |     visible)     | \\ wheel_RL (1)
					 |                  |
					  ------------------
						  Sonar: 0x12

						  Back side




Defined coordinate system of the robot:

				   +x (Forward)
					 ^
					 |
					 |
	+y (Left) <------o

   - x-axis: Forward
   - y-axis: Left
   - z-axis: Up (out of plane)

The omnidirectional robot, using a Type-O wheel arrangement (as shown in the top view) with the chosen coordinate system, has the following

inverse kinematics:

  	float w_FL = (1 / WHEEL_R) * (+ vx_current - vy_current  - (A_SUM * w_current));
	float w_FR = (1 / WHEEL_R) * (+ vx_current + vy_current  + (A_SUM * w_current));
	float w_RL = (1 / WHEEL_R) * (+ vx_current + vy_current  - (A_SUM * w_current));
	float w_RR = (1 / WHEEL_R) * (+ vx_current - vy_current  + (A_SUM * w_current));

forward kinematics:

	float vx = (WHEEL_R / 4) * (+ w_FL + w_FR + w_RL + w_RR);
	float vy = (WHEEL_R / 4) * (- w_FL + w_FR + w_RL - w_RR);
	float wz = (WHEEL_R / 4) * (- (1/A_SUM)*w_FL + (1/A_SUM)*w_FR - (1/A_SUM)*w_RL + (1/A_SUM)*w_RR);

----------------------------------------------------
Robot Speed Calculation
----------------------------------------------------
   Max Motor no-load speed:	 			8100 rpm
   Recommended Motor no-load speed:     7000 rpm
   Gear ratio:               			64:1
   Wheel radius (r):         			50 m
   Wheel circumference (C):  			2 * π * r ≈ 0.314 m (~314 mm)

To calculate the max linear and angular speed of the wheel:
1) Using max motor speed:

   Step 1: Wheel speed after gearbox
	   n_motor_RPM = n_motor / gear_ratio
				   = 8100.0f / 64.0f
				   ≈ 126.6f rev/m

   Step 2: Convert to revolutions per second
	   n_motor_RPS = 126.6f / 60.0f
	   	   	   	   ≈ 2.1f rev/s

   Step 3: Linear speed
			   v_max = n_wheel * C
					 = 2.1f * 0.314f
					 ≈ 0.66f m/s   (~660 mm/s)

   Step 4: Angular speed
			   w_max = v / r
					 = 660.0f mm/s / 50.0f mm
					 ≈ 13.2f rad/s

2) Using recommended motor speed:
		   n_motor_RPM = 109.4f rpm

		   n_motor_RPS = 1.8f   rev/s

		   v_max =  0.57f m/s (~570 mm/s)

		   w_max = 11.4f rad/s

 To calculate the max vx, vy and wz of the robot:

A)		   vx = (WHEEL_R / 4) * (+ w_FL + w_FR + w_RL + w_RR);
 To maximize vx, we set all w_i to w_max =>
	   vx_max = (WHEEL_R / 4) * (4 * w_max)
		   	  = WHEEL_R * w_max
		   	  = 50 mm * 13.2 = 660 mm/s

		   OR = 570 mm/s using recommended values

B) Same for vy =>
	   vy_max = vx_max = 660 mm/s

		   OR = 570 mm/s using recommended values


C) For wz =>
 	 	   wz = (WHEEL_R / 4) * (- (1/A_SUM)*w_FL + (1/A_SUM)*w_FR - (1/A_SUM)*w_RL + (1/A_SUM)*w_RR);

 To maximize wz, we set all signs to positive and w_i to w_max =>
 	  =>   wz = (WHEEL_R / 4) * ((1/A_SUM) * 4 * w_max));
 	 	   	  = (WHEEL_R * w_max) / (A_SUM)
 	 	   	  = 50 mm * 13.2 rad/s / 300 mm = 2.2 (rad/s)

 	  =>   OR = 1.9 rad/s using recommended values

----------------------------------------------------
PID Controller Equation (speed control)
----------------------------------------------------

   u(t) = Kp * e(t) + Ki * ∫ e(t) dt + Kd * de(t)/dt

   where:
     u(t) = controller output (e.g., PWM duty cycle)
     e(t) = error = (speed_setpoint - measured_speed)
     Kp   = proportional gain
     Ki   = integral gain
     Kd   = derivative gain


   Discrete PID (sample time = dt)

   u[k] = Kp * e[k]
        + Ki * Σ e[i] * dt   (integral term)
        + Kd * (e[k] - e[k-1]) / dt   (derivative term)

   where:
     u[k]   = controller output at step k
     e[k]   = error at step k
     e[k-1] = error at previous step

*/

// ----------------------------------------------------------------------------------
// ----------- Geometry of the wheels and (Faulhaber) motor specifications ----------
// ----------------------------------------------------------------------------------
#define NUM_MOTORS 4

#define WHEEL_R              		 	50   	 			// [mm]	wheel_radius
#define WHEEL_D              		 	100    				// [mm] wheel_diameter
#define L_HALF               		 	150   				// [mm]	from robot_center to wheel_center in +x axis (forward)
#define W_HALF               		 	150   				// [mm]	from robot_center to wheel_center in +y axis (left)
#define A_SUM                		 	300 	 			// [mm]  (L_HALF + W_HALF)

//#define WHEEL_R_M           		  		0.05f    			// [m]	wheel_radius
//#define WHEEL_D_M            		 		0.10f    			// [m] 	wheel_diameter
//#define L_HALF_M             		 		0.150f   			// [m]	from robot_center to wheel_center in +x axis (forward)
//#define W_HALF_M              			0.150f   			// [m]	from robot_center to wheel_center in +y axis (left)
//#define A_SUM_M               			0.300f   			// [m]  (L_HALF + W_HALF)

#ifndef M_PI
#define M_PI 				  			3.1416
#endif

//#define CIRM_M 				  			0.314f	 		// [m]  circumference of wheel in m = wheel_diameter * PI = 100 mm * 3.14 = 314 [mm]

#define CIRM 				  			314	    			// [mm] circumference of wheel in m = wheel_diameter * PI = 100 mm * 3.14 = 314 [mm]


// No-Load Values
#define RECOMMENDED_VALUES				1

#if RECOMMENDED_VALUES == 1
// Motor limits
	#define MAX_MOTOR_RPM  				7000.0f   			// n_e_max [Rev/Min] recommended safe continuous speed	(See datasheet: en_2342_cr_dff)
	#define MAX_MOTOR_RPS				1.8f				// n_e_max

// Wheel limits
	#define MAX_WHEEL_LINEAR_V_MMPS 	570					// v_wheel [mm/s]
	#define MIN_WHEEL_LINEAR_V_MMPS 	-570

	#define MAX_WHEEL_ANGULAR_V_RADPS 	11.4f				// w_wheel [rad/s]
	#define MIN_WHEEL_ANGULAR_V_RADPS 	-11.4f

// Robot limits
	#define MAX_ROBOT_V_MMPS 			570					// vx = vy [mm/s]
	#define MIN_ROBOT_V_MMPS 			-570

	#define MAX_ROBOT_WZ_RADPS 			1.9					// wz [rad/s]
	#define MIN_ROBOT_WZ_RADPS 			-1.9

// PWM limits
	#define CCR_MAX    					432.0    			// 86.4% PWM duty
	#define CC_MIN							0				// 0% 	PWM duty

#else
// Motor limits
	#define MAX_MOTOR_RPM 				8100.0f   			// n_0_max 	   [Rev/Min] theoretical no-load speed			(See datasheet: en_2342_cr_dff)
	#define MAX_MOTOR_RPS				2.1f				// n_e_max 	   [Rev/s]

// Wheel limits
	#define MAX_WHEEL_LINEAR_V_MMPS 	660					// v_wheel_max [mm/s]
	#define MIN_WHEEL_LINEAR_V_MMPS 	-660

	#define MAX_WHEEL_ANGULAR_V_RADPS 	13.2f				// w_wheel_max [rad/s]
	#define MIN_WHEEL_ANGULAR_V_RADPS 	-13.2f

// Robot limits
	#define MAX_ROBOT_V_MMPS 			660					// vx = vy [mm/s]
	#define MIN_ROBOT_V_MMPS 			-660

	#define MAX_ROBOT_WZ_RADPS 			2.2					// wz [rad/s]
	#define MIN_ROBOT_WZ_RADPS 			-2.2

// PWM limits
	#define CCR_MAX    					500.0    			// 100% PWM duty
	#define CC_MIN							0				// 0% 	PWM duty

#endif


#define V_RAMPING_STEPS   				5.0f     			// Δv per cycle [mm/s]
#define WZ_RAMPING_STEPS   				0.07f     			// Δw per cycle [rad/s]

// ----------------------------------------------------------------------------------
// ----------- PID gains and Control loop -------------------------------------------
// ----------------------------------------------------------------------------------
// Encoders specifications
#define ENCODER_CPR   					12
#define GEARBOX_RATIO 					64
#define DECODING_MODE  					4		 			// (rising and falling edges of channels A and B are counted)
#define TICKS_PER_REV     		  		3072	 			// ENCODER_CPR * GEARBOX_RATIO * DECODING_MODE = 12 * 64 * 4 = 3072 tick per (WHEEL) revolution

#define RAD_PER_TICK      		 	 	0.0041f  			// (2 * M_PI / (float)TICKS_PER_REV) = ~0.0041f [rad/tick]
#define DEG_PER_TICK   					0.234f 	 			// (360.0f ° / (float)TICKS_PER_REV) = ~0.234	 [deg/tick]


// ----------------------------------------------------------------------------------
// ----------- PID gains and Control loop -------------------------------------------
// ----------------------------------------------------------------------------------
#define KP_DEFAULT      		    	0.31f
#define KI_DEFAULT      		   	   (0.31f/0.02f)  		// ≈15.5  [1/s]
#define KD_DEFAULT       		   		0.0f

#define KC           					0.31f
#define TAUI         					0.02f
#define TAUD        					0.00f


#define	SAMPLETIME	  					0.001f 				// sample time for encoder readings    1 [ms]
#define CONTROL_DT       		   		0.001f				// control time for motor/PID control  1 [ms]

// Output limiting
#define INTEGRATOR_LIMIT  		  		0.30f   			// anti-windup clamp on I term (as duty)
#define DUTY_LIMIT       		  		1.00f

#define OUTPUT_MAX   					100    				// max output (e.g. PWM duty %)
#define OUTPUT_MIN   					0.0    				// min output
#define INTEGRAL_MAX 					50.0   				// anti-windup limit


// ----------------------------------------------------------------------------------
// ----------- Sensors parameters and Pin assignments -------------------------------
// ----------------------------------------------------------------------------------
#define NUM_SENSORS 					4					// number of sensors: 4x Dual ultrasonic sensors (DUS) and x4 Encoders

// ----------- SONAR ----------------------------------------------------------------
#define SONAR_HEADER1   				0x55
#define SONAR_HEADER2   				0xAA

// Default device address
#define SONAR_DEFAULT_ADDR 				0x11

// RS485 DE/RE control pin
#define SONAR_DE_RE_GPIO_Port 			GPIOC
#define SONAR_DE_RE_Pin 				GPIO_PIN_5

// Buffer sizes
#define SONAR_RX_BUF_SIZE 				16

// Error codes
#define SONAR_TIMEOUT     				-2
#define SONAR_INVALID     				-3


// ----------- RS485 ----------------------------------------------------------------
#define RS485_DIR_GPIO_Port 			GPIOC
#define RS485_DIR_Pin       			GPIO_PIN_5

#define COLLISION_THRESHOLD 			150   				// stop if obstacle closer than 150 mm



// ----------------------------------------------------------------------------------
// ----------- Current Sensor (CS) on VNH5019A-E motor driver  ----------------------
// ----------------------------------------------------------------------------------
#define ADC_RESOLUTION 					4095.0f      		// 12-bit ADC
#define ADC_REF_VOLTAGE 				3.3f        		// Reference voltage (Vref)

// Hardware constants
#define VOLTAGE_DIVIDER_GAIN   			11.0f	 			// (R9 + R10) / R10 = 11
#define R_SENSE 						1000.0f             // 1 kΩ resistor
#define K_SENSE 						7000.0f             // iOUT / iSENSE from driver datasheet (≈7k typical)
#define STALL_CURRENT 					3.0f          		// Stall current threshold (A). In the datasheet ~5.6 A

// ----------------------------------------------------------------------------------
// ----------- Other definitions ----------------------------------------------------
// ----------------------------------------------------------------------------------
#define REF_VOLT 						12
#define SEC_PER_MIN 					60
#define MICROS_PER_SEC 					1000000

#define CMD_TIMEOUT_MS     		 		200u				// Watchdog: stop if no command in 200ms




#endif /* GLOBAL_DEFINITIONS_H_ */
