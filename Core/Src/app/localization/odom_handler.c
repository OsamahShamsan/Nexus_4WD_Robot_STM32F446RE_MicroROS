
#include "odom_handler.h"
#include "robot_params.h"

#include <math.h>
#include <stdint.h>

#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <nav_msgs/msg/odometry.h>

#include "debug_pub.h"

rcl_publisher_t odom_pub;         // Publisher object → publishes /odom topic
nav_msgs__msg__Odometry odom_msg; // Struct holding odometry data to send back to ROS2

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern void dbg_printf(const char* fmt, ...);
extern void dbg_publish_floats(const char* tag, const float* vals, size_t n);

static EncWheel enc[4] = {
    { &htim4, 0, +1 },  // RL
    { &htim1, 0, -1 },  // FL
    { &htim3, 0, -1 },  // FR
    { &htim8, 0, +1 },  // RR
};

static double x=0, y=0, theta=0;
static int64_t last_ns = 0;

void compute_and_publish_odometry(void)
{
	int64_t now_ns =  rmw_uros_epoch_nanos();

    // First-call init
    if (last_ns == 0) {
        for (int i=0; i<4; i++) {
        	enc[i].last = (int32_t)( __HAL_TIM_GET_COUNTER(enc[i].tim));
        }
        last_ns = now_ns;

        if (!odom_msg.header.frame_id.data) {
            rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
            rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
        }
        if (!odom_msg.child_frame_id.data) {
            rosidl_runtime_c__String__init(&odom_msg.child_frame_id);
            rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
        }
        return;
    }

    // Real elapsed time
    double dt = (double)(now_ns - last_ns) * 1e-9;
    if (dt < 1e-4) dt = 1e-4;
    if (dt > 0.2)  dt = 0.2;
    last_ns = now_ns;

    uint16_t c_snap[4];
    int16_t  d_ticks[4];
    float    w[4];

    for (int i=0; i<4; i++) {
    	uint16_t ccur = (uint16_t)__HAL_TIM_GET_COUNTER(enc[i].tim);
    	int16_t  d    = (int16_t)(ccur - (uint16_t)enc[i].last); // wrap-safe
    	c_snap[i] = ccur;
    	d_ticks[i] = (int16_t)(d * enc[i].sign);
    	enc[i].last = ccur;

    	float wi = (float)d * RAD_PER_TICK / (float)dt;  // rad/s at motor=>wheel config
    	w[i] = wi * enc[i].sign;                      // per-wheel sign
    }

    // Kinematics (x fwd, y left, z up)
    float vx = (WHEEL_R / 4.0f)          * (w[RL] + w[FL] + w[FR] + w[RR]);
    float vy = (WHEEL_R / 4.0f)          * (w[RL] - w[FL] + w[FR] - w[RR]);
    float wz = (WHEEL_R / (4.0f*A_SUM))  * (-w[RL] - w[FL] + w[FR] + w[RR]);

    // Small deadband to avoid yaw creep
    if (fabsf(wz) < 1e-3f) wz = 0.0f;

    // Integrate body -> world
    double cth = cos(theta), sth = sin(theta);
    x     += (vx * cth - vy * sth) * (float)dt;
    y     += (vx * sth + vy * cth) * (float)dt;
    theta += wz * (float)dt;
    if (theta >  M_PI) theta -= 2.0 * M_PI;
    if (theta < -M_PI) theta += 2.0 * M_PI;

    // Fill and publish
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sin(theta * 0.5f);
    odom_msg.pose.pose.orientation.w = cos(theta * 0.5f);

    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.linear.y  = vy;
    odom_msg.twist.twist.angular.z = wz;

    odom_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000ULL);
    odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);

    rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);
    if (rc != RCL_RET_OK) {
    	// Handle publisher init error
    }

    // ---------- DEBUG (rate-limited) ----------
    debug_pub_cd_w(c_snap, d_ticks, w);
    // ---------- END DEBUG ----------
}

/*
void compute_and_publish_odometry(void)
{
    uint64_t now_ns = rmw_uros_epoch_nanos();
    if (last_ns == 0) {                 // first call: prime
        for (int i=0;i<4;i++) enc[i].last = __HAL_TIM_GET_COUNTER(enc[i].tim);
        last_ns = now_ns;
        // set frame ids once if not already set
        if (!odom_msg.header.frame_id.data) {
            rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
            rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
        }
        if (!odom_msg.child_frame_id.data) {
            rosidl_runtime_c__String__init(&odom_msg.child_frame_id);
            rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
        }
        return;
    }

    double dt = (double)(now_ns - last_ns) * 1e-9;
    if (dt < 1e-4) dt = 1e-4;
    if (dt > 0.2)  dt = 0.2;
    last_ns = now_ns;

    float w[4];
    for (int i=0;i<4;i++) {
        uint16_t c = __HAL_TIM_GET_COUNTER(enc[i].tim);
        int16_t  d = (int16_t)(c - enc[i].last);     // wrap-safe delta
        enc[i].last = c;

        // (equivalently, use your 32767/65536 wrap code here)

        float wi = (float)d * RAD_PER_TICK / (float)dt;
        w[i] = wi * enc[i].sign;
    }

    // inverse kinematics (x forward, y left, z up)
    float vx = (WHEEL_R / 4.0f)          * (w[RL] + w[FL] + w[FR] + w[RR]);
    float vy = (WHEEL_R / 4.0f)          * (w[RL] - w[FL] + w[FR] - w[RR]);
    float wz = (WHEEL_R / (4.0f*A_SUM))  * (-w[RL] - w[FL] + w[FR] + w[RR]);

    // optional tiny deadband to avoid yaw creep
    if (fabsf(wz) < 1e-3f) wz = 0.0f;

    // integrate body->world
    float cth = cosf(theta), sth = sinf(theta);
    x     += (vx * cth - vy * sth) * (float)dt;
    y     += (vx * sth + vy * cth) * (float)dt;
    theta += wz * (float)dt;
    if (theta >  M_PI) theta -= 2.0f*M_PI;
    if (theta < -M_PI) theta += 2.0f*M_PI;

    // fill and publish
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(theta * 0.5f);

    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.linear.y  = vy;
    odom_msg.twist.twist.angular.z = wz;

    odom_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000ULL);
    odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);

    rcl_ret_t rc1 = rcl_publish(&odom_pub, &odom_msg, NULL);

	if (rc1 != RCL_RET_OK) {
			 // Handle publisher init error
	}

	// ---------- DEBUG: print everything that can go wrong ----------
   static uint32_t dbg_cnt = 0;
   dbg_cnt++;
   if ((dbg_cnt % DBG_EVERY_CYCLES) == 0) {
	   // Recompute tick deltas just for logging (matches earlier values)
	   int16_t dRL = (int16_t)(__HAL_TIM_GET_COUNTER(enc[RL].tim) - enc[RL].last);
	   int16_t dFL = (int16_t)(__HAL_TIM_GET_COUNTER(enc[FL].tim) - enc[FL].last);
	   int16_t dFR = (int16_t)(__HAL_TIM_GET_COUNTER(enc[FR].tim) - enc[FR].last);
	   int16_t dRR = (int16_t)(__HAL_TIM_GET_COUNTER(enc[RR].tim) - enc[RR].last);

	   // Mecanum “combos” (before scaling factors), useful invariants:
	   // For pure +vx: S_vy ≈ 0 and S_wz ≈ 0
	   float S_vx = (w[RL] + w[FL] + w[FR] + w[RR]);
	   float S_vy = (w[RL] - w[FL] + w[FR] - w[RR]);
	   float S_wz = (-w[RL] - w[FL] + w[FR] + w[RR]);

	   // Check signs: for +vx motion, all w should have same sign
	   int sRL = (w[RL] >= 0.0f) ? 1 : -1;
	   int sFL = (w[FL] >= 0.0f) ? 1 : -1;
	   int sFR = (w[FR] >= 0.0f) ? 1 : -1;
	   int sRR = (w[RR] >= 0.0f) ? 1 : -1;
	   bool sign_mismatch = (sRL != sFL) || (sRL != sFR) || (sRL != sRR);

	   // Check symmetry: wheels should be ~equal for straight vx
	   float w_mean = 0.25f * (w[RL] + w[FL] + w[FR] + w[RR]);
	   float max_dev = 0.0f;
	   float ws[4] = { w[RL], w[FL], w[FR], w[RR] };
	   for (int i=0;i<4;i++) {
		   float dev = fabsf(ws[i] - w_mean);
		   if (dev > max_dev) max_dev = dev;
	   }
	   float rel_dev = (fabsf(w_mean) > 1e-6f) ? (max_dev / fabsf(w_mean)) : 0.0f;

	   // Raise flags only when we are clearly trying to go straight
	   bool vy_bias  = (fabsf(vx) >= VX_MIN_FOR_TEST) && (fabsf(vy) > VY_TOL);
	   bool wz_bias  = (fabsf(vx) >= VX_MIN_FOR_TEST) && (fabsf(wz) > WYAW_TOL);
	   bool combo_bad = (fabsf(vx) >= VX_MIN_FOR_TEST) &&
						(fabsf(S_vy) > WYAW_TOL || fabsf(S_wz) > WYAW_TOL);
	   bool symmetry_bad = (fabsf(vx) >= VX_MIN_FOR_TEST) && (rel_dev > WDEV_TOL);

	   // Text line (easy to watch with `ros2 topic echo /debug`)
	   dbg_printf(
		   "dt=%.4f d[RL,FL,FR,RR]=[%d,%d,%d,%d] "
		   "w=[%.3f,%.3f,%.3f,%.3f] S(vx,vy,wz)=[%.3f,%.3f,%.3f] "
		   "twist=[vx=%.3f vy=%.3f wz=%.3f] pose=[x=%.3f y=%.3f th=%.3f] "
		   "flags: sign=%d sym=%.2f vy_bias=%d wz_bias=%d combo_bad=%d",
		   (float)dt, dRL, dFL, dFR, dRR,
		   w[RL], w[FL], w[FR], w[RR],
		   S_vx, S_vy, S_wz,
		   vx, vy, wz, x, y, theta,
		   sign_mismatch, rel_dev, vy_bias, wz_bias, combo_bad
	   );

	   // Binary float packet (handy for plotting)
	   float pack[16] = {
		   (float)dt,
		   (float)dRL, (float)dFL, (float)dFR, (float)dRR,
		   w[RL], w[FL], w[FR], w[RR],
		   vx, vy, wz,
		   x, y, theta
	   };
	   dbg_publish_floats("odom_dbg", pack, 15);
   }
   // ---------- END DEBUG ----------
}
*/

