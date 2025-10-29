/*
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


static EncWheel enc[4] = {
    { &htim4, 0, +1 },  // RL
    { &htim1, 0, +1 },  // FL
    { &htim3, 0, +1 },  // FR
    { &htim8, 0, +1 },  // RR
};

static double x=0, y=0, theta=0;
static int64_t last_ns = 0;

void compute_and_publish_odometry(void)
{
	int64_t now_ns =  rmw_uros_epoch_nanos();

    // Real elapsed time
    double dt = (double)(now_ns - last_ns) * 1e-9;
    if (dt < 1e-4) dt = 1e-4;
    if (dt > 0.2)  dt = 0.2;
    last_ns = now_ns;

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

    odom_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);
    if (rc != RCL_RET_OK) {
    	// Handle publisher init error
    }

}

*/
