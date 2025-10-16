#pragma once


#include "main.h"


#ifndef DBG_EVERY_CYCLES
#define DBG_EVERY_CYCLES 10u     // print every 10 odom cycles (~10 Hz if odom=100 Hz)
#endif
#ifndef VX_MIN_FOR_TEST
#define VX_MIN_FOR_TEST 0.05f    // only test straight-line invariants when |vx| >= this
#endif
#ifndef VY_TOL
#define VY_TOL 0.01f             // m/s tolerance for vy≈0
#endif
#ifndef WYAW_TOL
#define WYAW_TOL 0.02f           // rad/s tolerance for wz≈0
#endif
#ifndef WDEV_TOL
#define WDEV_TOL 0.15f           // 15% max deviation allowed across wheels for "vx only"
#endif

/*** Debug print shim ***/
// If you added the micro-ROS debug publisher:
   #define HAVE_MICROROS_DEBUG_PUB 1
#if defined(HAVE_MICROROS_DEBUG_PUB)
  #include "debug_pub.h"
  #define DBG_PRINT(...) dbg_printf(__VA_ARGS__)
#elif defined(ENABLE_PRINTF_DEBUG)
  #include <stdio.h>
  #define DBG_PRINT(...) printf(__VA_ARGS__)
#else
  // No debug output (keeps it compile-clean on bareboard)
  #define DBG_PRINT(...) do {} while (0)
#endif


// Wheel order indices
enum { RL=0, FL=1, FR=2, RR=3 };

typedef struct {
    TIM_HandleTypeDef* tim;
    int32_t last;     // last counter snapshot
    int8_t   sign;     // +1/-1 so forward spin gives +w
} EncWheel;



void compute_and_publish_odometry(void);


