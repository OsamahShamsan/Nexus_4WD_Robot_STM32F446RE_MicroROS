// -----------------------------------------------------------------------------
// math_helpers.h
// -----------------------------------------------------------------------------
// Generic math utilities for embedded robotics projects.
//
// These are pure functions — they don't depend on HAL or micro-ROS.
// They ensure consistent handling of floating point operations,
// angle wrapping, clamping, interpolation, and filtering.
// -----------------------------------------------------------------------------

/*
#include <math.h>
#include "math_helpers.h"
// ----- Basic math ------------------------------------------------------------
#define SIGN(x)              ((x) >= 0 ? 1.0f : -1.0f)
#define SQUARE(x)            ((x) * (x))
#define CLAMP(x, lo, hi)     (((x) < (lo)) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// ----- Angle helpers (use radians for ROS2 compatibility) ---------------------
#define DEG2RAD(x)           ((x) * (float)M_PI / 180.0f)
#define RAD2DEG(x)           ((x) * 180.0f / (float)M_PI)

// Wraps angle to [-π, π]
static inline float wrap_angle_rad(float angle)
{
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// ----- Mapping and scaling ----------------------------------------------------
static inline float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ----- Filters (simple smoothing) ---------------------------------------------
static inline float low_pass_filter(float prev, float new_val, float alpha)
{
    return prev + alpha * (new_val - prev); // alpha in [0..1]
}
*/
