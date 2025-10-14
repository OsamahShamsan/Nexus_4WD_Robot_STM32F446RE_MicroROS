#ifndef MATH_HELPERS_H_
#define MATH_HELPERS_H_

// -----------------------------------------------------------------------------
// Unit conversion helpers (for consistent SI scaling in micro-ROS messages)
// -----------------------------------------------------------------------------
// Purpose:
//   Prevents unit mismatches when switching between internal control units
//   (e.g., mm/s) and ROS 2 standard units (m/s, rad/s).
// -----------------------------------------------------------------------------

/*

#include <math.h>

// Constants --------------------------------------------------------------------
#define MM_TO_M(x)         ((x) * 0.001f)
#define M_TO_MM(x)         ((x) * 1000.0f)

#define DEG_TO_RAD(x)      ((x) * (float)M_PI / 180.0f)
#define RAD_TO_DEG(x)      ((x) * 180.0f / (float)M_PI)

// For velocity conversions
#define MMPS_TO_MPS(x)     ((x) * 0.001f)
#define MPS_TO_MMPS(x)     ((x) * 1000.0f)

// Convenience wrappers ---------------------------------------------------------
#define CLAMP(x, lo, hi)   (((x) < (lo)) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// Example usage:
//     float v_robot_mps = MMPS_TO_MPS(v_robot_mmps);
//     float angle_rad   = DEG_TO_RAD(yaw_deg);
// -----------------------------------------------------------------------------



 */

#endif /* MATH_HELPERS_H_ */
