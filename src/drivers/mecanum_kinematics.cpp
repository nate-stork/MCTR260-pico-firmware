/**
 * @file mecanum_kinematics.cpp
 * @brief Mecanum wheel inverse kinematics implementation
 *
 * MECANUM WHEEL PHYSICS:
 *   Mecanum wheels have rollers angled at 45° to the wheel axis. When all
 *   four wheels spin at different speeds/directions, the net force vectors
 *   combine to produce movement in any direction (omnidirectional).
 *
 * COORDINATE SYSTEM (top view, front is up):
 *
 *    <──── +vx (right) ────>
 *                          
 *    [FL]───────────────[FR]    
 *     │                     │    │  +vy
 *     │      (center)       │    │  (forward)
 *     │                     │    │
 *    [BL]───────────────[BR]    
 *
 *              (CCW) +omega (CCW)
 *
 * INVERSE KINEMATICS (desired motion -> individual wheel speeds):
 *   Given desired robot velocity (vx, vy, omega):
 *     FL = vy + vx + omega     (front-left)
 *     FR = vy - vx - omega     (front-right)
 *     BL = vy - vx + omega     (back-left)
 *     BR = vy + vx - omega     (back-right)
 *
 *   Where:
 *     vx    = strafe velocity (positive = right)
 *     vy    = forward velocity (positive = forward)
 *     omega = rotation rate (positive = counterclockwise)
 *
 * NORMALIZATION:
 *   Combined motions (e.g., forward + strafe + rotate) can produce wheel
 *   speeds exceeding +/-100. We normalize by dividing all wheels by the max
 *   absolute value, then apply the speed multiplier. This preserves the
 *   motion direction while keeping all values in the valid range.
 */

/*
 * VEHICLE LAYOUT (top view, front is up):
 *
 *         ┌──────── FRONT ────────┐
 *         │                       │
 *         │ M1 (FL)       M2 (FR) │
 *         │                       │
 *         │        [center]       │         +vy = forward
 *         │                       │         +vx = right
 *         │ M3 (BL)       M4 (BR) │         +omega = CCW
 *         │                       │
 *         └──────── REAR ─────────┘
 *
 * Mecanum roller orientations (top view):
 *   FL: NE-SW      FR: NW-SE       (front pair makes a V)
 *   BL: NW-SE      BR: NE-SW       (rear pair makes an inverted V)
 */

#include "mecanum_kinematics.h"
#include <math.h>

/**
 * @details MECANUM INVERSE KINEMATICS EQUATIONS:
 *   FL = vy + vx + omega
 *   FR = vy - vx - omega
 *   BL = vy - vx + omega
 *   BR = vy + vx - omega
 *
 * These come from resolving the 45-degree roller forces on each wheel.
 * The sign pattern determines which wheel combination produces forward,
 * strafe, or rotation. Combined motions (e.g., forward + strafe + rotate)
 * can produce raw values exceeding +/-100. Normalization divides all
 * values by the largest, preserving the motion direction while keeping
 * all outputs in the valid range.
 *
 * The early-exit for all-zero inputs is an optimization: it avoids the
 * normalization math (fabsf calls) when the robot should be stationary.
 */
void mecanum_calculate(float vx, float vy, float omega, float speedMultiplier,
                       WheelSpeeds *output) {
  // Early exit: if all inputs are zero, output zero directly
  if (vx == 0 && vy == 0 && omega == 0) {
    output->frontLeft = 0;
    output->frontRight = 0;
    output->backLeft = 0;
    output->backRight = 0;
    return;
  }

  // ==========================================================================
  // MECANUM INVERSE KINEMATICS
  // ==========================================================================
  // For a mecanum robot with rollers at 45 degrees:
  //   FL = vy + vx + omega
  //   FR = vy - vx - omega
  //   BL = vy - vx + omega
  //   BR = vy + vx - omega
  //
  // Note: Signs may need adjustment based on motor/wheel orientation
  // ==========================================================================

  // float fl = vy + vx + omega;
  // float fr = vy - vx - omega;
  // float bl = vy - vx + omega;
  // float br = vy + vx - omega;

  float bl = vy + vx + omega;
  float br = vy - vx - omega;
  float fl = vy - vx + omega;
  float fr = vy + vx - omega;

  // Find the maximum absolute value
  float maxVal = fabsf(fl);
  if (fabsf(fr) > maxVal)
    maxVal = fabsf(fr);
  if (fabsf(bl) > maxVal)
    maxVal = fabsf(bl);
  if (fabsf(br) > maxVal)
    maxVal = fabsf(br);

  // Normalize to -100..+100 range if any value exceeds it
  if (maxVal > 100.0f) {
    float scale = 100.0f / maxVal;
    fl *= scale;
    fr *= scale;
    bl *= scale;
    br *= scale;
  }

  // Apply speed multiplier
  output->frontLeft = fl * speedMultiplier;
  output->frontRight = fr * speedMultiplier;
  output->backLeft = bl * speedMultiplier;
  output->backRight = br * speedMultiplier;
}
