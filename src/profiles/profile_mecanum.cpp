/**
 * @file profile_mecanum.cpp
 * @brief Mecanum drive motion profile: the command pipeline
 *
 * FULL DATA PIPELINE (one joystick update, ~20ms):
 *   1. Flutter app sends JSON: {"type":"control","right":{"x":50,"y":80},...}
 *   2. command_parser extracts vx=50, vy=80, omega=0 from the joystick
 *   3. This module calls mecanum_calculate(vx, vy, omega) -> 4 wheel speeds
 *   4. Speeds are SCALED to motor units:
 *      - DC motors:  +/-100 -> +/-255 PWM (multiply by 2.55)
 *      - Steppers:   +/-100 -> +/-STEPPER_MAX_SPEED steps/sec
 *   5. For steppers: speeds are written to g_targetSpeeds[] via MUTEX
 *      Core 1 reads these speeds and generates step pulses
 *
 * MULTICORE HANDOFF (stepper path only):
 *   Core 0 uses mutex_try_enter(), which is NON-BLOCKING. If Core 1 holds the mutex
 *   (it's mid-I2C write), Core 0 skips this update. At 50Hz command rate,
 *   dropping one update is imperceptible. The alternative
 * (mutex_enter_blocking) would stall Core 0 and delay BLE processing.
 *
 * LEFT/RIGHT INPUT MAPPING:
 *   The Flutter app's "left" and "right" controls map differently per vehicle:
 *     mecanum:  left=dial(rotation)  right=joystick(translation)
 *     fourwheel: left=dial(steering) right=slider(throttle)
 *     tracked:   left=slider(L)      right=slider(R)
 *
 * AUTONOMOUS MODE (toggle 0):
 *   When cmd->toggles[0] is true, the robot drives straight forward at a
 *   fixed speed for AUTONOMOUS_DRIVE_TIME_MS milliseconds, then stops.
 *   The timer resets each time the toggle is turned on. Manual joystick
 *   input is ignored while autonomous is active.
 */

#include "profile_mecanum.h"
#include "core/motor_dc.h"
#include "core/motor_manager.h"
#include "core/motor_stepper.h"
#include "drivers/mcp23017.h"
#include "drivers/mecanum_kinematics.h"
#include "project_config.h"

#include "pico/mutex.h"
#include <Arduino.h>

// =============================================================================
// INTER-CORE COMMUNICATION (defined in main.cpp)
// =============================================================================
extern mutex_t g_speedMutex;
extern volatile float g_targetSpeeds[4];
extern volatile bool g_speedsUpdated;

// =============================================================================
// AUTONOMOUS MODE PARAMETERS
// =============================================================================

// How long to drive forward when toggle 0 is activated (milliseconds)
static const unsigned long AUTONOMOUS_DRIVE_TIME_MS = 25000UL;

// Forward speed during autonomous (0-100 scale, same as joystick)
static const float AUTONOMOUS_FORWARD_SPEED = 100.0f;

// Speed multiplier during autonomous (0.0-1.0). Kept moderate so the
// robot doesn't lurch at full speed. Tune this for your chassis.
static const float AUTONOMOUS_SPEED_MULT = 0.3f;

/**
 * @details INPUT MAPPING:
 *   The Flutter app sends two control inputs: "left" and "right".
 *   For the mecanum profile:
 *     Right (joystick): x -> vx (strafe), y -> vy (forward)
 *     Left (dial):      value -> omega (rotation)
 *   If left is a joystick instead of a dial, its X axis is used for rotation.
 *
 * AUTONOMOUS MODE:
 *   When toggles[0] is pressed, the robot enters autonomous mode:
 *     - On the rising edge (toggleFlipped), the 10-second timer is reset.
 *     - vy is set to AUTONOMOUS_FORWARD_SPEED for the duration.
 *     - Once AUTONOMOUS_DRIVE_TIME_MS elapses, the robot stops (vy = 0).
 *     - Joystick input is ignored while autonomous is active.
 *   When toggles[0] is released, manual control resumes immediately.
 *
 * MOTOR TYPE BRANCHING:
 *   DC motors:     speed (-100..+100) multiplied by 2.55 to fill the
 *                  0-255 PWM range. motor_set_pwm() is called directly
 *                  on Core 0 (no cross-core handoff needed).
 *   Stepper motors: speed (-100..+100) scaled to steps/sec by
 *                   STEPPER_MAX_SPEED/100. Written to g_targetSpeeds[]
 *                   behind a mutex for Core 1 to pick up.
 *
 * NON-BLOCKING MUTEX:
 *   mutex_try_enter() returns false if Core 1 holds the mutex (it's
 *   mid-I2C write). In that case, this command is silently dropped.
 *   At 50Hz command rate, one dropped command is imperceptible.
 */
void profile_mecanum_apply(const control_command_t *cmd) {

  float vx = 0.0f;    // Strafe
  float vy = 0.0f;    // Forward
  float omega = 0.0f; // Rotation
  float speedMult = 0.0f;

  // Tracks whether the toggle was off on the previous call, so we can
  // detect the rising edge and reset the timer.
  static bool toggleFlipped = true;

  // Timestamp (ms) of when the current autonomous run started.
  static unsigned long timerStart = 0UL;

  // =========================================================================
  // INPUT SELECTION: Autonomous vs Manual
  // =========================================================================

  if (cmd->toggles[0]) {
    // --- AUTONOMOUS MODE ---

    // Rising edge: toggle just turned ON → reset the drive timer
    if (toggleFlipped) {
      timerStart = millis();
      toggleFlipped = false;
    }

    unsigned long elapsed = millis() - timerStart;

    if (elapsed < AUTONOMOUS_DRIVE_TIME_MS) {
      // Still within the 10-second window: drive straight forward
      vy = AUTONOMOUS_FORWARD_SPEED;
      speedMult = AUTONOMOUS_SPEED_MULT;
    } else {
      // Time expired: hold stop until the toggle is released
      vy = 0.0f;
      speedMult = 0.0f;
    }

    // vx and omega intentionally left at 0 — straight line only

  } else {
    // --- MANUAL CONTROL ---

    // Record that the toggle is currently off so we detect the next
    // rising edge correctly.
    toggleFlipped = true;

    // Right joystick: forward/back (Y). Strafe (X) is commented out
    // but left here for easy re-enabling.
    if (cmd->right.isJoystick) {
      vx = cmd->right.x; // Strafe from right X
      vy = cmd->right.y; // Forward from right Y
    }

    // Left control: rotation
    if (cmd->left.isJoystick) {
      omega = cmd->left.x; // Rotation from left joystick X
    } else {
      omega = cmd->left.value; // Rotation from dial
    }

    // Speed slider (0-100 → 0.0-1.0)
    speedMult = cmd->speed / 100.0f;
  }

  // =========================================================================
  // KINEMATICS: vx / vy / omega → per-wheel speeds
  // =========================================================================

  WheelSpeeds wheels;
  mecanum_calculate(vx, vy, omega, speedMult, &wheels);

  // =========================================================================
  // MOTOR OUTPUT
  // =========================================================================

  bool hasDC = motors_has_dc();
  bool hasSteppers = motors_has_steppers();

  // Debug: log motor type detection (once only)
  static bool debugDone = false;
  if (!debugDone) {
    Serial.printf("[Mecanum] Motor detection: DC=%d, Steppers=%d\n", hasDC,
                  hasSteppers);
    debugDone = true;
  }

  if (hasDC) {
    // DC motors: convert -100..+100 to -255..+255 PWM
    motor_set_pwm(0, (int16_t)(wheels.frontLeft  * 2.55f));
    motor_set_pwm(1, (int16_t)(wheels.frontRight * 2.55f));
    motor_set_pwm(2, (int16_t)(wheels.backLeft   * 2.55f));
    motor_set_pwm(3, (int16_t)(wheels.backRight  * 2.55f));

  } else if (hasSteppers) {
    // NOTE: stepperEnableAll() removed - already enabled in setup()
    // and calling it here causes I2C contention with Core 1

    // Steppers: convert -100..+100 to steps/sec
    float scale = STEPPER_MAX_SPEED / 100.0f;
    float s0 = wheels.frontLeft  * scale;
    float s1 = wheels.frontRight * scale;
    float s2 = wheels.backLeft   * scale;
    float s3 = wheels.backRight  * scale;

    // =================================================================
    // MULTICORE: Send speeds to Core 1 via shared memory
    // =================================================================
    // Use try_enter to avoid blocking Core 0 if Core 1 has the mutex
    if (mutex_try_enter(&g_speedMutex, nullptr)) {
      g_targetSpeeds[0] = s0;
      g_targetSpeeds[1] = s1;
      g_targetSpeeds[2] = s2;
      g_targetSpeeds[3] = s3;
      g_speedsUpdated = true;
      mutex_exit(&g_speedMutex);
    }
    // If mutex busy, skip this update - Core 1 will get the next one

  } else {
    // No motors detected - log warning once
    static bool warnDone = false;
    if (!warnDone) {
      Serial.println("[Mecanum] WARNING: No motors detected!");
      warnDone = true;
    }
  }
}