/**
 * @file project_config.h
 * @brief Robot Configuration for Raspberry Pi Pico W - USER EDITABLE
 *
 * Customize these values for YOUR robot.
 * This is the main configuration file for project-specific settings.
 */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// =============================================================================
// DEVICE IDENTITY
// =============================================================================

// BLE advertised name - CHANGE THIS TO RENAME YOUR ROBOT
#define DEVICE_NAME         "PicoBot_05"

// 6-digit pairing PIN (must match what you enter on the app)
#define BLE_PASSKEY         123456

// =============================================================================
// MOTOR TYPE SELECTION (Enable one for DC motors, one for steppers)
// =============================================================================

// DC Motor drivers (uncomment ONE if using DC motors)
// #define MOTOR_DRIVER_DRV8871      // Single H-bridge, 2 PWM pins per motor
// #define MOTOR_DRIVER_DRV8833      // Dual H-bridge, 2 PWM pins per channel
// #define MOTOR_DRIVER_L298N        // Dual H-bridge with enable pins

// Stepper drivers (uncomment ONE if using steppers)
#define STEPPER_DRIVER_TMC2209    // Step/dir control via MCP23017 @ 0x20
// #define STEPPER_DRIVER_A4988     // Step/dir, basic driver
// #define STEPPER_DRIVER_DRV8825   // Step/dir, up to 1/32 microstepping

// =============================================================================
// MOTION PROFILES (Enable one or more)
// =============================================================================

#define MOTION_PROFILE_MECANUM      // 4-wheel omnidirectional drive
// #define MOTION_PROFILE_DIRECT    // Raw motor control via aux channels

// =============================================================================
// VEHICLE GEOMETRY (Measure your robot!)
// =============================================================================

// Wheel radius in millimeters
#define WHEEL_RADIUS_MM        50.0f

// Distance from center to front/back axle (half of wheelbase)
#define WHEELBASE_HALF_MM       100.0f

// Distance from center to left/right wheel (half of track width)
#define TRACK_WIDTH_HALF_MM     100.0f

// Dead zone: joystick values below this are treated as zero (0-100 scale)
#define MOTOR_DEADZONE          5.0f

// =============================================================================
// GPIO PIN ASSIGNMENTS - DC MOTORS (DRV8871)
// TODO: Update with actual GPIO when available
// =============================================================================

#ifdef MOTOR_DRIVER_DRV8871
#define PIN_FL_IN1      2       // Front-Left IN1
#define PIN_FL_IN2      3       // Front-Left IN2
#define PIN_FR_IN1      4       // Front-Right IN1
#define PIN_FR_IN2      5       // Front-Right IN2
#define PIN_BL_IN1      6       // Back-Left IN1
#define PIN_BL_IN2      7       // Back-Left IN2
#define PIN_BR_IN1      8       // Back-Right IN1
#define PIN_BR_IN2      9       // Back-Right IN2
#endif

// =============================================================================
// GPIO PIN ASSIGNMENTS - DC MOTORS (DRV8833)
// =============================================================================

#ifdef MOTOR_DRIVER_DRV8833
// Chip 1: Front motors
#define PIN_DRV8833_1_AIN1  2   // Front-Left
#define PIN_DRV8833_1_AIN2  3
#define PIN_DRV8833_1_BIN1  4   // Front-Right
#define PIN_DRV8833_1_BIN2  5
// Chip 2: Back motors  
#define PIN_DRV8833_2_AIN1  6   // Back-Left
#define PIN_DRV8833_2_AIN2  7
#define PIN_DRV8833_2_BIN1  8   // Back-Right
#define PIN_DRV8833_2_BIN2  9
#endif

// =============================================================================
// GPIO PIN ASSIGNMENTS - DC MOTORS (L298N)
// =============================================================================

#ifdef MOTOR_DRIVER_L298N
// Module 1: Front motors
#define PIN_L298N_1_ENA     2   // Front-Left Enable (PWM)
#define PIN_L298N_1_IN1     3   // Front-Left Direction 1
#define PIN_L298N_1_IN2     4   // Front-Left Direction 2
#define PIN_L298N_1_ENB     5   // Front-Right Enable (PWM)
#define PIN_L298N_1_IN3     6   // Front-Right Direction 1
#define PIN_L298N_1_IN4     7   // Front-Right Direction 2
// Module 2: Back motors
#define PIN_L298N_2_ENA     8   // Back-Left Enable
#define PIN_L298N_2_IN1     9   // Back-Left Direction 1
#define PIN_L298N_2_IN2     10  // Back-Left Direction 2
#define PIN_L298N_2_ENB     11  // Back-Right Enable
#define PIN_L298N_2_IN3     12  // Back-Right Direction 1
#define PIN_L298N_2_IN4     13  // Back-Right Direction 2
#endif

// =============================================================================
// GPIO PIN ASSIGNMENTS - STEPPERS (via MCP23017 I2C GPIO Expander)
// MechaPico MCB: Steppers are controlled via MCP23017 @ I2C address 0x20
// See: drivers/mcp23017.h for pin mappings
// =============================================================================

#if defined(STEPPER_DRIVER_TMC2209) || defined(STEPPER_DRIVER_A4988) || defined(STEPPER_DRIVER_DRV8825)

// I2C pins for communication with MCP23017 GPIO expanders
#define PIN_I2C_SDA             4   // Pico GPIO 4 - I2C0 SDA
#define PIN_I2C_SCL             5   // Pico GPIO 5 - I2C0 SCL
#define PIN_I2C_INT             22  // Pico GPIO 22 - Interrupt (optional)

// MCP23017 I2C addresses (do not change - set by PCB hardware)
#define MCP_STEPPER_I2C_ADDR    0x20    // U6_1: Stepper control
#define MCP_DCMOTOR_I2C_ADDR    0x21    // U6_2: DC motors & LEDs

// NOTE: Individual STEP/DIR pins are on MCP23017, not Pico GPIO!
// Motor mapping (managed by MCP23017 driver):
//   M1 (Front-Left):  GPB0=DIR, GPB1=STEP
//   M2 (Front-Right): GPB2=DIR, GPB3=STEP
//   M3 (Back-Left):   GPB4=DIR, GPB5=STEP
//   M4 (Back-Right):  GPB6=DIR, GPB7=STEP
//   M5 (Spare):       GPA2=DIR, GPA1=STEP
//
// Shared signals on MCP23017 Port A:
//   GPA3 = STPR_ALL_EN (active LOW)
//   GPA4 = STPR_ALL_MS1 (microstepping)
//   GPA5 = STPR_ALL_MS2 (microstepping)
//   GPA6 = STPR_ALL_SPREAD (SpreadCycle mode)
//   GPA7 = STPR_ALL_PDN (power down, active LOW)

#endif

// =============================================================================
// STEPPER MOTOR PARAMETERS
// =============================================================================

// Base steps per revolution (typically 200 for 1.8° motors)
#define STEPPER_STEPS_PER_REV   200

// Microstepping divisor (8, 16, 32, 64 for TMC2209)
#define STEPPER_MICROSTEPPING   8

// Maximum step rate (steps per second)
#define STEPPER_MAX_SPEED       4000.0f

// Acceleration (steps per second squared)
#define STEPPER_ACCELERATION    8000.0f

// Stepper pulse generation interval (microseconds)
// Lower = faster response, but more CPU usage
#define STEPPER_PULSE_INTERVAL_US   500

// Speed deadzone - speeds below this are treated as zero (steps/sec)
// Prevents jitter from small floating point noise
#define STEPPER_SPEED_DEADZONE  10.0f

// =============================================================================
// TIMING PARAMETERS
// =============================================================================

// Motor update rate (50Hz = 20ms)
#define MOTOR_UPDATE_INTERVAL_MS    20

// Safety timeout - stop motors if no command received
#define SAFETY_TIMEOUT_MS           2000

// Telemetry update rate
#define TELEMETRY_INTERVAL_MS       500

#endif // PROJECT_CONFIG_H
