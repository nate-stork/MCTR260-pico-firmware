/**
 * @file motor_manager.cpp
 * @brief Motor initialization and management
 *
 * Configures motors based on project_config.h settings.
 * For MechaPico MCB, initializes MCP23017 I2C GPIO expanders.
 */

#include "motor_manager.h"
#include "motor_dc.h"
#include "motor_stepper.h"
#include "project_config.h"
#include "../drivers/mcp23017.h"
#include <Arduino.h>

// =============================================================================
// MOTOR STORAGE
// =============================================================================

static MotorBase* motors[NUM_MOTORS] = {nullptr};

// Static storage for motor objects (to avoid heap allocation)
#if defined(MOTOR_DRIVER_DRV8871) || defined(MOTOR_DRIVER_DRV8833) || defined(MOTOR_DRIVER_L298N)
static MotorDC* dcMotors[NUM_MOTORS];
#endif

#if defined(STEPPER_DRIVER_TMC2209) || defined(STEPPER_DRIVER_A4988) || defined(STEPPER_DRIVER_DRV8825)
static MotorStepper* stepperMotors[NUM_MOTORS];
#endif

// =============================================================================
// INITIALIZATION
// =============================================================================

bool motors_init() {
    Serial.println("[MotorManager] Initializing motors...");
    
    // =========================================================================
    // DC MOTORS (DRV8871)
    // =========================================================================
#ifdef MOTOR_DRIVER_DRV8871
    static MotorDCConfig dcConfigs[NUM_MOTORS] = {
        {0, true, DCDriverType::DRV8871, PIN_FL_IN1, PIN_FL_IN2, -1, 1},   // FL
        {1, true, DCDriverType::DRV8871, PIN_FR_IN1, PIN_FR_IN2, -1, -1},  // FR (reversed)
        {2, true, DCDriverType::DRV8871, PIN_BL_IN1, PIN_BL_IN2, -1, 1},   // BL
        {3, true, DCDriverType::DRV8871, PIN_BR_IN1, PIN_BR_IN2, -1, -1},  // BR (reversed)
    };
    
    static MotorDC dc0(dcConfigs[0]);
    static MotorDC dc1(dcConfigs[1]);
    static MotorDC dc2(dcConfigs[2]);
    static MotorDC dc3(dcConfigs[3]);
    
    dcMotors[0] = &dc0;
    dcMotors[1] = &dc1;
    dcMotors[2] = &dc2;
    dcMotors[3] = &dc3;
    
    motors[0] = dcMotors[0];
    motors[1] = dcMotors[1];
    motors[2] = dcMotors[2];
    motors[3] = dcMotors[3];
#endif

    // =========================================================================
    // STEPPER MOTORS (TMC2209 / A4988 / DRV8825) via MCP23017
    // =========================================================================
#if defined(STEPPER_DRIVER_TMC2209) || defined(STEPPER_DRIVER_A4988) || defined(STEPPER_DRIVER_DRV8825)
    
    // Initialize MCP23017 GPIO expander for stepper control
    Serial.println("[MotorManager] Initializing MCP23017 GPIO expander...");
    if (!mcpStepper.init()) {
        Serial.println("[MotorManager] ERROR: MCP23017 init failed!");
        return false;
    }
    Serial.println("[MotorManager] MCP23017 initialized at address 0x20");
    
    // Enable steppers (set STPR_ALL_EN low, active LOW)
    stepperEnableAll();
    Serial.println("[MotorManager] Steppers enabled");
    
    // Configure microstepping based on project_config.h
    // TMC2209 Microstepping Table (different from A4988!):
    //   MS1=0, MS2=0: 8 microsteps
    //   MS1=1, MS2=1: 16 microsteps
    //   MS1=1, MS2=0: 32 microsteps
    //   MS1=0, MS2=1: 64 microsteps
    #if STEPPER_MICROSTEPPING == 8
    stepperSetMicrostepping(false, false);  // MS1=0, MS2=0 = 8 microsteps
    #elif STEPPER_MICROSTEPPING == 16
    stepperSetMicrostepping(true, true);    // MS1=1, MS2=1 = 16 microsteps
    #elif STEPPER_MICROSTEPPING == 32
    stepperSetMicrostepping(true, false);   // MS1=1, MS2=0 = 32 microsteps
    #elif STEPPER_MICROSTEPPING == 64
    stepperSetMicrostepping(false, true);   // MS1=0, MS2=1 = 64 microsteps
    #else
    stepperSetMicrostepping(false, false);  // Default 8 microsteps
    #endif
    Serial.printf("[MotorManager] Microstepping: 1/%d\n", STEPPER_MICROSTEPPING);
    
    #ifdef STEPPER_DRIVER_TMC2209
    StepperDriverType driverType = StepperDriverType::TMC2209;
    #elif defined(STEPPER_DRIVER_A4988)
    StepperDriverType driverType = StepperDriverType::A4988;
    #else
    StepperDriverType driverType = StepperDriverType::DRV8825;
    #endif
    
    // Configure stepper motors
    // Note: Pin values are -1 because step/dir are on MCP23017, not Pico GPIO
    // The motor index maps to the physical motor on the MCP23017 (0=M1, 1=M2, etc.)
    static MotorStepperConfig stepperConfigs[NUM_MOTORS] = {
        {
            .index = 0,         // Motor index maps to M1 on MCP23017
            .enabled = true,
            .driverType = driverType,
            .pinStep = -1,      // Not used - via MCP23017 GPB1
            .pinDir = -1,       // Not used - via MCP23017 GPB0
            .pinEnable = -1,    // Shared enable via MCP23017 GPA3
            .stepsPerRev = STEPPER_STEPS_PER_REV,
            .microstepping = STEPPER_MICROSTEPPING,
            .direction = -1,    // Front-Left (inverted for vehicle forward)
            .maxSpeed = STEPPER_MAX_SPEED,
            .acceleration = STEPPER_ACCELERATION
        },
        {
            .index = 1,         // Motor index maps to M2 on MCP23017
            .enabled = true,
            .driverType = driverType,
            .pinStep = -1,      // Not used - via MCP23017 GPB3
            .pinDir = -1,       // Not used - via MCP23017 GPB2
            .pinEnable = -1,
            .stepsPerRev = STEPPER_STEPS_PER_REV,
            .microstepping = STEPPER_MICROSTEPPING,
            .direction = -1,    // Front-Right (reversed)
            .maxSpeed = STEPPER_MAX_SPEED,
            .acceleration = STEPPER_ACCELERATION
        },
        {
            .index = 2,         // Motor index maps to M3 on MCP23017
            .enabled = true,
            .driverType = driverType,
            .pinStep = -1,      // Not used - via MCP23017 GPB5
            .pinDir = -1,       // Not used - via MCP23017 GPB4
            .pinEnable = -1,
            .stepsPerRev = STEPPER_STEPS_PER_REV,
            .microstepping = STEPPER_MICROSTEPPING,
            .direction = -1,    // Back-Left (inverted for vehicle forward)
            .maxSpeed = STEPPER_MAX_SPEED,
            .acceleration = STEPPER_ACCELERATION
        },
        {
            .index = 3,         // Motor index maps to M4 on MCP23017
            .enabled = true,
            .driverType = driverType,
            .pinStep = -1,      // Not used - via MCP23017 GPB7
            .pinDir = -1,       // Not used - via MCP23017 GPB6
            .pinEnable = -1,
            .stepsPerRev = STEPPER_STEPS_PER_REV,
            .microstepping = STEPPER_MICROSTEPPING,
            .direction = -1,    // Back-Right (reversed)
            .maxSpeed = STEPPER_MAX_SPEED,
            .acceleration = STEPPER_ACCELERATION
        }
    };
    
    static MotorStepper stepper0(stepperConfigs[0]);
    static MotorStepper stepper1(stepperConfigs[1]);
    static MotorStepper stepper2(stepperConfigs[2]);
    static MotorStepper stepper3(stepperConfigs[3]);
    
    stepperMotors[0] = &stepper0;
    stepperMotors[1] = &stepper1;
    stepperMotors[2] = &stepper2;
    stepperMotors[3] = &stepper3;
    
    motors[0] = stepperMotors[0];
    motors[1] = stepperMotors[1];
    motors[2] = stepperMotors[2];
    motors[3] = stepperMotors[3];
#endif

    // Initialize all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i]) {
            if (!motors[i]->init()) {
                Serial.printf("[MotorManager] Motor %d init failed!\n", i);
                return false;
            }
        }
    }
    
    Serial.printf("[MotorManager] %d motors initialized\n", NUM_MOTORS);
    return true;
}

// =============================================================================
// PUBLIC API
// =============================================================================

MotorBase* motor_get(int index) {
    if (index < 0 || index >= NUM_MOTORS) {
        return nullptr;
    }
    return motors[index];
}

void motor_set_pwm(int index, int16_t pwm) {
    if (index < 0 || index >= NUM_MOTORS || !motors[index]) {
        return;
    }
    
    if (motors[index]->getType() == MotorType::DC) {
        static_cast<MotorDC*>(motors[index])->setTarget(pwm);
    }
}

void motor_set_speed(int index, float stepsPerSec) {
    if (index < 0 || index >= NUM_MOTORS || !motors[index]) {
        return;
    }
    
    if (motors[index]->getType() == MotorType::Stepper) {
        static_cast<MotorStepper*>(motors[index])->setTargetSpeed(stepsPerSec);
    }
}

void motors_update(float dtSec) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i]) {
            motors[i]->update(dtSec);
        }
    }
}

void motors_stop_all() {
    Serial.println("[MotorManager] >>> STOP ALL <<<");
    
    // NOTE: Do NOT touch I2C here - Core 1 owns the I2C bus for steppers
    // The emergency stop is signaled via shared memory in main.cpp
    // Core 1 will handle stopping the motors safely
    
    // For DC motors only (they don't use I2C on this board)
#if defined(MOTOR_DRIVER_DRV8871) || defined(MOTOR_DRIVER_DRV8833) || defined(MOTOR_DRIVER_L298N)
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i] && motors[i]->getType() == MotorType::DC) {
            motors[i]->stop();
        }
    }
#endif
}

bool motors_has_steppers() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i] && motors[i]->getType() == MotorType::Stepper) {
            return true;
        }
    }
    return false;
}

bool motors_has_dc() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i] && motors[i]->getType() == MotorType::DC) {
            return true;
        }
    }
    return false;
}
