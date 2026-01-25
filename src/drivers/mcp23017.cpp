/**
 * @file mcp23017.cpp
 * @brief MCP23017 16-bit I2C GPIO Expander Driver Implementation
 *
 * Controls the two MCP23017 chips on the MechaPico MCB:
 *   - U6_1 (0x20): Stepper motors M1-M5
 *   - U6_2 (0x21): DC motors 3-4, LEDs, general I/O
 */

#include "mcp23017.h"
#include "pico/stdlib.h"
#include <Arduino.h>
#include <cstring>

// =============================================================================
// STATIC MEMBERS
// =============================================================================

bool MCP23017::i2cInitialized_ = false;

// Global instances
MCP23017 mcpStepper(MCP23017_STEPPER_ADDR);  // U6_1 @ 0x20
MCP23017 mcpDCMotor(MCP23017_DCMOTOR_ADDR);  // U6_2 @ 0x21

// =============================================================================
// CONSTRUCTOR
// =============================================================================

MCP23017::MCP23017(uint8_t addr)
    : addr_(addr)
    , portA_(0)
    , portB_(0)
    , initialized_(false)
{
}

// =============================================================================
// I2C INITIALIZATION (shared across all instances)
// =============================================================================

void MCP23017::initI2C() {
    if (i2cInitialized_) return;
    
    // Initialize I2C peripheral
    i2c_init(MCP23017_I2C_PORT, MCP23017_I2C_FREQ);
    
    // Configure GPIO pins for I2C
    gpio_set_function(MCP23017_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MCP23017_I2C_SCL_PIN, GPIO_FUNC_I2C);
    
    // Enable internal pull-ups (external pull-ups recommended for reliable operation)
    gpio_pull_up(MCP23017_I2C_SDA_PIN);
    gpio_pull_up(MCP23017_I2C_SCL_PIN);
    
    i2cInitialized_ = true;
}

// =============================================================================
// DEVICE INITIALIZATION
// =============================================================================

bool MCP23017::init() {
    if (initialized_) return true;
    
    // Ensure I2C is initialized
    initI2C();
    
    // Configure all pins as outputs (IODIR = 0x00)
    if (!writeRegister(MCP23017_IODIRA, 0x00)) return false;
    if (!writeRegister(MCP23017_IODIRB, 0x00)) return false;
    
    // Initialize outputs to safe state
    if (addr_ == MCP23017_STEPPER_ADDR) {
        // TMC2209 Stepper controller:
        //   EN = LOW (enabled - active LOW)
        //   PDN = HIGH (standalone STEP/DIR mode, not UART mode)
        //   SPREAD = LOW (StealthChop mode)
        portA_ = STPR_ALL_PDN_BIT;  // EN=0 (enabled), PDN=1 (standalone mode)
        portB_ = 0x00;
    } else {
        // DC motor controller: All outputs low
        portA_ = 0x00;
        portB_ = 0x00;
    }
    
    if (!writeRegister(MCP23017_OLATA, portA_)) return false;
    if (!writeRegister(MCP23017_OLATB, portB_)) return false;
    
    initialized_ = true;
    return true;
}

// =============================================================================
// REGISTER ACCESS
// =============================================================================

bool MCP23017::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    int result = i2c_write_blocking(MCP23017_I2C_PORT, addr_, buffer, 2, false);
    return (result == 2);
}

uint8_t MCP23017::readRegister(uint8_t reg) {
    uint8_t value = 0;
    
    // Write register address
    int result = i2c_write_blocking(MCP23017_I2C_PORT, addr_, &reg, 1, true);
    if (result != 1) return 0;
    
    // Read register value
    result = i2c_read_blocking(MCP23017_I2C_PORT, addr_, &value, 1, false);
    if (result != 1) return 0;
    
    return value;
}

// =============================================================================
// PORT ACCESS
// =============================================================================

// I2C error tracking
static uint32_t i2cErrorCount = 0;
static uint32_t i2cWriteCount = 0;
static unsigned long lastI2CErrorLog = 0;

void MCP23017::setPortA(uint8_t value) {
    portA_ = value;
    if (!writeRegister(MCP23017_OLATA, portA_)) {
        i2cErrorCount++;
    }
    i2cWriteCount++;
}

void MCP23017::setPortB(uint8_t value) {
    portB_ = value;
    if (!writeRegister(MCP23017_OLATB, portB_)) {
        i2cErrorCount++;
        // NOTE: Serial.printf removed - causes jitter on Core 1
    }
    i2cWriteCount++;
}

// =============================================================================
// BIT MANIPULATION
// =============================================================================

void MCP23017::setBitA(uint8_t bit, bool state) {
    if (state) {
        portA_ |= bit;
    } else {
        portA_ &= ~bit;
    }
    writeRegister(MCP23017_OLATA, portA_);
}

void MCP23017::setBitB(uint8_t bit, bool state) {
    if (state) {
        portB_ |= bit;
    } else {
        portB_ &= ~bit;
    }
    writeRegister(MCP23017_OLATB, portB_);
}

void MCP23017::toggleBitA(uint8_t bit) {
    portA_ ^= bit;
    writeRegister(MCP23017_OLATA, portA_);
}

void MCP23017::toggleBitB(uint8_t bit) {
    portB_ ^= bit;
    writeRegister(MCP23017_OLATB, portB_);
}

// =============================================================================
// CONVENIENCE FUNCTIONS FOR STEPPER CONTROL
// =============================================================================

/**
 * @brief Enable all stepper motors (set STPR_ALL_EN LOW)
 */
void stepperEnableAll() {
    mcpStepper.setBitA(STPR_ALL_EN_BIT, false);  // Active LOW
}

/**
 * @brief Disable all stepper motors (set STPR_ALL_EN HIGH)
 */
void stepperDisableAll() {
    Serial.printf("[Stepper] DISABLED at %lu ms\n", millis());
    mcpStepper.setBitA(STPR_ALL_EN_BIT, true);  // Inactive HIGH
}

/**
 * @brief Set microstepping mode for all steppers
 * @param ms1 MS1 pin state
 * @param ms2 MS2 pin state
 * 
 * MS1=0, MS2=0: Full step
 * MS1=1, MS2=0: 1/2 step
 * MS1=0, MS2=1: 1/4 step
 * MS1=1, MS2=1: 1/8 step
 */
void stepperSetMicrostepping(bool ms1, bool ms2) {
    mcpStepper.setBitA(STPR_ALL_MS1_BIT, ms1);
    mcpStepper.setBitA(STPR_ALL_MS2_BIT, ms2);
}

/**
 * @brief Set direction for a specific motor
 * @param motorIndex Motor index (0-4 for M1-M5)
 * @param forward true for forward, false for reverse
 */
void stepperSetDirection(uint8_t motorIndex, bool forward) {
    if (motorIndex >= 5) return;
    
    const StepperPinConfig& cfg = STEPPER_PINS[motorIndex];
    
    if (cfg.port == 0) {
        mcpStepper.setBitA(cfg.dirBit, forward);
    } else {
        mcpStepper.setBitB(cfg.dirBit, forward);
    }
}

/**
 * @brief Generate a step pulse for a specific motor
 * @param motorIndex Motor index (0-4 for M1-M5)
 * 
 * Note: This toggles the STEP pin. Call twice for a complete pulse,
 * or use stepperPulse() for a complete high-low sequence.
 */
void stepperToggleStep(uint8_t motorIndex) {
    if (motorIndex >= 5) return;
    
    const StepperPinConfig& cfg = STEPPER_PINS[motorIndex];
    
    if (cfg.port == 0) {
        mcpStepper.toggleBitA(cfg.stepBit);
    } else {
        mcpStepper.toggleBitB(cfg.stepBit);
    }
}

/**
 * @brief Generate a complete step pulse (high then low)
 * @param motorIndex Motor index (0-4 for M1-M5)
 */
void stepperPulse(uint8_t motorIndex) {
    if (motorIndex >= 5) return;
    
    const StepperPinConfig& cfg = STEPPER_PINS[motorIndex];
    
    if (cfg.port == 0) {
        // Set high
        mcpStepper.setBitA(cfg.stepBit, true);
        // Brief delay for pulse width (minimum ~1us for most drivers)
        sleep_us(2);
        // Set low
        mcpStepper.setBitA(cfg.stepBit, false);
    } else {
        mcpStepper.setBitB(cfg.stepBit, true);
        sleep_us(2);
        mcpStepper.setBitB(cfg.stepBit, false);
    }
}

/**
 * @brief Generate step pulses for multiple motors with a single I2C transaction pair
 * 
 * This is the key optimization for I2C-based stepper control.
 * Instead of 2 I2C writes per motor (8 writes for 4 motors), we do just 2 total.
 */
void stepperPulseBatchPortB(uint8_t stepMask) {
    if (stepMask == 0) return;
    
    // Get current port B value and set all requested STEP bits HIGH
    uint8_t portB = mcpStepper.getPortB();
    mcpStepper.setPortB(portB | stepMask);
    
    // Brief delay for pulse width (TMC2209 minimum is 100ns, we use 2us for safety)
    sleep_us(2);
    
    // Clear all STEP bits (keep direction bits as they were)
    mcpStepper.setPortB(portB & ~stepMask);
}

/**
 * @brief Set direction bits for multiple motors efficiently
 */
void stepperSetDirectionBatch(uint8_t setHighMask, uint8_t setLowMask) {
    uint8_t portB = mcpStepper.getPortB();
    portB |= setHighMask;    // Set HIGH bits
    portB &= ~setLowMask;    // Clear LOW bits
    mcpStepper.setPortB(portB);
}
