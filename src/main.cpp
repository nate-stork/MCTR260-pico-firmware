/**
 * @file main.cpp
 * @brief Main Entry Point for Raspberry Pi Pico W Robot Controller
 *
 * MULTICORE ARCHITECTURE:
 * - Core 0: BLE (BTstack), command parsing, kinematics
 * - Core 1: Dedicated stepper pulse generation (500µs timing)
 * - Inter-core: Mutex-protected shared memory for speed commands
 */

#include <Arduino.h>
#include <Wire.h>
#include "pico/mutex.h"

// Project configuration
#include "project_config.h"

// Core modules
#include "core/ble_controller.h"
#include "core/command_parser.h"
#include "core/motor_manager.h"
#include "core/safety.h"
#include "core/simple_stepper.h"
#include "drivers/mcp23017.h"

// Profiles
#include "profiles/profile_mecanum.h"

// =============================================================================
// INTER-CORE COMMUNICATION (shared between Core 0 and Core 1)
// =============================================================================

// Mutex for thread-safe access to shared speed data
mutex_t g_speedMutex;

// Shared speed commands (written by Core 0, read by Core 1)
volatile float g_targetSpeeds[4] = {0, 0, 0, 0};
volatile bool g_speedsUpdated = false;
volatile bool g_emergencyStop = false;

// =============================================================================
// GLOBAL STATE (Core 0 only)
// =============================================================================

static bool s_bleConnected = false;
static unsigned long s_lastUpdateTime = 0;

// =============================================================================
// CORE 1: STEPPER PULSE GENERATION
// =============================================================================

/**
 * @brief Core 1 setup - runs in parallel with Core 0 setup()
 */
void setup1() {
    // Wait for Core 0 to initialize I2C and MCP23017 first
    delay(2000);
    
    // Initialize the stepper system on Core 1
    simple_stepper_init();
}

/**
 * @brief Core 1 loop - dedicated to stepper pulse generation
 * 
 * This runs completely isolated from BLE processing, ensuring
 * deterministic timing for step pulses.
 */
void loop1() {
    // Check for new speed commands (non-blocking)
    if (mutex_try_enter(&g_speedMutex, nullptr)) {
        if (g_emergencyStop) {
            simple_stepper_stop_all();
            g_emergencyStop = false;
        } else if (g_speedsUpdated) {
            for (int i = 0; i < 4; i++) {
                simple_stepper_set_speed(i, g_targetSpeeds[i]);
            }
            g_speedsUpdated = false;
        }
        mutex_exit(&g_speedMutex);
    }
    
    // Generate step pulses (runs at 500µs intervals internally)
    simple_stepper_update();
}

// =============================================================================
// CORE 0: BLE CALLBACKS
// =============================================================================

/**
 * @brief Called when a command is received over BLE (Core 0)
 */
void onBleCommand(const char* jsonData, uint16_t length) {
    // Feed safety watchdog
    safety_feed();
    
    // Parse command
    control_command_t cmd;
    if (!command_parse(jsonData, &cmd)) {
        return;
    }
    
    // Skip heartbeats (they just keep connection alive)
    if (command_is_heartbeat()) {
        return;
    }
    
    // Apply motion profile based on vehicle type
    if (strcmp(cmd.vehicle, "mecanum") == 0) {
#ifdef MOTION_PROFILE_MECANUM
        profile_mecanum_apply(&cmd);
#endif
    }
}

/**
 * @brief Called when BLE connection state changes (Core 0)
 */
void onBleConnectionChange(bool connected) {
    s_bleConnected = connected;
    
    if (connected) {
        Serial.println(">>> Client connected!");
    } else {
        Serial.println(">>> Client disconnected - stopping motors");
        
        // Signal Core 1 to stop all motors
        mutex_enter_blocking(&g_speedMutex);
        g_emergencyStop = true;
        mutex_exit(&g_speedMutex);
        
        motors_stop_all();
    }
}

// =============================================================================
// CORE 0: ARDUINO SETUP
// =============================================================================

void setup() {
    // Initialize mutex FIRST (before Core 1 starts using it)
    mutex_init(&g_speedMutex);
    
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Wait 10 seconds for serial monitor reconnection after flash
    delay(10000);
    while (!Serial && millis() < 13000) {
        // Wait for serial (additional 3 seconds if not ready)
    }
    
    Serial.println();
    Serial.println("===========================================");
    Serial.println("   RC Robot Controller - Pico W Edition");
    Serial.println("        MULTICORE ARCHITECTURE v2.0");
    Serial.println("===========================================");
    Serial.printf("Device Name: %s\n", DEVICE_NAME);
    Serial.printf("PIN Code: %06d\n", BLE_PASSKEY);
    Serial.println("Core 0: BLE + Commands");
    Serial.println("Core 1: Stepper Pulses");
    Serial.println();
    
    // Initialize LED for status
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Initialize safety watchdog
    safety_init();
        
    // =========================================================================
    // MOTOR INITIALIZATION (Core 0 - I2C setup)
    // =========================================================================
    Serial.println("[Core0] Initializing motors...");
    bool motorsOk = motors_init();
    if (!motorsOk) {
        Serial.println("ERROR: Motor initialization failed!");
        Serial.println("       MCP23017 not responding - check I2C wiring");
    } else {
        Serial.println("[Core0] Motors initialized OK");
    }
    
    // STEPPER INITIALIZATION (only if motors initialized)
#if defined(STEPPER_DRIVER_TMC2209) || defined(STEPPER_DRIVER_A4988) || defined(STEPPER_DRIVER_DRV8825)
    if (motorsOk) {
        // NOTE: Microstepping already configured by motor_manager based on STEPPER_MICROSTEPPING
        // Just set additional TMC2209-specific pins here
        mcpStepper.setBitA(STPR_ALL_SPRD_BIT, false);  // StealthChop (quiet mode)
        mcpStepper.setBitA(STPR_ALL_PDN_BIT, true);    // PDN=1 for standalone STEP/DIR mode
        // EN already set by motor_manager
        
        Serial.printf("[Core0] Steppers ready (%d microsteps)\n", STEPPER_MICROSTEPPING);
    }
#endif
    
    // =========================================================================
    // BLE INITIALIZATION (Core 0 - after motors to avoid I2C conflicts)
    // =========================================================================
    ble_init(onBleCommand, onBleConnectionChange);
    
    Serial.println();
    Serial.println("[Core0] Setup complete - Core 1 handles stepper pulses");
    Serial.println();
    
    s_lastUpdateTime = millis();
}

// =============================================================================
// CORE 0: ARDUINO LOOP (BLE only - no stepper updates!)
// =============================================================================

void loop() {
    // Process BLE events (this is Core 0's main job)
    ble_update();
    
    // Motor update at 50Hz (safety check only)
    unsigned long now = millis();
    if (now - s_lastUpdateTime >= MOTOR_UPDATE_INTERVAL_MS) {
        s_lastUpdateTime = now;
        
        // Check safety timeout
        if (s_bleConnected && safety_check_timeout()) {
            // Signal Core 1 to stop
            mutex_enter_blocking(&g_speedMutex);
            g_emergencyStop = true;
            mutex_exit(&g_speedMutex);
        }
    }
    
    // Status LED: blink when disconnected, solid when connected
    if (!s_bleConnected) {
        static unsigned long lastBlink = 0;
        if (now - lastBlink >= 500) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            lastBlink = now;
        }
    }
}
