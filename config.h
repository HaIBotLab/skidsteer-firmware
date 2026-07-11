// ==================================================
// Project          : ESP32 Dual Motor Controller
// File Name        : config.h
// Created          : Jul 11, 2026
// Author           : Pham Duc Duy
// Framework        : Arduino (ESP32 core)
// Description      : Centralized hardware pinouts, software execution thresholds, 
//                    and absolute RC transmitter input calibrations.
// ==================================================

#ifndef CONFIG_H
#define CONFIG_H

// ==================================================
// SYSTEM CONFIGURATION & COMPILATION SWITCHES
// ==================================================
#define ENABLE_DEBUG          1     // Set to 0 to strip all logging overhead from the binary
#define TASK_DELAY_MS         50    // RTOS Task scheduler cycle loop time (50ms = 20Hz update frequency)
#define STACK_SIZE            4096  // [FIX-7] Raised from 3072 to 4096: Modbus + Serial + snprintf cần margin an toàn

// ==================================================
// HARDWARE PINOUT DEFINITIONS
// ==================================================
const int rcPin2        = 17; // CH2: Pitch Input (Forward / Backward)
const int rcPin4        = 18; // CH4: Roll Input (Turn Left / Turn Right)
const int rcPin5        = 22; // CH5: 3-Way Selector Switch for Dynamic Profiling
#define LED_PIN           12
#define ButtonPhysic      15
#define MAX485_DE         32
#define MAX485_RE_NEG     33
#define MODBUS_RX_PIN     27
#define MODBUS_TX_PIN     13

// ==================================================
// MOTOR DRIVER SYSTEM HYPERPARAMETERS
// ==================================================
const uint8_t MODE      = 3;
const uint16_t L_MS_A1  = 100; const uint16_t L_MS_A2 = 100; // Acceleration ramps (ms)
const uint16_t L_MS_D1  = 100; const uint16_t L_MS_D2 = 100; // Deceleration ramps (ms)
const uint16_t R_MS_A1  = 100; const uint16_t R_MS_A2 = 100;
const uint16_t R_MS_D1  = 100; const uint16_t R_MS_D2 = 100;

// ==================================================
// RC STATE-GATE GATING THRESHOLDS (Microseconds)
// ==================================================
const uint16_t bottom_forward_threshold   = 1580;
const uint16_t top_forward_threshold      = 1810;
const uint16_t bottom_stop_threshold      = 1490;
const uint16_t top_stop_threshold         = 1570;
const uint16_t bottom_backward_threshold  = 1280;
const uint16_t top_backward_threshold     = 1480;

const uint16_t bottom_turnleft_threshold  = 1180;
const uint16_t top_turnleft_threshold     = 1390;
const uint16_t bottom_turnright_threshold = 1510;
const uint16_t top_turnright_threshold    = 1700;

// ==================================================
// RC HARDWARE LINEAR MAPPING BOUNDARIES (Microseconds)
// ==================================================
const uint16_t RC_CH2_MIN_FORWARD  = 1571; // Ch5 Profile 0 entry minimum forward limit
const uint16_t RC_CH2_MAX_FORWARD  = 1850; // Maximum physical stick excursion limit
const uint16_t RC_CH2_MIN_BACKWARD = 1280; // Minimum physical stick excursion limit
const uint16_t RC_CH2_MAX_BACKWARD = 1538; // Profile 0 entry minimum backward limit

const uint16_t RC_CH4_MIN_LEFT     = 1470; // Center edge limit for left turns
const uint16_t RC_CH4_MAX_LEFT     = 1690; // Maximum mechanical boundary for left turns
const uint16_t RC_CH4_MIN_RIGHT    = 1200; // Minimum mechanical boundary for right turns
const uint16_t RC_CH4_MAX_RIGHT    = 1448; // Center edge limit for right turns

// ==================================================
// CH5 PROFILE MULTIPLIER COEFFICIENTS (k-Factors)
// ==================================================
const float RC_CH5_K0              = 1.0f; // Baseline multiplier for Profile 0
const float RC_CH5_K1              = 1.5f; // Multiplier for Profile 1 (Medium speed boost)
const float RC_CH5_K2              = 2.0f; // Multiplier for Profile 2 (Maximum performance)

// CH5 Profile Threshold Boundaries (Microseconds)
const uint16_t bottom_mock_0_threshold = 1600;
const uint16_t top_mock_0_threshold    = 2200;
const uint16_t bottom_mock_1_threshold = 1000;
const uint16_t top_mock_1_threshold    = 1600;
const uint16_t bottom_mock_2_threshold = 900;
const uint16_t top_mock_2_threshold    = 1000;

// ==================================================
// BASELINE MOTION DYNAMICS (PROFILE LEVEL 0)
// ==================================================
const int16_t BASE_MOVE_RANGE      = 100; // Reference maximum target RPM boundary
const int16_t BASE_ROTATE_IN_PLACE = 9;   // High scaling denominator = smooth rotation in place
const int16_t BASE_MOVE_AND_TURN   = 4;   // Medium scaling denominator = standard steering split

// ==================================================
// DEBUG
// ==================================================
bool debug = true;

#endif // CONFIG_H
