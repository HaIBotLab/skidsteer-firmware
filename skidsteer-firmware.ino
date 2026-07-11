// ==================================================
// Project          : ESP32 Dual Motor Low-Level Controller
// File Name        : new.ino
// Target MCU       : ESP32-WROOM-32 (Dual Core Execution)
// Framework        : Arduino (ESP32 core)
// Description      : Real-time dual-core differential drive motor management utilizing 
//                    Modbus master protocols over an RS485 transceiver network.
//                    Pure PWM capture on CH5 via hardware interrupts.
// ==================================================

#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include <stdint.h>
#include "config.h"
#include "debug.h"

// ==================================================
// GLOBAL PERIPHERAL & DRIVER INSTANCES
// ==================================================
ModbusMaster nodeL;
ModbusMaster nodeR;
ZLAC8015D driverL;
ZLAC8015D driverR;

// FreeRTOS Handlers & Shared Resources
TaskHandle_t xTask1Handle     = NULL;
TaskHandle_t xTask2Handle     = NULL;
SemaphoreHandle_t uartSemaphore;

volatile int16_t move_range       = BASE_MOVE_RANGE;
volatile int16_t rotate_in_place  = BASE_ROTATE_IN_PLACE;
volatile int16_t move_and_turn    = BASE_MOVE_AND_TURN;

volatile float current_speed_mock = 1.0f;

// Interrupt Variable Captures (Volatile Memory Contexts)
volatile unsigned long pulseWidth2 = 1500;
volatile unsigned long pulseWidth4 = 1500;
volatile unsigned long pulseWidth5 = 0;
volatile unsigned long lastTime2   = 0;
volatile unsigned long lastTime4   = 0;
volatile unsigned long lastTime5   = 0;

// Forward declaration
float update_mock_ch5();

// ==================================================
// TASK 1: LEFT MOTOR CONTROL SUB-SYSTEM (CORE 0)
// ==================================================
void Task1(void *pvParameters) {
  uint32_t cntL = 0;
  int16_t fb_left[2] = {0, 0};
  int64_t io_latency_us = 0;

  int16_t rpm_left = 0;

  for(;;) {  
    // Atomic critical segment execution to shield volatile captures from corruption
    noInterrupts();
    unsigned long local_pw2 = pulseWidth2;
    unsigned long local_pw4 = pulseWidth4;
    unsigned long local_pw5 = pulseWidth5;
    interrupts();

    int16_t local_move_range = move_range * current_speed_mock;
    int16_t turn_range = 1 * current_speed_mock;

    // Check onboard Physical Button state acting as a hardware E-Stop loop
    bool emergency_halt = (digitalRead(ButtonPhysic) == LOW);

    if (!emergency_halt) {
      // Linear Mapping: Pitch Axis Control (CH2 Forward / Backward Handling)
      if (local_pw2 > bottom_forward_threshold && local_pw2 < top_forward_threshold) {
        rpm_left = map(local_pw2, RC_CH2_MIN_FORWARD, RC_CH2_MAX_FORWARD, 0, local_move_range);
        turn_range = move_and_turn;
      }
      else if (local_pw2 > bottom_stop_threshold && local_pw2 < top_stop_threshold) {
        rpm_left = 0;
        turn_range = rotate_in_place;
      }
      else if (local_pw2 > bottom_backward_threshold && local_pw2 < top_backward_threshold) {
        rpm_left = map(local_pw2, RC_CH2_MIN_BACKWARD, RC_CH2_MAX_BACKWARD, -local_move_range, 0);
        turn_range = move_and_turn;
      }

      // Defensive Boundaries: Prevent zero-division panics
      if (turn_range < 1) turn_range = 1;

      // Linear Mapping: Roll Axis Integration (CH4 Steering Matrix Calculations)
      if (local_pw4 > bottom_turnleft_threshold && local_pw4 < top_turnleft_threshold) {
        int turn_speed = map(local_pw4, RC_CH4_MIN_LEFT, RC_CH4_MAX_LEFT, 0, local_move_range);
        rpm_left += (turn_speed / turn_range);
      }
      else if (local_pw4 > bottom_turnright_threshold && local_pw4 < top_turnright_threshold) {
        int turn_speed = map(local_pw4, RC_CH4_MIN_RIGHT, RC_CH4_MAX_RIGHT, local_move_range, 0);
        rpm_left -= (turn_speed / turn_range);
      }   
    } else {
      rpm_left = 0;
    }

    rpm_left = constrain(rpm_left, -local_move_range, local_move_range);

    // Mutual Exclusion Zone: Secure RS485 Modbus Bus Resource
    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
      int64_t start_time = esp_timer_get_time();
      
      driverL.set_rpm(rpm_left, rpm_left);
      driverL.get_rpm(fb_left);
      
      io_latency_us = esp_timer_get_time() - start_time;
      xSemaphoreGive(uartSemaphore);
    }
    
    cntL++;
    if (cntL > 9999) cntL = 0;

    current_speed_mock = update_mock_ch5();

    if (debug) print_motor_log('L', cntL, current_speed_mock, local_pw2, local_pw4, local_pw5, rpm_left, fb_left[0], fb_left[1], io_latency_us);

    vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
  }
}

// ==================================================
// TASK 2: RIGHT MOTOR CONTROL SUB-SYSTEM (CORE 1)
// ==================================================
void Task2(void *pvParameters) {
  uint32_t cntR = 0;
  int16_t fb_right[2] = {0, 0};
  int64_t io_latency_us = 0;

  int16_t rpm_right = 0;

  for(;;) {
    noInterrupts();
    unsigned long local_pw2 = pulseWidth2;
    unsigned long local_pw4 = pulseWidth4;
    unsigned long local_pw5 = pulseWidth5;
    interrupts();

    int16_t local_move_range = move_range * current_speed_mock;
    int16_t turn_range = 1 * current_speed_mock;

    bool emergency_halt = (digitalRead(ButtonPhysic) == LOW);

    if (!emergency_halt) {
      // Linear Mapping: Pitch Axis Control (CH2 Forward / Backward Handling)
      if (local_pw2 > bottom_forward_threshold && local_pw2 < top_forward_threshold) {
        rpm_right = -map(local_pw2, RC_CH2_MIN_FORWARD, RC_CH2_MAX_FORWARD, 0, local_move_range);
        turn_range = move_and_turn;
      }
      else if (local_pw2 > bottom_stop_threshold && local_pw2 < top_stop_threshold) {
        rpm_right = 0;
        turn_range = rotate_in_place;
      }
      else if (local_pw2 > bottom_backward_threshold && local_pw2 < top_backward_threshold) {
        rpm_right = -map(local_pw2, RC_CH2_MIN_BACKWARD, RC_CH2_MAX_BACKWARD, -local_move_range, 0);
        turn_range = move_and_turn;
      }

      if (turn_range < 1) turn_range = 1;

      // Linear Mapping: Roll Axis Integration (CH4 Steering Matrix Calculations)
      if (local_pw4 > bottom_turnleft_threshold && local_pw4 < top_turnleft_threshold) {
        int turn_speed = map(local_pw4, RC_CH4_MIN_LEFT, RC_CH4_MAX_LEFT, 0, local_move_range);
        rpm_right += (turn_speed / turn_range);
      }
      else if (local_pw4 > bottom_turnright_threshold && local_pw4 < top_turnright_threshold) {
        int turn_speed = map(local_pw4, RC_CH4_MIN_RIGHT, RC_CH4_MAX_RIGHT, local_move_range, 0);
        rpm_right -= (turn_speed / turn_range);
      }    
    } else {
      rpm_right = 0;
    }

    rpm_right = constrain(rpm_right, -local_move_range, local_move_range);

    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
      int64_t start_time = esp_timer_get_time();
      
      driverR.set_rpm(rpm_right, rpm_right);
      driverR.get_rpm(fb_right);
      
      io_latency_us = esp_timer_get_time() - start_time;
      xSemaphoreGive(uartSemaphore);
    }

    cntR++;
    if (cntR > 9999) cntR = 0;

    if (debug) print_motor_log('R', cntR, current_speed_mock, local_pw2, local_pw4, local_pw5, rpm_right, fb_right[0], fb_right[1], io_latency_us);

    vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
  }
}

// ==================================================
// INTERRUPT SERVICE ROUTINES (ISR) - IRAM SAFE EXECUTION
// ==================================================
void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void IRAM_ATTR handleInterrupt2() {
  if (digitalRead(rcPin2) == HIGH) lastTime2 = micros();
  else pulseWidth2 = micros() - lastTime2;
}

void IRAM_ATTR handleInterrupt4() {
  if (digitalRead(rcPin4) == HIGH) lastTime4 = micros();
  else pulseWidth4 = micros() - lastTime4;
}

void IRAM_ATTR handleInterrupt5() {
  if (digitalRead(rcPin5) == HIGH) lastTime5 = micros();
  else pulseWidth5 = micros() - lastTime5;
}

float update_mock_ch5() {
  noInterrupts();
  unsigned long local_pw5 = pulseWidth5;
  interrupts();

  if (local_pw5 > bottom_mock_2_threshold && local_pw5 < top_mock_2_threshold) {
    // Profile 2: pw5 = 1600-2200µs → Maximum performance
    current_speed_mock = RC_CH5_K2;
  }
  else if (local_pw5 > bottom_mock_1_threshold && local_pw5 < top_mock_1_threshold) {
    // Profile 1: pw5 = 1000-1600µs → Medium speed boost
    current_speed_mock = RC_CH5_K1;
  }
  else if (local_pw5 > bottom_mock_0_threshold && local_pw5 < top_mock_0_threshold) {
    // Profile 0: pw5 = 900-1000µs → Baseline
    current_speed_mock = RC_CH5_K0;
  }
  return current_speed_mock;
}

// ==================================================
// PERIPHERAL FIRMWARE HARDWARE BOOTSTRAP
// ==================================================
void setup() {
  if (debug) Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  postTransmission();
  
  pinMode(rcPin2, INPUT);
  pinMode(rcPin4, INPUT);
  pinMode(rcPin5, INPUT);
  pinMode(ButtonPhysic, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(rcPin2), handleInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rcPin4), handleInterrupt4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rcPin5), handleInterrupt5, CHANGE);

  uartSemaphore = xSemaphoreCreateMutex();
  if (uartSemaphore == NULL) {
    Serial.println("CRITICAL ERROR: UART Mutex Allocation Failed!");
    while(1);
  }

  nodeL.begin(1, Serial1);
  nodeR.begin(2, Serial1);
  nodeL.preTransmission(preTransmission);
  nodeL.postTransmission(postTransmission);
  nodeR.preTransmission(preTransmission);
  nodeR.postTransmission(postTransmission);

  driverL.set_modbus(&nodeL);
  driverR.set_modbus(&nodeR);

  driverL.disable_motor();
  driverL.set_mode(MODE);
  driverL.enable_motor();
  delay(10);
  driverL.set_accel_time(L_MS_A1, L_MS_A2);
  driverL.set_decel_time(L_MS_D1, L_MS_D2);

  driverR.disable_motor();
  driverR.set_mode(MODE);
  driverR.enable_motor();
  delay(10);
  driverR.set_accel_time(R_MS_A1, R_MS_A2);
  driverR.set_decel_time(R_MS_D1, R_MS_D2);

  xTaskCreatePinnedToCore(Task1, "Task_Motor_L", STACK_SIZE, NULL, 3, &xTask1Handle, 0);
  xTaskCreatePinnedToCore(Task2, "Task_Motor_R", STACK_SIZE, NULL, 3, &xTask2Handle, 1);

  digitalWrite(LED_PIN, LOW);
  Serial.println("System Base Subsystem Ready (CH5 PWM Reader Enabled).");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
