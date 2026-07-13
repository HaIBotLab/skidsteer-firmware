// ==================================================
// Project          : ESP32 Dual Motor Controller
// File Name        : skidsteer-firmware.ino (new_2.ino)
// Framework        : Arduino (ESP32 core)
// Description      : Real-time dual-core differential drive motor management.
//                    Features Thread-Safe micro-ROS logging via Queue.
// ==================================================

#include <ModbusMaster.h>
#include <ZLAC8015D.h>
#include <stdint.h>
#include "config.h"
#include "debug.h"
#include "micro_ros_handler.h"   // Xử lý micro-ROS và Logging Queue

// ==================================================
// GLOBAL PERIPHERAL & DRIVER INSTANCES
// ==================================================
ModbusMaster nodeL;
ModbusMaster nodeR;
ZLAC8015D    driverL;
ZLAC8015D    driverR;

// FreeRTOS Handlers & Shared Resources
TaskHandle_t      xTask1Handle  = NULL;
TaskHandle_t      xTask2Handle  = NULL;
SemaphoreHandle_t uartSemaphore = NULL;

// Dynamic profile multipliers (updated by CH5 ISR path)
volatile int16_t  move_range      = BASE_MOVE_RANGE;
volatile int16_t  rotate_in_place = BASE_ROTATE_IN_PLACE;
volatile int16_t  move_and_turn   = BASE_MOVE_AND_TURN;
volatile float    current_speed_mock = 1.0f;

// Interrupt Variable Captures (Volatile — IRAM-safe)
volatile unsigned long pulseWidth2 = 1500;
volatile unsigned long pulseWidth4 = 1500;
volatile unsigned long pulseWidth5 = 0;
volatile unsigned long pulseWidth6 = 0;   // CH6: mode switch
volatile unsigned long lastTime2   = 0;
volatile unsigned long lastTime4   = 0;
volatile unsigned long lastTime5   = 0;
volatile unsigned long lastTime6   = 0;

// ==================================================
// FORWARD DECLARATIONS
// ==================================================
float update_mock_ch5();

// ==================================================
// HELPER: RC → RPM computation for LEFT wheel
// ==================================================
static int16_t compute_rc_rpm_left(unsigned long pw2, unsigned long pw4,
                                   int16_t local_move_range,
                                   int16_t &turn_range_out)
{
    int16_t rpm = 0;
    turn_range_out = static_cast<int16_t>(1 * current_speed_mock);

    if (pw2 > bottom_forward_threshold && pw2 < top_forward_threshold) {
        rpm = map(pw2, RC_CH2_MIN_FORWARD, RC_CH2_MAX_FORWARD, 0, local_move_range);
        turn_range_out = move_and_turn;
    } else if (pw2 > bottom_stop_threshold && pw2 < top_stop_threshold) {
        rpm = 0;
        turn_range_out = rotate_in_place;
    } else if (pw2 > bottom_backward_threshold && pw2 < top_backward_threshold) {
        rpm = map(pw2, RC_CH2_MIN_BACKWARD, RC_CH2_MAX_BACKWARD, -local_move_range, 0);
        turn_range_out = move_and_turn;
    }

    if (turn_range_out < 1) turn_range_out = 1;

    if (pw4 > bottom_turnleft_threshold && pw4 < top_turnleft_threshold) {
        int turn_speed = map(pw4, RC_CH4_MIN_LEFT, RC_CH4_MAX_LEFT, 0, local_move_range);
        rpm += (turn_speed / turn_range_out);
    } else if (pw4 > bottom_turnright_threshold && pw4 < top_turnright_threshold) {
        int turn_speed = map(pw4, RC_CH4_MIN_RIGHT, RC_CH4_MAX_RIGHT, local_move_range, 0);
        rpm -= (turn_speed / turn_range_out);
    }

    return rpm;
}

// ==================================================
// HELPER: RC → RPM computation for RIGHT wheel
// ==================================================
static int16_t compute_rc_rpm_right(unsigned long pw2, unsigned long pw4,
                                    int16_t local_move_range,
                                    int16_t &turn_range_out)
{
    int16_t rpm = 0;
    turn_range_out = static_cast<int16_t>(1 * current_speed_mock);

    if (pw2 > bottom_forward_threshold && pw2 < top_forward_threshold) {
        rpm = -map(pw2, RC_CH2_MIN_FORWARD, RC_CH2_MAX_FORWARD, 0, local_move_range);
        turn_range_out = move_and_turn;
    } else if (pw2 > bottom_stop_threshold && pw2 < top_stop_threshold) {
        rpm = 0;
        turn_range_out = rotate_in_place;
    } else if (pw2 > bottom_backward_threshold && pw2 < top_backward_threshold) {
        rpm = -map(pw2, RC_CH2_MIN_BACKWARD, RC_CH2_MAX_BACKWARD, -local_move_range, 0);
        turn_range_out = move_and_turn;
    }

    if (turn_range_out < 1) turn_range_out = 1;

    if (pw4 > bottom_turnleft_threshold && pw4 < top_turnleft_threshold) {
        int turn_speed = map(pw4, RC_CH4_MIN_LEFT, RC_CH4_MAX_LEFT, 0, local_move_range);
        rpm += (turn_speed / turn_range_out);
    } else if (pw4 > bottom_turnright_threshold && pw4 < top_turnright_threshold) {
        int turn_speed = map(pw4, RC_CH4_MIN_RIGHT, RC_CH4_MAX_RIGHT, local_move_range, 0);
        rpm -= (turn_speed / turn_range_out);
    }

    return rpm;
}

// ==================================================
// TASK 1: LEFT MOTOR CONTROL SUB-SYSTEM (CORE 0)
// ==================================================
void Task1(void *pvParameters)
{
    uint32_t cntL       = 0;
    int16_t  fb_left[2] = {0, 0};
    int64_t  io_latency_us = 0;

    for (;;) {
        // Lấy dữ liệu ngắt
        noInterrupts();
        const unsigned long local_pw2 = pulseWidth2;
        const unsigned long local_pw4 = pulseWidth4;
        const unsigned long local_pw5 = pulseWidth5;
        const unsigned long local_pw6 = pulseWidth6;
        interrupts();

        const bool use_rc = ros_use_rc_mode(local_pw6);
        const int16_t local_move_range = static_cast<int16_t>(move_range * current_speed_mock);

        int16_t rpm_left = 0;

        if (use_rc) {
            int16_t tr_unused = 0;
            rpm_left = compute_rc_rpm_left(local_pw2, local_pw4, local_move_range, tr_unused);
        } else {
            // Chế độ ROS Tự động
            rpm_left = g_ros_rpm_left;
        }

        rpm_left = constrain(rpm_left, -local_move_range, local_move_range);

        // Chiếm quyền bus RS485 để đẩy lệnh
        if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
            const int64_t t0 = esp_timer_get_time();
            driverL.set_rpm(rpm_left, rpm_left);
            driverL.get_rpm(fb_left);
            io_latency_us = esp_timer_get_time() - t0;
            xSemaphoreGive(uartSemaphore);
        }

        // Đẩy RPM thực tế cho ROS Task đọc
        g_fb_rpm_left = fb_left[0];

        cntL++;
        if (cntL > 9999) cntL = 0;

        current_speed_mock = update_mock_ch5();

        // Gửi Log vào Hàng đợi (Queue) - THÊM THAM SỐ use_rc
        if (debug) print_motor_log('L', cntL, current_speed_mock,
                                   local_pw2, local_pw4, local_pw5, local_pw6,
                                   rpm_left, fb_left[0], fb_left[1], io_latency_us, use_rc);

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}

// ==================================================
// TASK 2: RIGHT MOTOR CONTROL SUB-SYSTEM (CORE 1)
// ==================================================
void Task2(void *pvParameters)
{
    uint32_t cntR        = 0;
    int16_t  fb_right[2] = {0, 0};
    int64_t  io_latency_us = 0;

    for (;;) {
        noInterrupts();
        const unsigned long local_pw2 = pulseWidth2;
        const unsigned long local_pw4 = pulseWidth4;
        const unsigned long local_pw5 = pulseWidth5;
        const unsigned long local_pw6 = pulseWidth6;
        interrupts();

        const bool use_rc = ros_use_rc_mode(local_pw6);
        const int16_t local_move_range = static_cast<int16_t>(move_range * current_speed_mock);

        int16_t rpm_right = 0;

        if (use_rc) {
            int16_t tr_unused = 0;
            rpm_right = compute_rc_rpm_right(local_pw2, local_pw4, local_move_range, tr_unused);
        } else {
            rpm_right = g_ros_rpm_right;
        }

        rpm_right = constrain(rpm_right, -local_move_range, local_move_range);

        if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
            const int64_t t0 = esp_timer_get_time();
            driverR.set_rpm(rpm_right, rpm_right);
            driverR.get_rpm(fb_right);
            io_latency_us = esp_timer_get_time() - t0;
            xSemaphoreGive(uartSemaphore);
        }

        g_fb_rpm_right = fb_right[0];

        cntR++;
        if (cntR > 9999) cntR = 0;

        // Gửi Log vào Hàng đợi (Queue) - THÊM THAM SỐ use_rc
        if (debug) print_motor_log('R', cntR, current_speed_mock,
                                   local_pw2, local_pw4, local_pw5, local_pw6,
                                   rpm_right, fb_right[0], fb_right[1], io_latency_us, use_rc);

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}

// ==================================================
// RS485 BUS DIRECTION CONTROL
// ==================================================
void preTransmission()
{
    digitalWrite(MAX485_RE_NEG, 1);
    digitalWrite(MAX485_DE,     1);
}

void postTransmission()
{
    digitalWrite(MAX485_RE_NEG, 0);
    digitalWrite(MAX485_DE,     0);
}

// ==================================================
// INTERRUPT SERVICE ROUTINES (IRAM-SAFE)
// ==================================================
void IRAM_ATTR handleInterrupt2()
{
    if (digitalRead(rcPin2) == HIGH) lastTime2 = micros();
    else pulseWidth2 = micros() - lastTime2;
}

void IRAM_ATTR handleInterrupt4()
{
    if (digitalRead(rcPin4) == HIGH) lastTime4 = micros();
    else pulseWidth4 = micros() - lastTime4;
}

void IRAM_ATTR handleInterrupt5()
{
    if (digitalRead(rcPin5) == HIGH) lastTime5 = micros();
    else pulseWidth5 = micros() - lastTime5;
}

void IRAM_ATTR handleInterrupt6()
{
    if (digitalRead(rcPin6) == HIGH) lastTime6 = micros();
    else pulseWidth6 = micros() - lastTime6;
}

// ==================================================
// CH5 PROFILE UPDATE
// ==================================================
float update_mock_ch5()
{
    noInterrupts();
    const unsigned long local_pw5 = pulseWidth5;
    interrupts();

    if (local_pw5 > bottom_mock_2_threshold && local_pw5 < top_mock_2_threshold) {
        current_speed_mock = RC_CH5_K2;
    } else if (local_pw5 > bottom_mock_1_threshold && local_pw5 < top_mock_1_threshold) {
        current_speed_mock = RC_CH5_K1;
    } else if (local_pw5 > bottom_mock_0_threshold && local_pw5 < top_mock_0_threshold) {
        current_speed_mock = RC_CH5_K0;
    }
    return current_speed_mock;
}

// ==================================================
// ERROR LOOP 
// ==================================================
void error_loop()
{
    // Đã loại bỏ in Serial lỗi ở đây vì ROS sử dụng giao thức Serial USB nhị phân.
    while (true) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

// ==================================================
// SETUP
// ==================================================
void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);

    pinMode(LED_PIN,       OUTPUT);
    pinMode(MAX485_RE_NEG, OUTPUT);
    pinMode(MAX485_DE,     OUTPUT);
    postTransmission();

    pinMode(rcPin2, INPUT);
    pinMode(rcPin4, INPUT);
    pinMode(rcPin5, INPUT);
    pinMode(rcPin6, INPUT);

    attachInterrupt(digitalPinToInterrupt(rcPin2), handleInterrupt2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rcPin4), handleInterrupt4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rcPin5), handleInterrupt5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rcPin6), handleInterrupt6, CHANGE);

    uartSemaphore = xSemaphoreCreateMutex();
    if (uartSemaphore == NULL) {
        while (true);
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

    // Launch micro-ROS handler task (Core 0)
    ros_handler_init();

    // Launch motor control tasks
    xTaskCreatePinnedToCore(Task1, "Task_Motor_L", STACK_SIZE, NULL, 3, &xTask1Handle, 0);
    xTaskCreatePinnedToCore(Task2, "Task_Motor_R", STACK_SIZE, NULL, 3, &xTask2Handle, 1);

    digitalWrite(LED_PIN, LOW);
}

// ==================================================
// LOOP
// ==================================================
void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}