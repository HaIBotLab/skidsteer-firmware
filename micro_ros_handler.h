// ==================================================
// Project          : ESP32 Dual Motor Controller
// File Name        : micro_ros_handler.h
// Description      : Header-only micro-ROS abstraction layer with Thread-Safe Logging.
// ==================================================

#ifndef MICRO_ROS_HANDLER_H
#define MICRO_ROS_HANDLER_H

// ---------- micro-ROS ----------
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

// ---------- FreeRTOS ----------
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ---------- project ----------
#include "config.h"

// ==================================================
// MODULE CONSTANTS & QUEUE CONFIG
// ==================================================
#define ROS_TASK_STACK_SIZE        8192
#define ROS_TASK_PRIORITY          2
#define ROS_TASK_CORE              0
#define ROS_AGENT_PING_MS          1000
#define ROS_AGENT_PING_ATTEMPTS    3
#define ROS_EXECUTOR_HANDLES       1

#define ROS_LOG_QUEUE_LEN          10    // Lưu tối đa 10 dòng log trong hàng đợi
#define ROS_LOG_MAX_LEN            180   // Chiều dài tối đa 1 dòng log

// ==================================================
// SHARED STATE
// ==================================================
volatile int16_t g_ros_rpm_left  = 0;
volatile int16_t g_ros_rpm_right = 0;
volatile bool    g_ros_cmd_fresh = false;
volatile bool    g_ros_connected = false;
volatile int16_t g_fb_rpm_left   = 0;
volatile int16_t g_fb_rpm_right  = 0;

QueueHandle_t    g_ros_log_queue = NULL;

// ==================================================
// FILE-SCOPED ROS OBJECTS
// ==================================================
static rcl_node_t           s_node;
static rcl_allocator_t      s_allocator;
static rclc_support_t       s_support;
static rclc_executor_t      s_executor;
static rcl_publisher_t      s_pub_encoder;
static rcl_publisher_t      s_pub_debug;      
static rcl_subscription_t   s_sub_wheel_rpm;

static std_msgs__msg__Int32MultiArray s_msg_encoder;
static std_msgs__msg__Int32MultiArray s_msg_wheel_rpm;
static std_msgs__msg__String          s_msg_debug;

static int32_t s_enc_data[2] = {0, 0};
static int32_t s_cmd_data[2] = {0, 0};
static char    s_debug_buffer[ROS_LOG_MAX_LEN];

// ==================================================
// INTERNAL: bind static buffer to Int32MultiArray msg
// ==================================================
static void _bind_array_msg(std_msgs__msg__Int32MultiArray *msg, int32_t *buf, size_t len)
{
    msg->data.data     = buf;
    msg->data.size     = len;
    msg->data.capacity = len;
    msg->layout.dim.data     = NULL;
    msg->layout.dim.size     = 0;
    msg->layout.dim.capacity = 0;
    msg->layout.data_offset  = 0;
}

// ==================================================
// INTERNAL: /wheel_rpm subscriber callback
// ==================================================
static void _cb_wheel_rpm(const void *msg_in)
{
    const std_msgs__msg__Int32MultiArray *m = (const std_msgs__msg__Int32MultiArray *)msg_in;
    if (m->data.size < 2) return;

    const int16_t RPM_LIMIT = (int16_t)(BASE_MOVE_RANGE * RC_CH5_K2);
    g_ros_rpm_left  = (int16_t)constrain(m->data.data[0], -RPM_LIMIT, RPM_LIMIT);
    g_ros_rpm_right = (int16_t)constrain(m->data.data[1], -RPM_LIMIT, RPM_LIMIT);
    g_ros_cmd_fresh = true;
}

// ==================================================
// INTERNAL: MACRO An Toàn (Không reset ESP khi rớt gói tin)
// ==================================================
#define RC_INIT_CHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}

// ==================================================
// INTERNAL: init ROS node, pub, sub, executor
// ==================================================
static bool _ros_do_init(void)
{
    s_allocator = rcl_get_default_allocator();

    if (RMW_RET_OK != rmw_uros_ping_agent(ROS_AGENT_PING_MS, ROS_AGENT_PING_ATTEMPTS)) {
        return false;
    }

    RC_INIT_CHECK(rclc_support_init(&s_support, 0, NULL, &s_allocator));
    RC_INIT_CHECK(rclc_node_init_default(&s_node, ROS_NODE_NAME, "", &s_support));

    RC_INIT_CHECK(rclc_publisher_init_default(
        &s_pub_encoder, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "zlac_encoder"));

    RC_INIT_CHECK(rclc_publisher_init_default(
        &s_pub_debug, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "zlac_debug"));

    RC_INIT_CHECK(rclc_subscription_init_default(
        &s_sub_wheel_rpm, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel_rpm"));

    RC_INIT_CHECK(rclc_executor_init(&s_executor, &s_support.context, ROS_EXECUTOR_HANDLES, &s_allocator));
    RC_INIT_CHECK(rclc_executor_add_subscription(
        &s_executor, &s_sub_wheel_rpm, &s_msg_wheel_rpm, &_cb_wheel_rpm, ON_NEW_DATA));

    return true;
}

// ==================================================
// INTERNAL: teardown and reset shared state
// ==================================================
static void _ros_do_teardown(void)
{
    rclc_executor_fini(&s_executor);
    rcl_publisher_fini(&s_pub_encoder, &s_node);
    rcl_publisher_fini(&s_pub_debug, &s_node);
    rcl_subscription_fini(&s_sub_wheel_rpm, &s_node);
    rcl_node_fini(&s_node);
    rclc_support_fini(&s_support);

    g_ros_connected = false;
    g_ros_cmd_fresh = false;
    g_ros_rpm_left  = 0;
    g_ros_rpm_right = 0;
}

// ==================================================
// ROS SPIN TASK — Core 0, priority 2
// ==================================================
static void _ros_task(void *pv)
{
    (void)pv;
    uint32_t pub_tick = 0;
    char rx_log_buf[ROS_LOG_MAX_LEN];
`
    for (;;) {
        if (!g_ros_connected) {
            if (_ros_do_init()) {
                g_ros_connected = true;
            } else {
                //_ros_do_teardown();
                vTaskDelay(pdMS_TO_TICKS(ROS_AGENT_PING_MS));
                continue;
            }
        }

        // 1. Quét sự kiện từ máy tính (Timeout 2ms để Task chạy nhanh)
        rcl_ret_t ret = rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(2));
        
        // CHỈ ngắt kết nối khi có lỗi thực sự, phớt lờ RCL_RET_TIMEOUT
        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
            _ros_do_teardown();
            continue;
        }

        // 2. Gửi phản hồi Encoder (Định kỳ ~250ms = 25 chu kỳ * 10ms)
        if (++pub_tick >= 25) {
            pub_tick = 0;
            s_msg_encoder.data.data[0] = (int32_t)g_fb_rpm_left;
            s_msg_encoder.data.data[1] = (int32_t)g_fb_rpm_right;
            RCSOFTCHECK(rcl_publish(&s_pub_encoder, &s_msg_encoder, NULL));
        }

        // 3. Xử lý Hàng đợi (Queue) Log & Gửi lên Topic
        int logs_processed = 0;
        while (g_ros_log_queue != NULL && 
               xQueueReceive(g_ros_log_queue, rx_log_buf, 0) == pdTRUE && 
               logs_processed < 5) 
        {
            // COPY dữ liệu vào Buffer an toàn tuyệt đối
            strncpy(s_debug_buffer, rx_log_buf, ROS_LOG_MAX_LEN - 1);
            s_debug_buffer[ROS_LOG_MAX_LEN - 1] = '\0'; // Chốt chặn kết thúc chuỗi
            
            s_msg_debug.data.size = strlen(s_debug_buffer);
            RCSOFTCHECK(rcl_publish(&s_pub_debug, &s_msg_debug, NULL));
            logs_processed++;
        }

        // Vòng lặp nhỏ 10ms (nhanh hơn 50ms cũ) để xử lý log liên tục không bị nghẽn
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================================================
// PUBLIC API
// ==================================================

static void ros_handler_init(void)
{
    set_microros_transports();

    g_ros_log_queue = xQueueCreate(ROS_LOG_QUEUE_LEN, ROS_LOG_MAX_LEN);

    _bind_array_msg(&s_msg_encoder,   s_enc_data, 2);
    _bind_array_msg(&s_msg_wheel_rpm, s_cmd_data, 2);

    // Trỏ con trỏ chuỗi của topic Debug vào vùng nhớ đệm
    s_msg_debug.data.data = s_debug_buffer;
    s_msg_debug.data.capacity = sizeof(s_debug_buffer);
    s_msg_debug.data.size = 0;

    xTaskCreatePinnedToCore(_ros_task, "Task_ROS", ROS_TASK_STACK_SIZE, NULL, ROS_TASK_PRIORITY, NULL, ROS_TASK_CORE);
}

void ros_send_debug_log(const char* log_str)
{
    if (g_ros_log_queue != NULL && g_ros_connected) {
        xQueueSend(g_ros_log_queue, log_str, 0); 
    }
}

static bool ros_use_rc_mode(unsigned long pw6)
{
    if (pw6 > ch6_threshold) return true;
    return false;
}

#endif // MICRO_ROS_HANDLER_H