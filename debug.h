// ==================================================
// Project          : ESP32 Dual Motor Controller
// File Name        : debug.h
// Description      : Thread-safe Logging to micro-ROS topic /zlac_debug
// ==================================================

#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "config.h"

// Khai báo hàm đẩy log từ file micro_ros_handler.h
extern void ros_send_debug_log(const char* log_str);

/**
 * @brief Formats operational telemetry and pushes it to the FreeRTOS ROS-Logging Queue.
 */
inline void print_motor_log(char core_label, uint32_t counter, float profile, 
                            unsigned long pw2, unsigned long pw4, unsigned long pw5, unsigned long pw6,
                            int16_t target_rpm, int16_t fb1, int16_t fb2, 
                            int64_t latency, bool is_rc_mode)
{
#if ENABLE_DEBUG
  char log_buf[180];
  snprintf(log_buf, sizeof(log_buf), 
           "[%c #%04u] MODE:%s | PROF:%.2f | RC[CH2:%4lu CH4:%4lu CH5:%4lu CH6:%4lu] | RPM:%4d | FB:%4d | Lat:%lld us",
           core_label, counter, is_rc_mode ? "RC_MANUAL" : "ROS_AUTO", profile, pw2, pw4, pw5, pw6, target_rpm, fb1, latency);
           
  // Gửi vào hàng đợi thay vì in ra Serial USB gây nhiễu micro-ROS
  ros_send_debug_log(log_buf);
#endif
}

#endif // DEBUG_H