// ==================================================
// Project          : ESP32 Dual Motor Controller
// File Name        : debug.h
// Created          : Jul 11, 2026
// Author           : Pham Duc Duy
// Description      : Unified logging layout using static buffer allocation
// ==================================================

#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Outputs operational telemetry of a specific motor channel formatted onto a single scannable line.
 */
inline void print_motor_log(char core_label, uint32_t counter, float profile, 
                            unsigned long pw2, unsigned long pw4, unsigned long pw5, 
                            int16_t target_rpm, int16_t fb1, int16_t fb2, 
                            int64_t latency)
{
#if ENABLE_DEBUG
  char log_buf[160];
  snprintf(log_buf, sizeof(log_buf), 
           "[%c #%04u] PROFILE:%.2f | RAW_RC[CH2:%4lu CH4:%4lu CH5:%4lu] | Target_RPM:%4d | FB_RPM[%c1:%4d %c2:%4d] | Latency:%lld us",
           core_label, counter, profile, pw2, pw4, pw5, target_rpm, core_label, fb1, core_label, fb2, latency);
  Serial.println(log_buf);
#endif
}

#endif // DEBUG_H
