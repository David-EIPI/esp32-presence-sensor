#pragma once

#include <stdint.h>

/* Settable configuration parameters */
extern int32_t ld24xx_MaxDistance;
extern int32_t ld24xx_Timeout;

/* Reported sensor data */
extern uint8_t ld24xx_MotionDetected;
//extern uint8_t ld24xx_StillDetected;
extern int32_t ld24xx_Distance;
extern uint8_t ld24xx_Interference;

/* Calibration status (set ld24xx_calibrationTimer > 0 to initiate calibration */
extern int32_t ld24xx_calibrationTimer;
//extern uint8_t ld24xx_calibrationStatus;

/* Diagnostic output */
extern int32_t ld24xx_gate0;
extern int32_t ld24xx_gate3;
extern uint8_t ld24xx_eng_mode;

/* Calibration parameters */
extern int32_t ld24xx_trigger_threshold;
extern int32_t ld24xx_keep_threshold;
extern int32_t ld24xx_micro_threshold;

extern volatile int32_t resetRequest; /* Factory reset requested via Zigbee */

void ld24xx_task(void *arg);

