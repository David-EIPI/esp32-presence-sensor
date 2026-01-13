#pragma once

#include <stdint.h>

/* Settable configuration parameters */
extern int32_t ld2402_MaxDistance;
extern int32_t ld2402_Timeout;

/* Reported sensor data */
extern uint8_t ld2402_MotionDetected;
//extern uint8_t ld2402_StillDetected;
extern int32_t ld2402_Distance;
extern uint8_t ld2402_Interference;

/* Calibration status (set ld2402_calibrationTimer > 0 to initiate calibration */
extern int32_t ld2402_calibrationTimer;
//extern uint8_t ld2402_calibrationStatus;

/* Diagnostic output */
extern int32_t ld2402_gate0;
extern int32_t ld2402_gate3;
extern uint8_t ld2402_eng_mode;

/* Calibration parameters */
extern int32_t ld2402_trigger_threshold;
extern int32_t ld2402_keep_threshold;
extern int32_t ld2402_micro_threshold;


void ld2402_task(void *arg);

