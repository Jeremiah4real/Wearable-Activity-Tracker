/*
 * fall_detection_imu.h
 *
 *  Created on: Jun 26, 2025
 *      Author: jerem
 */

#ifndef SRC_FALL_DETECTION_IMU_H_
#define SRC_FALL_DETECTION_IMU_H_

#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Fall types
typedef enum {
    FALL_NONE,
    FALL_FORWARD,
    FALL_BACKWARD,
    FALL_LEFT,
    FALL_RIGHT
} FallType;

// Input structure (1 sample)
typedef struct {
    float accel_mag;   // √(ax^2 + ay^2 + az^2)
    float roll;        // degrees
    float pitch;
    float yaw;
    uint32_t timestamp_ms;
} MotionSample;

// Internal state
typedef enum {
    STATE_IDLE,
    STATE_MONITOR,
    STATE_SUSPECT,
    STATE_CONFIRM
} FallState;

typedef struct {
    FallState state;
    uint32_t lower_thresh_time;
    bool alert_triggered;
    FallType detected_fall;
} FallDetector;

// IMU input (raw data from MPU6050)
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_Raw;

// Filtered output angles (in degrees)
typedef struct {
    float roll;
    float pitch;
    float yaw;  // No accelerometer correction for yaw
} Attitude;

// Initialize the detector
void init_fall_detector(FallDetector* fd);

// Update function to be called per sample (ideally every ~10–50ms)
void update_fall_detector(FallDetector* fd, MotionSample sample);

// Reset detection system
void reset_fall_detector(FallDetector* fd);

// Call this function at constant rate (dt in seconds)
void compute_attitude(IMU_Raw raw, Attitude* angle, float dt);

// returns time in ms
uint32_t get_time_ms(void);

// computes the magnitude of the acceleration in m/s^2
float compute_accel_mag(int16_t ax_raw, int16_t ay_raw, int16_t az_raw);

#ifdef __cplusplus
}
#endif

#endif /* SRC_FALL_DETECTION_IMU_H_ */
