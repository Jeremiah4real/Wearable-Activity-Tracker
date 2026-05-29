/*
 * fall_detection_imu.c
 *
 *  Created on: Jun 26, 2025
 *      Author: jerem
 */
#include "bsp.h"
#include "i2c.h"
#include "i2c_two.h"
#include <stdint.h>
#include "io.h"
#include "riscv.h"
#include "plic.h"
#include "gpio.h"
#include "clint.h"
#include "uart.h"
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "fall_detection_imu.h"


// Threshold constants
#define ACCEL_LOWER_THRESHOLD 7.0f     // m/s^2
#define ACCEL_UPPER_THRESHOLD 15.0f    // m/s^2
#define TIME_THRESHOLD_MS      3000     // ms
#define ROLL_LYING_THRESHOLD   20.0f   // degrees

// Constants gyro values (attitude)
#define RAD_TO_DEG 57.2958f
#define ACCEL_SCALE 16384.0f   // for ±2g
//#define ACCEL_SCALE_2G 16384.0f
#define GYRO_SCALE 131.0f      // for ±250°/s
#define ALPHA 0.96f            // 96% gyro, 4% accel
#define G_TO_MS2       9.80665f


// Initialize the detector
void init_fall_detector(FallDetector* fd) {
    fd->state = STATE_IDLE;
    fd->lower_thresh_time = 0;
    fd->alert_triggered = false;
    fd->detected_fall = FALL_NONE;
}

// Update function to be called per sample (ideally every ~10–50ms)
void update_fall_detector(FallDetector* fd, MotionSample sample) {
    switch (fd->state) {
        case STATE_IDLE:
            if (sample.accel_mag < ACCEL_LOWER_THRESHOLD) {
                fd->lower_thresh_time = sample.timestamp_ms;
                fd->state = STATE_MONITOR;
            }
            break;

        case STATE_MONITOR:
            if (sample.accel_mag > ACCEL_UPPER_THRESHOLD) {
                uint32_t delta_t = sample.timestamp_ms - fd->lower_thresh_time;
                if (delta_t <= TIME_THRESHOLD_MS) {
                    fd->state = STATE_SUSPECT;
                } else {
                    fd->state = STATE_IDLE; // timeout
                }
            }
            break;

        case STATE_SUSPECT:
            if (sample.roll < ROLL_LYING_THRESHOLD) {
                // Determine type of fall based on pitch & yaw
                if (sample.pitch > 60 && sample.yaw < -60)
                    fd->detected_fall = FALL_FORWARD;
                else if (sample.pitch < -60 && sample.yaw > 60)
                    fd->detected_fall = FALL_BACKWARD;
                else if (sample.pitch > 0 && sample.yaw > 0)
                    fd->detected_fall = FALL_RIGHT;
                else if (sample.pitch < 0 && sample.yaw < 0)
                    fd->detected_fall = FALL_LEFT;
                else
                    fd->detected_fall = FALL_NONE;

                fd->alert_triggered = true;
                fd->state = STATE_CONFIRM;
            } else {
                // Not lying → reset
                fd->state = STATE_IDLE;
            }
            break;

        case STATE_CONFIRM:
            // Stay in this state until manually cleared
            break;
    }
}
// Reset fall detection algorithm
void reset_fall_detector(FallDetector* fd) {
    fd->state = STATE_IDLE;
    fd->alert_triggered = false;
    fd->detected_fall = FALL_NONE;
}

// Call this function at constant rate (dt in seconds)
void compute_attitude(IMU_Raw raw, Attitude* angle, float dt) {
    // 1. Convert raw to physical units
    float ax = raw.acc_x / ACCEL_SCALE;
    float ay = raw.acc_y / ACCEL_SCALE;
    float az = raw.acc_z / ACCEL_SCALE;

    float gx = raw.gyro_x / GYRO_SCALE;
    float gy = raw.gyro_y / GYRO_SCALE;
    float gz = raw.gyro_z / GYRO_SCALE;

    // 2. Compute accelerometer angles (in degrees)
    float acc_roll  = atan2(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // 3. Integrate gyroscope angles
    angle->roll  += gx * dt;
    angle->pitch += gy * dt;
    angle->yaw   += gz * dt;

    // 4. Complementary filter (only roll/pitch)
    angle->roll  = ALPHA * angle->roll  + (1.0f - ALPHA) * acc_roll;
    angle->pitch = ALPHA * angle->pitch + (1.0f - ALPHA) * acc_pitch;
    // yaw remains gyro-only (or use magnetometer if available)
}

//returns time in ms
uint32_t get_time_ms() {
    return (uint32_t)(clint_getTime(SYSTEM_CLINT_CTRL) / (SYSTEM_CLINT_HZ / 1000));
}

//computes the magnitude of the acceleration in m/s^2
float compute_accel_mag(int16_t ax_raw, int16_t ay_raw, int16_t az_raw) {
    // 1. Convert to g
    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;

    // 2. Convert to m/s²
    ax *= G_TO_MS2;
    ay *= G_TO_MS2;
    az *= G_TO_MS2;

    // 3. Compute magnitude
    return sqrtf(ax * ax + ay * ay + az * az);
}

