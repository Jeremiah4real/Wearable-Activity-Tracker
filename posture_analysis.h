/*
 * posture_analysis.h
 *
 *  Created on: Jul 17, 2025
 *      Author: jerem
 */

#ifndef POSTURE_ANALYSIS_H
#define POSTURE_ANALYSIS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Structure to store accelerometer data (g units)
typedef struct {
    float ax;  // Acceleration on X-axis
    float ay;  // Acceleration on Y-axis
    float az;  // Acceleration on Z-axis
} AccelData;

// Enum to classify posture correction states in Algorithm#2
typedef enum {
    POSTURE_GOOD = 0,     // Normal posture
    POSTURE_CASE_1 = 1,   // Bad posture, not improving
    POSTURE_CASE_2 = 2    // Improving posture but not yet corrected
} PostureCase;

// Initializes the posture detection system with user's baseline CVA
void posture_init(float initial_cva);

// Updates the posture state using new accelerometer reading
void update_posture_state(AccelData accel_data);

// Calculates forward tilt angle from accelerometer values
float calculate_tilt_angle(AccelData accel);

// Converts tilt angle to estimated CVA (Craniovertebral Angle)
float get_cva_from_tilt(float tilt_angle);

// Returns the most recent CVA estimate
float get_current_cva(void);

// Returns whether the user is currently in FHP state
bool is_bad_posture(void);

// Returns the current case for correction logic
PostureCase get_posture_case(void);

#ifdef __cplusplus
}
#endif

#endif // POSTURE_ANALYSIS_H
