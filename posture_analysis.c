/*
 * posture_analysis.c
 *
 *  Created on: Jul 17, 2025
 *      Author: jerem
 */

#include "posture_analysis.h"
#include <math.h>
#include <stdio.h>

// === Constants and Parameters ===

#define CVA_OFFSET              89.0f   // CVA = θ + 89°
#define BAD_POSTURE_DROP        5.0f    // 5° drop from baseline means FHP
#define HISTORY_SIZE            8      // For 10 seconds at 0.5s interval
#define THRESHOLD2_OFFSET       1.0f    // Offset above moving average to detect improvement

// === Internal State ===

static float baseline_cva = 0.0f;              // User's good posture reference angle
static float baseline_threshold = 0.0f;        // baseline_cva - BAD_POSTURE_DROP

static float cva_history[HISTORY_SIZE];        // Circular buffer to store recent CVAs
static uint8_t cva_index = 0;                  // Index for circular buffer

static float current_cva = 0.0f;               // Latest calculated CVA
static uint8_t bad_posture_counter = 0;        // Number of consecutive bad readings

static bool is_fhp = false;                    // Whether user is currently in FHP
static PostureCase current_case = POSTURE_GOOD;// For correction feedback state

// === Math Utility ===

/**
 * @brief Calculate forward tilt angle θ from accelerometer data.
 * Formula: θ = atan(x / sqrt(y² + z²))
 */
float calculate_tilt_angle(AccelData accel) {
    return atan2f(accel.ax, sqrtf(accel.ay * accel.ay + accel.az * accel.az)) * (180.0f / M_PI);
}

/**
 * @brief Convert tilt angle to CVA using: CVA = θ + 89°
 */
float get_cva_from_tilt(float tilt_angle) {
    return tilt_angle + CVA_OFFSET;
}

// === Initialization ===

/**
 * @brief Initializes the posture detection with user’s baseline CVA (good posture).
 */
void posture_init(float initial_cva) {
    baseline_cva = initial_cva;
    baseline_threshold = baseline_cva - BAD_POSTURE_DROP;

    for (int i = 0; i < HISTORY_SIZE; i++) {
        cva_history[i] = baseline_cva;
    }

    cva_index = 0;
    current_cva = baseline_cva;
    bad_posture_counter = 0;
    is_fhp = false;
    current_case = POSTURE_GOOD;
}

// === Update Posture State ===

/**
 * @brief Updates posture classification using new sensor input.
 * Runs Algorithm#1 and Algorithm#2 as described in the research paper.
 */
void update_posture_state(AccelData accel_data) {
    // Calculate CVA from raw acceleration
    float tilt = calculate_tilt_angle(accel_data);
    current_cva = get_cva_from_tilt(tilt);

    // Store CVA in circular buffer
    cva_history[cva_index] = current_cva;
    cva_index = (cva_index + 1) % HISTORY_SIZE;

    // === Algorithm #1: Detect FHP after 10s of sustained bad posture ===
    if (!is_fhp) {
        if (current_cva < baseline_threshold) {
            bad_posture_counter++;
        } else {
            bad_posture_counter = 0;
        }

        if (bad_posture_counter >= HISTORY_SIZE) {
            is_fhp = true;
            current_case = POSTURE_CASE_1; // Initially bad
        } else {
            current_case = POSTURE_GOOD;
        }
    }

    // === Algorithm #2: Determine if posture is correcting ===
    else {
        float sum = 0.0f;
        for (int i = 0; i < HISTORY_SIZE; i++) {
            sum += cva_history[i];
        }
        float avg_prev = sum / HISTORY_SIZE;
        float threshold2 = avg_prev + THRESHOLD2_OFFSET;

        if (current_cva > baseline_cva) {
            // Posture fully corrected
            is_fhp = false;
            bad_posture_counter = 0;
            current_case = POSTURE_GOOD;
        } else if (current_cva > threshold2) {
            // Improving posture
            current_case = POSTURE_CASE_2;
        } else {
            // Still bad
            current_case = POSTURE_CASE_1;
        }
    }
}

// === Accessor Functions ===

/**
 * @brief Returns the latest CVA value.
 */
float get_current_cva(void) {
    return current_cva;
}

/**
 * @brief Returns whether the current posture is in the FHP (bad posture) state.
 */
bool is_bad_posture(void) {
    return is_fhp;
}

/**
 * @brief Returns the correction case: GOOD, CASE_1, or CASE_2.
 */
PostureCase get_posture_case(void) {
    return current_case;
}





