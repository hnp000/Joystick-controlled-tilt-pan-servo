#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#define SERVO_MIN_MS       0.5f    // Minimum pulse width in ms
#define SERVO_MAX_MS       2.5f    // Maximum pulse width in ms
#define SERVO_MAX_ANGLE    180.0f  // Full scale angle in degrees
#define BASE_SERVO_OFFSET  0.0f    // Offset for base servo calibration

#define Number_of_samples  40      // Not used, using MAX_S_CURVE_POINTS instead
#define TRANSITION_TIME    5000    // Transition time in ms

#define MAX_S_CURVE_POINTS 50

typedef struct {
    float current_pwm_value;           // Last PWM value (in ticks)
    float current_angle;               // Current servo angle (degrees)
    float target_angle;                // Target servo angle (degrees)
    volatile int sample_index;         // Current index into the S-curve buffer
    volatile int num_samples;          // Total number of S-curve points
    float servo_max_angle;             // Maximum allowed servo angle
    float servo_min_angle;             // Minimum allowed servo angle
    uint16_t pwm_values_A[MAX_S_CURVE_POINTS]; // First S-curve buffer
    uint16_t pwm_values_B[MAX_S_CURVE_POINTS]; // Second S-curve buffer
    uint16_t *active_buffer;           // Buffer used by PWM ISR
    uint16_t *inactive_buffer;         // Buffer for generating new S-curve
    uint32_t transition_time;          // Transition time in ms (if used)
} SERVO_TYPEDEF;

void Servo_Init(SERVO_TYPEDEF *servo, float max_angle, float min_angle);
float ms_to_degree(float pulse_width_ms);
void generate_adaptive_s_curve(SERVO_TYPEDEF *servo, float new_target_angle);
uint16_t degree_to_ms(float angle);

#endif /* SERVO_H_ */
