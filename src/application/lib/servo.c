#include "servo.h"
#include "pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/**
 * @brief Initialize the servo structure and generate an initial S-curve.
 */
void Servo_Init(SERVO_TYPEDEF *servo, float max_angle, float min_angle)
{
    servo->active_buffer = servo->pwm_values_A;
    servo->inactive_buffer = servo->pwm_values_B;
    servo->servo_max_angle = max_angle;
    servo->servo_min_angle = min_angle;
    // Generate an initial S-curve with no movement.
    generate_adaptive_s_curve(servo, 0);
}

/**
 * @brief Compute the number of S-curve points based on error magnitude.
 */
static int computeNumSCurvePoints(float error)
{
    const int basePoints = 10;
    const float scalingFactor = 0.1f;
    int extraPoints = (int)(fabsf(error) * scalingFactor);
    int totalPoints = basePoints + extraPoints;
    if (totalPoints > MAX_S_CURVE_POINTS) {
        totalPoints = MAX_S_CURVE_POINTS;
    }
    return totalPoints;
}

/**
 * @brief Convert a servo angle (in degrees) to a PWM compare value (in timer ticks).
 */
uint16_t degree_to_ms(float angle)
{
    angle += BASE_SERVO_OFFSET;
    float pulse_width_ms = SERVO_MIN_MS + ((angle * (SERVO_MAX_MS - SERVO_MIN_MS)) / SERVO_MAX_ANGLE);
    return (uint16_t)(pulse_width_ms * 1000);  // 1 tick = 1 µs at 1MHz
}

/**
 * @brief Convert a PWM pulse width (in timer ticks) to a servo angle (degrees).
 */
float ms_to_degree(float pulse_width_ms)
{
    pulse_width_ms /= 1000.0f;  // Convert µs to ms
    if (pulse_width_ms < SERVO_MIN_MS) pulse_width_ms = SERVO_MIN_MS;
    if (pulse_width_ms > SERVO_MAX_MS) pulse_width_ms = SERVO_MAX_MS;
    float angle = ((pulse_width_ms - SERVO_MIN_MS) * SERVO_MAX_ANGLE) / (SERVO_MAX_MS - SERVO_MIN_MS);
    return angle;
}

/**
 * @brief Generate an adaptive S-curve profile for servo movement.
 *
 * This function computes intermediate PWM values (stored in the inactive buffer)
 * to smoothly transition from the current angle (derived from current_pwm_value)
 * to the new_target_angle. It then swaps the active and inactive buffers
 * in a critical section.
 */
void generate_adaptive_s_curve(SERVO_TYPEDEF *servo, float new_target_angle)
{
    // Clamp new target to allowed range.
    if (new_target_angle >= servo->servo_max_angle) new_target_angle = servo->servo_max_angle;
    if (new_target_angle <= servo->servo_min_angle) new_target_angle = servo->servo_min_angle;
    
    // Update current angle based on last PWM value.
    servo->current_angle = ms_to_degree(servo->current_pwm_value);
    float error = new_target_angle - servo->current_angle;
    int totalPoints = computeNumSCurvePoints(error);
    
    // Update servo structure
    servo->target_angle = new_target_angle;
    servo->num_samples = totalPoints;
    servo->sample_index = 0;
    
    // Generate S-curve points into the inactive buffer.
    for (int i = 0; i < totalPoints; i++) {
        float t = (float)i / (totalPoints - 1); // Normalize t in [0,1]
        float s_factor = 3 * t * t - 2 * t * t * t; // Cubic S-curve function
        float angle = servo->current_angle + s_factor * error;
        servo->inactive_buffer[i] = degree_to_ms(angle);
    }
    
    // Swap active/inactive buffers in a critical section.
    taskENTER_CRITICAL();
    {
        uint16_t *temp = servo->active_buffer;
        servo->active_buffer = servo->inactive_buffer;
        servo->inactive_buffer = temp;
        servo->sample_index = 0;
    }
    taskEXIT_CRITICAL();
}
