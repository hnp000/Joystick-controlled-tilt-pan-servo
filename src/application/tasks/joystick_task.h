#ifndef JOYSTICK_TASK_H
#define JOYSTICK_TASK_H

#include <stdint.h>

// Task configuration
#define JOYSTICK_TASK_PRIORITY    (9)
#define JOYSTICK_TASK_STACK_SIZE  (128)  // In words

// ADC sample definitions
#define ADC_ONE_AXIS_SAMPLES      20
#define ADC_SAMPLES               20

// ADC resolution and mapping constants
#define ADC_RESOLUTION            4095
#define ADC_CENTER                2048.0f
#define SCALE_MAX                 100.0f

// Dead-zone and filtering parameters
#define DEAD_ZONE                 20.0f
#define ALPHA                     0.98f        // Exponential filter coefficient
#define DEAD_ZONE_ENTER           10.0f        // Enter threshold for dead zone
#define DEAD_ZONE_EXIT            15.0f        // Exit threshold for dead zone
#define NOISE_THRESHOLD           5.0f         // Minimum change to consider

// Joystick data structure
typedef struct 
{
    float x_sample;          // Raw X-axis sample (mapped)
    float y_sample;          // Raw Y-axis sample (mapped)
    float x_filtered;        // Filtered X-axis value
    float y_filtered;        // Filtered Y-axis value
    float previous_x_sample; // Previous X-axis value (for relative comparisons)
    float previous_y_sample; // Previous Y-axis value (for relative comparisons)
    float current_angle;     // Current computed angle (if needed)
} Joystick_Data_Typedef;

// Function prototypes
void joystick_task_init(void);
void joystick_task_run(void *params);

#endif /* JOYSTICK_TASK_H */
