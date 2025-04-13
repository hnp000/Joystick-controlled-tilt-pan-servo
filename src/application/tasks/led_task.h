#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>

// Task configuration
#define LED_TASK_PRIORITY    (7)
#define LED_TASK_STACK_SIZE  (128)  // In words

// LED GPIO definitions
#define LD2_Pin            GPIO_PIN_5
#define LD2_GPIO_Port      GPIOA

// Function declarations
void led_task_init(void);
void led_task_run(void *params);

#endif /* LED_TASK_H */
