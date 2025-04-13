#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#define PWM_time_period               20    // PWM period in milliseconds
#define PWM_period_reload_value       ((PWM_time_period * 1000) - 1)

#define SERVO_TASK_PRIORITY           (10)
#define SERVO_TASK_STACK_SIZE         (128) // in words

#define BASE_SERVO_MAX_ANGLE          180.0f
#define BASE_SERVO_MIN_ANGLE          0.0f
#define TILT_SERVO_MAX_ANGLE          150.0f
#define TILT_SERVO_MIN_ANGLE          30.0f

void servo_task_init(void);
void TIM1_UP_TIM10_IRQHandler(void);
void servo_task_run(void *params);

#endif /* SERVO_TASK_H */
