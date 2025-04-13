#include "servo_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "servo.h"
#include "stm32f4xx_hal.h"
#include "timers.h"

TIM_HandleTypeDef htim1;
TaskHandle_t servo_task_handle = NULL;
QueueHandle_t servo_task_queue = NULL;
TimerHandle_t servo_Timer;

// Two servo instances for base and tilt
SERVO_TYPEDEF servo1;
SERVO_TYPEDEF servo2;

// External variables from UART and Joystick tasks
extern uint8_t pwm_from_uart_base;
extern uint8_t pwm_from_uart_tilt;
extern float joystick_base_angle;
extern float joystick_tilt_angle;

/**
 * @brief Timer callback is not used here, events are directly received from the queue.
 */

/**
 * @brief PWM ISR: Advances S-curve indices and updates PWM compare registers.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Update servo1 S-curve
    if (servo1.sample_index < servo1.num_samples - 1) {
        servo1.sample_index++;
        servo1.current_pwm_value = servo1.active_buffer[servo1.sample_index];
    } else {
        servo1.current_angle = servo1.target_angle;
    }
    // Update servo2 S-curve
    if (servo2.sample_index < servo2.num_samples - 1) {
        servo2.sample_index++;
        servo2.current_pwm_value = servo2.active_buffer[servo2.sample_index];
    } else {
        servo2.current_angle = servo2.target_angle;
    }
    // Apply PWM compare values
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo1.active_buffer[servo1.sample_index]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, servo2.active_buffer[servo2.sample_index]);
}

/**
 * @brief Initialize TIM1 for PWM generation.
 */
void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 15;  // 16MHz system clock divided by 16 -> 1MHz timer clock
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_period_reload_value;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;  // Initial pulse width (50% duty cycle)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_Base_Start_IT(&htim1);
}

/**
 * @brief Initialize the servo task and its peripherals.
 */
void servo_task_init(void)
{
    MX_TIM1_Init();
    // Initialize servos with their maximum and minimum angles.
    Servo_Init(&servo1, BASE_SERVO_MAX_ANGLE, BASE_SERVO_MIN_ANGLE);
    Servo_Init(&servo2, TILT_SERVO_MAX_ANGLE, TILT_SERVO_MIN_ANGLE);
    servo_task_queue = xQueueCreate(5, sizeof(uint8_t));
    if (servo_task_queue == NULL) { return; }
    BaseType_t result = xTaskCreate(servo_task_run, "servo_task_Task",
                                    SERVO_TASK_STACK_SIZE, NULL,
                                    SERVO_TASK_PRIORITY, &servo_task_handle);
    if (result != pdPASS) { /* Handle task creation failure */ }
}

/**
 * @brief Servo task: receives events and generates a new S-curve based on input.
 */
void servo_task_run(void *params)
{
    uint8_t event;
    while (1) {
        if (xQueueReceive(servo_task_queue, &event, portMAX_DELAY) == pdPASS) {
            switch (event) {
                case 1:
                    generate_adaptive_s_curve(&servo1, pwm_from_uart_base);
                    break;
                case 2:
                    generate_adaptive_s_curve(&servo2, pwm_from_uart_tilt);
                    break;
                case 3:
                    // Preempt with joystick-based movement for base servo.
                    servo1.current_angle = ms_to_degree(servo1.current_pwm_value);
                    generate_adaptive_s_curve(&servo1, servo1.current_angle + joystick_base_angle);
                    break;
                case 4:
                    // Preempt with joystick-based movement for tilt servo.
                    servo2.current_angle = ms_to_degree(servo2.current_pwm_value);
                    generate_adaptive_s_curve(&servo2, servo2.current_angle + joystick_tilt_angle);
                    break;
                default:
                    break;
            }
        }
    }
}
