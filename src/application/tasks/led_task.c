#include "led_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f4xx_hal.h"

// FreeRTOS task handle and queue
TaskHandle_t led_task_handle = NULL;
QueueHandle_t led_task_queue = NULL;

/**
 * @brief Initialize the GPIO for the onboard LED.
 */
void led_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Set the LED pin high initially
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    
    // Configure the LED pin as push-pull output with no pull-up/down
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Initialize the LED task and its associated peripherals.
 */
void led_task_init(void)
{
    // Initialize LED GPIO
    led_gpio_init();
    
    // Create a queue for LED events (if needed)
    led_task_queue = xQueueCreate(5, sizeof(uint8_t));
    if (led_task_queue == NULL) {
        // Handle queue creation failure (error handling can be added here)
        return;
    }
    
    // Create the LED task
    BaseType_t result = xTaskCreate(led_task_run,
                                    "led_task_Task",
                                    LED_TASK_STACK_SIZE,
                                    NULL,
                                    LED_TASK_PRIORITY,
                                    &led_task_handle);
    if (result != pdPASS) {
        // Handle task creation failure (error handling can be added here)
    }
}

/**
 * @brief LED task for debugging: toggles LED at a fixed interval.
 *
 * This task toggles the LED every 500 ms and also checks for any
 * events on its queue with a short timeout so that the LED blinking
 * remains regular regardless of queued events.
 */
void led_task_run(void *params)
{
    uint8_t event;
    const TickType_t delayTicks = pdMS_TO_TICKS(500);      // 500 ms delay between toggles
    const TickType_t queueTimeout = pdMS_TO_TICKS(10);       // 10 ms timeout for queue checking

    while (1) {
        // Toggle LED state
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        
        // Non-blocking check for any queued event (if any debugging event is needed)
        if (xQueueReceive(led_task_queue, &event, queueTimeout) == pdPASS) {
            // Process LED-related event here (if needed)
            // Currently, we ignore the event and continue blinking.
        }
        
        // Delay to maintain a regular blink interval
        vTaskDelay(delayTicks);
    }
}
