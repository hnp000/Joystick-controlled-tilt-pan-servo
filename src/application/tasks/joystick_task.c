#include "joystick_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include "timers.h"

// ADC and DMA handles for ADC1 and ADC2 (X and Y axes)
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

// Timer for triggering ADC conversions
TimerHandle_t joystickTimer;

// Global flag to merge ADC conversion events (bit 0 for ADC1, bit 1 for ADC2)
volatile uint8_t adc_conversion_flags = 0;

// Separate DMA buffers for each ADC
uint32_t adc_buf_x[ADC_SAMPLES];
uint32_t adc_buf_y[ADC_SAMPLES];

// Task handle and queue for the joystick task
TaskHandle_t joystick_task_handle = NULL;
QueueHandle_t joystick_task_queue = NULL;

// External servo task queue for inter-task communication
extern QueueHandle_t servo_task_queue;

// Joystick data instance
Joystick_Data_Typedef Joystick_Data;

// Global variables for computed angles
float joystick_base_angle = 0.0f;
float joystick_tilt_angle = 0.0f;

/**
 * @brief Timer callback that starts ADC conversions for both ADC1 (X) and ADC2 (Y).
 */
void joystick_timer_callback(TimerHandle_t xTimer)
{
    HAL_ADC_Start_DMA(&hadc1, adc_buf_x, ADC_SAMPLES);
    HAL_ADC_Start_DMA(&hadc2, adc_buf_y, ADC_SAMPLES);
}

/**
 * @brief ADC conversion complete callback for merging events.
 * Sets bits in a flag and, when both ADC conversions are complete,
 * sends a single merged event to the joystick task.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hadc->Instance == ADC1) {
        adc_conversion_flags |= 0x01;  // Set bit 0 for ADC1
    }
    if (hadc->Instance == ADC2) {
        adc_conversion_flags |= 0x02;  // Set bit 1 for ADC2
    }
    // If both ADCs have completed conversion, send a merged event
    if ((adc_conversion_flags & 0x03) == 0x03) {
        uint8_t event = 0;  // Merged event code
        xQueueSendFromISR(joystick_task_queue, &event, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        adc_conversion_flags = 0;  // Reset flags for the next cycle
    }
}

/**
 * @brief Initialize ADC1 for the X axis.
 */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;  // Single-channel mode
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    sConfig.Channel = ADC_CHANNEL_0;  // X-axis channel
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief Initialize ADC2 for the Y axis.
 */
void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    __HAL_RCC_ADC2_CLK_ENABLE();

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = DISABLE;  // Single-channel mode
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) { Error_Handler(); }

    sConfig.Channel = ADC_CHANNEL_1;  // Y-axis channel (PA1)
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief Initialize DMA for ADC1.
 */
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

/**
 * @brief Initialize DMA for ADC2.
 */
void MX_DMA_ADC2_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);
}

/**
 * @brief Initialize the joystick task and associated peripherals.
 */
void joystick_task_init(void)
{
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_DMA_Init();
    MX_DMA_ADC2_Init();

    // Create a queue for ADC conversion events
    joystick_task_queue = xQueueCreate(5, sizeof(uint8_t));
    if (joystick_task_queue == NULL) { return; }

    // Create a periodic FreeRTOS timer for ADC triggering (10ms period)
    joystickTimer = xTimerCreate("JoyTimer", pdMS_TO_TICKS(10), pdTRUE, NULL, joystick_timer_callback);

    // Create the joystick task
    BaseType_t result = xTaskCreate(joystick_task_run, "joystick_task_Task",
                                    JOYSTICK_TASK_STACK_SIZE, NULL,
                                    JOYSTICK_TASK_PRIORITY, &joystick_task_handle);
    if (result != pdPASS) { /* Handle error */ }
}

/**
 * @brief Map an ADC reading to a signed range (-100 to 100).
 */
float map_adc_to_signed_range(float adc_value)
{
    float center = ADC_CENTER;
    float max_offset = ADC_CENTER;
    float centered = adc_value - center;
    float mapped = (centered * SCALE_MAX) / max_offset;
    if (mapped > SCALE_MAX) mapped = SCALE_MAX;
    if (mapped < -SCALE_MAX) mapped = -SCALE_MAX;
    return mapped;
}

/**
 * @brief Joystick task that processes ADC data from ADC1 and ADC2,
 * applies filtering and dead-zone logic, and sends events to the servo task.
 */
void joystick_task_run(void *params)
{
    uint8_t event;
    float x_avg = 0, y_avg = 0;
    xTimerStart(joystickTimer, 0);

    while (1) {
        if (xQueueReceive(joystick_task_queue, &event, portMAX_DELAY) == pdPASS) {
            if (event == 0) {
                // Save previous values for relative comparison
                Joystick_Data.previous_x_sample = Joystick_Data.x_sample;
                Joystick_Data.previous_y_sample = Joystick_Data.y_sample;

                x_avg = 0;
                y_avg = 0;
                // Compute averages from separate ADC buffers
                for (int i = 0; i < ADC_ONE_AXIS_SAMPLES; i++) {
                    x_avg += adc_buf_x[i];
                    y_avg += adc_buf_y[i];
                }
                x_avg /= ADC_ONE_AXIS_SAMPLES;
                y_avg /= ADC_ONE_AXIS_SAMPLES;

                // Map averaged ADC values to the range -100 to 100
                Joystick_Data.x_sample = map_adc_to_signed_range(x_avg);
                Joystick_Data.y_sample = map_adc_to_signed_range(y_avg);

                // Apply exponential low-pass filtering to smooth the data
                Joystick_Data.x_filtered = ALPHA * Joystick_Data.x_sample +
                                           (1.0f - ALPHA) * Joystick_Data.x_filtered;
                Joystick_Data.y_filtered = ALPHA * Joystick_Data.y_sample +
                                           (1.0f - ALPHA) * Joystick_Data.y_filtered;

                // Apply dead-zone with hysteresis for X axis
                if (fabsf(Joystick_Data.x_filtered) < DEAD_ZONE_ENTER) {
                    if (fabsf(Joystick_Data.previous_x_sample) < DEAD_ZONE_EXIT)
                        Joystick_Data.x_filtered = 0.0f;
                }
                // Apply dead-zone with hysteresis for Y axis
                if (fabsf(Joystick_Data.y_filtered) < DEAD_ZONE_ENTER) {
                    if (fabsf(Joystick_Data.previous_y_sample) < DEAD_ZONE_EXIT)
                        Joystick_Data.y_filtered = 0.0f;
                }
                // Update the final sample values with the filtered values
                Joystick_Data.x_sample = Joystick_Data.x_filtered;
                Joystick_Data.y_sample = Joystick_Data.y_filtered;

                // Relative comparison: only send an event if there's a significant incremental change
                if (Joystick_Data.x_sample > 0) {
                    if ((Joystick_Data.x_sample > Joystick_Data.previous_x_sample) &&
                        (fabsf(Joystick_Data.x_sample - Joystick_Data.previous_x_sample) > NOISE_THRESHOLD)) {
                        Joystick_Data.previous_x_sample = Joystick_Data.x_sample;
                        event = 3; // base servo event for positive direction
                        joystick_base_angle = Joystick_Data.x_sample;
                        xQueueSend(servo_task_queue, &event, 0);
                    }
                } else if (Joystick_Data.x_sample < 0) {
                    if ((Joystick_Data.x_sample < Joystick_Data.previous_x_sample) &&
                        (fabsf(Joystick_Data.x_sample - Joystick_Data.previous_x_sample) > NOISE_THRESHOLD)) {
                        Joystick_Data.previous_x_sample = Joystick_Data.x_sample;
                        event = 3; // base servo event for negative direction
                        joystick_base_angle = Joystick_Data.x_sample;
                        xQueueSend(servo_task_queue, &event, 0);
                    }
                } else {
                    Joystick_Data.previous_x_sample = 0.0f;
                }
                // Process Y axis similarly
                if (Joystick_Data.y_sample > 0) {
                    if ((Joystick_Data.y_sample > Joystick_Data.previous_y_sample) &&
                        (fabsf(Joystick_Data.y_sample - Joystick_Data.previous_y_sample) > NOISE_THRESHOLD)) {
                        Joystick_Data.previous_y_sample = Joystick_Data.y_sample;
                        event = 4; // tilt servo event for positive direction
                        joystick_tilt_angle = Joystick_Data.y_sample;
                        xQueueSend(servo_task_queue, &event, 0);
                    }
                } else if (Joystick_Data.y_sample < 0) {
                    if ((Joystick_Data.y_sample < Joystick_Data.previous_y_sample) &&
                        (fabsf(Joystick_Data.y_sample - Joystick_Data.previous_y_sample) > NOISE_THRESHOLD)) {
                        Joystick_Data.previous_y_sample = Joystick_Data.y_sample;
                        event = 4; // tilt servo event for negative direction
                        joystick_tilt_angle = Joystick_Data.y_sample;
                        xQueueSend(servo_task_queue, &event, 0);
                    }
                } else {
                    Joystick_Data.previous_y_sample = 0.0f;
                }
            }
        }
    }
}
