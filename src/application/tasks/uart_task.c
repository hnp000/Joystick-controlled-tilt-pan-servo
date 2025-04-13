#include "uart_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "uart.h"
#include "stm32f4xx_hal.h"
#include "circular_queue.h"

TaskHandle_t uart_task_handle = NULL;
QueueHandle_t uart_task_queue = NULL;

UART_Parser_typedef uart_parser;
CircularQueue CQ;
uint8_t cq_rx_buffer[64];
uint8_t uart_rx;
uint8_t pwm_from_uart_base;
uint8_t pwm_from_uart_tilt;

extern UART_HandleTypeDef huart2;
extern QueueHandle_t servo_task_queue;

/**
 * @brief UART receive complete callback.
 * Enqueues received data into a circular queue and notifies the UART task.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t event = 0;
        HAL_UART_Receive_IT(&huart2, &uart_rx, 1);  // Re-arm UART receive
        CQUEUE_Enqueue(&CQ, uart_rx);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(uart_task_queue, &event, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Initialize UART task and related peripherals.
 */
void uart_task_init(void)
{
    MX_USART2_UART_Init();
    HAL_UART_Receive_IT(&huart2, &uart_rx, 1);
    circular_queue_create(&CQ, cq_rx_buffer, 64);
    uart_task_queue = xQueueCreate(5, sizeof(uint8_t));
    if (uart_task_queue == NULL) { return; }
    BaseType_t result = xTaskCreate(uart_task_run, "uart_task_Task",
                                    UART_TASK_STACK_SIZE, NULL,
                                    UART_TASK_PRIORITY, &uart_task_handle);
    if (result != pdPASS) { /* Handle task creation error */ }
}

/**
 * @brief UART task: Processes received data using a state machine.
 */
void uart_task_run(void *params)
{
    uint8_t event;
    uint8_t data;
    // Initialize header bytes for packet recognition
    uart_parser.header[0] = 0xAA;
    uart_parser.header[1] = 0x55;
    uart_parser.header[2] = 0xAA;
    while (1) {
        if (xQueueReceive(uart_task_queue, &event, portMAX_DELAY) == pdPASS) {
            switch (event) {
                case 0:
                    CQUEUE_Dequeue(&CQ, &data);
                    switch (uart_parser.state) {
                        case STATE_WAIT_FOR_HEADER_BYTE_1:
                            uart_parser.frame_completed = 0;
                            if (uart_parser.header[0] == data)
                                uart_parser.state = STATE_WAIT_FOR_HEADER_BYTE_2;
                            break;
                        case STATE_WAIT_FOR_HEADER_BYTE_2:
                            if (uart_parser.header[1] == data)
                                uart_parser.state = STATE_WAIT_FOR_HEADER_BYTE_3;
                            break;
                        case STATE_WAIT_FOR_HEADER_BYTE_3:
                            if (uart_parser.header[2] == data)
                                uart_parser.state = STATE_WAIT_FOR_COMMAND;
                            break;
                        case STATE_WAIT_FOR_COMMAND:
                            uart_parser.command = data;
                            uart_parser.state = STATE_WAIT_FOR_DATA_LENGTH;
                            break;
                        case STATE_WAIT_FOR_DATA_LENGTH:
                            uart_parser.data_length = data;
                            uart_parser.data_index = 0;
                            uart_parser.state = (uart_parser.data_length > 0) ?
                                                  STATE_WAIT_FOR_DATA : STATE_PACKET_COMPLETE;
                            break;
                        case STATE_WAIT_FOR_DATA:
                            uart_parser.data[uart_parser.data_index++] = data;
                            if (uart_parser.data_index >= uart_parser.data_length)
                                uart_parser.state = STATE_PACKET_COMPLETE;
                            break;
                        case STATE_PACKET_COMPLETE:
                            uart_parser.state = STATE_WAIT_FOR_HEADER_BYTE_1;
                            uart_parser.frame_completed = 1;
                            break;
                        default:
                            uart_parser.state = STATE_WAIT_FOR_HEADER_BYTE_1;
                            break;
                    }
                    // When a complete frame is received, process the command.
                    if (uart_parser.frame_completed) {
                        if (uart_parser.command == 0x01) {
                            pwm_from_uart_base = uart_parser.data[0];
                            event = 1;
                            xQueueSend(servo_task_queue, &event, 0);
                        }
                        if (uart_parser.command == 0x02) {
                            pwm_from_uart_tilt = uart_parser.data[0];
                            event = 2;
                            xQueueSend(servo_task_queue, &event, 0);
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
}
