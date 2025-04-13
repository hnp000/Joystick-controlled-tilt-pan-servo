#ifndef UART_TASK_H
#define UART_TASK_H

#include <stdint.h>
#include <stdbool.h>

// Task configuration
#define UART_TASK_PRIORITY    (8)
#define UART_TASK_STACK_SIZE  (256)  // in words

// UART state machine states
typedef enum {
    STATE_WAIT_FOR_HEADER_BYTE_1,
    STATE_WAIT_FOR_HEADER_BYTE_2,
    STATE_WAIT_FOR_HEADER_BYTE_3,
    STATE_WAIT_FOR_COMMAND,
    STATE_WAIT_FOR_DATA_LENGTH,
    STATE_WAIT_FOR_DATA,
    STATE_PACKET_COMPLETE
} UART_State_typedef;

typedef struct {
    UART_State_typedef state;
    uint8_t header[3];
    uint8_t command;
    uint8_t data_length;
    uint8_t data[128];
    uint8_t data_index;
    uint8_t frame_completed;
} UART_Parser_typedef;

void uart_task_init(void);
void uart_task_run(void *params);

#endif /* UART_TASK_H */
