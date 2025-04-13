
#ifndef UART_H_
#define UART_H_

#include <stdint.h>


#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA





void MX_USART2_UART_Init(void);



#endif 