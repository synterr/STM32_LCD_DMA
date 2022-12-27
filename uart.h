#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

#define UART_BUFF_SIZE 100

void USART2_IRQHandler(void);
volatile static char uart_buffer[UART_BUFF_SIZE];
volatile static uint8_t uart_buffer_len = 0;
char* uart_get_buffer(void);
uint8_t uart_get_buffer_len(void);
void uart_clear_buffer(void);
void uart_send_string(char* buff, uint8_t len, bool end_string);
  
volatile static uint8_t uval;

void UART2_init (void);
uint8_t UART2_GetChar (void);
void UART2_SendChar (uint8_t c);

#endif

