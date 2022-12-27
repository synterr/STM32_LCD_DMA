#include <stdbool.h>
#include "stm32f4xx.h"                  // Device header
#include "gpio.h"
#include "uart.h"


void UART2_init (void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART
  
  GPIOA->MODER 	 |= (2<<4);   // Bits (5:4)= 1:0 --> Alternate Function for Pin PA2
  GPIOA->MODER 	 |= (2<<6);   // Bits (7:6)= 1:0 --> Alternate Function for Pin PA3
  
  GPIOA->OSPEEDR |= (3<<4)|(3<<6);  // HIGH Speed for PA2, PA3
  
  GPIOA->AFR[0]  |= (7<<8);   // Bites (11:10:9:8) = 0:1:1:1  --> AF7 Alternate function for USART2 at Pin PA2
  GPIOA->AFR[0]  |= (7<<12);   // Bites (15:14:13:12) = 0:1:1:1  --> AF7 Alternate function for USART2 at Pin PA3
  
  USART2->CR1 = 0x00;   // Clear ALL
  USART2->CR1 &= ~(1<<12);   // M = 0 - 8 bit word

  uint16_t uartdiv = (SystemCoreClock/2) / 115200; //System core clock is at 84MHZ but APB1Clock is at 42MHz so dividing by 2
    USART2->BRR = ( ( ( uartdiv / 16 ) << USART_BRR_DIV_Mantissa_Pos ) |
                    ( ( uartdiv % 16 ) << USART_BRR_DIV_Fraction_Pos ) );
  
  USART2->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE);
}

uint8_t UART2_GetChar (void)
{
		/*********** STEPS FOLLOWED *************
	
	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit
	
	****************************************/
	
	
	while (!(USART2->SR & (1<<5)));  // Wait for RXNE to SET.. This indicates that the data has been Received
	uval = (uint8_t)USART2->DR;
  return uval;
}

void UART2_SendChar (uint8_t c)
{
	/*********** STEPS FOLLOWED *************
	
	1. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
		 for each data to be transmitted in case of single buffer.
	2. After writing the last data into the USART_DR register, wait until TC=1. This indicates
		 that the transmission of the last frame is complete. This is required for instance when
		 the USART is disabled or enters the Halt mode to avoid corrupting the last transmission.
	
	****************************************/

	USART2->DR = c;   // LOad the Data
	while (!(USART2->SR & (1<<6)));  // Wait for TC to SET.. This indicates that the data has been transmitted
}

char* uart_get_buffer(void){
  return uart_buffer;
}

uint8_t uart_get_buffer_len(void){
  return uart_buffer_len;
}

void uart_clear_buffer(void){
  for(uint8_t i = 0; i < UART_BUFF_SIZE; i++)
    uart_buffer[i] = '\0';
  uart_buffer_len = 0;
}

void uart_send_string(char* buff, uint8_t len, bool end_string){
  for(uint8_t i = 0; i < len; i++){
    UART2_SendChar(buff[i]);
    if (buff[i] == '\0')
      return;
  }
  if (end_string)
    UART2_SendChar('\0');
}

void USART2_IRQHandler(void) {

 //check if we are here because of RXNE interrupt
 if (USART2->SR & USART_SR_RXNE) //if RX is not empty
 {
  char temp = USART2->DR; //fetch the data received
  //USART2->DR = temp;  //send it back out
  while (!(USART2->SR & USART_SR_TC));
  if(uart_buffer_len < UART_BUFF_SIZE)
    uart_buffer[uart_buffer_len++] = temp;
 }

 //check if we are here because of TXEIE interrupt
 if (USART2->SR & USART_SR_TXE) //if RX is not empty
 {
  //handle transmit completion here

 }

}