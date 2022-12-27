#include "stm32f4xx.h"                  // Device header
#include "gpio.h"
#include "spi.h"

void spi_init (void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  
  //                  SCK    MISO    MOSI
  GPIOA->MODER |=   (2<<10)|(2<<12)|(2<<14);  // Alternate functions for PA5, PA6, PA7
	GPIOA->OSPEEDR |= (3<<10)|(3<<12)|(3<<14);  // HIGH Speed for PA5, PA6, PA7, 
	GPIOA->AFR[0] |=  (5<<20)|(5<<24)|(5<<28);  // AF5(SPI1) for PA5, PA6, PA7
  
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 CLock

  //CPHA CPOL
  SPI1-> CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
	
  SPI1->CR1 |= SPI_CR1_MSTR;  // Master Mode
	
  SPI1->CR1 |= (5<<3);  // BR[2:0] = 010: fPCLK/8, PCLK2 = 80MHz, SPI clk = 6MHz
	
  //MSB first
  SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
	
  SPI1->CR1 |= (SPI_CR1_SSM);  // SSM=1,
  SPI1->CR1 |= (SPI_CR1_SSI);  //   SSi=1 -> Software Slave Management
	
  //Full duplex mode
  SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);
  SPI1->CR1 &= ~(SPI_CR1_RXONLY);
	
  SPI1->CR1 &= ~(SPI_CR1_DFF);  // DFF=0, 8 bit data
	
  SPI1->CR2 = 0;
  SPI1->CR1 |= (SPI_CR1_SPE); //SPE: SPI enable
}

void spi_transmit (uint8_t *data, int size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/		
	
	int i=0;
	while (i<size)
	{
	   while (!((SPI1->SR)&(1<<1))) {}  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI1->DR = data[i];  // load the data into the Data Register
	   i++;
	}	
	
		while (!((SPI1->SR)&(1<<1))){}  // wait for TXE bit to set -> This will indicate that the buffer is empty
  	while (((SPI1->SR)&(1<<7))) {}  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication	

//  Clear the Overrun flag by reading DR and SR
//	uint8_t temp = SPI1->DR;
//					temp = SPI1->SR;

	
}