#include "stm32f4xx.h"                  // Device header
#include "dma.h"
#include "gpio.h"

#define ch3		(0x03<<25)

void dma_init(void) {

  ///* DMA Config *///
  RCC->AHB1ENR|=RCC_AHB1ENR_DMA2EN;
	DMA2_Stream3->CR&=~DMA_SxCR_EN;
	while((DMA2_Stream3->CR)&DMA_SxCR_EN){}

    DMA2_Stream3->CR = ch3|DMA_SxCR_MINC|DMA_SxCR_DIR_0|DMA_SxCR_TCIE
	                    |DMA_SxCR_HTIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE;
    //DMA2_Stream3->CR &= ~(DMA_SxCR_PINC|DMA_SxCR_CIRC);
    
	//DMA2_Stream3->FCR=0;
	DMA2_Stream3->FCR&=~DMA_SxFCR_DMDIS;

}

uint8_t get_transfer(void)
{
  return transfer;
}

void set_transfer(uint8_t tr)
{
  transfer = tr;
}
  
  
void dma_spi_enable(void)
{
  NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    //enable DMA_TX buffer
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);	
  SPI1->CR2 |= SPI_CR2_TXDMAEN;
    
}

void dma_spi_disable(void)
{
    NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    //disable DMA_TX buffer
		SPI1->CR2 &= ~SPI_CR2_TXDMAEN;
    NVIC_DisableIRQ(DMA2_Stream3_IRQn);
}


void dma_start(uint8_t *data, uint32_t size) {
  
  transfer = 1;
  DMA2->LIFCR |=DMA_LIFCR_CTCIF3|DMA_LIFCR_CHTIF3|DMA_LIFCR_CTEIF3|DMA_LIFCR_CDMEIF3|DMA_LIFCR_CFEIF3;
  DMA2_Stream3->PAR= (uint32_t)&SPI1->DR;
  DMA2_Stream3->M0AR=(uint32_t)data;
	DMA2_Stream3->NDTR=size;
	DMA2_Stream3->CR|=DMA_SxCR_EN;
}

