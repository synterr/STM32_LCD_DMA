#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "stm32f4xx.h"                  // Device header
#include "gpio.h"
#include "dma.h"
#include "spi.h"
#include "tools.h"
#include "lcd.h"
#include "Graphics.h"

void EXTI15_10_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);

static volatile uint32_t msTicks;                                 // counts 1ms timeTicks
static long cnt = 0;

void SysTick_Handler(void);
void Delay (uint32_t dlyTicks);
void SystemCoreClockConfigure(void);
/*----------------------------------------------------------------------------
 * SysTick_Handler:
 *----------------------------------------------------------------------------*/
//void SysTick_Handler(void) {
//  msTicks++;
//}

/*----------------------------------------------------------------------------
 * Delay: delays a number of Systicks
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {
	
	// Setting max 84 MHz system clock !!!
  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  FLASH->ACR  = FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_ICEN;                            // Instruction cache enable
  FLASH->ACR |= FLASH_ACR_DCEN;                            // Data cache enable
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                     // Flash 5 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK (AHB prescaler)
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;                        // APB1 = HCLK/2 (PCLK1 is 42MHz max)
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK/1

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  // PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
  RCC->PLLCFGR = ( 8ul                   |                 // PLL_M =  8
                 (84ul <<  6)            |                 // PLL_N = 84
                 (  2ul << 16)            |                // PLL_P =   2
                 (RCC_PLLCFGR_PLLSRC_HSI) |                // PLL_SRC = HSI
                 (  8ul << 24)             );              // PLL_Q =   8

  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}

/*----------------------------------------------------------------------------
 * main: blink LED and check button state
 *----------------------------------------------------------------------------*/
int main (void) {
  
  
  SystemCoreClockConfigure();                              // configure HSI as System Clock
  SystemCoreClockUpdate();
  
  //SysTick_Config(SystemCoreClock / 1000);                  // SysTick 1 msec interrupts
  
  gpio_init_input(GPIO_PIN_BTN);
  dma_init();
  spi_init();
  dma_spi_enable();
  ST7789_SpiInit();
  graphics_init();
  
  //ST7789_Clear();

  ST7789_Display(true);

//  ST7789_DrawPixel(100,10,0);

  //cube size
  float cs = 1.0f; 
  float cn = -1.0f; // negative or zero
 
 
  vec3 points[8]= {
    {cn, cn, cn},
    {cs, cn, cn},
    {cs, cs, cn},
    {cn, cs, cn},
    
    {cn, cn, cs},
    {cs, cn, cs},
    {cs, cs, cs},
    {cn, cs, cs},
  };
  
  connection connections[12]= {
    {0,4},
    {1,5},
    {2,6},
    {3,7},

    {0,1},
    {1,2},
    {2,3},
    {3,0},
    
    {4,5},
    {5,6},
    {6,7},
    {7,4},    
  };
  while(1) {

    NVIC_DisableIRQ(EXTI15_10_IRQn);
    
    //Update display buffer with DMA
    if (get_transfer() == 0)
    {
      cnt++;
      //ST7789_Clear();
      uint8_t dots = 16;
      for (uint8_t k = 0; k < dots; k++)
      {
        cnt--;
        ST7789_DrawFilledCircle((uint16_t)(88.0f+ 82.0f *sinf(-cnt/20.0f+2*3.1415f/dots*k)), (uint16_t)(88.0f+ 82.0f *cosf(-cnt/20.0f+2*3.1415f/dots*k)), 5,
         0x00);
        cnt++;
        ST7789_DrawFilledCircle((uint16_t)(88.0f+ 82.0f *sinf(-cnt/20.0f+2*3.1415f/dots*k)), (uint16_t)(88.0f+ 82.0f *cosf(-cnt/20.0f+2*3.1415f/dots*k)), 5,
         ((uint32_t)(128+128 *sinf(-cnt/7.0f+2*3.1415f/dots*k)))+ ((uint32_t)(128+128 *sinf(-cnt/8.0f+2*3.1415f/dots*k))<<16) + ((uint32_t)(128+128 *sinf(-cnt/11.0f+2*3.1415f/dots*k))<<8));
      }
      ST7789_Display(false);
    }
    //Write to buffer here
    if (get_transfer() == 1)
    {
      //ST7789_Fill((uint16_t)(88.0f+ 50.0f *sinf(-cnt/10.0f)),(uint16_t)(88.0f+ 50.0f *cosf(-cnt/10.0f)),16, 16, 0x00ff00);
    }

    
    NVIC_EnableIRQ(EXTI15_10_IRQn);  
  }
}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & (1 << BTN_PINPOS) ) {
    /* clear the interrupt flag by writing a 1 */
    EXTI->PR |= (1 << BTN_PINPOS);

  }
}

void DMA2_Stream3_IRQHandler(void)
		{
			
		if(DMA2->LISR&(DMA_LISR_TCIF3))
				{
					//printf("finished transfered\r\n");

          /* gpio_up(GPIO_PIN_OLED_CS);  Theoretically needed after transfer*/
					DMA2_Stream3->CR&=~DMA_SxCR_EN;
					DMA2->LIFCR |=DMA_LIFCR_CTCIF3;
          set_transfer(0);
				}
				
		if(DMA2->LISR&(DMA_LISR_HTIF3))
				{
					//printf("half transfered\r\n");
					DMA2->LIFCR |=DMA_LIFCR_CHTIF3;
				}
				
		
		if(DMA2->LISR&(DMA_LISR_TEIF3))
						{
						//printf("transfer error interrupt\r\n");
						DMA2->LIFCR|=(DMA_LIFCR_CTEIF3);
						}
						
		if(DMA2->LISR&(DMA_LISR_DMEIF3))
						{
						//printf("Direct mode interrupt error\r\n");
						DMA2->LIFCR|=(DMA_LIFCR_CDMEIF3);
						}
						
		if(DMA2->LISR&(DMA_LISR_FEIF3))
						{
						//printf("FIFO error interrupt\r\n");
						DMA2->LIFCR|=(DMA_LIFCR_CFEIF3);
						}

			NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
		}