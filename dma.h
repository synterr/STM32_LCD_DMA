/**
  ******************************************************************************
  * @file       dma.h
  * @author     Marcin Bukowski
  ******************************************************************************
  * @attention
  * COPYRIGHT 2022 DCT DELTA GmbH. All rights reserved.
  * CONTACT-INFO: info@dct-delta.de
  ******************************************************************************
  */
#ifndef DMA_H
#define DMA_H

#include <stdint.h>
static volatile uint8_t transfer = 0;

void dma_spi_enable(void);
void dma_spi_disable(void);

void dma_init(void);
void dma_start(uint8_t *data, uint32_t size);

uint8_t get_transfer(void);
void set_transfer(uint8_t tr);
#endif
