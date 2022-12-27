#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void spi_init (void);
void spi_transmit (uint8_t *data, int size);
#endif

