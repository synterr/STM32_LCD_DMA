#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

/* LED */
#define LED_PINPOS                 5
#define LED_PORT                   GPIOA

/* OLED */
#define OLED_CS_PINPOS              9
#define OLED_CS_PORT                GPIOA
#define OLED_DC_PINPOS              8
#define OLED_DC_PORT                GPIOA
#define OLED_RST_PINPOS             10
#define OLED_RST_PORT               GPIOB

#define BTN_PINPOS                 13
#define BTN_PORT                   GPIOC

typedef enum gpio_pin_e {
  GPIO_PIN_OLED_CS,
  GPIO_PIN_OLED_DC,
  GPIO_PIN_OLED_RST,
  GPIO_PIN_LED,
  GPIO_PIN_BTN,
} gpio_pin_t;

void gpio_up(gpio_pin_t pin);
void gpio_down(gpio_pin_t pin);

uint8_t gpio_get(gpio_pin_t pin);

void gpio_init(gpio_pin_t pin);

void gpio_init_input(gpio_pin_t pin);
void gpio_init_input_pu(gpio_pin_t pin);
void gpio_init_input_pd(gpio_pin_t pin);
void gpio_init_af(gpio_pin_t pin, unsigned af_no);
#endif

