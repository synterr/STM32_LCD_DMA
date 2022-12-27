#include "stm32f4xx.h"                  // Device header
#include "gpio.h"


static void gpio_init_speed_otype_pupd(GPIO_TypeDef *gpio_hndl,
                                       unsigned pinpos) {

  gpio_hndl->OSPEEDR |= (unsigned)(3 << (pinpos * 2));
  gpio_hndl->OTYPER &= ~(unsigned)(1 << (pinpos));
  gpio_hndl->PUPDR &=  ~(unsigned)(3 << (pinpos * 2));

}

static void gpio_enable_clock(GPIO_TypeDef *gpio_hndl) {

  if (gpio_hndl == GPIOA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  else if (gpio_hndl == GPIOB)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  else if (gpio_hndl == GPIOC)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  else if (gpio_hndl == GPIOD)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  else
    return;
}

static GPIO_TypeDef* get_port_base(gpio_pin_t pin) {
  
  switch(pin) {

    case GPIO_PIN_OLED_CS:
      return OLED_CS_PORT;
    case GPIO_PIN_OLED_DC:
      return OLED_DC_PORT;
    case GPIO_PIN_OLED_RST:
      return OLED_RST_PORT;
    case GPIO_PIN_LED:
      return LED_PORT;
    case GPIO_PIN_BTN:
      return BTN_PORT;

    default: return 0;
  }
}

static unsigned get_pinpos(gpio_pin_t pin) {
  switch(pin) {

    case GPIO_PIN_OLED_CS:
      return OLED_CS_PINPOS;
    case GPIO_PIN_OLED_DC:
      return OLED_DC_PINPOS;
    case GPIO_PIN_OLED_RST:
      return OLED_RST_PINPOS;
    case GPIO_PIN_LED:
      return LED_PINPOS;
    case GPIO_PIN_BTN:
      return BTN_PINPOS;

    default: return 0;
  }
}

void gpio_up(gpio_pin_t pin) {
  GPIO_TypeDef *gpio = get_port_base(pin);

  if (!gpio)
    return;
  
  gpio->BSRR = (1 << get_pinpos(pin));
}

void gpio_down(gpio_pin_t pin) {
  GPIO_TypeDef *gpio = get_port_base(pin);

  if (!gpio)
    return;

  gpio->BSRR = (1 << (get_pinpos(pin) + 16));
}

uint8_t gpio_get(gpio_pin_t pin) {

  GPIO_TypeDef *gpio = get_port_base(pin);

  if (!gpio)
    return 2;

  return ((gpio->IDR) & (1 << get_pinpos(pin))) ? 1 : 0;
}

void gpio_init(gpio_pin_t pin) {
  GPIO_TypeDef *gpio = get_port_base(pin);
  unsigned pinpos = get_pinpos(pin);

  gpio_enable_clock(gpio);

  gpio->MODER &= ~(unsigned)(3 << (pinpos * 2));
  gpio->MODER |=  (unsigned)(1 << (pinpos * 2));

  gpio_init_speed_otype_pupd(gpio, pinpos);

  gpio->BSRR = (1 << (pinpos + 16));
}

void gpio_init_input(gpio_pin_t pin) {
  GPIO_TypeDef *gpio = get_port_base(pin);
  unsigned pinpos = get_pinpos(pin);

  gpio_enable_clock(gpio);

  gpio->MODER &= ~(unsigned)(3 << (pinpos * 2));
  gpio->PUPDR &= ~(unsigned)(3 << (pinpos * 2));
  
  if (pin == GPIO_PIN_BTN) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    
    EXTI->RTSR &= ~(1<<pinpos);   /* Disble Rising Edge Trigger for BUTTONS */
    EXTI->FTSR |=  (1<<pinpos);   /* Enable Falling Edge Trigger for BUTTONS */

    EXTI->IMR |=   (1<<pinpos);   /* Disable the mask for pin on EXTI */
    EXTI->PR  |=   (1<<pinpos);   /* Clear the interrupt flag */
  }
}

void gpio_init_input_pu(gpio_pin_t pin) {
  GPIO_TypeDef *gpio = get_port_base(pin);
  unsigned pinpos = get_pinpos(pin);

  gpio_init_input(pin);

  gpio->PUPDR |=  (1 << (pinpos * 2));
}

void gpio_init_input_pd(gpio_pin_t pin) {
  GPIO_TypeDef *gpio = get_port_base(pin);
  unsigned pinpos = get_pinpos(pin);

  gpio_init_input(pin);

  gpio->PUPDR |=  (unsigned)(2 << (pinpos * 2));
}

void gpio_init_af(gpio_pin_t pin, unsigned af_no) {
  GPIO_TypeDef *gpio = get_port_base(pin);
  unsigned pinpos = get_pinpos(pin);

  gpio_enable_clock(gpio);

  gpio->MODER &= ~(unsigned)(3 << (pinpos * 2));
  gpio->MODER |=  (unsigned)(2 << (pinpos * 2));

  gpio_init_speed_otype_pupd(gpio, pinpos);

  if (af_no < 8) {
    gpio->AFR[0] &= ~(unsigned)(0xF   << (pinpos * 4));
    gpio->AFR[0] |=  (unsigned)(af_no << (pinpos * 4));
  } else {
    gpio->AFR[1] &= ~(unsigned)(0xF   << ((pinpos - 8) * 4));
    gpio->AFR[1] |=  (unsigned)(af_no << ((pinpos - 8) * 4));
  }
}
