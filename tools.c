#include "stm32f4xx.h"                  // Device header
#include "tools.h"


void delay_nops (uint32_t dlyTicks) {
  
  for(uint32_t dly =0; dly < dlyTicks; dly++) { __NOP(); }
}

char* float_to_string(float val, char* output) {
  sprintf(output, "%g", val);
  return output;
}

float string_to_float(char* input) {
   return (float)atof(input);
}

char* int_to_string(uint16_t val, char* output) {
  sprintf(output, "%d", val);
  return output;
}

uint16_t string_to_int(char* input) {
   return (uint16_t)atoi(input);
}

/**
 * Find maximum between two numbers.
 */
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

/**
 * Find minimum between two numbers.
 */
int min(int num1, int num2) 
{
    return (num1 > num2 ) ? num2 : num1;
}