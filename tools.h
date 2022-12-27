#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>
#include <stdio.h>
#include <string.h> /* strlen */
#include <stdlib.h> /* atof */

#define DLMUL 6000

void delay_nops (uint32_t dlyTicks);
char* float_to_string(float val, char* output);
float string_to_float(char* input);
char* int_to_string(uint16_t val, char* output);
uint16_t string_to_int(char* input);

int max(int num1, int num2);
int min(int num1, int num2);
#endif

