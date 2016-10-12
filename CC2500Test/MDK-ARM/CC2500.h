#ifndef CC2500
#define CC2500

#include "stdint.h"
#include "main.c"
#include <../CC2500.c>

void CC2500_WriteReg(uint8_t address, uint8_t data);


#endif