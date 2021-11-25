#ifndef INC_I2CLIB_H
#define INC_I2CLIB_H
#include "main.h"

void turnOFF(void);
void turnON(void);
void setFreq(uint16_t PWMFreq);
void setDuty(uint16_t DutyCycle);
void setNumberLed(uint16_t led);

#endif
