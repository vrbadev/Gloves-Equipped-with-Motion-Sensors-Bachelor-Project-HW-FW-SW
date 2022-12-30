#ifndef DELAY_H__
#define DELAY_H__

#include "BlueNRG1_conf.h"

void systick_init(void);
void delay_ms(uint32_t delay);
uint32_t get_ms(void);

void mft_init(void);
void delay_us(uint16_t delay);

#endif
