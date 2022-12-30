#ifndef ADC_H__
#define ADC_H__

#include "stdio.h"
#include "BlueNRG1_conf.h"

typedef union {
	float f;
	uint8_t u8[4];
} adc_measurement_t;

void adc_init(void);
void adc_read(adc_measurement_t* measurement);

#endif
