#include "inc/adc.h"

ADC_InitType adc_settings;

void adc_init(void)
{
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
	
	adc_settings = (ADC_InitType) {
		.ADC_OSR = ADC_OSR_200,
		.ADC_Input = ADC_Input_AdcPin1,
		.ADC_ConversionMode = ADC_ConversionMode_Single,
		.ADC_Attenuation = ADC_Attenuation_0dB,
		.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6
	};
	
	ADC_Init(&adc_settings);
	ADC_AutoOffsetUpdate(ENABLE);
	ADC_Calibration(ENABLE);
}

void adc_read(adc_measurement_t* measurement)
{
  ADC_Cmd(ENABLE);
	while (!ADC_GetFlagStatus(ADC_FLAG_EOC));
	measurement->f = ADC_GetConvertedData(adc_settings.ADC_Input, adc_settings.ADC_ReferenceVoltage);
	//measurement->f = ADC_GetRawData() * 1.0;
}
