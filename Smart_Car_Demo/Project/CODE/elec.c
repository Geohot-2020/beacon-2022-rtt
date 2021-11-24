#include "elec.h"

#define ADC_CH1			ADC2_CH04_A04
#define ADC_CH2			ADC2_CH05_A05
#define ADC_CH3			ADC2_CH06_A06
#define ADC_CH4			ADC2_CH07_A07
#define ADC_CH5			ADC3_CH10_C00
#define ADC_CH6			ADC3_CH11_C01
#define ADC_CH7			ADC3_CH12_C02
#define ADC_CH8			ADC3_CH13_C03

void elec_init(void)
{
    adc_init(ADC_1, ADC_CH1, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_1, ADC_CH2, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_1, ADC_CH3, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_1, ADC_CH4, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_1, ADC_CH5, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_1, ADC_CH6, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_1, ADC_CH7, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
	adc_init(ADC_2, ADC_CH8, ADC_10BIT);    // 初始化 ADC 已经对应通道引脚 10位精度
}

uint16 elec_data[8];
void elec_get(void)
{
    elec_data[0] = adc_convert(ADC_1, ADC_CH1);
    elec_data[1] = adc_convert(ADC_1, ADC_CH2);
    elec_data[2] = adc_convert(ADC_1, ADC_CH3);
    elec_data[3] = adc_convert(ADC_1, ADC_CH4);
    elec_data[4] = adc_convert(ADC_1, ADC_CH5);
    elec_data[5] = adc_convert(ADC_1, ADC_CH6);
    elec_data[6] = adc_convert(ADC_1, ADC_CH7);
    elec_data[7] = adc_convert(ADC_2, ADC_CH8);
    
}

void elec_calculate(void)
{
    
}