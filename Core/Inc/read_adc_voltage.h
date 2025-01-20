/*
 * read_adc_voltage.h
 *
 *  Created on: Jan 20, 2025
 *      Author: matwa
 */

#ifndef INC_READ_ADC_VOLTAGE_H_
#define INC_READ_ADC_VOLTAGE_H_
ADC_HandleTypeDef hadc1;

//TERMISTOR VCO TEMP PC2
float read_ADC_voltage(void)
{
    uint32_t adcValue = 0;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(&hadc1);
    }

    float voltage = (float)adcValue * 3.3f / 4095.0f;
    return voltage;
}

#endif /* INC_READ_ADC_VOLTAGE_H_ */
