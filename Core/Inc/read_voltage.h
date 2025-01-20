/*
 * read_voltage.h
 *
 *  Created on: Jan 20, 2025
 *      Author: matwa
 */

#ifndef INC_READ_VOLTAGE_H_
#define INC_READ_VOLTAGE_H_

// Funkcja odczytu napięcia dla danego kanału ADC (RSIA, RSIB)
float read_voltage(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;

    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }


    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
        uint32_t adcValue = HAL_ADC_GetValue(hadc);
        float voltage = (adcValue * 3.3) / 4096;
        return voltage;
    }

    return 0;
}

#endif /* INC_READ_VOLTAGE_H_ */
