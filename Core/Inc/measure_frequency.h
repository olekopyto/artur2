/*
 * maesure_frequency.h
 *
 *  Created on: Jan 20, 2025
 *      Author: matwa
 */

#ifndef INC_MEASURE_FREQUENCY_H_
#define INC_MEASURE_FREQUENCY_H_

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

//FREQ_MEAS_A PA6
float measure_frequency() {
    __HAL_TIM_SET_COUNTER(&htim2, 0);


    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

    HAL_Delay(1);


    uint32_t pulse_count = __HAL_TIM_GET_COUNTER(&htim2);

    float frequency = pulse_count / 0.0001;

    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);
    return frequency;

}

#endif /* INC_MEASURE_FREQUENCY_H_ */
