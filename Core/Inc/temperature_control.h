/*
 * temperature_control.h
 *
 *  Created on: Jan 20, 2025
 *      Author: Adam Kulczycki
 */

#ifndef INC_TEMPERATURE_CONTROL_H_
#define INC_TEMPERATURE_CONTROL_H_


#define HEATER_GPIO_PORT          GPIOC
#define HEATER_GPIO_PIN           GPIO_PIN_12

#define THERMISTOR_GPIO_PORT      GPIOC
#define THERMISTOR_GPIO_PIN       GPIO_PIN_2
#define THERMISTOR_ADC_CHANNEL    ADC_CHANNEL_10

// Thermistor parameters
#define THERMISTOR_R25             (10000.0f)
#define THERMISTOR_B_COEFFICIENT   (3950.0f)
#define THERMISTOR_REF_TEMPERATURE (298.15f)

// Target & Tolerance
#define HEAT_EXPECTED_TEMP         (25.0f)
#define HEAT_TEMPERATURE_TOLERANCE (0.2f)
#define HEAT_LOOP_INTERVAL         (10)

// External references
extern ADC_HandleTypeDef hadc1;


void Heater_On(void)
{
    HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_SET);
}

void Heater_Off(void)
{
    HAL_GPIO_WritePin(HEATER_GPIO_PORT, HEATER_GPIO_PIN, GPIO_PIN_RESET);
}


/* Configure the ADC channel for the thermistor (PC2 => ADC_CHANNEL_10) */
void Thermistor_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = THERMISTOR_ADC_CHANNEL;
    sConfig.Rank = 1;      // Single conversion
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        // You might want to handle errors differently
        Error_Handler();
    }
}


void Heater_Init(void)
{
    // ----------------------------
    // 1) Configure PC12 for heater control
    // ----------------------------
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = HEATER_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HEATER_GPIO_PORT, &GPIO_InitStruct);

    // Start with heater off
    Heater_Off();

    // ----------------------------
    // 2) Configure ADC channel 10 (PC2) once
    // ----------------------------
    Thermistor_ADC_Init();
}



/* Single conversion read of the thermistor channel, returning temp in Celsius */
float Read_Temperature_Celsius(void)
{
    // 1) Start ADC conversion
    HAL_ADC_Start(&hadc1);

    // 2) Wait for ADC conversion to finish
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
    {
        // handle error or return an error indicator
        return -999.0f;
    }

    // 3) Read the raw ADC value (0..4095 for 12-bit)
    uint32_t rawValue = HAL_ADC_GetValue(&hadc1);

    // 4) Convert the ADC count to voltage (assuming 3.3V reference)
    float voltage = (3.3f * (float)rawValue) / 4095.0f;

    // 5) Compute thermistor resistance (voltage divider with 10k resistor to ground)
    float R_fixed = 10000.0f; // 10k
    float R_therm = R_fixed * (voltage / (3.3f - voltage));

    // 6) Convert to temperature in °C using B‐coefficient equation
    float term = logf(R_therm / THERMISTOR_R25);  // ln(R/R25)
    float invT = (term / THERMISTOR_B_COEFFICIENT) + (1.0f / THERMISTOR_REF_TEMPERATURE);
    float tempKelvin = 1.0f / invT;
    float tempCelsius = tempKelvin - 273.15f;

    return tempCelsius;
}

/* BLOCKING approach: Turn heater on, wait until temperature is above threshold, then off */
void Heater_Update_Blocking(void)
{
    Heater_On();
    while ((HEAT_EXPECTED_TEMP - Read_Temperature_Celsius()) > HEAT_TEMPERATURE_TOLERANCE)
    {
        HAL_Delay(HEAT_LOOP_INTERVAL);
    }
    Heater_Off();
}

/* NON-BLOCKING approach: Call periodically; if below threshold => On, else Off */
void Heater_Update_Non_Blocking(void)
{
    float currentTemp = Read_Temperature_Celsius();
    if ((HEAT_EXPECTED_TEMP - currentTemp) > HEAT_TEMPERATURE_TOLERANCE)
    {
        Heater_On();
    }
    else
    {
        Heater_Off();
    }
}

#endif /* INC_TEMPERATURE_CONTROL_H_ */
