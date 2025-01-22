/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "arm_math.h"
#include "temperature_control.h"
#include "toggle_pins.h"
#include "read_adc_voltage.h"
#include "read_voltage.h"
#include "measure_frequency.h"
#include "calculate_angles.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SAMPLES 128  // Liczba próbek na kanał
#define ADC_CHANNELS 2   // Liczba kanałów (PA0, PA1)
#define FFT_SIZE 128


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t measuredFrequency = 0;
volatile uint8_t adc_ready = 0;
extern UART_HandleTypeDef huart2;
float fft_input[2*FFT_SIZE];
float fft_output[FFT_SIZE / 2]; // Magnitude output of the FFT
#define SAMPLE_RATE 10000       // Define your sampling rate in Hz
float thermistor_measurement[256];
uint16_t adc_buffer[FFT_SIZE];


typedef struct {
    float rsia;
    float rsib;
    float voltage_measurements;
    float frequency;
}PhaseSettings;

typedef struct {
    float amplitude_3kHz;
    PhaseSettings phaseSettinggs[4];

} DAC_Settings;



DAC_Settings measurements[1000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
int _write(int file, char *ptr, int len);
void print_adc_results(void);
//void measure_frequency(void);
float Read_ADC_Voltage(void);
void Start_Timer(void);
uint32_t Stop_Timer(void);
void Measure_Time(void);
uint32_t ADC_Read(void);
void sendToAD5641(uint16_t value);
//void performFFT(const float32_t* input, float32_t* output, uint16_t size);
void toggle_pins(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t capture1 = 0, capture2 = 0;

    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        capture1 = capture2;
        capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (capture2 > capture1)
        {
            uint32_t period = capture2 - capture1;
            measuredFrequency = HAL_RCC_GetPCLK1Freq() / (htim->Init.Prescaler + 1) / period;
        }
        else if (capture1 > capture2)
        {
            uint32_t period = (htim->Init.Period - capture1) + capture2 + 1;
            measuredFrequency = HAL_RCC_GetPCLK1Freq() / (htim->Init.Prescaler + 1) / period;
        }
    }
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
//wypsiywanie pomiarów z PAO PA1
//AF_A AF_B




//LICZNIKI DO FREQ_MEAS
void Start_Timer(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Zerowanie licznika
    HAL_TIM_Base_Start(&htim2);  // Uruchomienie timera
}

uint32_t Stop_Timer(void)
{
    HAL_TIM_Base_Stop(&htim2);  // Zatrzymanie timera
    return __HAL_TIM_GET_COUNTER(&htim2);  // Odczytanie wartości licznika
}

void Measure_Time(void)
{
    // Symulacja początku zdarzenia
    Start_Timer();

    HAL_Delay(1);  // Symulacja operacji, np. 1 ms opóźnienia

    // Symulacja końca zdarzenia
    uint32_t elapsed_time = Stop_Timer();

    // Wyświetlenie wyniku
    printf("Zmierzony czas: %lu µs\n\r", elapsed_time);
}


uint32_t ADC_Read(void)
{
    HAL_ADC_Start(&hadc1);  // Start pojedynczej konwersji
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // Czekaj na wynik
    uint32_t value = HAL_ADC_GetValue(&hadc1);  // Pobranie wyniku
    return value;
}

//ustawienie wartości na PA4

void sendToAD5641(uint16_t value)
{
    uint8_t data[2];

    // Podziel wartość 16-bitową na 2 bajty
    data[0] = (value >> 8) & 0xFF; // MSB
    data[1] = value & 0xFF;        // LSB

    // Wybierz układ AD5641 (CS niski)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Przykladowy pin CS

    // Wyślij dane przez SPI
    HAL_SPI_Transmit(&hspi2, data, 2, HAL_MAX_DELAY);

    // Odznacz układ (CS wysoki)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}



#include "arm_math.h"

void perform_fft(void) {
    // Initialize RFFT instance
    arm_rfft_fast_instance_f32 S;
    if (arm_rfft_fast_init_f32(&S, FFT_SIZE) != ARM_MATH_SUCCESS) {
        printf("FFT Initialization Error!\n\r");
        return;
    }

    // Convert ADC input to float and normalize
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_input[i] = ((float)adc_buffer[i] / 4095.0f) * 3.3f; // Scale to voltage
    }

    // Perform FFT
    arm_rfft_fast_f32(&S, fft_input, fft_output, 0);

    // Compute magnitude of the FFT output
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        fft_output[i] = sqrtf(fft_output[2 * i] * fft_output[2 * i] +
                              fft_output[2 * i + 1] * fft_output[2 * i + 1]);
    }

    // Find the dominant frequency
    float max_amplitude = 0.0f;
    uint32_t max_index = 0;
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        if (fft_output[i] > max_amplitude) {
            max_amplitude = fft_output[i];
            max_index = i;
        }
    }

    // Compute the dominant frequency
    float dominant_frequency = (float)max_index * SAMPLE_RATE / FFT_SIZE;

    // Print results
    printf("Dominant Frequency: %.2f Hz, Amplitude: %.3f\n\r", dominant_frequency, max_amplitude);
}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC2) {
        adc_ready = 1;  // Set a flag to indicate data is ready for processing
    }
}


#define DAC_MAX_VALUE          4095
#define DAC_STEP_SIZE          1

#define SAMPLES_COUNT          128       // Number of audio samples
#define SAMPLE_FREQUENCY       10000.0f  // 10 kHz sampling
#define TARGET_FREQ_BIN        13        // Approx. bin index for 1 kHz in 128-pt FFT at 10 kHz
#define RSSI_THRESHOLD         (0.05f)   // Example threshold for -50 dB, tune for your hardware
#define RATIO_THRESHOLD        2.0f      // Must be 2x average amplitude

// Structure to hold scanning candidates
typedef struct {
    float approximateSignalFrequency;  // e.g., from mapping DAC value to frequency
    uint16_t dacValue;
    float vcoTemperature;
    float amplitude1kHz;
    float averageAmplitude;
    float ratio;  // amplitude1kHz / averageAmplitude
} CandidateResult;

// Global or static arrays to store candidates
static CandidateResult candidateList[64]; // Adjust size if you expect more
static uint32_t candidateCount = 0;

// Forward declarations of functions assumed to exist.

//-------------------------------------------------------------------------
// Function: Perform a scanning procedure over a DAC range
//-------------------------------------------------------------------------

// Example frequency mapping for demonstration.
// Tweak to match your VCO's tuning curve or actual measured function.
static float CalculateFrequencyFromDAC(uint16_t dacValue)
{
    // Suppose 0 -> 2.3 GHz, 4095 -> 2.6 GHz, purely as an example:
    float freqMin = 2.3e9f; // 2.3 GHz
    float freqMax = 2.6e9f; // 2.6 GHz
    float ratio   = (float)dacValue / (float)DAC_MAX_VALUE;
    return freqMin + ratio * (freqMax - freqMin);
}

static void SetDACValue(uint16_t dacValue)
{
    // Example stub. You would set your hardware DAC here.
    // e.g., HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacValue);
}

static float MeasureRSSI(void)
{
    //TODO
	return 0;
}

static void AcquireAudioSamples(float *buffer, uint32_t numSamples, float sampleRate)
{
	//remember that buffer size = 2*numSamples
   //TODO
}


void ScanFrequencyRange(void)
{
    float bestRatio = 0.0f;
    uint16_t bestDACValue = 0;

    // Clear candidate list
    candidateCount = 0;

    // Iterate through the DAC range
    for(uint16_t dacValue = 0; dacValue <= DAC_MAX_VALUE; dacValue += DAC_STEP_SIZE)
    {
        // 1) Set the DAC to the current value
        SetDACValue(dacValue);

        // 2) Measure the RSSI
        float rssi = MeasureRSSI();

        // Check if RSSI > -50 dB threshold
        if(rssi > RSSI_THRESHOLD)
        {
            // 3) Acquire 128 samples of the demodulated audio at 10 kHz
            AcquireAudioSamples(fft_input, FFT_SIZE, SAMPLE_FREQUENCY);



            // 5) Check amplitude at ~1 kHz vs. average amplitude
            //    The bin for 1 kHz is roughly ~13 for 128 samples at 10 kHz
            float amplitude1kHz = fft_output[TARGET_FREQ_BIN];

            // Compute average amplitude across all bins from 0..(SAMPLES_COUNT/2 - 1)
            // (You might skip bin 0 if it’s large DC, but that’s up to you.)
            float sum = 0.0f;
            for(uint32_t i = 0; i < (SAMPLES_COUNT / 2); i++) {
                sum += fft_output[i];
            }
            float avgAmplitude = sum / (float)(SAMPLES_COUNT / 2);

            float ratio = amplitude1kHz / (avgAmplitude + 1e-9f); // Avoid divide-by-zero

            // Check if amplitude at 1 kHz is >= 2x average
            if(ratio >= RATIO_THRESHOLD)
            {
                // 6) Record in candidate list
                if(candidateCount < (sizeof(candidateList)/sizeof(candidateList[0])))
                {
                    candidateList[candidateCount].approximateSignalFrequency = CalculateFrequencyFromDAC(dacValue);
                    candidateList[candidateCount].dacValue                 = dacValue;
                    candidateList[candidateCount].vcoTemperature           = Read_Temperature_Celsius();
                    candidateList[candidateCount].amplitude1kHz            = amplitude1kHz;
                    candidateList[candidateCount].averageAmplitude         = avgAmplitude;
                    candidateList[candidateCount].ratio                    = ratio;
                    candidateCount++;
                }

                // 7) Print results
                printf("Candidate found!\n");
                printf("  DAC = %u\n", dacValue);
                printf("  RSSI = %.3f\n", rssi);
                printf("  Approx. Frequency = %.2f Hz\n",
                       CalculateFrequencyFromDAC(dacValue));
                printf("  Temperature = %.2f degC\n", Read_Temperature_Celsius());
                printf("  1 kHz amplitude = %.4f\n", amplitude1kHz);
                printf("  Average amplitude = %.4f\n", avgAmplitude);
                printf("  Ratio (1kHz/avg) = %.2f\n", ratio);

                // Keep track of the best ratio
                if(ratio > bestRatio)
                {
                    bestRatio   = ratio;
                    bestDACValue = dacValue;
                }
            }
        } // end if RSSI > threshold
    } // end for(dacValue)

    // 8) After scanning, set DAC to the value that yielded the highest ratio
    if(bestRatio > 0.0f)
    {
        SetDACValue(bestDACValue);
        printf("\nSetting DAC to best ratio candidate:\n");
        printf("  Best DAC Value = %u\n", bestDACValue);
        printf("  Best Ratio     = %.2f\n\n", bestRatio);
    }
    else
    {
        printf("No valid candidate found above threshold.\n");
    }
}

//-------------------------------------------------------------------------
// Example stubs for hardware-specific functions
// (Implement these for your hardware.)
//-------------------------------------------------------------------------




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, NUM_SAMPLES);
  Heater_Update_Blocking();
  ScanFrequencyRange();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t num =0;
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //toggle_pins();
	 Heater_Update_Non_Blocking();

	  for (uint8_t pair = 0; pair < 4; pair++) {
		  toggle_pins();
		  measurements[num].phaseSettinggs[pair].voltage_measurements=read_ADC_voltage();
		  measurements[num].phaseSettinggs[pair].rsia=read_voltage(&hadc2, ADC_CHANNEL_0);
		  measurements[num].phaseSettinggs[pair].rsib=read_voltage(&hadc3, ADC_CHANNEL_1);
		  measurements[num].phaseSettinggs[pair].frequency=measure_frequency();
		  float max_voltage = 3.3f;
		  float cos_alpha = measurements[num].phaseSettinggs[pair].rsia / max_voltage;
		  float cos_beta = measurements[num].phaseSettinggs[pair].rsib / max_voltage;
		  AngleResults angles = calculate_angles(cos_alpha, cos_beta);

		  printf("Pair %d: Detector=%.2f V, RSSI_A=%.2f V, RSSI_B=%.2f V, Frequency=%.2f Hz\n\r",
				  pair,
				  measurements[num].phaseSettinggs[pair].voltage_measurements,
				  measurements[num].phaseSettinggs[pair].rsia,
				  measurements[num].phaseSettinggs[pair].rsib,
				  measurements[num].phaseSettinggs[pair].frequency
				  );

		  //set_servo_pwm(&htim4, TIM_CHANNEL_3, angles.azimuth);
		  //set_servo_pwm(&htim4, TIM_CHANNEL_4, angles.elevation);
		  HAL_Delay(100);
	  }
	  num++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  //--- ADC Config ---
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;              // enable scanning
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;                // total 4 channels
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  //--- Channel 10 ---
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  //--- Channel 11 ---
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  //--- Channel 12 ---
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  //--- Channel 13 ---
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
