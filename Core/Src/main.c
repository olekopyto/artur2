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

//#include "arm_math.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc1;

uint16_t adc_buffer[NUM_SAMPLES * ADC_CHANNELS];


DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t measuredFrequency = 0;
volatile uint8_t adc_ready = 0;
extern UART_HandleTypeDef huart2;
float fft_input[2 * FFT_SIZE];   // Input for FFT (complex format: real and imaginary parts)
float fft_output[FFT_SIZE / 2]; // Magnitude output of the FFT
#define SAMPLE_RATE 10000       // Define your sampling rate in Hz
float thermistor_measurement[256];

typedef struct {
    float rsia;
    float rsib;
    float voltage_measurements;
}PhaseSettings;

typedef struct {
    float frequency;
    float amplitude_3kHz;  // Amplituda sygnału 3 kHz (zmiennoprzecinkowa)
    PhaseSettings phaseSettinggs[4];

} DAC_Settings;


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
void measure_frequency(void);
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

//FREQ_MEAS_A PA6
void measure_frequency() {
    // Zrestartuj licznik
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // Uruchom timery
    HAL_TIM_Base_Start(&htim3); // Timer odmierzający 100 µs
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1); // Timer zliczający impulsy

    // Poczekaj na timeout (lub przerwanie)
    HAL_Delay(1); // Tymczasowe, zamiast przerwań

    // Odczytaj liczbę zliczonych impulsów
    uint32_t pulse_count = __HAL_TIM_GET_COUNTER(&htim2);

    // Przelicz na częstotliwość
    float frequency = pulse_count / 0.0001; // 100 µs = 0.0001 s

    // Zatrzymaj timery
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);

    printf("Zmierzona częstotliwosc: %.2f Hz\n", frequency);
}
//TERMISTOR VCO TEMP PC2
float Read_ADC_Voltage(void)
{
    uint32_t adcValue = 0;

    // Uruchomienie konwersji ADC
    HAL_ADC_Start(&hadc1);

    // Czekanie na zakończenie konwersji
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(&hadc1);  // Odczytanie wartości z ADC
    }

    // Obliczenie napięcia
    float voltage = (float)adcValue * 3.3f / 4095.0f;  // Napięcie na podstawie odczytu ADC
    return voltage;
}

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
    printf("Zmierzony czas: %lu µs\n", elapsed_time);
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

// Funkcja odczytu napięcia dla danego kanału ADC
float ReadVoltage(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    //sConfig.Rank = IS_ADC_REGULAR_RANK;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    // Konfiguracja kanału
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Start konwersji
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
        uint32_t adcValue = HAL_ADC_GetValue(hadc);
        float voltage = (adcValue * 3.3) / 4096; // Zakładając referencję 3.3V i 12-bitowy ADC
        return voltage;
    }

    return 0;
}

#include "arm_math.h"

void perform_fft(void) {
    // Initialize RFFT instance
    arm_rfft_fast_instance_f32 S;
    if (arm_rfft_fast_init_f32(&S, FFT_SIZE) != ARM_MATH_SUCCESS) {
        printf("FFT Initialization Error!\n");
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
    printf("Dominant Frequency: %.2f Hz, Amplitude: %.3f\n", dominant_frequency, max_amplitude);
}



//przełączanie pomiędzy pinami
void toggle_pins(void) {
    static uint8_t state = 0;

    switch (state) {
        case 0:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // PB0 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // PB1 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // PB2 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // PB10 LOW
            printf("0\n");
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // PB0 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // PB1 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  // PB2 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // PB10 LOW
            printf("1\n");
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // PB0 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // PB1 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // PB2 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // PB10 HIGH
            printf("2\n");
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // PB0 HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // PB1 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // PB2 LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // PB10 HIGH
            printf("3\n");
            break;
    }

    // Zwiększanie stanu i zapewnienie cykliczności
    state = (state + 1) % 4;

    // Dodanie opóźnienia, żeby zmiany były zauważalne
    HAL_Delay(1000);  // Opóźnienie 500 ms
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC2) {
        adc_ready = 1;  // Set a flag to indicate data is ready for processing
    }
}


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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  while (1)
  {
    /* USER CODE END WHILE */

	 toggle_pins();
	 if (adc_ready)
	 {
	     adc_ready = 0;       // Clear flag
	     perform_fft();       // Call your FFT function
	 }
    /* USER CODE BEGIN 3 */
	  //toggle_pins();
	 Heater_Update_Non_Blocking();
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
    ADC_ChannelConfTypeDef sConfig = {0};

    /* ADC2 Initialization */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  // Divide clock to balance speed and accuracy
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;           // 12-bit resolution
    hadc2.Init.ScanConvMode = DISABLE;                    // Single channel
    hadc2.Init.ContinuousConvMode = ENABLE;               // Continuous conversion mode
    hadc2.Init.DiscontinuousConvMode = DISABLE;           // Disable discontinuous conversions
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // No external trigger
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;     // Software-triggered conversions
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;           // Right-aligned data
    hadc2.Init.NbrOfConversion = 1;                       // Single conversion
    hadc2.Init.DMAContinuousRequests = ENABLE;            // Enable DMA for ADC
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;        // End of Conversion after each sample

    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure ADC2 regular channel (PA0 -> ADC_CHANNEL_0) */
    sConfig.Channel = ADC_CHANNEL_0;                     // Use ADC Channel 0 (PA0)
    sConfig.Rank = 1;                                    // Rank 1 in the regular group
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;      // Increase sampling time for better accuracy

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
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
  htim4.Init.Prescaler = 8399; // Adjust for 1ms tick
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999; // 1-second period
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
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
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
