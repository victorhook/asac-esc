#include "machine.h"
#include "hal.h"

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;



void SystemClock_Config();
static void timer14_init();
static void nvic_interrupt_init();
static void uart2_init();
static void adc_init();
static void gpios_init();
static void init_gpio_output_pin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState init_state);
static void init_pwm_timer_channel(TIM_HandleTypeDef* timx, const uint32_t channel,
                                   GPIO_TypeDef* port,
                                   const uint32_t pin, const uint32_t alternate_function);
static void init_pwm_timer(TIM_HandleTypeDef* timx, TIM_TypeDef* timer_inst);


// INIT
void hal_init()
{
    // GPIO_AF5_TIM1  // T1_CH1
    // GPIO_AF5_TIM1  // T1_CH2
    // GPIO_AF1_TIM1  // T1_CH3
    // GPIO_AF2_TIM1  // T1_CH4
    // GPIO_AF11_TIM3 // T3_CH2
    // GPIO_AF3_TIM3  // T3_CH3

    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

    init_pwm_timer(&htim1, TIM1);
    init_pwm_timer(&htim3, TIM3);
    init_pwm_timer_channel(&htim1, TIM_CHANNEL_2, GPIOA, GPIO_PIN_1, GPIO_AF5_TIM1);   // HIN_A
    init_pwm_timer_channel(&htim1, TIM_CHANNEL_3, GPIOB, GPIO_PIN_6, GPIO_AF1_TIM1);   // HIN_C
    init_pwm_timer_channel(&htim3, TIM_CHANNEL_2, GPIOC, GPIO_PIN_14, GPIO_AF11_TIM3); // HIN_B

    // Prescaler - TIMx_PSC -> Divides counter clock frequency [0, 65536]
    // Counter mode - Upcoing
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2048 / 10);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2048 / 10);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2048 / 10);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    gpios_init();
    timer14_init();
    adc_init();
    nvic_interrupt_init();
    uart2_init();

}

static void init_gpio_output_pin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState init_state)
{
    HAL_GPIO_WritePin(port, pin, init_state);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void gpios_init()
{
    init_gpio_output_pin(PORT_LIN_A,    PIN_LIN_A,    GPIO_PIN_RESET);
    init_gpio_output_pin(PORT_LIN_B,    PIN_LIN_B,    GPIO_PIN_RESET);
    init_gpio_output_pin(PORT_LIN_C,    PIN_LIN_C,    GPIO_PIN_RESET);
    init_gpio_output_pin(PORT_LED_RED,  PIN_LED_RED,  GPIO_PIN_RESET);
    init_gpio_output_pin(PORT_LED_BLUE, PIN_LED_BLUE, GPIO_PIN_RESET);

    // Current sensing input + BEMF
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Input signal pin
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void init_pwm_timer_channel(TIM_HandleTypeDef* timx, const uint32_t channel,
                                   GPIO_TypeDef* port,
                                   const uint32_t pin, const uint32_t alternate_function)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (HAL_TIM_PWM_Init(timx) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(timx, &sConfigOC, channel) != HAL_OK)
    {
        Error_Handler();
    }

    GPIO_InitStruct.Pin = pin;
    // Alternate Function Push Pull Mode
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // Peripheral to be connected to the selected pins
    GPIO_InitStruct.Alternate = alternate_function;

    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void init_pwm_timer(TIM_HandleTypeDef* timx, TIM_TypeDef* timer_inst)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  timx->Instance = timer_inst;
  timx->Init.Prescaler = 0;
  timx->Init.CounterMode = TIM_COUNTERMODE_UP;
  timx->Init.Period = 2048;
  timx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timx->Init.RepetitionCounter = 0;
  timx->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(timx) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(timx, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void uart2_init()
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  huart2.FifoMode = UART_FIFOMODE_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void timer14_init()
{
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 48;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 0xFFFF;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_Base_Start(&htim14) != HAL_OK)
    {
      Error_Handler();
    }

    // Very important to enable the interrupt !
    //htim14.Instance->DIER = TIM_DIER_CC1IE;

    //htim14.Instance->CR1 |= TIM_CR1_URS;
    //htim14.Instance->CR1 &= ~TIM_CR1_UDIS;
    //htim14.Instance->CR1 |= TIM_CR1_CEN;

    //htim14.Instance->CCR1 = 1000;
    //TIM14->SR &= ~TIM_SR_UIF;
    //HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

static void adc_init()
{
    ADC_ChannelConfTypeDef sConfig = {0};

     /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    //sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }


}

static void nvic_interrupt_init()
{
    /* EXTI2_3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


// PRINTF over UART

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *data, int len)
{
   // arbitrary timeout 1000
   HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);
   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}
