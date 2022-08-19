/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const unsigned int note_freq[9] = {0 ,262, 294, 330, 349, 392, 440, 494, 523};
//                                 rest,do,  re,  mi,  fa,  so,  la,  si,  do_high
#define DEFAULT_DUTY 90
#define BUZZER_MODE 0
#define SONG_MODE_1 1
#define SONG_MODE_2 2
#define TOTAL_MODES 3

#define INCRE 1.05946



const int note[201]            = {1,1,3,5,5,  6,1,6,5,5,  3,3,5,3,1,  6,1,2,5,5,
                                  6,6,6,1,5,  3,2,3,2,1,  2,5,5,4,5,  6,6,7,6,5,
                                  1,1,6,1,    5,5,6,5,1,  6,6,5,3,    2,3,4,5,5,
                                  1,1,1,3,    2,3,2,1,1,  6,6,5,3,    2,2,3,1,1,
                                  1,1,1,0,    6,6,6,0,    5,5,6,5,    2,2,3,5,5,
                                  1,1,1,0,    6,6,6,0,    5,5,6,5,    2,2,3,5,5,
-1};

const int beat[201]            = {2,1,1,2,2,  2,1,1,2,2,  2,2,1,1,2,  2,1,1,2,2,
                                  2,2,1,1,2,  2,2,1,1,2,  2,2,1,1,2,  2,2,1,1,2,
                                  3,1,2,2,    2,1,1,2,2,  3,1,2,2,    2,1,1,2,2,
                                  3,1,2,2,    2,1,1,2,2,  3,1,2,2,    2,1,1,2,2,
                                  2,2,2,2,    2,2,2,2,    2,2,2,2,    2,1,1,2,2,
                                  2,2,2,2,    2,2,2,2,    2,2,2,2,    2,1,1,2,2,
-1};

const int tone[201]            = {0,0,0,0,0,  0,1,0,0,0,  0,0,0,0,0, -1,0,0,0,0,
                                  0,0,0,1,0,  0,0,0,0,0,  0,0,0,2,0,  0,0,0,0,0,
                                  1,1,0,1,    0,0,0,0,0,  0,0,0,0,    0,0,0,0,0,
                                  0,0,0,0,    0,0,0,0,0,  0,0,0,0,    0,0,0,0,0,
                                  1,1,1,0,    0,0,0,0,    0,0,0,0,    0,0,0,0,0,
                                  1,1,1,0,    0,0,0,0,    0,0,0,0,    0,0,0,0,0,
-1}; //+1�???8度，+2升半�???

int play_mode = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//unsigned int i = 0;
//int play_mode = 1;//midi与放歌的切换
//int frqc = 0;
//int ratio = 0;
//long int dot_unit = 300;
//int j = 0;
//int play_stop = 1;//播放�???????????1，暂停是-1
//int which_song = -1;//放那首歌�??????
//千位是升降（1降，2升，还原则没有千位），百位是音符，十位是音长，个位是音区,此处是编谱区域；
//const int score_org_2[] = {121,111,311,521,521,621,112,611,521,521,321,321,511,311,121,
//                         620,111,211,521,-1};
//const int score_org[] = {341,441,511,511,541,511,541,641,511,021,341,441,
//                         511,511,341,241,111,221,341,241,141,111,141,141,142,112,741,711,
//                         142,741,641,511,611,541,411,441,541,641,541,521,031,
//                         112,122,641,541,511,641,541,521,611,611,541,111,241,
//                         321,021,211,111,141,141,311,341,341,241,211,611,241,211,211,741,621,531,021,
//                         341,441,511,511,541,511,541,641,511,021,341,441,
//                          511,511,341,241,111,221,341,241,141,111,141,141,142,112,741,711,
//                          142,741,641,511,611,541,411,441,541,641,541,521,031,
//                          112,122,641,541,511,641,541,521,611,611,541,111,241,
//                          321,021,211,111,141,141,311,341,341,241,221,641,511,541,341,241,540,341,241,131,
//                          641,641,641,741,112,641,541,741,711,142,212,711,142,741,641,511,141,241,341,331,
//                          311,711,741,711,341,212,212,112,741,641,631,621,011,
//                          641,641,641,741,142,711,641,741,711,142,212,741,741,322,312,242,142,122,112,142,741,
//                          611,611,641,611,641,511,511,241,321,131,031,
//                         -1};

//int score_len = 0;
//long int dot[] = {0,300,300*2,300*4,(int)300/2};

void setPWM(double freq, int duty_percent)
{
	if (freq == 0) {
		TIM3->CCR1 = 0;
		return ;
	}
	TIM3->ARR = 10000 / freq - 1;
	TIM3->CCR1 = (double)duty_percent * (TIM3->ARR + 1) / 100.0;
}

void init_buzzer()
{
	unsigned int i;
	for (i=1;i<8;i++) {
		setPWM(note_freq[i], DEFAULT_DUTY);
		HAL_Delay(200);
	}
	setPWM(0, 100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // Keys interrupt
{
//	if (clock - lastTime <= 80) {
//		lastTime = clock;
//		return ;
//	}
//	lastTime = clock;
	if (GPIO_Pin == GPIO_PIN_8) {
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) {
			// switch mode
			play_mode = (play_mode + 1) % TOTAL_MODES;
		}
		return ;
	}
	double freq = 0.0;
	switch (GPIO_Pin) {
	case GPIO_PIN_1: freq = note_freq[1]; break;
	case GPIO_PIN_2: freq = note_freq[2]; break;
	case GPIO_PIN_3: freq = note_freq[3]; break;
	case GPIO_PIN_4: freq = note_freq[4]; break;
	case GPIO_PIN_5: freq = note_freq[5]; break;
	case GPIO_PIN_6: freq = note_freq[6]; break;
	case GPIO_PIN_7: freq = note_freq[7]; break;
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) {
		// big button pushed down
		freq *= 2.0;
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0) {
		// small button pushed down
		freq /= 2.0;
	}
	setPWM(freq, DEFAULT_DUTY);
}



void play_music(const int* pnote, const int* pbeat, const int* ptone)
{
	int i;
	for (i=0;pnote[i]!=-1 && play_mode != BUZZER_MODE;i++) {
		double freq = note_freq[pnote[i]];

		switch (ptone[i]) {
		case 1:  freq *= 2; break;
		case -1: freq /= 2; break;
		case 2:  freq *= INCRE; break;
		case -2: freq /= INCRE; break;
		}
		setPWM(freq, DEFAULT_DUTY);
		HAL_Delay(200 * pbeat[i]);
        setPWM(0, 100);
        HAL_Delay(50);
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //启动定时器TIM3通道1的PWM输出
  //定义函数
  setPWM(0, DEFAULT_DUTY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (play_mode) {
	  case BUZZER_MODE:
		  init_buzzer();
		  while (play_mode==BUZZER_MODE);
		  break;
	  HAL_Delay(1000);
	  case SONG_MODE_1:
		  play_music(note, beat, tone);
		  break;
	  case SONG_MODE_2:
		  break;
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB3 PB4
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
