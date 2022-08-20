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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const double note_freq[9] = {0 ,262, 294, 330, 349, 392, 440, 494, 523};
//                          rest,do,  re,  mi,  fa,  so,  la,  si,  do_high
const int white_note[9] =        {0,   2,   4,   5,   7,    9,  11,  12}; // shift from MIDDLE_C

#define MIDDLE_C 0x3C

#define DEFAULT_DUTY 95

#define PIANO_MODE 0
#define TOTAL_MODES 5

#define OUTPUT_BUZZER 0
#define OUTPUT_MIDI 1

#define KEY0 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) // key0 pushed down is high ttl
#define KEY1 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)// other keys pushed down is low ttl
#define KEY2 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define KEY3 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)
#define KEY4 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)
#define KEY5 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)
#define KEY6 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)
#define KEY7 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)
#define KEY8 !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)

//#define INCRE 1.05946

const int score[10][4][402] =
{
	{{0}},
	{ // 校歌
		 {200, 3}, // 整体性质：score[i][0][0]速度(bpm),score[i][0][1]整体音调(0C,1C#这样)(挪动几个半音)�??????
		 {1,1,3,5,5,  6,1,6,5,5,  3,3,5,3,1,  6,1,2,5,
		  6,6,6,1,5,  3,2,3,2,1,  2,5,5,4,5,  6,6,7,6,5,
		  1,1,6,1,    5,5,6,5,    6,6,5,3,    2,2,3,5,
		  1,1,1,3,    2,3,2,1,    6,6,5,3,    2,2,3,1,
		  1,1,0,      6,6,0,      5,5,6,5,    2,2,3,5,
		  1,1,0,      6,6,0,      5,5,6,5,    2,3,2,1,
		  -1}, // 基本音符

		 {2,1,1,2,2,  2,1,1,2,2,  2,2,1,1,2,  2,1,1,4,
		  2,2,1,1,2,  2,2,1,1,2,  2,2,1,1,2,  2,2,1,1,2,
		  3,1,2,2,    2,1,1,4,    3,1,2,2,    2,1,1,2,
		  3,1,2,2,    2,1,1,4,    3,1,2,2,    2,1,1,4,
		  2,4,2,      2,4,2,      2,2,2,2,    2,1,1,4,
		  2,4,2,      2,4,2,      2,2,2,2,    2,1,1,4,
		  -1},// 几拍

		 {0,0,0,0,0,  0,1,0,0,0,  0,0,0,0,0, -1,0,0,0,
		  0,0,0,1,0,  0,0,0,0,0,  0,0,0,2,0,  0,0,0,0,0,
		  1,1,0,1,    0,0,0,0,    0,0,0,0,    0,0,0,0,
		  0,0,0,0,    0,0,0,0,    0,0,0,0,    0,0,0,0,
		  1,1,0,      0,0,0,      0,0,0,0,    0,0,0,0,
		  1,1,0,      0,0,0,      0,0,0,0,    0,0,0,0,
		  -1} //升降�??????? +1�???????8度，+2升半音，+3升八度且升半�??????,-1�???????8度，-2降半�???????
	},
	{ // 明明�??????
		 {280, 0},
		 {3,3,4,3,6,1,2,7,0,3,3,4,3,3,4,5,6,0,3,3,4,3,6,6,0,3,6,6,7,1,0,
		  3,3,4,3,6,1,2,7,0,3,3,4,5,6,7,5,7,6,0,3,4,3,6,6,6,7,2,1,6,6,7,6,6,
		  -1}, // 基本音符

		 {2,2,2,2,1,3,1,2,1,2,2,2,2,1,3,1,2,1,2,2,2,2,1,3,2,2,1,2,2,9,2,
	      2,2,2,2,1,3,1,2,1,2,2,2,2,1,3,1,2,3,2,1,1,2,1,3,1,1,2,1,2,1,1,1,16,
		  -1},// 几拍

		 {1,1,3,1,0,3,1,0,0,1,1,3,1,0,2,2,0,0,1,1,3,1,0,0,0,1,0,0,0,3,0,
	      1,1,3,1,0,3,1,0,0,1,1,3,3,1,1,3,1,1,0,1,3,1,0,0,0,0,1,3,0,0,0,0,0,
		  -1} //升降�??????? +1�???????8度，+2升半音，-1�???????8度，-2降半�???????
	},
	{ // 十年
		 {280, -3},
		 {1,2,5,3,  3,3,4,5,6,  2,2,4,3,2,  1,3,2,1,7,  1,6,6,0,1,7,6,  7,4,3,2,7,1,  0,1,2,3,  6,3,4,5,3,  2,
		  1,2,5,3,  3,3,4,5,6,  2,2,4,3,2,  1,3,2,1,7,  1,6,6,0,1,7,6,  7,4,3,2,3,2,  1,0,1,2,3,  6,6,3,1,2,1,7,1,
		  -1}, // 基本音符

		 {1,1,1,1,  3,2,1,1,1,  2,3,1,1,1,  3,2,1,1,1,  1,1,2,1,1,1,1,  2,1,1,2,1,3,  1,1,1,1,  3,1,2,1,3,  2,
		  1,1,1,1,  3,2,1,1,1,  3,2,1,1,1,  3,2,1,1,1,  1,1,2,1,1,1,1,  2,1,1,2,1,2,  3,1,1,1,1,  3,1,2,1,7,1,1,4,
		  -1},// 几拍

		 {0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,-1,0,0,-1,  0,-1,-1,0,0,-1,-1,  -1,0,0,0,-1,0,  0,0,0,0,  0,0,0,0,0,  0,
		  0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,-1,0,0,-1,  0,-1,-1,0,0,-1,-1,  -1,0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,0,-1,0,
		  -1} //升降调 +1升8度，+2升半音，-1降8度，-2降半音
	},
	{ // Big Big World
		 {480, 0},
		 {1,2,  3,3,3,3,4,     2,2,2,2,2,3,  1,1,1,1,2,  3,2,2,1,2,  3,3,3,3,4,        2,2,2,2,3,           3,2,1,0,0,      0,3,2,2,3,3,1,2,
		  3,5,3,5,3,5,3,5,     0,3,4,3,2,    2,3,2,1,    0,1,1,      3,2,2,1,2,3,      0,3,2,2,1,           2,2,1,2,3,2,3,  0,2,2,2,1,6,1,    3,2,2,1,  1,2,3,  1,2,
		        3,3,3,3,4,     2,2,2,2,2,3,  1,1,1,1,2,  3,2,2,1,2,  3,5,3,2,3,2,3,2,  0,3,2,1,6,5,5,2,     1,1,0,0,        3,2,1,
		  -1}, // 基本音符

		 {2,2,  4,4,4,2,2,     4,4,2,2,2,2,  4,4,4,2,2,  6,2,4,2,2,  4,4,4,2,2,        4,4,4,2,2,           2,4,2,4,4,      2,1,3,2,1,2,2,2,
		  2,2,2,2,2,2,2,2,     4,2,2,2,6,    2,4,2,8,    4,2,10,     2,2,1,1,4,6,      4,2,2,2,6,           2,4,3,1,1,1,4,  4,2,2,1,1,2,3,    1,1,1,1,  2,2,4,  2,2,
		        4,4,4,2,2,     4,4,2,2,2,2,  4,4,4,2,2,  6,2,4,2,2,  6,4,1,1,1,1,1,5,  2,2,1,1,1,1,2,2,     6,2,4,4,        6,6,20,
		  -1},// 几拍

		 {0,0,  0,0,0,0,0,     0,0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,        0,0,0,0,0,           0,0,0,0,0,      0,0,0,0,0,0,0,0,
		  0,-1,0,-1,0,-1,0,-1, 0,0,0,0,0,    0,0,0,0,    0,0,0,      0,0,0,0,0,0,      0,0,0,0,0,           0,0,0,0,0,0,0,  0,0,0,0,0,-1,0,   0,0,0,0,  0,0,0,  0,0,
		        0,0,0,0,0,     0,0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,0,0,0,  0,0,0,0,-1,-1,-1,0,  0,0,0,0,        0,0,0,
		  -1} //升降调 +1升8度，+2升半音，-1降8度，-2降半音
	},
};

int play_mode = 0;
int output_device = OUTPUT_BUZZER;
int current_note = 0;
int key0_long_pushed = 0;
int sounding_buffer = 0; // 1 needs to sound
int pausing = 0;
double speed = 1.0;
int tone_switching = 0;
uint8_t data_buff[1];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

void setPWM(double freq, int duty_percent)
{
	if (freq == 0) {
		TIM3->CCR1 = 0;
		return ;
	}
	TIM3->ARR = 100000.0 / freq - 1;
	TIM3->CCR1 = (double)duty_percent * (TIM3->ARR + 1) / 100.0;
}

double note_to_frequency(int note)
{
	return 440.0 * pow(2, (note - 69)/12.0);
}

void produce_sound(int note, int lasting_millisecond)
{
	if (output_device == OUTPUT_BUZZER) {
		if (note == 0) {
			setPWM(0, 100);
			HAL_Delay(lasting_millisecond);
			return ;
		}
		double freq = note_to_frequency(note);
		setPWM(freq, DEFAULT_DUTY);
		HAL_Delay(lasting_millisecond);
		setPWM(0, 100);
		HAL_Delay(50);
	}
	else {
		// send MIDI
		if (note == 0) {
			HAL_Delay(lasting_millisecond);
			return ;
		}
		unsigned char operation;
		unsigned char sound;
		unsigned char force;
		operation = 0x90;
		sound = note;
		force = 0x7F;
		HAL_UART_Transmit(&hlpuart1, &operation, 1, 0xffff);
		HAL_UART_Transmit(&hlpuart1, &sound, 1, 0xffff);
		HAL_UART_Transmit(&hlpuart1, &force, 1, 0xffff);
		HAL_Delay(lasting_millisecond);
		operation = 0x80;
		force = 0x00;
		HAL_UART_Transmit(&hlpuart1, &operation, 1, 0xffff);
		HAL_UART_Transmit(&hlpuart1, &sound, 1, 0xffff);
		HAL_UART_Transmit(&hlpuart1, &force, 1, 0xffff);
		HAL_Delay(50);
	}
}

void init_piano()
{
	produce_sound(0, 1000);
	unsigned int i;
	for (i=0;i<8;i++) {
		produce_sound(MIDDLE_C + white_note[i], 200);
	}
	produce_sound(0, 0);
}

void init_walkman()
{
	pausing = 0;
	speed = 1.0;
//	tone_switching = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // Keys interrupt
{
	if (GPIO_Pin == GPIO_PIN_8) {
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) {
			// switch mode
			play_mode = (play_mode + 1) % TOTAL_MODES;
		}
		return ;
	}

	if (play_mode == PIANO_MODE) {
		current_note = MIDDLE_C;
		switch (GPIO_Pin) {
		case GPIO_PIN_1: current_note += white_note[0]; break;
		case GPIO_PIN_2: current_note += white_note[1]; break;
		case GPIO_PIN_3: current_note += white_note[2]; break;
		case GPIO_PIN_4: current_note += white_note[3]; break;
		case GPIO_PIN_5: current_note += white_note[4]; break;
		case GPIO_PIN_6: current_note += white_note[5]; break;
		case GPIO_PIN_7: current_note += white_note[6]; break;
		}
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)) {
			// big button pushed down
			current_note += 12;
		}
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0) {
			// small button pushed down
			current_note -= 12;
		}
		sounding_buffer = 1;
	}
	else {
		// switch the walkman
		switch (GPIO_Pin) {
		case GPIO_PIN_1: pausing = !pausing; break;
		case GPIO_PIN_2:  break;
		case GPIO_PIN_3: speed += 0.25; break;
		case GPIO_PIN_4: speed -= 0.25; break;
		case GPIO_PIN_5: tone_switching++; break;
		case GPIO_PIN_6: tone_switching--; break;
//		case GPIO_PIN_7: tone += 12; break;
//		case GPIO_PIN_8: tone -= 12; break;
		}
	}
	int i;
	for (i=0;i<0x3fff;i++);
}



void play_music(const int* pnote, const int* pbeat, const int* ptone,
		int bpm, int tone_shift)
{
	int i;
	int init_mode = play_mode;
	int init_tone = tone_switching;
	if (init_mode == 0)
		return ;
	for (i=0;pnote[i]!=-1 && play_mode == init_mode && tone_switching==init_tone ;i++) {
		while (pausing) ;
		int note = pnote[i]>0?(white_note[pnote[i]-1] + MIDDLE_C):0;
		switch (ptone[i]) {
		case 1:  note += 12; break;
		case 3:  note += 13; break;
		case -1: note -= 12; break;
		case 2:  note++; break;
		case -2: note--; break;
		case -3: note -= 13; break;
		}
		if (note != 0)
			note += tone_switching;
		produce_sound(note, 60*1000*pbeat[i]/bpm/speed);
	}
}
int key0_last_status = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //定时器的定时回调函数
{
	if (htim->Instance==TIM4) //确定�??? TIM4 引起的中�???
	{
		if (key0_last_status && KEY0) {
			// long pressed
			key0_long_pushed = 1;
			output_device = !output_device;
			while (KEY0);
		}
		key0_last_status = KEY0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    switch (data_buff[0]) {
	case 0x01: pausing = !pausing; break;
	case 0x02:  break;
	case 0x03: speed += 0.25; break;
	case 0x04: speed -= 0.25; break;
	case 0x05: tone_switching++; break;
	case 0x06: tone_switching--; break;
	case 0x07: tone_switching+=12; break;
	case 0x08: tone_switching-=12; break;
	case 0x10: play_mode = 1; break;
	case 0x11: play_mode = 2; break;
	case 0x12: play_mode = 3; break;
	//		case GPIO_PIN_7: tone += 12; break;
	//		case GPIO_PIN_8: tone -= 12; break;
	}
  while(HAL_UART_Receive_IT(&huart1, data_buff, 1) != HAL_OK); // Wait completly receive 1 byte data, and put data in rDataBuffer
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //启动定时器TIM3通道1的PWM输出
  HAL_TIM_Base_Start_IT(&htim4);
  //定义函数
  setPWM(0, DEFAULT_DUTY);
  HAL_UART_Receive_IT(&huart1,data_buff, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (play_mode == PIANO_MODE) {
		  init_piano();
		  while (play_mode==PIANO_MODE) {
			  if (sounding_buffer) {
				  produce_sound(current_note, 500);
				  sounding_buffer = 0;
			  }
		  }
	  }
	  init_walkman();
	  HAL_Delay(1000);
	  play_music(score[play_mode][1], score[play_mode][2], score[play_mode][3],
			  score[play_mode][0][0], score[play_mode][0][1]);
	  // score[music_num(SONG_MODE_1/2/...)][note/beat/tone]
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  htim3.Init.Prescaler = 1699;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
