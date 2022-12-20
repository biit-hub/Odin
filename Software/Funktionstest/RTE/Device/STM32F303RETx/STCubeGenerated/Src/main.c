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
#include "stdbool.h"
#include <stdio.h>
#include "ST7735/ST7735.h"
#include "ST7735/DefaultFonts.h"
#include "ST7735/bitmaps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTONS	(*((volatile uint32_t *) (0x48000800 + 0x10 + 1)))		// BASE Address PORT C + Offset for Input Register + Byte Shift (>> 8, to skip LEDs)
#define LEDS		(*((volatile uint32_t *) (0x48000800 + 0x14)))				// BASE Address PORT C + Offset for Output Register
#define ENTER_BUTTON	0x80
#define SKIP_BUTTON		0x40
#define FAIL_BUTTON		0x20
#define PREPERATION		255

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

enum state_e {
	LED_SWITCH_TEST = 0, 
	POTENTIOMETER_TEST, 
	RGB_TEST,
	USB_TEST, 
	EEPROM_TEST, 
	DISPLAY_TEST, 
	MATRIX_TEST,
	SEGMENT_TEST, 
	SHIFT_REGISTER_TEST, 
	SERVO_TEST,
	END_TEST
};

enum testStates_e {
	NONE,
	DONE,
	SKIPPED,
	FAILED
};

uint8_t testText[END_TEST][14] = {
	"LED+Schalter",
	"Potentiometer",
	"RGB LED",
	"USB",
	"EEPROM",
	"Display"
};

uint8_t testStates[END_TEST];
uint8_t uart_received;
uint8_t buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
bool led_switch_test(void);
uint8_t potentiometer_test(void);
void rgb_test(uint8_t);
void usb_transmit(uint8_t);
bool eeprom_test(uint16_t, uint8_t);
void display_init(void);
void display_test(void);
void test_init(bool);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	uint8_t state = PREPERATION;
	bool result;
	uint8_t readInput;
	
	for(uint8_t i=0; i<END_TEST; i++)
	{
		testStates[i] = NONE;
	}
	
	display_init();
	HAL_Delay(1000);
	
	test_init(true);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		switch(state)
		{
			case PREPERATION:
				lcd7735_print("Press all", 0, 125,0);
				lcd7735_print("Switches", 0, 135,0);
				state = LED_SWITCH_TEST;
				break;
			
			case LED_SWITCH_TEST:
				if(led_switch_test())
				{	
					// complete the Test
					testStates[LED_SWITCH_TEST] = DONE;
					test_init(false);
					
					// preperation for next Test
					state = POTENTIOMETER_TEST;
					lcd7735_print("S8 = Done", 0, 125,0);
					lcd7735_print("S7 = Skip", 0, 135,0);
					lcd7735_print("S6 = Failed", 0, 145,0);
				}
				break;
			
			case POTENTIOMETER_TEST:
				LEDS = potentiometer_test();
				readInput = BUTTONS;
				if(readInput & (ENTER_BUTTON | SKIP_BUTTON | FAIL_BUTTON))
				{
					// complete the Test
					if(readInput & ENTER_BUTTON)
					{
						testStates[POTENTIOMETER_TEST] = DONE;
					}
					else if(readInput & SKIP_BUTTON)
					{
						testStates[POTENTIOMETER_TEST] = SKIPPED;
					}
					else if(readInput & FAIL_BUTTON)
					{
						testStates[POTENTIOMETER_TEST] = FAILED;
					}
					test_init(false);
					HAL_Delay(100);
					while(BUTTONS & ENTER_BUTTON);
					
					// preperation for next Test
					LEDS = 0x00;
					lcd7735_print("S8 = Done", 0, 125,0);
					lcd7735_print("S7 = Skip", 0, 135,0);
					lcd7735_print("S6 = Failed", 0, 145,0);
					state = RGB_TEST;
				}
				break;
			
			case RGB_TEST:
				readInput = BUTTONS;
				rgb_test(readInput& 0x1F);
				if(readInput & (ENTER_BUTTON | SKIP_BUTTON | FAIL_BUTTON))
				{
					// complete the Test
					if(readInput & ENTER_BUTTON)
					{
						testStates[RGB_TEST] = DONE;
					}
					else if(readInput & SKIP_BUTTON)
					{
						testStates[RGB_TEST] = SKIPPED;
					}
					else if(readInput & FAIL_BUTTON)
					{
						testStates[RGB_TEST] = FAILED;
					}
					test_init(false);
					HAL_Delay(100);
					while(BUTTONS & ENTER_BUTTON);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
					
					// preperation for next Test
					lcd7735_print("S8 = Done", 0, 125,0);
					lcd7735_print("S7 = Skip", 0, 135,0);
					lcd7735_print("S6 = Failed", 0, 145,0);
					state = USB_TEST;
				}
				break;
			
			case USB_TEST:
				readInput = BUTTONS;
				usb_transmit(readInput & 0x1F);
				LEDS = uart_received;
				if(readInput & (ENTER_BUTTON | SKIP_BUTTON | FAIL_BUTTON))
				{
					// complete the Test
					if(readInput & ENTER_BUTTON)
					{
						testStates[USB_TEST] = DONE;
					}
					else if(readInput & SKIP_BUTTON)
					{
						testStates[USB_TEST] = SKIPPED;
					}
					else if(readInput & FAIL_BUTTON)
					{
						testStates[USB_TEST] = FAILED;
					}
					test_init(false);
					HAL_Delay(100);
					while(BUTTONS & ENTER_BUTTON);
					LEDS = 0x00;
					
					// preperation for next Test
					lcd7735_print("S8: Start Test", 0, 125,0);
					lcd7735_print("Automatic Test", 0, 135,0);
					lcd7735_print("              ", 0, 145,0);
					state = EEPROM_TEST;
				}
				break;
			
			case EEPROM_TEST:
				if(BUTTONS & ENTER_BUTTON)
				{
					LEDS = 0x00;
					while(BUTTONS & ENTER_BUTTON);
					
					result = eeprom_test(0x0000, 0xAA); // hand over only Switch 1 to 6, without Enter-Switch and Start-Switch
					if(result)
					{
						testStates[EEPROM_TEST] = DONE;
						test_init(false);
					}
					else
					{
						testStates[EEPROM_TEST] = FAILED;
						test_init(false);
					}
					state = DISPLAY_TEST;
					HAL_Delay(500);
				}
				break;
			
			case DISPLAY_TEST:
				display_test();
				
				readInput = BUTTONS;
				if(readInput & (ENTER_BUTTON | SKIP_BUTTON | FAIL_BUTTON))
				{
					// complete the Test
					if(readInput & ENTER_BUTTON)
					{
						testStates[DISPLAY_TEST] = DONE;
					}
					else if(readInput & SKIP_BUTTON)
					{
						testStates[DISPLAY_TEST] = SKIPPED;
					}
					else if(readInput & FAIL_BUTTON)
					{
						testStates[DISPLAY_TEST] = FAILED;
					}
					test_init(true);
					HAL_Delay(100);
					while(BUTTONS & ENTER_BUTTON);
					
					// preperation for next Test
					lcd7735_print("S8 = Done", 0, 125,0);
					lcd7735_print("S7 = Skip", 0, 135,0);
					lcd7735_print("S6 = Failed", 0, 145,0);
					state = MATRIX_TEST;
				}				
				break;
			
			case MATRIX_TEST:
				state = SEGMENT_TEST;
				break;
			
			case SEGMENT_TEST:
				state = SHIFT_REGISTER_TEST;
				break;
			
			case SHIFT_REGISTER_TEST:
				state = SERVO_TEST;
				break;
			
			case SERVO_TEST:
				state = END_TEST;
				break;
			
			case END_TEST:
				lcd7735_print("Test completed!", 0, 125,0);
				lcd7735_print("               ", 0, 135,0);
				lcd7735_print("               ", 0, 145,0);
				while(1);
				state = LED_SWITCH_TEST;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
	
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	UART_Start_Receive_DMA(&huart3, &uart_received, 1);
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_0_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin
                          |LED_4_Pin|LED_5_Pin|LED_6_Pin|LED_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IO_0_Pin|IO_1_Pin|IO_2_Pin|IO_3_Pin
                          |IO_4_Pin|IO_5_Pin|IO_6_Pin|IO_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_DISPLAY_Pin|CD_SDCARD_Pin|RESET_DISPLAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_5_Pin Button_6_Pin Button_7_Pin Button_0_Pin
                           Button_1_Pin Button_2_Pin Button_3_Pin Button_4_Pin */
  GPIO_InitStruct.Pin = Button_5_Pin|Button_6_Pin|Button_7_Pin|Button_0_Pin
                          |Button_1_Pin|Button_2_Pin|Button_3_Pin|Button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_0_Pin LED_1_Pin LED_2_Pin LED_3_Pin
                           LED_4_Pin LED_5_Pin LED_6_Pin LED_7_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin
                          |LED_4_Pin|LED_5_Pin|LED_6_Pin|LED_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IO_0_Pin IO_1_Pin IO_2_Pin IO_3_Pin
                           IO_4_Pin IO_5_Pin IO_6_Pin IO_7_Pin */
  GPIO_InitStruct.Pin = IO_0_Pin|IO_1_Pin|IO_2_Pin|IO_3_Pin
                          |IO_4_Pin|IO_5_Pin|IO_6_Pin|IO_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_DISPLAY_Pin CD_SDCARD_Pin RESET_DISPLAY_Pin */
  GPIO_InitStruct.Pin = CS_DISPLAY_Pin|CD_SDCARD_Pin|RESET_DISPLAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Read the Switches and mirror it to the LEDs. The function trace which Switch was set High.
 * 
 * @return true Every Switch was set once to High
 * @return false Not Every Switch was set once to High
 */
bool led_switch_test(void)
{
	static uint8_t mask = 0;
	uint8_t buttons;
	
	buttons = BUTTONS;
	LEDS = buttons;
	
	mask |= buttons;
	if(mask == 0xFF)
	{
		while(buttons != 0x00)
		{
			buttons = BUTTONS;
			LEDS = buttons;
		}
		mask = 0x00;
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief Read the ADC the Potentiometer position and give back the upper 8 bits of the ADC Value
 * 
 * @return uint8_t The upper 8 bits of the ADC Value
 */
uint8_t potentiometer_test(void)
{
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 100);
	return HAL_ADC_GetValue(&hadc3) >> 4;
}

/**
 * @brief Controls the RGB LED over the parameter
 * 
 * @param inputParameter Controls the RGB LED mode
 * Bit 0: Controls Red LED (10% or off)
 * Bit 1:	Controls Green LED (10% or off)
 * Bit 2: Controls Blue LED (10% or off)
 * Bit 3: Set Brightness to 50%
 * Bit 4: Plays a Rainbow-Animation. This Bit ignors Bit 1-4
 * Bit 5: -
 * Bit 6: -
 * Bit 7: -
 */
void rgb_test(uint8_t inputParameter)
{
	uint16_t capture_value = 6553;	// set brightness to 10%
	static uint16_t step_cnt = 0;
	static uint16_t red = 65535;
	static uint16_t green = 0;
	static uint16_t blue = 0;
	
	if(inputParameter & 0x10)
	{
		// Animation
		switch(step_cnt/257)
		{
			case 0:
				green += 255;
				break;
			
			case 1:
				red -= 255;
				break;
			
			case 2:
				blue += 255;
				break;
			
			case 3:
				green -= 255;
				break;
			
			case 4:
				red += 255;
				break;
			
			case 5:
				blue -= 255;
			break;
		}
		
		
		TIM2->CCR1 = red;
		TIM2->CCR3 = green;
		TIM2->CCR4 = blue;
		
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		
		HAL_Delay(5);
		
		if(step_cnt == 1541)
		{
			step_cnt = 0;
		}
		else
		{
			step_cnt++;
		}
	}
	else
	{		
		// brightness
		if(inputParameter & 0x08)
		{
			capture_value = 32767;	// set brightness to 50%
		}
		
		// Red
		if(inputParameter & 0x01)
		{
			TIM2->CCR1 = capture_value;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		}
		
		// Green
		if(inputParameter & 0x02)
		{
			TIM2->CCR3 = capture_value;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		}
		
		// Blue
		if(inputParameter & 0x04)
		{
			TIM2->CCR4 = capture_value;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		}
	}	
}

/**
 * @brief Send Strings over the UART to the USB-Port
 * 
 * @param inputParameter Controls the UART Output. Is positiv edge triggerd.
 * Bit 0: Sends "Hello "
 * Bit 1:	Sends "World "
 * Bit 2: Sends "I'm Odin. "
 * Bit 3: Sends "Nice to meet you! "
 * Bit 4: Sends "\r\n"
 * Bit 5: -
 * Bit 6: -
 * Bit 7: - 
 */
void usb_transmit(uint8_t inputParameter)
{
	static uint8_t old_inputParameter = 0;
	uint8_t changes = (inputParameter ^ old_inputParameter) & inputParameter; 	// positiv edge detection

	switch(changes)
	{
		case 0x01:
			HAL_UART_Transmit(&huart3, "Hello ", 6, 100);
			break;
		
		case 0x02:
			HAL_UART_Transmit(&huart3, "World ", 6, 100);
			break;
		
		case 0x04:
			HAL_UART_Transmit(&huart3, "I'm Odin. ", 10, 100);
			break;
		
		case 0x08:
			HAL_UART_Transmit(&huart3, "Nice to meet you! ", 18, 100);
			break;
		
		case 0x10:
			HAL_UART_Transmit(&huart3, "\r\n", 2, 100);
			break;
	}
	
	old_inputParameter = inputParameter;
}

/**
 * @brief Test EEPROM. Write the input Parameter to the EEPROM, read the EEPROM and compare the parameter value with the read value.
 * 
 * @param eepromAddress EEPROM-Address for the test value
 * @param inputParameter Test value which is written into the EEPROM
 * @return true Test successful. Values are equal
 * @return false Test failed. Values are different
 */
bool eeprom_test(uint16_t eepromAddress, uint8_t inputParameter)
{
	uint8_t readValue;
	
	HAL_I2C_Mem_Write(&hi2c1, 0xA0, eepromAddress, I2C_MEMADD_SIZE_16BIT ,&inputParameter, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, 0xA0, eepromAddress, I2C_MEMADD_SIZE_16BIT ,&readValue, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	
	if(readValue == inputParameter)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief Initialize the display and shows the Logo
 * 
 */
void display_init(void)
{
	lcd7735_initR(INITR_REDTAB);
	lcd7735_setFont((uint8_t *)&SmallFont[0]);
	lcd7735_drawBitmap(0,0,128,160,(bitmapdatatype)Odin_160x128,1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	TIM3->CCR1 = 65535;
}

/**
 * @brief Test Display. Colors the display in Red, Green and Blue and shows informations
 * 
 */
void display_test(void)
{
	const uint32_t delayTime = 1000;
	static uint8_t state = 0;
	static uint32_t time;
	
	switch(state)
	{
		case 0:
			lcd7735_fillScreen(ST7735_RED);
			lcd7735_print("S8 = Done", 0, 125,0);
			lcd7735_print("S7 = Skip", 0, 135,0);
			lcd7735_print("S6 = Failed", 0, 145,0);
			time = HAL_GetTick();
			state++;
			break;
		
		case 1:
			if((time + delayTime) < HAL_GetTick())
			{
				state++;
			}
			break;
			
		case 2:
			lcd7735_fillScreen(ST7735_GREEN);
			lcd7735_print("S8 = Done", 0, 125,0);
			lcd7735_print("S7 = Skip", 0, 135,0);
			lcd7735_print("S6 = Failed", 0, 145,0);
			time = HAL_GetTick();
			state++;
			break;
		
		case 3:
			if((time + delayTime) < HAL_GetTick())
			{
				state++;
			}
			break;
			
		case 4:
			lcd7735_fillScreen(ST7735_BLUE);
			lcd7735_print("S8 = Done", 0, 125,0);
			lcd7735_print("S7 = Skip", 0, 135,0);
			lcd7735_print("S6 = Failed", 0, 145,0);
			time = HAL_GetTick();
			state++;
			break;
		
		case 5:
			if((time + delayTime) < HAL_GetTick())
			{
				state = 0;
			}
			break;			
	}
}

/**
 * @brief Initialize the test screen with the test results
 * 
* @param fillScreen if true, than the Background will b redrawn
 */
void test_init(bool fillScreen)
{
	uint8_t start_y = 5;
	const uint8_t line_hight = 12;
	const uint8_t text_start_x = 12;
	const uint8_t icon_start_x = 0;
	uint8_t line_y = start_y;
	
	if(fillScreen)
	{
		lcd7735_fillScreen(ST7735_BLACK);
	}
	
	for(int i=0; i < END_TEST; i++)
	{
		switch(testStates[i])
		{
			case NONE:
				break;
			case DONE:
				lcd7735_drawBitmap(icon_start_x,line_y,10,10,(bitmapdatatype)complete_10x10,1);
				break;
			case SKIPPED:
				lcd7735_drawBitmap(icon_start_x,line_y,10,10,(bitmapdatatype)skip_10x10,1);
				break;
			case FAILED:
				lcd7735_drawBitmap(icon_start_x,line_y,10,10,(bitmapdatatype)cancel_10x10,1);
			break;
		}

		lcd7735_print(testText[i],text_start_x,line_y,0);
		
		line_y += line_hight;
	}	
}

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
