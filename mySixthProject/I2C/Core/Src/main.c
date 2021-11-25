/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c1control.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t conToChar [4] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

uint16_t scanUart(void) {
	uint8_t enter = 0, scan[5] = { 0 }, i = 0, countLett = 0;
	uint16_t result = 0;

	while (enter != '\r') {
		HAL_UART_Receive(&huart3, &scan[i], 1, 5000);
		enter = scan[i];
		if (scan[i] == 0 || scan[i] == '\r')
			continue;
		if (enter == '\b'){
			if (i){
				HAL_UART_Transmit(&huart3, &scan[i], 1, 10);
				HAL_UART_Transmit(&huart3, (uint8_t *)" ", 1, 10);
				HAL_UART_Transmit(&huart3, &scan[i], 1, 10);
				scan[i--] = 0;
 				scan[i] = 0;
			}
			else
				scan[i] = 0;
			continue;
		}
		HAL_UART_Transmit(&huart3, &scan[i], 1, 10);
		i++;
	}
	countLett = i - 1;

	for (int i = 0, l = 0; i <= countLett; i++) {
		if (scan[i] < '0' || scan[i] > '9') {
			if (l)
				result = 0;
			else
				result = scan[i];
			goto Return;
		}
		l++;
	}

	for (i = 0; i <= countLett; i++) {
		scan[i] -= '0';
		result += (scan[i] * (pow(10, countLett - i)));
	}


	Return:
	return result;
}

void convertToChar(uint16_t valForConv){
	uint8_t length;
	for (int i = 3; i >= 0; i--){
		 if((valForConv / pow(10, i)) != 0){
			 length = i + 1;
			 break;
		 }
	}

	for (int i = length; i != 0; i--) {
		conToChar[i - 1] = valForConv % 10;
		conToChar[i - 1] += '0';
		valForConv /= 10;
	}
}
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	uint8_t devId = 0x80, numberLed = 0, turn = 0;
	uint8_t TxBuff[8], choose = 0, cycle = 5;
	uint16_t freq = 25;
	;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	setFreq(freq);
	setDuty(cycle);
	TxBuff[0] = 0x00;
	TxBuff[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	TxBuff[0] = 0x00;
	TxBuff[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	TxBuff[0] = 0xFD;
	TxBuff[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	turn = 10;

	while (1) {
		switch(turn){
				case 0:
					HAL_UART_Transmit(&huart3, (uint8_t*) "\n\n\n\n\r!!!PWM OFF!!!",13 + 5, 10);
					break;
				case 1:
					HAL_UART_Transmit(&huart3, (uint8_t*) "\n\n\n\n\r!!!PWM ON!!!",12 + 5, 10);
					break;
				case 10:
					HAL_UART_Transmit(&huart3, (uint8_t*) "\n\n\n\n\r!!!PWM ON BUT NO LED SELECTED!!!",32 + 5, 10);
					break;
				}
		HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\tN - Turn on the PWM;",20 + 3, 10);
		HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\t1 - LED selection;(Currently: ", 30 + 3, 10);
		if(numberLed < 1 || numberLed > 16)
			switch (numberLed){
			case 0:
				HAL_UART_Transmit(&huart3, (uint8_t*) "no LED is on)\n\r", 13 + 2, 10);
				break;
			case 'A':
				HAL_UART_Transmit(&huart3, (uint8_t*) "All LED is on)\n\r", 18 + 2, 10);
				break;
			}
		else{
			convertToChar(numberLed);
			switch(numberLed){
			case 1:
				HAL_UART_Transmit(&huart3, conToChar, strlen((uint8_t *)conToChar), 10);
				HAL_UART_Transmit(&huart3, (uint8_t*) "-st)\n\r", 4 + 2, 10);
				break;
			case 2:
				HAL_UART_Transmit(&huart3, conToChar, strlen((uint8_t *)conToChar), 10);
				HAL_UART_Transmit(&huart3, (uint8_t*) "-nd)\n\r", 4 + 2, 10);
				break;
			case 3:
				HAL_UART_Transmit(&huart3, conToChar, strlen((uint8_t *)conToChar), 10);
				HAL_UART_Transmit(&huart3, (uint8_t*) "-rd)\n\r", 4 + 2, 10);
				break;
			default:
				HAL_UART_Transmit(&huart3, conToChar, strlen((uint8_t *)conToChar), 10);
				HAL_UART_Transmit(&huart3, (uint8_t*) "-th)\n\r", 4 + 2, 10);
				break;
			}

		}

		HAL_UART_Transmit(&huart3, (uint8_t*) "\t2 - Change the PWM frequency;(Currently: ", 41 + 1, 10);
		convertToChar(freq);
		HAL_UART_Transmit(&huart3, conToChar, strlen((uint8_t *)conToChar), 10);
		HAL_UART_Transmit(&huart3, (uint8_t*) " Hz)\n\r", 4 + 2, 10);

		HAL_UART_Transmit(&huart3, (uint8_t*) "\t3 - Change Duty Cycle %;(Currently: ",36 + 1, 10);
		convertToChar(cycle);
		HAL_UART_Transmit(&huart3, conToChar, strlen((uint8_t *)conToChar), 10);
		HAL_UART_Transmit(&huart3, (uint8_t*) " %)\n\r", 3 + 2, 10);
		HAL_UART_Transmit(&huart3, (uint8_t*) "\tF - Turn off the PWM;",21 + 1, 10);
		HAL_UART_Transmit(&huart3, (uint8_t*) "\n\n\rYour choice(After entering, press ENTER): ", 42 + 3, 10);

		choose = scanUart();
		uint32_t temp;
		switch (choose) {
		case 1:
			HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\t1 - 1-st Led;", 13 + 3,10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\t2 - 2-nd Led;", 13 + 3,10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\t*\n\r\t*", 2 + 6, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\t16 - 16-th led;",15 + 3, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r\tA - All LEDs on.",18 + 3, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\n\rEnter LED number: ",18 + 2, 10);
			temp = scanUart();
			if ((temp >= 1 && temp <= 16) || (temp == 'A')){
				if (temp == 'A')
					setNumberLed(0xfa);
				numberLed = temp;
				setNumberLed(numberLed);
				turn = 1;
			}
			else
				HAL_UART_Transmit(&huart3,(uint8_t*) "\n\rWrong number!", 13 + 2, 10);
			break;

		case 2:
			HAL_UART_Transmit(&huart3,(uint8_t*) "\n\rEnter frequency(24-1526Hz): ", 28 + 2, 10);
			temp = scanUart();
			if (temp >= 24 && temp <= 1526){
				freq = temp;
				setFreq(freq);
			}
			else
				HAL_UART_Transmit(&huart3,(uint8_t*) "\n\rWrong number!", 13 + 2, 10);
			break;

		case 3:
			HAL_UART_Transmit(&huart3,(uint8_t*) "\n\rEnter a value for duty cycle changes(1-100%): ", 46 + 2, 10);
			temp = scanUart();
			if (temp >= 1 && temp <= 100){
				cycle = temp;
				setDuty(cycle);
			}
			else
				HAL_UART_Transmit(&huart3,(uint8_t*) "\n\rWrong number!", 13 + 2, 10);
			break;

		case 'N':
			turnON();
			turn = 1;
			break;

		case 'F':
			turnOFF();
			turn = 0;
			break;
		default:
			HAL_UART_Transmit(&huart3,(uint8_t*) "\n\rWrong number!", 13 + 2, 10);
			break;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
