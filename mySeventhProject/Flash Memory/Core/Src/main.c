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
#include <stdio.h>
#include "string.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define READ 0x03
#define SECTOR_4KB_ERASE 0x20
#define CHIP_ERASE 0x60
#define BYTE_PROGRAM 0x02
#define RDSR 0x05
#define EWSR 0x50
#define WRSR 0x01
#define WREN 0x06
#define READ_ID 0x90

#define START_ADDRESS 0x00
#define MAX_SECTORS 0x1ff
#define MAX_SIZE_SECTOR 0xfff
#define MAX_SIZE 0x1fffff

#define ENABLE_WRITE_TO_CHIP 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

enum {
	Read,
	Write
};

enum {
	CE_ON,
	CE_OFF
};

const uint8_t *time_capsule[] =
		{
				"From: Roman Muravka, dinaroma5323@gmail.com\r",
				"Mentor: Vladyslav Kotsiurba, iDontKnow@gmail.com\r",
				"Date: 28.11.2021\r", "TIME CAPSULE\r",
				">>> Music that gave me strength in 2021 <<<\r",
				"Imagine Dragons - Radioactive\r",
				"Miyagi & Andy Panda - Сartridge\r",
				"Eminem - Without Me\r",
				"Macklemore - Thrift Shop (feat. Wanz)\r",
				"10AGE - A gun\r", "10AGE - Zoo\r",
				"MiyaGi - Dear sing\r", "Scryptonite - Moscow loves ...\r",
				"Miyagi & Andy Panda - Fire Man\r", "IOWA - Simple song\r",
				"IOWA - 140\r", "Scryptonite and Ryda - Baby mama\r",
				"Scryptonite - Pets\r", "Captown - I taxi\r",
				"Captown - Various rubbish\r",
				"Eminem - The Real Slim Shady\r", "10AGE and Ramil - Au\r",
				"Miyagi - Malboro\r", "Eminem - Crazy In Love\r",
				"T-Fest and Scryptonite - Lambada\r",
				"I gave you a good mood, everything else is on you, go to the top without sparing your legs.\r",
				"Good luck!!!\r"
		};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
void TxRxSPI(uint8_t* tx, uint8_t* rx, uint32_t sizeTx, uint32_t sizeRx, _Bool WR);
void EraseMemory(void);
void ReadMemory(void);
void WriteToMemory(void);
void CE(_Bool State);
void isItBusy(void);
void EraseSector(void);
uint16_t scanUart(void);

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, 10);
	return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CE(_Bool State) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, State);
}

void TxRxSPI(uint8_t* tx, uint8_t* rx, uint32_t sizeTx, uint32_t sizeRx, _Bool WR) {
	uint8_t EnableWrite = WREN;
	if (WR == Write) {
		CE(CE_ON);
		HAL_SPI_Transmit(&hspi1, &EnableWrite, 1, 100);
		CE(CE_OFF);

		CE(CE_ON);
		HAL_SPI_Transmit(&hspi1, tx, sizeTx, 100);
		CE(CE_OFF);
	}
	else{
		CE(CE_ON);
		HAL_SPI_Transmit(&hspi1, tx, sizeTx, 100);
		HAL_SPI_Receive(&hspi1, rx, sizeRx, 100);
		CE(CE_OFF);
	}
}

void EraseMemory(void){
	uint8_t tx = CHIP_ERASE;
	TxRxSPI(&tx, NULL, 1, NULL, Write);
	isItBusy();
}

void EraseSector(void){
	uint8_t tx [4], choose;
	uint16_t sector_addr = 0x00;
	printf ("\r\n\nPlease enter address(DEC) 0..511: ");
	fflush(stdout);
	sector_addr = scanUart();
	if (sector_addr < 0x1ff) {
		printf("\r\nYou put in the number 0x%x(HEX), right?(y/n): ", sector_addr);
		fflush(stdout);
		choose = scanUart();
		if (choose == 'y' || choose == 'Y') {
			tx[0] = SECTOR_4KB_ERASE;
			tx[1] = (uint8_t) (sector_addr >> 4);
			tx[2] = (uint8_t) (sector_addr << 4);
			tx[3] = 0x00;
			TxRxSPI(&tx, NULL, 4, NULL, Write);
			isItBusy();
			printf ("\r\nSector 0x%x is clear\n", sector_addr);
		}
	}
	else
		printf("\n\r!!!WRONG NUMBER!!!\n");
}

void WriteToMemory(void){
	EraseMemory();
	uint8_t* string;
	uint8_t byteCommand[5] = { 0 };
	uint16_t sector_addr = 0x00;
	uint16_t address = 0x00;
	for (sector_addr = 0; sector_addr < ARRAY_SIZE(time_capsule); sector_addr++){
		string = time_capsule[sector_addr];
		for (address = 0; *string != '\0'; address++, string++){
			byteCommand[0] = BYTE_PROGRAM;
			byteCommand[1] = (uint8_t)(sector_addr >> 4);
			byteCommand[2] = (uint8_t)((sector_addr << 4) | (address >> 8));
			byteCommand[3] = (uint8_t)(address & 0xff);
			byteCommand[4] = *string;
			TxRxSPI(byteCommand, NULL, 5, NULL, Write);
			if (*string == '\r') {
				address++;
				byteCommand[0] = BYTE_PROGRAM;
				byteCommand[1] = (uint8_t) (sector_addr >> 4);
				byteCommand[2] = (uint8_t) (sector_addr << 4) | (address >> 8);
				byteCommand[3] = (uint8_t) (address & 0xff);
				byteCommand[4] = '\n';
				TxRxSPI(byteCommand, NULL, 5, NULL, Write);
			}
			if (address == MAX_SIZE_SECTOR){
				printf("The sector is full, the line may not be completed\r\n");
				break;
			}
			if (sector_addr == MAX_SECTORS){
				printf("The memory is full, the text may not be completed\r\n");
				break;
			}
		}
	}
}

void ReadMemory(void){
	uint8_t readData;
	uint16_t emptySector;
	uint16_t sector_addr = 0x00;
	uint16_t address = 0x00;
	uint8_t readCommand[4];

	for (sector_addr = 0; sector_addr <= MAX_SECTORS; sector_addr++){
		if (emptySector < 2){
		printf("\r\nsector #0x%x (%dKb) //", sector_addr, sector_addr * 4);
		fflush(stdout);
		}
		for (address = 0; address <= MAX_SIZE_SECTOR; address++){
			readCommand[0] = READ;
			readCommand[1] = (uint8_t) (sector_addr >> 4);
			readCommand[2] = (uint8_t) ((sector_addr << 4) | (address >> 8));
			readCommand[3] = (uint8_t) (address & 0xff);
			TxRxSPI(readCommand, &readData, 4, 1, Read);
			if(readData != 0xff){
				if (emptySector < 2){
					printf("%c", readData);
					emptySector = 0;
				}
				else
					switch(emptySector){
					case 2:
						printf("\r\nsector #0x%x (%dKb) //Empty", sector_addr - 1, (sector_addr - 1) * 4);
						printf("\r\nsector #0x%x (%dKb) //%c", sector_addr, sector_addr * 4);
						emptySector = 0;
						break;
					case 3:
						printf("\r\nsector #0x%x (%dKb) //Empty", sector_addr - 2, (sector_addr - 2) * 4);
						printf("\r\nsector #0x%x (%dKb) //Empty", sector_addr - 1, (sector_addr - 1) * 4);
						printf("\r\nsector #0x%x (%dKb) //%c", sector_addr, sector_addr * 4);
						emptySector = 0;
						break;
					default:
						printf("\r\nsector #0x%x...#0x%x //Empty", sector_addr - (emptySector - 2), sector_addr - 1);
						printf("\r\nsector #0x%x (%dKb) //%c", sector_addr, sector_addr * 4);
						emptySector = 0;
						break;
					}
			}
			else if(address == 0){
				if(emptySector < 2)
					printf("Empty!!!\r\n");
				emptySector++;
				break;
			}
			else
				break;
		}
	}
	if (emptySector > 2){
				printf("\r\nsector #0x%x...#0x%x //Empty!!!", MAX_SECTORS - (emptySector - 2), MAX_SECTORS);
			}
}

void isItBusy(void){
	uint8_t readCommand = RDSR;
	uint8_t readRegister = 0;
	do
	TxRxSPI(&readCommand, &readRegister, 1,1 , Read);
	while(readRegister & 0x01);
}



uint16_t scanUart(void) { //Универсальный ввод
	uint8_t enter = 0, scan[4] = { 0 }, i = 0, countLett = 0;
	uint16_t result = 0;

	for (int i = 0; i <= 4; i++)
		scan[i] = 0;

	/*Ввод значений с использованием Enter и Backspace*/
	while (enter != '\r') {
		HAL_UART_Receive(&huart3, &scan[i], 1, 10); //Ввод символа
		enter = scan[i]; //запись в переменную для проверки нажатие Enter или Backspace
		/*Проверка на ввод чего либо или нажатие Enter*/
		if (scan[i] == 0 || scan[i] == '\r')
			continue;
		/*Проверка на нажатие Backspace*/
		if (enter == '\b') {
			/*Проверка на наличие елементов для очистки*/
			if (i) {
				/*Стирание символа в консоли*/
				printf("\b \b");
				fflush(stdout);

				/*Очистка символа Backspace и стираемого символа*/
				scan[i--] = 0;
				scan[i] = 0;
			} else
				/*Очистка символа Backspace так как нет символа для очистки*/
				scan[i] = 0;
			continue;
		}
		printf("%c", scan[i]); //Вывод вводимого символа в консоль
		fflush(stdout);
		i++;
	}

	countLett = i - 1; //Сохранение количества вводимых символов

	/*Проверка на ввод цифр*/
	for (int i = 0, l = 0; i <= countLett; i++) {
		/*Если введена последовательность цифр и букв, то возращаем 0, если нет - Проверка*/
		if (scan[i] < '0' || scan[i] > '9') {
			if (l)
				result = 0;
			else if (scan[i + 1] == '\r') // Если после вводимого символа ничего нет - возращаем символ, есть - возращаем 0
				result = scan[i];
			else
				result = 0;
			goto Return;
		}
		l++;
	}
	/*Конвертация в целочисленные значения и последующий возврат*/
	for (i = 0; i <= countLett; i++) {
		scan[i] -= '0';
		result += (scan[i] * (pow(10, countLett - i)));
	}

	Return: return result;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART3_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(100);
	uint8_t choose = 0;
	uint8_t TxArray [2] = { 0 };

	CE(0);
	TxArray[0] = EWSR;
	HAL_SPI_Transmit(&hspi1, TxArray, 1, 100);
	CE(1);
	CE(0);
	TxArray[0] = WRSR;
	TxArray[1] = 0;
	HAL_SPI_Transmit(&hspi1, TxArray, 2, 100);
	CE(1);
	while (1) {
		printf("\r\n\n\nAction options:\r\n");
		printf("\t1 - Clear the memory\r\n");
		printf("\t2 - Write to memory\r\n");
		printf("\t3 - Read the contents of memory\r\n");
		printf("\t4 - Сlear memory sector(4Kb)\r\n\n");
		printf("\n\rYour choice(After entering, press ENTER): ");
		fflush(stdout);
		choose = scanUart(); //Ввод

		switch (choose) {
		case 1: //
			printf("\r\n\tAre you sure you want to clear the contents of memory(y/n)? ");
			fflush(stdout);
			choose = scanUart();
			if (choose == 'y' || choose == 'Y'){
				printf("\r\n\n\n...CLEARING MEMORY...\r\n");
				EraseMemory();
				printf("\r\n\n\n.../MEMORY IS CLEAR\\...\r\n");
			}
			break;

		case 2: //
			printf("\r\n\tAre you sure that you want to write new data, all data will be cleared before writing(y/n)? ");
			fflush(stdout);
			choose = scanUart();
			if (choose == 'y' || choose == 'Y'){
				printf("\r\n\n\n...WRITING TO MEMORY...\r\n");
				WriteToMemory();
				printf("\r\n\n\n.../RECORDING COMPLETED\\...\r\n");
			}
			break;

		case 3: //
			printf("\r\n\n\n...DATA READING STARTED...\r\n");
			ReadMemory();
			printf("\r\n\n\n.../MEMORY CONTENT READ\\...\r\n");
			break;

		case 4: //
			printf("\r\n\n\n...CLEARING SECTOR...\r\n");
			EraseSector();
			printf("\r\n\n\n.../SECTOR IS CLEAR\\...\r\n");
			break;

		default: //Неправильный ввод
			printf("\n\r!!!WRONG NUMBER!!!\n");
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin : PD7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
