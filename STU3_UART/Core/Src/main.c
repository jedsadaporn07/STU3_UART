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
#include <stdio.h>
#include <string.h>
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] =
{ 0 };
//uint16_t
uint8_t RxDataBuffer[32] =
{ 0 };
uint8_t temp_s[2]="Xu";
uint8_t temp_f[2]="Fn";
uint8_t Posdata;
int DataIn;
uint8_t M_state = 0;
uint8_t N_state = 0;
uint8_t chkStart;
uint8_t chkStart2;
uint8_t NameM;
uint8_t StartM;
uint8_t chkM = 0;
uint8_t dataFN = 0;
uint8_t Nstation;
uint8_t dataF1;
uint8_t dataF2;
uint8_t dataFSum;
uint8_t chksum;
uint8_t chksum1;
uint8_t chksum2;
uint8_t chksum3;
uint8_t countN = 0;


///////////////////// DMA//////////////////////////////
typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 =
{ 0 };

uint8_t MainMemory[255] =
{ 0 };

typedef enum
{
	DNMXP_idle,
	DNMXP_1stHeader,
	DNMXP_2ndHeader,
	DNMXP_3rdHeader,
	DNMXP_Reserved,
	DNMXP_ID,
	DNMXP_LEN1,
	DNMXP_LEN2,
	DNMXP_Inst,
	DNMXP_ParameterCollect,
	DNMXP_CRCAndExecute

} DNMXPState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void All_mode(uint8_t DataIn, UARTStucrture *uart);
void UARTRecieveIT();


//////////////////////////////////DMA//////////////////////////////////
void UARTInit(UARTStucrture *uart);

void UARTResetStart(UARTStucrture *uart);

uint32_t UARTGetRxHead(UARTStucrture *uart);

int16_t UARTReadChar(UARTStucrture *uart);

void UARTTxDumpBuffer(UARTStucrture *uart);

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);
  UARTReadChar(&UART2);
  //HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 9 ,1000);
  //HAL_UART_Transmit(&huart2, (uint8_t*)temp_f, 9 ,1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int16_t inputChar = UARTReadChar(&UART2);
	  		//if input char == -1 ==> No New data
	  		if (inputChar != -1)
	  		{
	  			dataFN += 1;
	  			for(int i = 0; i < dataFN; i++){
	  				All_mode(inputChar, &UART2);
	  			}

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}
uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}
int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;
	}

	return Result;

}
void All_mode(uint8_t DataIn, UARTStucrture *uart)
{
	N_state = uart->RxBuffer[uart->RxTail-1];
	switch (chkM){
		case 0:
			StartM = DataIn;
			chkStart = StartM >> 4;
			if (chkStart == 9){
				chkM = 1;
			}else{
				chkM = 0;
			}
			break;
		case 1:
			NameM = (StartM & 15);
			if (NameM >= 1 && NameM <= 14){
				chkM = 2;
			}else{
				chkM = 0;
			}
			break;
		case 2:
			switch (NameM){
				case 1: //10010001 01000000 00000000 00101110
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						M_state = 1;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						//HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,1000); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 2: //10010010 01101101
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 2;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						//HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,1000); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 3: // 10010011 01101100
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 3;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						//HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,1000); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 4:
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						M_state = 4;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 5:
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						M_state = 5;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 6:
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						M_state = 6;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 7:
					if (dataFN == 2){
						Nstation = DataIn;
					}
					if (dataFN < Nstation + 3){
						if (dataFN == countN + 3){
							dataFSum += DataIn;
							countN += 1;
						}
					}
					chksum = DataIn;
					chksum3 = ~(StartM + Nstation + dataFSum);
					if (chksum == chksum3){
						M_state = 7;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						dataFSum = 0;
						dataFN = 0;
						countN = 0;
						chkM = 0;
					}
					break;
				case 8: //10011000 01100111
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 8;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						HAL_Delay(1000);
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_f, 2);
						//HAL_UART_Transmit(&huart2, (uint8_t*)temp_f, 2 ,1000);//Fn
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 9: //10011001 01100110
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 9;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 10: //10011010 01100101
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 10;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 11: //10011011 01100100
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 11;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 12: //10011100 01100011
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 12;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						//HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,1000); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 13: //10011101 01100010
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 13;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						//HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,1000); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 14: //10011110 01100001
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						M_state = 14;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
						chkM = 0;
						dataFN = 0;
					}
					break;
			}
	}

}
void UARTRecieveIT()
{
//	static uint32_t dataPos =0;
//	int16_t data=-1;
	//HAL_UART_Receive_DMA(&huart2,RxDataBuffer,32);
	//HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);
//	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
//	{
//		data=RxDataBuffer[dataPos];
//		DataIn = data;
//		dataPos= (dataPos+1)%huart2.RxXferSize;
//		Posdata = dataPos;
//		dataFN += 1;
//
//	}
//	HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDataBuffer, 32);
//	if (dataPos == 32){
//		dataPos = 0;
//	}
//	DataIn=RxDataBuffer[dataPos];
//	dataPos += 1;
//	All_mode();

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);//s
	//HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
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

