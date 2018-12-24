/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define START_TIME  3
#define TRANSFER_WAIT_TIME  5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
#pragma import(__use_no_semihosting)                  
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
  
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{ 	
	while((UART5->ISR&0X40)==0);  
	UART5->TDR=(uint8_t)ch;      
	return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t RxBuf5[USART_DMA_RX_BUFFER_MAXIMUM];
extern uint8_t RxBuf[USART_DMA_RX_BUFFER_MAXIMUM];
extern uint8_t RxBuf3[USART_DMA_RX_BUFFER_MAXIMUM];
extern uint8_t DeveloperLinkedIn;
extern uint8_t FirmwareTransferBegin;
extern uint8_t FirmwareTransferEnd;
extern uint32_t FirmwareSize;
uint32_t NeedUpdatingFlag = 0;
uint8_t  ReceiveFinishFlag = 0;
char *logo = 
 "---------/+syyyys/------------\n"
"----:+ymNdyo//:/sdNy/---------\n"
"--:yNd+-          -yMh/-------\n"
"-:Nm- /hyyyo`    `+o:sNdo:----\n"
"-oM+ sh`   -y    /mNo `/yNNmy:\n"
"-+Ms sy                  /MMM+\n"
"--Nm  +o`          -+-   -hMd-\n"
"--yM:           .::/+hNmNmy/--\n"
"--oMo         .mNhyyyso+:-----\n"
"--/My         yM+-------------\n"
"--:Mh         hM:-------------\n"
"--/Mh         yM/-------------\n"
"--/My         /Ms-------------\n"
"--+Mo          dN/------------\n"
"--sM+          `mN/-----------\n"
"--yM:           .mN:----------\n"
"--dM.     /:     .Nm:---------\n"
"--NN     :h    /` :Mh---------\n"
"-/My     d.    m`  oMo--------\n\n";
/*
char *logo = "------------------------------\n"
						 "    ___   ___     ________ \n"
						 "   /  |  /  |    / _______)\n"
						 "  / /|| //| |   / / _____  \n"
					   " / / ||// | |  / /___/  /  \n"
					   "/_/  |_/  |_| (________/   \n\n";
*/
char *name = "MONGOOSE Bootloader";
char *ver  = "1.0"; 

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_UART5_Init();
 // MX_USART1_UART_Init();
 // MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	DeveloperLinkedIn = 0;
	
	printf("%s- %s %s -\n",logo,name,ver);
	printf("-System Started.\n");
	printf("-[Type in 'CM' to enter setup]\n");
	for(int i=1;i<=START_TIME*10;i++)
	{	
		printf("Waiting for developer command...%d s",(START_TIME*10-i)/10+1);		
		if(i%4==0) printf("/");
		else if(i%3==0) printf("-");
		else if(i%2==0) printf("\\");
		else printf("|");		
		
		if(DeveloperLinkedIn == 1)
		{
			printf("\n-Developer Linked In.\n");
			break;
		}
		printf("\r");
		HAL_Delay(100);
	}
	if(DeveloperLinkedIn == 1)
	{
		printf("-[Type in 'BG' to start firmware transfer]\n");
		int m=0;
		for(int i=1;i<=TRANSFER_WAIT_TIME*10;i++)
		{
			m=i;
			uint8_t WaitingOverTime=0;
			printf("Wating for firmware transfer start...%d s",(TRANSFER_WAIT_TIME*10-i)/10+1);
			if(i%4==0) printf("/");
			else if(i%3==0) printf("-");
			else if(i%2==0) printf("\\");
			else printf("|");	
			if(FirmwareTransferBegin == 1)
			{
				printf("\nReady for firmware transferring...\n");
			}
			while(FirmwareTransferBegin == 1)
			{	
				static uint32_t oldsize = 0xFF;
				static uint32_t i=0;
				if(FirmwareSize>10)
					oldsize = FirmwareSize;
				float fSize = (float)FirmwareSize/1024;
				printf("Data received: %.2f kb.",fSize);				
				HAL_Delay(20);
				i++;
				if(i >= 500 && fSize == 0.0) 
				{
					WaitingOverTime=1;
					printf("\n-Over waiting time...\n");
					break;
				}
				int j=i/5;
				if(j%4==0) printf("/");
				else if(j%3==0) printf("-");
				else if(j%2==0) printf("\\");
				else printf("|");	
			
				printf("\r");
				if(oldsize == FirmwareSize)
				{
					FirmwareTransferEnd=1;
					fSize = (float)(FirmwareSize)/1024;
					printf("-Data received: %.2f kb.\n",fSize);
					printf("-Transfer ended.\n");
					static uint16_t sum=0;
					for(int j=0;j<FirmwareSize-2;j++)
					{						
						sum+=RxBuf5[j];
					}
					printf("-CheckSum=0x%x\n",sum);
					break;							
				}
			}
			if(FirmwareTransferEnd == 1 || WaitingOverTime==1 ) break;
			printf("\r");
			HAL_Delay(100);
		}
		if(m==TRANSFER_WAIT_TIME*10)
		{
			printf("\n-Over waiting time...\n");
		}
	}
	else
		printf("\n-No developer linked.\n");
	
	if(FirmwareTransferEnd == 1)
	{		
		HAL_Delay(20);
		//TODO: Checking 'CheckSum' value equals.
		iap_write_appbin(USER_FLASH_FIRST_PAGE_ADDRESS,RxBuf5,FirmwareSize-2);
		printf("-Firmware Updated Successfully.\n");
		HAL_Delay(20);
		printf("Jumping to main program.\n");
		HAL_Delay(20);
		iap_load_app(USER_FLASH_FIRST_PAGE_ADDRESS);
	}
	
	printf("\nChecking firmware updating files in flash...\n");
	STMFLASH_Read(NEED_FOR_UPDATE_ADDR,&NeedUpdatingFlag,1);
	if(NeedUpdatingFlag == 1)
	{
		printf("-New Firmware Founded.\n");
		//loadApp();
		//checkFirmwareValid();
		printf("-Firmware Updated Successfully.\n");
	}
	else
	{
		printf("-No Available Firmware to be updated.\n");
		printf("Jumping to main program...\n");
		HAL_Delay(20);
		iap_load_app(USER_FLASH_FIRST_PAGE_ADDRESS);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		printf("-No Valid Program. (MONGOOSE-Bootloader-v1.0.)\n");
		HAL_Delay(5000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
	//__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	__HAL_UART_DISABLE_IT(&huart5,UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_RXNE);	
	//HAL_UART_Receive_DMA(&huart5, (uint8_t *)RxBuf5, USART_DMA_RX_BUFFER_MAXIMUM);
  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)RxBuf, USART_DMA_RX_BUFFER_MAXIMUM);
  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)RxBuf3, USART_DMA_RX_BUFFER_MAXIMUM);
  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RunningLed_GPIO_Port, RunningLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FaultLed_GPIO_Port, FaultLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RunningLed_Pin */
  GPIO_InitStruct.Pin = RunningLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RunningLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FaultLed_Pin */
  GPIO_InitStruct.Pin = FaultLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FaultLed_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
