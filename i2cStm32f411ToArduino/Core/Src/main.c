/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include <i2c1.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */






#define GPIOBEN__RCC_AHB1ENR        0x01



#define RCC_BASE_ADDR 0x40023800
#define GPIOB_BASE_ADDR 0x40020400
#define I2C1_BASE_ADDR 0x40005400




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//AHB bus


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void blinkk(void);
void blinkk2(void);

///////////

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
  /* USER CODE BEGIN 2 */

  //enablePinToDebug_portD();




///test case for i2c master read and master write///
#if 0
    i2c1Config("master");
  	HAL_Delay(500);
	uint8_t data1[8] = {0x54, 0x56, 0x98, 0x65};
	uint8_t data[8] = {0};
	uint8_t addrr = 0x04;
	HAL_Delay(1000);

	i2c1MasterWrite(addrr, 4, data1);

	HAL_Delay(2000);

	i2c1MasterRead(addrr | 0x01, 5, data);  //OR with 0x01 indicated that is reading address


	HAL_Delay(1000);

	i2c1MasterWrite(addrr, 4, data1);
#endif
//////////////////////////////////////////////////////


///test case for i2c slave write///
#if 1
	i2c1Config("slave"); //need for both read and write
	uint8_t sentData[20] = {0x78, 0x79, 0x80, 0x81, 0x82, 0x83};
	uint8_t received[20] = {0};
	i2c1SlaveSendBytes(sentData);
#endif
//////////////////////////////////////////////////////

/////tetst case for lcd-i2c///////////////////////////
#if 0
  i2c1SendCmd(0x02);  //return home - forced command for 4-bit mode lcd
  HAL_Delay(5);
  i2c1SendCmd(0x28);  //set interface 4-bit, 5x7
  HAL_Delay(5);
  i2c1SendCmd(0x0e);  //turn on display with cursor(cursor isn't blink)
  HAL_Delay(5);

  i2c1SendCmd(0x01);  //clear display
  HAL_Delay(5);

  i2c1SendData(0x61);
  HAL_Delay(5);
  //i2c1SendCmd(0x14);  //shift cursor only ; 0x1c shift display and cursor
  HAL_Delay(5);

  //i2c1SendCmd(0x80|0x01);  //set cursor to begin of second line, 0x80 indicate that setting SDRAM address
  HAL_Delay(5);

  gotoXYLcd(0,1);
  //i2c1SendCmd(0x0e);
  HAL_Delay(5);
  i2c1SendData('B');
  HAL_Delay(5);



  char* str = NULL;
  sprintf(str, "what us %d", 0x65);
  i2c1SendString(str);

  printf("check value");

  HAL_Delay(5);
  i2c1SendString("hello this\nis");
   HAL_Delay(5);
#endif
//////////////////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//// test case for receiver byte or multi bytes///
#if 0
	  i2c1SlaveReceiveBytes(received);
		if(received[0] == 0x97)
		{
			blinkk();
		}
		if(received[1] == 0x94)
		{
			blinkk2();
		}
#endif
/////////////////////////////////////////////////////
		//HAL_Delay(100);
	  //blinkk();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */










void blinkk(void)
{
	uint32_t *GPIOx_ODR = (uint32_t*)(0x40020C00 + 0x14);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 15);
	*GPIOx_ODR &= ~(1 << 14);
	*GPIOx_ODR &= ~(1 << 13);
	*GPIOx_ODR &= ~(1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR |= (1 << 15);
	*GPIOx_ODR |= (1 << 14);
	*GPIOx_ODR |= (1 << 13);
	*GPIOx_ODR |= (1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 15);
	*GPIOx_ODR &= ~(1 << 14);
	*GPIOx_ODR &= ~(1 << 13);
	*GPIOx_ODR &= ~(1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR |= (1 << 15);
	*GPIOx_ODR |= (1 << 14);
	*GPIOx_ODR |= (1 << 13);
	*GPIOx_ODR |= (1 << 12);
	HAL_Delay(500);
}


void blinkk2(void)
{
	uint32_t *GPIOx_ODR = (uint32_t*)(0x40020C00 + 0x14);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 15);
	*GPIOx_ODR &= ~(1 << 14);
	*GPIOx_ODR &= ~(1 << 13);
	*GPIOx_ODR &= ~(1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR |= (1 << 15);
	*GPIOx_ODR |= (1 << 14);
	//*GPIOx_ODR |= (1 << 13);
	//*GPIOx_ODR |= (1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 15);
	*GPIOx_ODR &= ~(1 << 14);
	*GPIOx_ODR |= (1 << 13);
	*GPIOx_ODR |= (1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR |= (1 << 15);
	*GPIOx_ODR |= (1 << 14);
	*GPIOx_ODR &= ~(1 << 13);
	*GPIOx_ODR &= ~(1 << 12);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 15);
		*GPIOx_ODR &= ~(1 << 14);
		*GPIOx_ODR |= (1 << 13);
		*GPIOx_ODR |= (1 << 12);
		HAL_Delay(500);
		*GPIOx_ODR |= (1 << 15);
		*GPIOx_ODR |= (1 << 14);
		*GPIOx_ODR &= ~(1 << 13);
		*GPIOx_ODR &= ~(1 << 12);
		HAL_Delay(500);
}

void testFunc(uint8_t* data)
{

	uint32_t *GPIOx_ODR = (uint32_t*)(0x40020C00 + 0x14);
	if(data[0] == 0x89)
	{

		*GPIOx_ODR |= (1 << 15);
	}
	if(data[1] == 0x99)
	{

		*GPIOx_ODR |= (1 << 14);
	}
	if(data[2] == 0xA9)
	{

		*GPIOx_ODR |= (1 << 13);
	}
	if(data[3] == 0xB9)
	{

		*GPIOx_ODR |= (1 << 12);
	}
	if(data[4] == 0xC9)
	{
		blinkk();
	}
	HAL_Delay(2000);

	*GPIOx_ODR |= (1 << 14);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 14);
	HAL_Delay(500);
	*GPIOx_ODR |= (1 << 14);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 14);
	HAL_Delay(500);
	*GPIOx_ODR |= (1 << 14);
	HAL_Delay(500);
	*GPIOx_ODR &= ~(1 << 14);
	HAL_Delay(500);
	HAL_Delay(1000);
	*GPIOx_ODR &= ~(1 << 15);

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