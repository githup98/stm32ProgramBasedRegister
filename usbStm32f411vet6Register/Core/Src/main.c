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
#include "newUsb.h"

extern uint32_t usbDataReceived[12];
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t *pRCC_AHB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x34);


uint32_t check1;
uint32_t countInt = 0;

//get vendor ID from st/stm32cubeide_1.10.1./plugins/com.st.stm32cube.common.mx_6.8.1.202304191431/db/mcu/IP/USBX-v1.0.0_U5_Cube_Modes.xml file

#if 0
uint8_t ConfigurationDescriptor[9] =
{
	0x09, //bLength 1 byte
	0x02, //bDescriptorType 1 byte (force equal to 2)
	0x32, //50 wTotalLenght 2 byte, total device, config, interface, endpoint
	0x00,
	0x01, //bNumInterfaces 1 byte
	0x01, //bConfigurationValue 1 byte, is used to select configuration (multi)
	0x00, //iConfiguration 1 byte, index string -> no need
	0x40, //bmAttributes 1 byte, self-power (6th-bit enabled)
	0xc8, //bMaxPower 1 byte: 200 * 2 mil-amps, active with 7th-bit is enabled
};

uint8_t InterfaceDescriptor[9] =
{
	0x09, //bLength 1 byte
	0x04, //bDescriptorType 1 byte (force equal to 4)
	0x00, //bInterfaceNumber 1 byte index of interface desc (increase 1 for each added interface desc)
	0x00, //bAlternateSetting 1 byte
	0x02, //bNumEndpoints 1 byte, this value should exclude (mean "ignore") ep 0, 1 IN, 1 OUT
    0x02, //bInterfaceClass 1 byte, = CDC class
	0x00, //bInterfaceSubClass 1 byte
	0x00, //bInterfaceProtocol 1 byte
	0x02, //iInterface 1 byte
};

//end point 1 IN
uint8_t EndpointDescriptor1[7] =
{
	0x07, //bLength 1 byte
	0x05, //bDescriptorType 1 byte (force equal to 5)
	0x81, //bEndpointAddress 1 byte, ep1 (4low-bit), IN (4high-bit)
	0x02, //bmAttributes 1 byte: bulk, no sync, data end-point
	0x40, //wMaxPacketSize 2 byte: max 64 byte this ep can handle
	0x00,
	0x00, //bInterval 1 byte
};

//end point 1 OUT
uint8_t EndpointDescriptor2[7] =
{
	0x07, //bLength 1 byte
	0x05, //bDescriptorType 1 byte (force equal to 5)
	0x01, //bEndpointAddress 1 byte, ep1 (4low-bit), IN (4high-bit)
	0x02, //bmAttributes 1 byte: bulk, no sync, data end-point
	0x40, //wMaxPacketSize 2 byte: max 64 byte this ep can handle
	0x00,
	0x00, //bInterval 1 byte
};
#endif //if 0; these arrays are not used


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

uint8_t check2 = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t *pOTG_FS_GINTSTS_ = (uint32_t*)(OTG_FS_BASE_ADDR + 0x014);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

extern uint8_t checkOderRun[20];
extern uint8_t indexCheckOderRun;

int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint8_t numIRQ = 67;
//	uint32_t *pNVIC_ISER0 = (uint32_t*) 0xE000E100;
	//*(pNVIC_ISER0 + (numIRQ / 32)) |= (1 << (numIRQ % 32));
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

  *pRCC_AHB2ENR |= (1 << OTGFSEN__RCC_AHB2ENR); //will change AHB master to idle mode
  //uint32_t *pOTG_FS_DCTL_ = (uint32_t*)(OTG_FS_BASE_ADDR + 0x804);

  initUsb();

  while (1)
  {
	 if(check2 == 3)
	 {

	     GPIOD->ODR |= (1 << 12);
	     HAL_Delay(1000);
	     GPIOD->ODR &= ~(1 << 12);
	     for(int j = 0; j < indexCheckOderRun; j++)
	     {
	     	GPIOD->ODR |= checkOderRun[j] << 12;
			HAL_Delay(1500);
			GPIOD->ODR &= ~(0x0f << 12);
		 
		 }

		GPIOD->ODR &= ~(1 << 12);
		HAL_Delay(500);
		GPIOD->ODR |= (1 << 12);
		HAL_Delay(500);
		GPIOD->ODR &= ~(1 << 12);
		HAL_Delay(500);
		GPIOD->ODR |= (1 << 12);
		HAL_Delay(500);
		GPIOD->ODR &= ~(1 << 12);
		HAL_Delay(500);
		GPIOD->ODR |= (1 << 12);
		HAL_Delay(500);
		GPIOD->ODR &= ~(1 << 12);
		HAL_Delay(500);
		GPIOD->ODR |= (1 << 12);
		HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4; //forUSB full speed 48MHz
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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




void OTG_FS_IRQHandler(void)
{
//	USB_OTG_FS->GAHBCFG &= ~(1 << 0);
//NPTXFE, ESUSP, USBSUSP, EOPF, PTXFE, SRQINT is 1
	check2 = 3;
//	countInt++;
//	if(countInt > 40000)
//	{
//		if( (((GPIOD->ODR) >> 14) & 0x01) == 0)
//		{
//			GPIOD->ODR |= (1 << 14);
//		}
//		else
//		{
//			GPIOD->ODR &= ~(1 << 14);
//		}
//		countInt = 0;
//	}

	(void)usbInterruptHandler();

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
