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
//#include "i2c1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//#define LCD16X2                                     1
//#define TEST_REC_SLAVE_MODE_WITH_INT                1
//#define TEST_SEND_SLAVE_MODE_WITH_INT               1
//#define TEST_READ_WRITE_MASTER_MODE_WITHOUT_INT     1
//#define TEST_REC_SLAVE_MODE_WITHOU_TINT             1

//for usart
#define TEST_USART                                    0

//for spi4
#define TEST_SPI4 								      1 


//define when i2c is enable


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



#if TEST_REC_SLAVE_MODE_WITH_INT ||  TEST_READ_WRITE_MASTER_MODE_WITHOUT_INT || TEST_REC_SLAVE_MODE_WITHOU_TINT || TEST_SEND_SLAVE_MODE_WITH_INT
#include "i2c1.h"
#define I2C1_BASE_ADDR 0x40005400
uint32_t *pI2C1_CR1 = (uint32_t*)(I2C1_BASE_ADDR);
uint32_t *pI2C1_CR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x04);
uint32_t *pI2C1_DR = (uint32_t*)(I2C1_BASE_ADDR + 0x10);
uint32_t *pI2C1_SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
uint32_t *pI2C1_SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);

//uint8_t sentData[20] = {0x78, 0x79, 0x80, 0x81, 0x82, 0x83};
uint8_t sentData[5] = {0x40, 0x20, 0x70, 0x90};
uint8_t recData[5] = {0};

uint8_t flagCount = 0;

uint8_t totalByte = 0;

#endif




#if TEST_USART
#include "usart.h"
uint32_t *USART_SR = (uint32_t*)(USART1_BASE_ADDR + 0x00);
uint32_t *USART_DR = (uint32_t*)(USART1_BASE_ADDR + 0x04);

uint32_t *pNVIC_ISER1 = (uint32_t*)0xE000E104; //for I2C1 ER no.32

uint8_t receivedData[9];
extern uint8_t indexReceivedByte;

#endif //TEST_USART


#if TEST_SPI4
#include "spi.h"
#include "dma.h"
uint32_t *SPI4__SPI_DR = (uint32_t*)(SPI4_BASE_ADDR + 0x0C);
uint32_t *SPI4__SPI_SR = (uint32_t*)(SPI4_BASE_ADDR + 0x08);
uint8_t receivedData[10];
uint8_t sendData[10] = {0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46};
//extern uint8_t spiIndexReceivedByte;

uint8_t spiIndexReceivedByte2 = 0;
#endif //TEST_SPI4

uint8_t checkTrace = 0;




#define GPIOBEN__RCC_AHB1ENR        0x01



#define RCC_BASE_ADDR 0x40023800
#define GPIOA_BASE_ADDR 0x40020000
#define GPIOB_BASE_ADDR 0x40020400



#define EXTI_BASE_ADDR   0x40013C00
#define SYSCFG_BASE_ADDR 0x40013800


#define USE_BUTTON_PA0     1

uint32_t *pEXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14); //pending



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//AHB bus

uint32_t *GPIOx_ODR = (uint32_t*)(0x40020C00 + 0x14);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */


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
	//I2C1_EV_IRQn = 31// ??????
	//HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x40, 0);



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

#if USE_BUTTON_PA0

	//config for PA0 connected to VDD
	uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCC_AHB1ENR |= (1 << 0); //enable GPIOA port
	uint32_t *pRCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);
	*pRCC_APB2ENR |= (1 << 14); //enable SYSCFG

	uint32_t *pGPIOx_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*pGPIOx_MODER &= ~(0x03); //input
	uint32_t *pGPIOx_OTYPER = (uint32_t*)(GPIOA_BASE_ADDR + 0x04);
	*pGPIOx_OTYPER &= ~(1 << 0);//pull up

	uint32_t *pGPIOx_OSPEEDR_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x08);
	*pGPIOx_OSPEEDR_A &= ~(0x03 << 0);
	*pGPIOx_OSPEEDR_A |= (0x03 << 0);

	uint32_t *pGPIOx_PUPDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x0c);
	*pGPIOx_PUPDR &= ~(0x03 << 0);
	*pGPIOx_PUPDR |= (0x02 << 0);  //pull down for user button

	uint32_t *pGPIOx_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);

#endif



	//config for interrupt
#if USE_INT_PA0
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	uint32_t *pNVIC_ISER0 = (uint32_t*)(0xE000E100);
	*pNVIC_ISER0 |= (1 << 6);


	uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCC_AHB1ENR |= (1 << 0); //enable GPIOA port
	uint32_t *pRCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);
	*pRCC_APB2ENR |= (1 << 14); //enable SYSCFG

	uint32_t *pGPIOx_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*pGPIOx_MODER &= ~(0x03); //input
	uint32_t *pGPIOx_OTYPER = (uint32_t*)(GPIOA_BASE_ADDR + 0x04);
	*pGPIOx_OTYPER &= ~(1 << 0);//pull up

	uint32_t *pGPIOx_OSPEEDR_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x08);
	*pGPIOx_OSPEEDR_A &= ~(0x03 << 0);
	*pGPIOx_OSPEEDR_A |= (0x03 << 0);

	uint32_t *pGPIOx_PUPDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x0c);
	*pGPIOx_PUPDR &= ~(0x03 << 0);
	*pGPIOx_PUPDR |= (0x01 << 0);  //pull up for falling edge interrupt


	uint32_t *pEXTI_IMR = (uint32_t*)(EXTI_BASE_ADDR + 0x00);
	*pEXTI_IMR |= (1 << 0); //not mask interrupt
	uint32_t *pEXTI_FTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x0c);
	*pEXTI_FTSR |= (1 << 0); //disable falling edge
	uint32_t *pEXTI_RTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x08);
	*pEXTI_RTSR &= ~(1 << 0); //rising edge duee tto user button onboard is connnected to VDD



	//liine map interrupt source
	uint32_t *pSYSCFG_EXTICR1 = (uint32_t*)(SYSCFG_BASE_ADDR + 0x08);
	*pSYSCFG_EXTICR1 &= ~(0x0f << 0); //interrupt to PA0
#endif
	/////


///test case for i2c master read and master write///
#if TEST_READ_WRITE_MASTER_MODE_WITHOUT_INT
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

#if TEST_REC_SLAVE_MODE_WITH_INT || TEST_SEND_SLAVE_MODE_WITH_INT
	uint32_t *pNVIC_ISER0 = (uint32_t*)0xE000E100; //for I2C1_EV no.31
	uint32_t *pNVIC_ISER1 = (uint32_t*)0xE000E104; //for I2C1 ER no.32
	i2c1Config("slave"); //need for both read and write

	uint8_t received[6] = {0};



		*pNVIC_ISER0 |= (1 << 31); //enable EV I2C1
		*pNVIC_ISER1 |= (1 << 0);  // enable ER I2C1

	//i2c1SlaveSendBytes(sentData);
#endif
//////////////////////////////////////////////////////



#if TEST_USART
	uint8_t usartSendData[9] = {0x49, 0x02, 0x98, 0x97, 0x95};

	*pNVIC_ISER1 |= (1 << 5);  // enable USART INT
	usart1Config();
	HAL_Delay(5);
	usart1SendOneByte(usartSendData, 4);
	HAL_Delay(1000);
	//usart1SendOneByte(0x55);
#endif ///TEST_USART


#if TEST_SPI4
#define SPI4NumIRQ			 84
configSPI4();
uint32_t varDMA2_Tx = 0x70;
uint32_t varDMA2_Rx = 0;
dmaConfig(3, 5, &varDMA2_Rx, 0); //Rx stream 3, channel 5, dir: peripheral to mem
dmaConfig(1, 4, &varDMA2_Tx, 1); //Tx stream 1, channel 4, dir: mem to peripheral
//
uint32_t *pNVIC_ISER0 = (uint32_t*) 0xE000E100;
//*(pNVIC_ISER0 + (SPI4NumIRQ / 32)) |= (1 << (SPI4NumIRQ % 32));

#endif //TEST_SPI4




/////tetst case for lcd-i2c///////////////////////////
#if LCD16X2
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


#if TEST_USART
  usart1ListenData();
#endif //TEST_USART

  //*SPI4__SPI_DR = 0x22;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if 0
	  *GPIOx_ODR |= (1 << 15);
	  //printf("hello this %lx\n", countPulse);
	  for(int l = 0;l < 8; l++)
	  {
		  *GPIOx_ODR &= ~(1 << 12);
		  *GPIOx_ODR &= ~(1 << 13);
	      HAL_Delay(500);
		  *GPIOx_ODR |= (((spiIndexReceivedByte2 >> l) & 0x01) << 12);
		  *GPIOx_ODR |= (1 << 13);
		  HAL_Delay(500);


	  }

      *GPIOx_ODR &= ~(1 << 15);
      HAL_Delay(300);
#endif

      //while(10);

#if USE_BUTTON_PA0

  if ((*pGPIOx_IDR & 0x01) == 1) //user button PA0 Pin
	 {
		 HAL_Delay(100);
		 if ((*pGPIOx_IDR & 0x01) == 1)
		 {
			 *GPIOx_ODR &= ~(1 << 15);
			 *GPIOx_ODR &= ~(1 << 14);
			 *GPIOx_ODR &= ~(1 << 13);
			 *GPIOx_ODR &= ~(1 << 12);
#if TEST_USART
			 uint8_t usartSendData2[6] = {0x55, 0x44, 0x00, 0x22};
			 usart1SendOneByte(usartSendData2, 4);
			 HAL_Delay(2000);
			usart1SendOneByte(usartSendData, 4);
#endif //TEST_USART
		 }

	 }


#if TEST_USART
	usart1ReceiveBytes(receivedData); //receive without INT
	if(indexReceivedByte > 3)
	{
	  indexReceivedByte = 0;
	}
#endif //if TEST_USART

#if TEST_SPI4
	////spi4ReceiveData(receivedData, sendData);
	if(spiIndexReceivedByte2 > 2)
	{
		spiIndexReceivedByte2 = 0;
	}

#endif //if TEST_SPI4

#if TEST_USART || TEST_SPI4

	if(varDMA2_Rx == 0x59)
	{
		*GPIOx_ODR |= (1 << 14);
	}
	if(receivedData[0] == 0x59)
  {
	  *GPIOx_ODR |= (1 << 15);
  }
  else
  {
	  *GPIOx_ODR &= ~(1 << 15);
  }
  if(receivedData[1] == 0x48)
    {
  	  //*GPIOx_ODR |= (1 << 14);
    }
  else
  {
	  *GPIOx_ODR &= ~(1 << 14);
  }
  if(receivedData[2] == 0x56)
    {
  	  *GPIOx_ODR |= (1 << 13);
    }
  else
  {
	  *GPIOx_ODR &= ~(1 << 13);
  }
  if(receivedData[3] == 0x23)
    {
  	  *GPIOx_ODR |= (1 << 12);
    }
  else
  {
	  *GPIOx_ODR &= ~(1 << 12);
  }
#endif  //TEST_USART

#endif  //useButtonPA0

#if TEST_REC_SLAVE_MODE_WITH_INT
  if(flagCount)
  {
	  if(recData[0] == 0x97)
	  {
		  *GPIOx_ODR &= ~(1 << 14);
		  *GPIOx_ODR |= (1 << 14);
	  }
	  if(recData[1] == 0x94)
		{
		  *GPIOx_ODR &= ~(1 << 13);
		  *GPIOx_ODR |= (1 << 13);
		}
	  else if(recData[1] == 0x99)
	  {
		  *GPIOx_ODR &= ~(1 << 15);
		  HAL_Delay(300);
		  *GPIOx_ODR |= (1 << 15);
		  HAL_Delay(300);
		  *GPIOx_ODR &= ~(1 << 15);
		  HAL_Delay(300);
		  *GPIOx_ODR |= (1 << 15);
		  HAL_Delay(300);
	  }
	  if(recData[2] == 0)
	  {
		  *GPIOx_ODR &= ~(1 << 12);
		  *GPIOx_ODR |= (1 << 12);
	  }
	  flagCount = 0;
  }
#endif


//// test case for receiver byte or multi bytes///

#if TEST_REC_SLAVE_MODE_WITHOUT_INT
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
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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


#if TEST_REC_SLAVE_MODE_WITH_INT || TEST_SEND_SLAVE_MODE_WITH_INT

void I2C1_EV_IRQHandler(void)
{
	if ((*pI2C1_SR1 >> ADDR__I2C_SR1) & 0x01)
	{
		uint8_t tmp = (*pI2C1_SR1 | *pI2C1_SR2);

		return;

	}
	if ((*pI2C1_SR1 >> TxE__I2C_SR1) & 0x01)
	{
		*pI2C1_DR = sentData[totalByte];
		totalByte += 1;

		return;
	}
	if ((*pI2C1_SR1 >> RxNE__I2C_SR1) & 0x01)
	{
		recData[totalByte] = *pI2C1_DR;
		totalByte += 1;
		return;
	}
	if ((*pI2C1_SR1 >> STOPF__I2C_SR1) & 0x01)
	{
		//clear STOPF bit
		uint8_t tmp = *pI2C1_SR1;
		*pI2C1_CR1 |= (1 << PE__I2C_CR1);

		totalByte = 0;
		flagCount = 1; //for check point by LED
		return;
	}


}

void I2C1_ER_IRQHandler(void)
{
	*pI2C1_SR1 &= ~(1 << AF__I2C_SR1);
	totalByte = 0;
}

#endif


//received using USART INT
#if testUsart


void USART1_IRQHandler(void)
{
	if((*USART_SR >> ORE__USART_SR) & 0x01)
	{
		return;
	}
	if((*USART_SR >> RXNE__USART_SR) & 0x01)
	{
		receivedData[indexReceivedByte] = *USART_DR;
		indexReceivedByte ++;
		return ;
	}
}

#endif //testUsart




#if TEST_SPI4
void SPI4_IRQHandler(void)
{
//	if((*SPI4__SPI_SR >> OVR__SPI_SR) & 0x01)
//	{
//		*GPIOx_ODR |= (1 << 12);
//		return;
//	}
//
//	if((*SPI4__SPI_SR >> TXE__SPI_SR) & 0x01)
//		{
//			*SPI4__SPI_DR = sendData[spiIndexReceivedByte2];
//			spiIndexReceivedByte2 ++;
//		}
//
//	if((*SPI4__SPI_SR >> RXNE__SPI_SR) & 0x01)
//	{
//		receivedData[spiIndexReceivedByte2 - 2] = *SPI4__SPI_DR;
//	}
}
#endif // TEST_SPI4



void EXTI0_IRQHandler(void)
{

	 *pEXTI_PR |= (1 << 0);  //write 1 to clear pending bit (for next time)
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
