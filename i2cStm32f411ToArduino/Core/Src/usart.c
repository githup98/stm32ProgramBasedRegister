/*
 * usart.c
 *
 *  Created on: Jul 3, 2023
 *      Author: hello
 */

#include "usart.h"


uint8_t indexReceivedByte = 0;

uint32_t *RCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
uint32_t *pRCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);

uint32_t *pGPIOx_MODER_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
uint32_t *pGPIOx_OTYPER_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x04);
uint32_t *pGPIOx_OSPEEDR_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x08);
uint32_t *pGPIOx_PUPDR_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x0C);
uint32_t *pGPIOx_IDR_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
uint32_t *pGPIOx_AFRH_A = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);

uint32_t *pGPIOx_MODER_B = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
uint32_t *pGPIOx_OTYPER_B = (uint32_t*)(GPIOB_BASE_ADDR + 0x04);
uint32_t *pGPIOx_OSPEEDR_B = (uint32_t*)(GPIOB_BASE_ADDR + 0x08);
uint32_t *pGPIOx_PUPDR_B = (uint32_t*)(GPIOB_BASE_ADDR + 0x0C);
uint32_t *pGPIOx_IDR_B = (uint32_t*)(GPIOB_BASE_ADDR + 0x10);
uint32_t *pGPIOx_AFRL_B = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);

//for USART1
uint32_t *pUSART_SR = (uint32_t*)(USART1_BASE_ADDR + 0x00);
uint32_t *pUSART_DR = (uint32_t*)(USART1_BASE_ADDR + 0x04);
uint32_t *pUSART_BRR = (uint32_t*)(USART1_BASE_ADDR + 0x08);
uint32_t *pUSART_CR1 = (uint32_t*)(USART1_BASE_ADDR + 0x0C);
uint32_t *pUSART_CR2 = (uint32_t*)(USART1_BASE_ADDR + 0x10);
uint32_t *pUSART_CR3 = (uint32_t*)(USART1_BASE_ADDR + 0x14);


void configPinUsart(void)
{
	//config pin Tx Rx is pulled up (PA9 Tx PA10 Rx)

	//enable GPIOA registers
	*RCC_AHB1ENR |= (1 << GPIOAEN__RCC_AHB1ENR);

	//find out why can not use PA9 PA10, although turn off all related peripheral


	*RCC_AHB1ENR |= (1 << 1); //portB

	//config PA9 Tx PA10 Rx as alter func
	*pGPIOx_MODER_A &= ~(0x03 << 30);
	*pGPIOx_MODER_A |= (0x02 << 30);

	//PA9 Tx
	*pGPIOx_MODER_B &= ~(0x03 << 14);
	*pGPIOx_MODER_B |= (0x02 << 14);

	//config Output push-pull type
	*pGPIOx_OTYPER_A |= (1 << 15);

	*pGPIOx_OTYPER_B &= ~(1 << 7);

	//config fast speed
	*pGPIOx_OSPEEDR_A &= ~(0x03 << 30);
	*pGPIOx_OSPEEDR_A |= (0x03 << 30);
	*pGPIOx_OSPEEDR_B &= ~(0x03 << 14);
	*pGPIOx_OSPEEDR_B |= (0x03 << 14);

	//config pull up pull up
	*pGPIOx_PUPDR_A &= ~(0x03 << 30); //PA15
	*pGPIOx_PUPDR_A |= (0x01 << 30);
	*pGPIOx_PUPDR_B &= ~(0x03 << 14);
	//*pGPIOx_PUPDR_B |= (0x01 << 14);

	//config alter func 07 - USART1
	*pGPIOx_AFRH_A &= ~(0x0f << 28);
	*pGPIOx_AFRH_A |= (0x07 << 28);
	*pGPIOx_AFRL_B &= ~(0x0f << 28);
	*pGPIOx_AFRL_B |= (0x07 << 28);


	//	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//
	//	GPIO_InitStruct.Pin = GPIO_PIN_15;
	//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	//	GPIO_InitStruct.Pull = GPIO_NOPULL;
	//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//
	//	GPIO_InitStruct.Pin = GPIO_PIN_7;
	//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	//	GPIO_InitStruct.Pull = GPIO_NOPULL;
	//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void usart1Config(void)
{

	configPinUsart();

	//enable USART1 registers
	*pRCC_APB2ENR |= (1 << USART1EN__RCC_APB2ENR);

	/* all steps
	1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	2. Program the M bit in USART_CR1 to define the word length.
	3. Program the number of stop bits in USART_CR2.
	4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
	place. Configure the DMA register as explained in multibuffer communication.
	5. Select the desired baud rate using the USART_BRR register.
	6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
	for each data to be transmitted in case of single buffer.
	8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
	that the transmission of the last frame is complete. This is required for instance when
	the USART is disabled or enters the Halt mode to avoid corrupting the last
	transmission.
	*/

	*pUSART_CR1 |= (1 << UE__USART_CR1);  //enable USART1
	*pUSART_CR1 &= ~(1 << M__USART_CR1);  //8 bits data
	*pUSART_CR2 &= ~(0x03 << STOP__USART_CR2); //1 stop bit

	*pUSART_CR1 |= (1 << RXNEIE__USART_CR1);  //enable interrupt for receive data

	//select baurate = 9600, [exp] baurate * 8*(2 - OVER8)*USARTDIV = fPCLK
	*pUSART_BRR = 0x341; //Mantissa = 0x34 (dec 52), Fraction = 1 => USARTDIV = 52,0625
	//*pUSART_BRR = 0x1A0B; //1,2KBps
	//*pUSART_BRR = 0xD05; // 2,4KBps
	//*pUSART_BRR = 0x1A1; //19,2


}

void usart1SendOneByte(uint8_t *data, uint8_t numToSend)
{
	*pUSART_CR1 |= (1 << TE__USART_CR1);  //send an idle frame

	while(numToSend > 0)
	{
		while(!((*pUSART_SR >> TXE__USART_SR) & 0x01));
		*pUSART_DR = *data;
		data++;
		numToSend --;
	}
	while (!((*pUSART_SR >> TC__USART_SR) & 0x01));
	*pUSART_CR1 &= ~(1 << TE__USART_CR1); //disable transmission
}

void usart1ListenData(void)
{
	*pUSART_CR1 |= (1 << RE__USART_CR1); //start searching start bits
}
void usart1ReceiveBytes(uint8_t *data)
{
	/* all steps
	Procedure:
	1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	2. Program the M bit in USART_CR1 to define the word length.
	3. Program the number of stop bits in USART_CR2.
	4. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to take
	place. Configure the DMA register as explained in multibuffer communication. STEP 3
	5. Select the desired baud rate using the baud rate register USART_BRR
	6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a
	start bit.
	When a character is received
	Note:
	• The RXNE bit is set. It indicates that the content of the shift register is transferred to the
	RDR. In other words, data has been received and can be read (as well as its
	associated error flags).
	• An interrupt is generated if the RXNEIE bit is set.
	• The error flags can be set if a frame error, noise or an overrun error has been detected
	during reception.
	• In multibuffer, RXNE is set after every byte received and is cleared by the DMA read to
	the Data Register.
	• In single buffer mode, clearing the RXNE bit is performed by a software read to the
	USART_DR register. The RXNE flag can also be cleared by writing a zero to it. The
	RXNE bit must be cleared before the end of the reception of the next character to avoid
	an overrun error.
	*/

	while(! ((*pUSART_SR >> RXNE__USART_SR) & 0x01) );
	data[indexReceivedByte] = *pUSART_DR;
	indexReceivedByte++;

}












