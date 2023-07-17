#include "spi.h"

uint8_t spiIndexReceivedByte = 0;

uint32_t *pSPI4__SPI_SR = (uint32_t*)(SPI4_BASE_ADDR + 0x08);
uint32_t *pSPI4__SPI_DR = (uint32_t*)(SPI4_BASE_ADDR + 0x0C);

void configSPI4Pin(void)
{
	//enable GPIOE registers
	uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCC_AHB1ENR |= (1 << GPIOEEN);
	
	//alter function
	uint32_t *pGPIOE__GPIOx_MODER = (uint32_t*)(GPIOE_BASE_ADDR + 0x00);
	*pGPIOE__GPIOx_MODER &= ~(3 << MODER6);
	*pGPIOE__GPIOx_MODER |= (2 << MODER6);
	
	*pGPIOE__GPIOx_MODER &= ~(3 << MODER11);
	*pGPIOE__GPIOx_MODER |= (2 << MODER11);
	
	*pGPIOE__GPIOx_MODER &= ~(3 << MODER12);
	*pGPIOE__GPIOx_MODER |= (2 << MODER12);

	*pGPIOE__GPIOx_MODER &= ~(3 << MODER13);
	*pGPIOE__GPIOx_MODER |= (2 << MODER13);
	
	//output push-pull
	uint32_t *pGPIOE__GPIOx_OTYPER = (uint32_t*)(GPIOE_BASE_ADDR + 0x04);
	*pGPIOE__GPIOx_OTYPER &= ~(1 << OT6);
	*pGPIOE__GPIOx_OTYPER &= ~(1 << OT11);
	*pGPIOE__GPIOx_OTYPER &= ~(1 << OT12);
	*pGPIOE__GPIOx_OTYPER &= ~(1 << OT13);

	//high speed
	uint32_t *pGPIOE__GPIOx_OSPEEDR = (uint32_t*)(GPIOE_BASE_ADDR + 0x08);
	*pGPIOE__GPIOx_OSPEEDR &= ~(3 << OSPEEDR6);
	*pGPIOE__GPIOx_OSPEEDR |= (3 << OSPEEDR6);
	
	*pGPIOE__GPIOx_OSPEEDR &= ~(3 << OSPEEDR11);
	*pGPIOE__GPIOx_OSPEEDR |= (3 << OSPEEDR11);

	*pGPIOE__GPIOx_OSPEEDR &= ~(3 << OSPEEDR12);
	*pGPIOE__GPIOx_OSPEEDR |= (2 << OSPEEDR12);
	
	*pGPIOE__GPIOx_OSPEEDR &= ~(3 << OSPEEDR13);
	*pGPIOE__GPIOx_OSPEEDR |= (3 << OSPEEDR13);

	//pull-up
	uint32_t *pGPIOE__GPIOx_PUPDR = (uint32_t*)(GPIOE_BASE_ADDR + 0x0C);
	*pGPIOE__GPIOx_PUPDR &= ~(3 << PUPDR6);	
	*pGPIOE__GPIOx_PUPDR |= (1 << PUPDR6);	

	*pGPIOE__GPIOx_PUPDR &= ~(3 << PUPDR11);	
	*pGPIOE__GPIOx_PUPDR |= (1 << PUPDR11);	
	
	*pGPIOE__GPIOx_PUPDR &= ~(3 << PUPDR12);	
	*pGPIOE__GPIOx_PUPDR |= (1 << PUPDR12);	
	
	*pGPIOE__GPIOx_PUPDR &= ~(3 << PUPDR13);	
	*pGPIOE__GPIOx_PUPDR |= (1 << PUPDR13);	
	
	//alter function
	uint32_t *pGPIOE__GPIOx_AFRL = (uint32_t*)(GPIOE_BASE_ADDR + 0x20);
	*pGPIOE__GPIOx_AFRL &= ~(0x0f << AFRL6); 
	*pGPIOE__GPIOx_AFRL |= (5 << AFRL6);
	
	uint32_t *pGPIOE__GPIOx_AFRH = (uint32_t*)(GPIOE_BASE_ADDR + 0x24);
	*pGPIOE__GPIOx_AFRH &= ~(0x0f << AFRL11); 
	*pGPIOE__GPIOx_AFRH |= (5 << AFRL11);

	*pGPIOE__GPIOx_AFRH &= ~(0x0f << AFRL12); 
	*pGPIOE__GPIOx_AFRH |= (5 << AFRL12); 
	
	*pGPIOE__GPIOx_AFRH &= ~(0x0f << AFRL13); 
	*pGPIOE__GPIOx_AFRH |= (5 << AFRL13); 
}

void configSPI4(void)
{
/*Procedure
1. Set the DFF bit to define 8- or 16-bit data frame format
2. Select the CPOL and CPHA bits to define one of the four relationships between the
data transfer and the serial clock (see Figure 194). For correct data transfer, the CPOL
and CPHA bits must be configured in the same way in the slave device and the master
device. This step is not required when the TI mode is selected through the FRF bit in
the SPI_CR2 register.
3. The frame format (MSB-first or LSB-first depending on the value of the LSBFIRST bit in
the SPI_CR1 register) must be the same as the master device. This step is not
required when TI mode is selected.
4. In Hardware mode (refer to Slave select (NSS) pin management), the NSS pin must be
connected to a low level signal during the complete byte transmit sequence. In NSS
software mode, set the SSM bit and clear the SSI bit in the SPI_CR1 register. This step
is not required when TI mode is selected.
5. Set the FRF bit in the SPI_CR2 register to select the TI mode protocol for serial
communications.
6. Clear the MSTR bit and set the SPE bit (both in the SPI_CR1 register) to assign the
pins to alternate functions.
*/
	
	//config pin PE6 MOSI PE11 NSS PE12 SCK PE13 MISO
	configSPI4Pin();
	//enable SPI4 registers
	uint32_t *pRCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);
	*pRCC_APB2ENR |= (1 << SPI4EN);
	
	//Set DFF is 8 bit frame
	uint32_t *pSPI4__SPI_CR1 = (uint32_t*)(SPI4_BASE_ADDR + 0x00);
	*pSPI4__SPI_CR1 &= ~(1 << DFF__SPI_CR1);

	//set CPOL = 0, CPHA = 0
	*pSPI4__SPI_CR1 &= ~(1 << CPOL__SPI_CR1);
	*pSPI4__SPI_CR1 &= ~(1 << CPHA__SPI_CR1);
	*pSPI4__SPI_CR1 |= (1 << CPHA__SPI_CR1);
	
	//MBS first
	*pSPI4__SPI_CR1 &= ~(1 << LSBFIRST__SPI_CR1);

	//use hardware mode for NSS pin, SSM = 0; SSI = 0;
	*pSPI4__SPI_CR1 &= ~(1 << SSM__SPI_CR1);
	*pSPI4__SPI_CR1 &= ~(1 << SSI__SPI_CR1);

	//use Motorola mode
	uint32_t *pSPI4__SPI_CR2 = (uint32_t*)(SPI4_BASE_ADDR + 0x04);
	*pSPI4__SPI_CR2 &= ~(1 << FRF__SPI_CR2);

	//enable interrupt
	//*pSPI4__SPI_CR2 |= (1 << TXEIE__SPI_CR2);
	//*pSPI4__SPI_CR2 |= (1 << RXNEIE__SPI_CR2);
	*pSPI4__SPI_CR2 |= (1 << ERRIE__SPI_CR2);

	*pSPI4__SPI_CR2 |= (1 << RXDMAEN__SPI_CR2);
	*pSPI4__SPI_CR2 |= (1 << TXDMAEN__SPI_CR2);

	//set slave mode, enable SPI
	*pSPI4__SPI_CR1 &= ~(1 << MSTR__SPI_CR1);

	*pSPI4__SPI_CR1 &= ~(1 << SPE__SPI_CR1);
	*pSPI4__SPI_CR1 |= (1 << SPE__SPI_CR1);



}

void spi4ReceiveData(uint8_t *data, uint8_t *data2)
{

	while(! ((*pSPI4__SPI_SR >> TXE__SPI_SR) & 0x01));
	*pSPI4__SPI_DR = data2[spiIndexReceivedByte];

	while(! ((*pSPI4__SPI_SR >> RXNE__SPI_SR) & 0x01));
	data[spiIndexReceivedByte] = *pSPI4__SPI_DR;

	spiIndexReceivedByte ++;
}
