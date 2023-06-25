#include "i2c1.h"

uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);

uint32_t *pI2C1__I2C_CR1 = (uint32_t*)(I2C1_BASE_ADDR);
uint32_t *pI2C1__I2C_DR = (uint32_t*)(I2C1_BASE_ADDR + 0x10);
uint32_t *pI2C1__I2C_SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
uint32_t *pI2C1__I2C_SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);


void i2c1Config(const char* masterORSlave) //master/slave
{
	/* all step -----------------
	 * configure pin feature
	 * Reset i2c line by software reset
	 * Program the peripheral input clock in I2C_CR2 Register in order to generate correct
	timings
	• Configure the clock control registers
	• Configure the rise time register
	• Program the I2C_CR1 register to enable the peripheral
	• Set the START bit in the I2C_CR1 register to generate a Start condition
	*/


	//enable peripheral clock GPIOB
	*pRCC_AHB1ENR |= (1 << GPIOBEN__RCC_AHB1ENR);


	//configure pins PB7 PB6 are alter function and output opened drain pull up
	uint32_t *pGPIOB__GPIOx_AFRL = (uint32_t*)(GPIOB_BASE_ADDR + 0x20); //pointer to alter func register
	*pGPIOB__GPIOx_AFRL &= ~(0x0f << AFRL7__GPIOx_AFRL);    //clear AFRL7[3:0]
	*pGPIOB__GPIOx_AFRL |= (AF4_I2C1 << AFRL7__GPIOx_AFRL);
	*pGPIOB__GPIOx_AFRL &= ~(0x0f << AFRL6__GPIOx_AFRL);    //clear AFRL6[3:0]
	*pGPIOB__GPIOx_AFRL |= (AF4_I2C1 << AFRL6__GPIOx_AFRL);


	uint32_t *pGPIOB__GPIOx_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
	*pGPIOB__GPIOx_MODER &= ~(0x03 << MODER7__GPIOx_MODER);  //clear MODER7
	*pGPIOB__GPIOx_MODER |= (AlternateFunctionMode << MODER7__GPIOx_MODER);
	*pGPIOB__GPIOx_MODER &= ~(0x03 << MODER6__GPIOx_MODER);  //clear MODER6
	*pGPIOB__GPIOx_MODER |= (AlternateFunctionMode << MODER6__GPIOx_MODER);

	uint32_t *pGPIOB__GPIOx_OTYPER = (uint32_t*)(GPIOB_BASE_ADDR + 0x04);
	*pGPIOB__GPIOx_OTYPER |= (OutputOpen_drain << OT7__GPIOx_OTYPER);
	*pGPIOB__GPIOx_OTYPER |= (OutputOpen_drain << OT6__GPIOx_OTYPER);

	uint32_t *pGPIOB__GPIOx_OSPEEDR = (uint32_t*)(GPIOB_BASE_ADDR + 0x08);
	*pGPIOB__GPIOx_OSPEEDR &= ~(0x03 << OSPEEDR7__GPIOx_OSPEEDR);
	*pGPIOB__GPIOx_OSPEEDR |= (High_speed << OSPEEDR7__GPIOx_OSPEEDR);
	*pGPIOB__GPIOx_OSPEEDR &= ~(0x03 << OSPEEDR6__GPIOx_OSPEEDR);
	*pGPIOB__GPIOx_OSPEEDR |=(High_speed << OSPEEDR6__GPIOx_OSPEEDR);

//	uint32_t *pGPIOB__PIOx_PUPDR = (uint32_t*)(GPIOB_BASE_ADDR + 0x0C);
//	*pGPIOB__PIOx_PUPDR &= ~(0x03 << PUPDR7__GPIOx_PUPDR);
//	*pGPIOB__PIOx_PUPDR |= (Pull_up << PUPDR7__GPIOx_PUPDR);
//	*pGPIOB__PIOx_PUPDR &= ~(0x03 << PUPDR6__GPIOx_PUPDR);
//	*pGPIOB__PIOx_PUPDR |= (Pull_up << PUPDR6__GPIOx_PUPDR);


	//enable peripheral input clock for I2C1, set peripheral input clock for SCL in I2C_CR2 register
	uint32_t *pRCC_APB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x40);
	*pRCC_APB1ENR |= (1 << I2C1EN__RCC_APB1ENR);

	//Reset i2c lines by software reset (master mode)
	if(strcmp(masterORSlave, "master") == 0)
	{
		*pI2C1__I2C_CR1 |= (1 << SWRST__I2C_CR1);
		*pI2C1__I2C_CR1 &= ~(1 << SWRST__I2C_CR1);

		//configure clock control register
		uint32_t *pI2C1__I2C_CCR = (uint32_t*)(I2C1_BASE_ADDR + 0x1C);
		*pI2C1__I2C_CCR &= (0xf000);   //clear previous value
		//tSCL = 10us -> tHighSCL = 5us; tAPB1 = 125ns -> tHighSCL = CCR * 125
		*pI2C1__I2C_CCR |= (0x28 << CCR__I2C_CCR);

		//Configure the rise time register
		uint32_t *pI2C1__I2C_TRISE = (uint32_t*)(I2C1_BASE_ADDR + 0x20);
		*pI2C1__I2C_TRISE &= ~(0x3f << TRISE__I2C_TRISE);
		//1000 / fAPB1 + 1 with 1000 is maximum allowed rised time on SCL in nanoseconds
		*pI2C1__I2C_TRISE |= (0x09 << TRISE__I2C_TRISE);
	}


	uint32_t *pI2C1__I2C_CR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x04);
	*pI2C1__I2C_CR2 &=~ (0x1f << FREQ__I2C_CR2); //clear previous value
	*pI2C1__I2C_CR2 |= (0x08 << FREQ__I2C_CR2);   //APB1 clock = 8


	if(strcmp(masterORSlave, "slave") == 0)
	{
		//Set address 7-bit mode
		uint32_t *pI2C1__I2C_OAR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x08);
		*pI2C1__I2C_OAR1 &= ~ (1 << ADDMODE__I2C_OAR1); //select 7 bit mode
		*pI2C1__I2C_OAR1 &= ~ (0xfe << ADD_7_1__I2C_OAR1); //clear value
		*pI2C1__I2C_OAR1 |= (0x0f << ADD_7_1__I2C_OAR1); //address = 0x10
		HAL_Delay(5);
	}

	//Program the I2C_CR1 register to enable the peripheral
	uint32_t *pI2C1__I2C_CR1 = (uint32_t*)(I2C1_BASE_ADDR);
	*pI2C1__I2C_CR1 |= (1 << PE__I2C_CR1);
	if(strcmp(masterORSlave, "slave") == 0)
	{
		*pI2C1__I2C_CR1 |= (1 << ACK__I2C_CR1); //ACK bit must be set by programmer after PE bit is set to 1
	}
}




void i2c1Start(void)
{
	*pI2C1__I2C_CR1  |= (1 << ACK__I2C_CR1);
	*pI2C1__I2C_CR1  |= (1 << START__I2C_CR1); //generate start condition
	while(!(((*pI2C1__I2C_CR1) >> SB__I2C_SR1) & 0x01)); //wait for SB equal to 1
}

void i2c1Address(uint8_t address)
{
	*pI2C1__I2C_DR = address;

	//wait for acknowledge bit (ADDR bit = 1)
	while (!(((*pI2C1__I2C_SR1) >> ADDR__I2C_SR1) & 0x01));

	//clear ADDR bit
	uint8_t tmp = *pI2C1__I2C_SR1 | *pI2C1__I2C_SR2;
}


void i2c1AddressReadOneByte(uint8_t address)
{
	*pI2C1__I2C_DR = address;
	////I2C1->DR = (uint16_t)address;

	//wait for acknowledge bit (ADDR bit = 1)
	while (!(((*pI2C1__I2C_SR1) >> ADDR__I2C_SR1) & 0x01));
	////while (!(((I2C1->SR1) >> ADDR__I2C_SR1) & 0x01));

	//change ACK to 0 before clear ADDR bit due to read 1 byte
	*pI2C1__I2C_CR1  &= ~(1 << ACK__I2C_CR1);

	//clear ADDR bit
	uint8_t tmp = *pI2C1__I2C_SR1 | *pI2C1__I2C_SR2;
}

void i2c1AddressReadTwoByte(unsigned char address)
{
	*pI2C1__I2C_DR = address;
	////I2C1->DR = (uint16_t)address;
	//wait for acknowledge bit (ADDR bit = 1)
	while (!(((*pI2C1__I2C_SR1) >> ADDR__I2C_SR1) & 0x01));
	////while (!(((I2C1->SR1) >> ADDR__I2C_SR1) & 0x01));

	//change ACK to 0 before clear ADDR bit due to read 1 byte
	*pI2C1__I2C_CR1  &= ~(1 << ACK__I2C_CR1);
	*pI2C1__I2C_CR1 |= (1 << POS__I2C_CR1);

	//clear ADDR bit
	uint8_t tmp = *pI2C1__I2C_SR1 | *pI2C1__I2C_SR2;
}





void i2c1ReadOneByte(uint8_t* data) //clear ACK bit before clear ADDR
{
	while(!(((*pI2C1__I2C_SR1) >> RxNE__I2C_SR1) & 0x01));

	data[0] = *pI2C1__I2C_DR;
	HAL_Delay(100);
}

void i2c1ReadTwoByte(uint8_t* data) //clear ACK bit and set POS bit before clear ADDR bit
{
	//waiting to BTF bit is set to 1
	while(!(((*pI2C1__I2C_SR1) >> BTF__I2C_SR1) & 0x01));
	i2c1Stop();

	HAL_Delay(100);
	data[0] = *pI2C1__I2C_DR;
	data[1] = *pI2C1__I2C_DR;

}


void i2c1ReadMultiBytes(uint8_t numByteToReads, uint8_t* data) //wait for BTF bit = 1 at the moment receive 3th last byte -> clear ACK, wait next BTF bit = 1 -> stop -> read two last byte
{
	uint8_t count = 0;
	while(count < numByteToReads)
	{
		if(count >= numByteToReads - 3)
		{
			while(!(((*pI2C1__I2C_SR1) >> BTF__I2C_SR1) & 0x01));
			*pI2C1__I2C_CR1  &= ~(1 << ACK__I2C_CR1);
			data[count] = *pI2C1__I2C_DR;

			while(!(((*pI2C1__I2C_SR1) >> BTF__I2C_SR1) & 0x01));
			i2c1Stop();

			HAL_Delay(100);
			data[count + 1] = *pI2C1__I2C_DR;
			data[count + 2] = *pI2C1__I2C_DR;
			break;//exit while loop
		}
		while(!(((*pI2C1__I2C_SR1) >> RxNE__I2C_SR1) & 0x01));
		data[count] = *pI2C1__I2C_DR;
		count += 1;
	}
}

void i2c1MasterWrite(uint8_t address, uint8_t numByteToWrites, uint8_t* data)
{
	uint8_t count = 0;
	switch(numByteToWrites)
	{
	case 0:
		break;
	case 1:
		i2c1Start();
		HAL_Delay(5);
		i2c1Address(address);
		HAL_Delay(5);
		i2c1Write(*data);
		HAL_Delay(5);
		i2c1Stop();
		HAL_Delay(10);
	default:
		i2c1Start();
		HAL_Delay(5);
		i2c1Address(address);
		HAL_Delay(5);
		for(count = 0; count < numByteToWrites; count ++)
		{
			i2c1Write(data[count]);
			HAL_Delay(100);
		}
		i2c1Stop();
		HAL_Delay(10);
		break;
	}
}

void i2c1MasterRead(uint8_t address, uint8_t numByteToReads, uint8_t* data)
{
	switch (numByteToReads)
	{
	case 0:
		break;
	case 1:
		i2c1Start();
		HAL_Delay(5);
		i2c1AddressReadOneByte(address);
		HAL_Delay(5);
		i2c1ReadOneByte(data);
		HAL_Delay(5);
		i2c1Stop();
		HAL_Delay(10);
		break;
	case 2:
		i2c1Start();
		HAL_Delay(5);
		i2c1AddressReadTwoByte(address);
		HAL_Delay(5);
		i2c1ReadTwoByte(data);
		HAL_Delay(10);
	default:
		i2c1Start();
		HAL_Delay(5);
		i2c1Address(address);
		HAL_Delay(5);
		i2c1ReadMultiBytes(numByteToReads, data);
		HAL_Delay(10);
		break;
	}
}


void i2c1Write(uint8_t data)
{
	//wait for TxE  = 1 (DR register empty, received Ack bit == address match in master mode)
	while (! (((*pI2C1__I2C_SR1) >> TxE__I2C_SR1) & 0x01) );
	*pI2C1__I2C_DR = data;

	//wait for BTF bit = 1, last byte transfer successed
	while (! (((*pI2C1__I2C_SR1) >> BTF__I2C_SR1) & 0x01) );
}

void i2c1Stop(void)
{
	*pI2C1__I2C_CR1 |= (1 << STOP__I2C_CR1);
}





void i2c1SendCmd(uint8_t addr, uint8_t cmd)
{
	i2c1Start();
	HAL_Delay(10);
	i2c1Address(addr);
	HAL_Delay(10);
	i2c1Write((cmd & 0xf0) | 0x0c); //P3 = 1 (for LED), EN = 1, RW = 0, RS = 0
	HAL_Delay(5);
	i2c1Write((cmd & 0xf0) | 0x08);
	HAL_Delay(5);
	i2c1Write(((cmd << 4) & 0xf0) | 0x0c);
	HAL_Delay(5);
	i2c1Write(((cmd << 4) & 0xf0) | 0x08);
	HAL_Delay(5);
	i2c1Stop();
	HAL_Delay(5);
}

void i2c1SendData(uint8_t addr, uint8_t data)
{
	i2c1Start();
	HAL_Delay(10);
	i2c1Address(addr); //arduino has address is 1, due to write operation -> 0x02;
	HAL_Delay(10);
	i2c1Write((data & 0xf0) | 0x0d); //P3 = 1 (for LED), EN = 1, RW = 0, RS = 0
	HAL_Delay(5);
	i2c1Write((data & 0xf0) | 0x09);
	HAL_Delay(5);
	i2c1Write(((data << 4) & 0xf0) | 0x0d);
	HAL_Delay(5);
	i2c1Write(((data << 4) & 0xf0) | 0x09);
	HAL_Delay(5);
	i2c1Stop();
	HAL_Delay(5);
}

void i2c1SlaveReceiveBytes(uint8_t *data)
{
	//HAL_Delay(10);
	while(!(((*pI2C1__I2C_SR1) >> ADDR__I2C_SR1) & 0x01));
	uint8_t tmp = (*pI2C1__I2C_SR1 | *pI2C1__I2C_SR2);    //clear ADDR bit
	HAL_Delay(10);  //this delay help void stuck in while loop
	uint8_t count = 0;
	while(!((*pI2C1__I2C_SR1 >> STOPF__I2C_SR1) & 0x01) ) //receive multi byte
	{
		//wait for RXEN bit equal to 1
		while(!((*pI2C1__I2C_SR1 >> RxNE__I2C_SR1) & 0x01) );  //due to preoder of "*" > ">>"
		data[count] = *pI2C1__I2C_DR;
		count += 1;
	}

	//clear STOPF bit by read SR1 and Write CR1
	count = *pI2C1__I2C_SR1;
	*pI2C1__I2C_CR1 |= (1 << PE__I2C_CR1);
}


void i2c1SlaveSendBytes(uint8_t* data)
{
	while(!(((*pI2C1__I2C_SR1) >> ADDR__I2C_SR1) & 0x01));
	uint8_t tmp = (*pI2C1__I2C_SR1 | *pI2C1__I2C_SR2);    //clear ADDR bit
	HAL_Delay(10);  //this delay help void stuck in while loop
	uint8_t count = 0;

	//while(!(((*pI2C1__I2C_SR1) >> BTF__I2C_SR1) & 0x01));
	while(!((*pI2C1__I2C_SR1 >> AF__I2C_SR1) & 0x01) )  //due to master will send NACK , so that check AF bit instead of checking STOPF,
	{
		while(!(((*pI2C1__I2C_SR1) >> TxE__I2C_SR1) & 0x01));
		*pI2C1__I2C_DR = data[count];
		count += 1;
		if(count > 5)
		{
			count = 0;
		}
		HAL_Delay(3);
	}
	*pI2C1__I2C_SR1 &= ~(1 << AF__I2C_SR1); //clear AF bit
}
void enablePinToDebug_portD(void)
{
///////
	uint32_t *pGPIOx_MODER = (uint32_t*)(0x40020C00);
	*pGPIOx_MODER &= ~(0x03 << 30);
	*pGPIOx_MODER |=(0x01 << 30);

	*pGPIOx_MODER &= ~(0x03 << 28);
	*pGPIOx_MODER |=(0x01 << 28);
	*pGPIOx_MODER &= ~(0x03 << 26);
	*pGPIOx_MODER |=(0x01 << 26);
	*pGPIOx_MODER &= ~(0x03 << 24);
	*pGPIOx_MODER |=(0x01 << 24);
	uint32_t *pGPIOx_OTYPER = (uint32_t*)(0x40020C00 + 0x04);
	*pGPIOx_OTYPER &= ~(0x01 << 15);
	*pGPIOx_OTYPER &= ~(0x01 << 14);
	*pGPIOx_OTYPER &= ~(0x01 << 13);
	*pGPIOx_OTYPER &= ~(0x01 << 12);


	uint32_t *GPIOx_PUPDR = (uint32_t*)(0x40020C00 + 0x0c);
	*GPIOx_PUPDR &= ~(0x03 << 30);
	*GPIOx_PUPDR |=(0x01 << 30);
	*GPIOx_PUPDR &= ~(0x03 << 28);
	*GPIOx_PUPDR |=(0x01 << 28);
	*GPIOx_PUPDR &= ~(0x03 << 26);
	*GPIOx_PUPDR |=(0x01 << 26);
	*GPIOx_PUPDR &= ~(0x03 << 24);
	*GPIOx_PUPDR |=(0x01 << 24);

	//uint32_t *GPIOx_ODR = (uint32_t*)(0x40020C00 + 0x14);

///////////
}


void gotoXYLcd(uint8_t addressI2C, uint8_t x, uint8_t y)
{
	uint8_t cmd = x + y * 0x40;
	i2c1SendCmd(addressI2C, 0x80 | cmd); //0x80 indicate that setting SDRAM address without it, cannot display content
}

void i2c1SendString(uint8_t addressI2C, char* str)
{
	while(*str)
	{
		if(*str == 10) //get new line
		{
				str++;
				gotoXYLcd(addressI2C, 0,0x40);
		}
		i2c1SendData(addressI2C, *str);
		str++;
	}
}


