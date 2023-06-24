/*
 * i2c1.h
 *
 *  Created on: Jun 24, 2023
 *      Author: hello
 */



#ifndef INC_I2C1_H_
#define INC_I2C1_H_



#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define AFRL7__GPIOx_AFRL           28
#define AFRL6__GPIOx_AFRL           24
#define AF4_I2C1					0x04

#define MODER7__GPIOx_MODER         14
#define MODER6__GPIOx_MODER         12
#define AlternateFunctionMode       0x02

#define OT7__GPIOx_OTYPER			0x07
#define OT6__GPIOx_OTYPER			0x06
#define OutputOpen_drain			0x01


#define OSPEEDR7__GPIOx_OSPEEDR		14
#define OSPEEDR6__GPIOx_OSPEEDR		12
#define High_speed                  0x03


#define PUPDR7__GPIOx_PUPDR			14
#define PUPDR6__GPIOx_PUPDR			12
#define  Pull_up					0x01


#define I2C1EN__RCC_APB1ENR			21
#define FREQ__I2C_CR2				0

#define CCR__I2C_CCR				0

#define TRISE__I2C_TRISE			0

#define PE__2C_CR1					0
#define SWRST__2C_CR1				15
#define POS__2C_CR1                 11
#define ACK__2C_CR1 				10
#define STOP__2C_CR1				9
#define START__2C_CR1				8

#define SB__I2C_SR1                 0
#define TxE__I2C_SR1 				0x07
#define RxNE__I2C_SR1               0x06
#define BTF__I2C_SR1				0x02
#define ADDR__I2C_SR1 				0x01

#define GPIOBEN__RCC_AHB1ENR    0x01



#define RCC_BASE_ADDR 0x40023800
#define GPIOB_BASE_ADDR 0x40020400
#define I2C1_BASE_ADDR 0x40005400


void i2c1Config(void);
void i2c1Start(void);
void i2c1Address(uint8_t address);

void i2c1AddressReadOneByte(uint8_t address);
void i2c1ReadOneByte(uint8_t* data);


void i2c1AddressReadTwoByte(uint8_t address);
void i2c1ReadTwoByte(uint8_t* data);

void i2c1ReadMultiBytes(uint8_t numByteToReads, uint8_t* data);

void i2c1MasterRead(uint8_t address, uint8_t numByteToReads, uint8_t* data);

void i2c1MasterWrite(uint8_t address, uint8_t numByteToWrites, uint8_t* data);

void i2c1Write(uint8_t data);
void i2c1Stop(void);
void i2c1SendCmd(uint8_t addr, uint8_t cmd);
void i2c1SendData(uint8_t addr, uint8_t data);
void i2c1SendString(uint8_t addressI2C, char* str);
void gotoXYLcd(uint8_t addressI2C, uint8_t x, uint8_t y);

//////*****///
void enablePinToDebug_portD(void);

#endif /* INC_I2C1_H_ */
