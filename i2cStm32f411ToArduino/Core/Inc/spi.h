#ifndef SPI_H_
#define SPI_H_


#include <stdint.h>


//spi4 pinout  PE6 MOSI PE11 NSS(chip select) PE12 SCK PE13 MISO

#define RCC_BASE_ADDR 				0x40023800

#define GPIOE_BASE_ADDR 			0x40021000

#define SPI4_BASE_ADDR  			0x40013400

#define GPIOEEN 					0x04
#define SPI4EN 						13


#define MODER6 						12
#define MODER11						22
#define MODER12 					24
#define MODER13 					26

#define OT6 						6
#define OT11						11
#define OT12 						12
#define OT13						13

#define OSPEEDR6  					12
#define OSPEEDR11 					22
#define OSPEEDR12  					24
#define OSPEEDR13 					26


#define PUPDR6  					12
#define PUPDR11 					22
#define PUPDR12  					24
#define PUPDR13 					26

#define AFRL6 						24
#define AFRL11						12
#define AFRL12 						16
#define AFRL13 						20

#define CPHA__SPI_CR1 				0x00
#define CPOL__SPI_CR1 				0x01
#define MSTR__SPI_CR1 				0x02
#define SPE__SPI_CR1 				0x06
#define LSBFIRST__SPI_CR1 			0x07
#define SSI__SPI_CR1 				0x08
#define SSM__SPI_CR1 				0x09
#define DFF__SPI_CR1 				11

#define FRF__SPI_CR2 				0x04

#define RXNE__SPI_SR 				0x00
#define TXE__SPI_SR 				0x01
#define OVR__SPI_SR 				0x06

#define TXEIE__SPI_CR2				0x07
#define RXNEIE__SPI_CR2				0x06

#define ERRIE__SPI_CR2				0x05

void configSPI4(void);
void spi4ReceiveData(uint8_t *data, uint8_t *data2);


#endif  //SPI_H_
