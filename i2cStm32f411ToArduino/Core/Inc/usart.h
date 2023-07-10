/*
 * usart.h
 *
 *  Created on: Jul 3, 2023
 *      Author: hello
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include <stdint.h>

#define GPIOA_BASE_ADDR                  0x40020000
#define GPIOB_BASE_ADDR                  0x40020400

#define USART1_BASE_ADDR                 0x40011000
#define RCC_BASE_ADDR                    0x40023800



#define USART1EN__RCC_APB2ENR            0x04
#define GPIOAEN__RCC_AHB1ENR             0x00


#define MODER10__GPIOx_MODER             20
#define MODER9__GPIOx_MODER              18
#define OT10__GPIOx_OTYPER               10
#define OT9__GPIOx_OTYPER                0x09

#define OSPEEDR10__GPIOx_OSPEEDR         20
#define OSPEEDR9__GPIOx_OSPEEDR          18

#define PUPDR10__GPIOx_PUPDR             20
#define PUPDR9__GPIOx_PUPDR              18

#define AFRH10__GPIOx_AFRH               0x08
#define AFRH9__GPIOx_AFRH                0x04

#define UE__USART_CR1                    13
#define M__USART_CR1                     12
#define RXNEIE__USART_CR1				 0x05
#define TE__USART_CR1                    0x03
#define RE__USART_CR1                    0x02

#define TXE__USART_SR                    0x07
#define TC__USART_SR                     0x06
#define RXNE__USART_SR					 0x05
#define ORE__USART_SR                    0x03

#define STOP__USART_CR2                  12




void usart1Config(void);

void usart1ListenData(void);

void usart1SendOneByte(uint8_t *data, uint8_t numToSend);

void usart1ReceiveBytes(uint8_t *data);

#endif /* INC_USART_H_ */
