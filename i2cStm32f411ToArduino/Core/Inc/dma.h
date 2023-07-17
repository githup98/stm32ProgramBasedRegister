/*
 * dma.h
 *
 *  Created on: Jul 17, 2023
 *      Author: hello
 */

#ifndef DMA_H_
#define DMA_H_

#include <stdint.h>

#define RCC_BASE_ADDR 				0x40023800
#define DMA2_BASE_ADDR              0x40026400  //for SPI4

#define SECTOR_7_FLASH              0x08060000

#define SPI4_BASE_ADDR              0x40013400

#define DMA2EN__RCC_AHB1ENR         22

#define CHSEL__DMA_SxCR             25
#define MBURST__DMA_SxCR	        23
#define PBURST__DMA_SxCR	        21
#define DBM__DMA_SxCR	            18
#define PL__DMA_SxCR			    16
#define MSIZE__DMA_SxCR             13
#define PSIZE__DMA_SxCR             11
#define MINC__DMA_SxCR              0x0A
#define PINC__DMA_SxCR              0x09
#define CIRC__DMA_SxCR              0x08
#define DIR__DMA_SxCR               0x06
#define PFCTRL__DMA_SxCR			0x05
#define TCIE__DMA_SxCR			    0x04
#define HTIE__DMA_SxCR			    0x03
#define TEIE__DMA_SxCR			    0x02
#define DMEIE__DMA_SxCR			    0x01
#define EN__DMA_SxCR                0x00


#define DMDIS__DMA_SxFCR            0x02



void dmaConfig(uint8_t numStream, uint8_t numChannel, uint32_t *des, uint8_t dir);

#endif /* DMA_H_ */
