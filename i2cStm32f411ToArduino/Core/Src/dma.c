/*
 * dma.c
 *
 *  Created on: Jul 17, 2023
 *      Author: hello
 */


#include "dma.h"

/* all steps
1. If the stream is enabled, disable it by resetting the EN bit in the DMA_SxCR register,
then read this bit in order to confirm that there is no ongoing stream operation. Writing
this bit to 0 is not immediately effective since it is actually written to 0 once all the
current transfers have finished. When the EN bit is read as 0, this means that the
stream is ready to be configured. It is therefore necessary to wait for the EN bit to be
cleared before starting any stream configuration. All the stream dedicated bits set in the
status register (DMA_LISR and DMA_HISR) from the previous data block DMA
transfer should be cleared before the stream can be re-enabled.
2. Set the peripheral port register address in the DMA_SxPAR register. The data will be
moved from/ to this address to/ from the peripheral port after the peripheral event.
3. Set the memory address in the DMA_SxMA0R register (and in the DMA_SxMA1R
register in the case of a double buffer mode). The data will be written to or read from
this memory after the peripheral event.
4. Configure the total number of data items to be transferred in the DMA_SxNDTR
register. After each peripheral event or each beat of the burst, this value is
decremented.
5. Select the DMA channel (request) using CHSEL[2:0] in the DMA_SxCR register.
6. If the peripheral is intended to be the flow controller and if it supports this feature, set
the PFCTRL bit in the DMA_SxCR register.
7. Configure the stream priority using the PL[1:0] bits in the DMA_SxCR register.
8. Configure the FIFO usage (enable or disable, threshold in transmission and reception)
9. Configure the data transfer direction, peripheral and memory incremented/fixed mode,
single or burst transactions, peripheral and memory data widths, Circular mode,
Double buffer mode and interrupts after half and/or full transfer, and/or errors in the
DMA_SxCR register.
10. Activate the stream by setting the EN bit in the DMA_SxCR register.
*/


void dmaConfig(uint8_t numStream, uint8_t numChannel, uint32_t *des, uint8_t dir) //Tx channel 4 stream 1, Rx channel 5 stream 3
{

	uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCC_AHB1ENR |= (1 << DMA2EN__RCC_AHB1ENR);

	uint32_t *pDMA_SxCR = (uint32_t*)(DMA2_BASE_ADDR +  0x10 + (0x18 * numStream));

	//disable DMA
	*pDMA_SxCR &= ~(1 << EN__DMA_SxCR);

	//confirm all transfer on the stream have done
	while((*pDMA_SxCR >> EN__DMA_SxCR) & 0x01);

	//peripheral address (data register where data is read/written from/to), in this case is SPI_DR register
	uint32_t *pDMA_SxPAR = (uint32_t*)(DMA2_BASE_ADDR + 0x18 + (0x18 * numStream));
	*pDMA_SxPAR = (SPI4_BASE_ADDR + 0x0C);

	//memory address
	uint32_t *pDMA_SxM0AR = (uint32_t*)(DMA2_BASE_ADDR + 0x1C + (0x18 * numStream));
	//*pDMA_SxM0AR = (SECTOR_7_FLASH + 0x00);
	*pDMA_SxM0AR = (uint32_t)des;

	//total number of data items
	uint32_t *pDMA_SxNDTR = (uint32_t*)(DMA2_BASE_ADDR + 0x14 + (0x18 * numStream));
	*pDMA_SxNDTR = 0xA0; //10 data items are transfered

	//select channel
	*pDMA_SxCR &= ~(7 << CHSEL__DMA_SxCR); //clear previous value
	*pDMA_SxCR |= (numChannel << CHSEL__DMA_SxCR);

	//select controller:peripheral
	*pDMA_SxCR |= (1 << PFCTRL__DMA_SxCR);

	//set stream priority (low)
	*pDMA_SxCR &= ~(1 << PL__DMA_SxCR);

	//set FIFO usage: disable, using direct mode
	uint32_t *pDMA_SxFCR = (uint32_t*)(DMA2_BASE_ADDR + 0x24 + (0x24 * numStream));
	*pDMA_SxFCR &= ~(1 << DMDIS__DMA_SxFCR);

	//config data transfer datails
	*pDMA_SxCR &= ~(1 << DIR__DMA_SxCR); //direct: peripheral to memory
	*pDMA_SxCR |= (dir << DIR__DMA_SxCR);

	*pDMA_SxCR &= ~(1 << MINC__DMA_SxCR); //fixed mode
	*pDMA_SxCR &= ~(1 << PINC__DMA_SxCR); //fixed mode

	*pDMA_SxCR &= ~(1 << MBURST__DMA_SxCR); //single transfer
	*pDMA_SxCR &= ~(1 << PBURST__DMA_SxCR); //single transfer

	//data widths
	*pDMA_SxCR &= ~(1 << MSIZE__DMA_SxCR); //8-bits
	*pDMA_SxCR &= ~(1 << PSIZE__DMA_SxCR); //8-bits

	*pDMA_SxCR &= ~(1 << CIRC__DMA_SxCR);  //Circular mode: off

	*pDMA_SxCR &= ~(1 << DBM__DMA_SxCR);  //double buffer: off

	*pDMA_SxCR |= (1 << TCIE__DMA_SxCR);  //enable completely transfer interrupt
	*pDMA_SxCR |= (1 << HTIE__DMA_SxCR);  //enable half transfer interrupt
	*pDMA_SxCR |= (1 << TEIE__DMA_SxCR);  //enable transfer error interrupt
	*pDMA_SxCR |= (1 << DMEIE__DMA_SxCR); //enable direct mode error interrupt

	//enable DMA
	*pDMA_SxCR |= (1 << EN__DMA_SxCR);
}
































