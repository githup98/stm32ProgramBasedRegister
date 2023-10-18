/*
 * usb.h
 *
 *  Created on: Jul 26, 2023
 *      Author: hello
 */

#ifndef INC_NEWUSB_H_
#define INC_NEWUSB_H_

#include <stdint.h>
#include "usbEpCtl.h"
#define RCC_BASE_ADDR						0x40023800

#define OTGFSEN__RCC_AHB2ENR				0x07

#define GPIOAEN__RCC_AHB1ENR				0x00

#define GPIOA_BASE_ADDR						0x40020000

#define OTG_FS_BASE_ADDR					0x50000000
#define OTG_FIFO_BASE						0x1000 
#define OTG_FIFO_SIZE						0x1000 

//define for  Config Des
#ifndef USB_CDC_CONFIG_DESC_SIZ
#define USB_CDC_CONFIG_DESC_SIZ				67
#endif

#define USB_DESC_TYPE_CONFIGURATION			0x02
#define USBD_SELF_POWERED					0x01
#define USBD_MAX_POWER						0x32
#define USB_DESC_TYPE_INTERFACE				0x04
#define USB_DESC_TYPE_ENDPOINT				0x05

#ifndef LOBYTE
#define LOBYTE(x)							((uint8_t)((x) & 0x00FF))
#endif

#ifndef HIBYTE
#define HIBYTE(x)							((uint8_t)(((x) & 0xff00) >> 8)) //>> 8 due to we just got 1 byte
#endif


#define pOTG_FS_DIEPINT(x) 					((uint32_t*)(OTG_FS_BASE_ADDR + 0x908 + 0x20*(x)))
#define pOTG_FS_DIEPCTL(x) 					((uint32_t*)(OTG_FS_BASE_ADDR + 0x900 + 0x20*(x)))
#define pOTG_FS_DIEPTSIZ(x) 				((uint32_t*)(OTG_FS_BASE_ADDR + 0x910 + 0x20*(x)))
#define pOTG_FS_DOEPINT(x) 					((uint32_t*)(OTG_FS_BASE_ADDR + 0xB08 + 0x20*(x)))
#define pOTG_FS_DOEPCTL(x) 					((uint32_t*)(OTG_FS_BASE_ADDR + 0xB00 + 0x20*(x)))
#define pOTG_FS_DOEPTSIZ(x) 				((uint32_t*)(OTG_FS_BASE_ADDR + 0xB10 + 0x20*(x)))

#define CDC_CMD_PACKET_SIZE					0x08
#define CDC_DATA_FS_MAX_PACKET_SIZE			64
#define CDC_FS_BINTERVAL					0x10
#define CDC_CMD_EP							0x82
#define CDC_OUT_EP							0x01
#define CDC_IN_EP							0x81

#define CDC_IN_EP_ADDR 						CDC_IN_EP
#define CDC_OUT_EP_ADDR 					CDC_OUT_EP


#define EP_TYPE_ISOC 						0x01
#define USBD_EP_TYPE_BULK					0x02
#define USBD_EP_TYPE_INTR					0x03
#define CDC_DATA_FS_IN_PACKET_SIZE 			64
#define CDC_DATA_FS_OUT_PACKET_SIZE 		64
#define CDC_CMD_EP_ADDR 					CDC_CMD_EP

#define Tx_BUFFER_SIZE 						2048
#define Rx_BUFFER_SIZE 						2048


#define MAX_PACKET_SIZE_EP0 				64 //64
#define MAX_PACKET_SIZE_EPx 				64 //64


///////////////////


#define AHB_IDEL_FAIL						1
#define SOFT_RESET_FAIL						2
#define SOFT_RESET_OK						0

#define SET_MODE_FAIL						1
#define SET_MODE_OK							0

#define SETUP_PKG_RECEIVED					0x06


#define MODER09								18
#define MODER10								20
#define MODER11								22
#define MODER12								24

#define OT09								9
#define OT10								10
#define OT11								11
#define OT12								12

#define OSPEEDR09							18
#define OSPEEDR10							20
#define OSPEEDR11							22
#define OSPEEDR12							24

#define PUPDR09								18
#define PUPDR10								20
#define PUPDR11								22
#define PUPDR12								24

#define AFRH09								4
#define AFRH10								8
#define AFRH11								12
#define AFRH12								16

#define GINTMSK_							0x00

#define RXFLVL								0x04

//USBCFG
#define FDMOD								30
#define FHMOD								29
#define PHYSEL								0x06
#define HNPCAP								0x09
#define SRPCAP								0x08
#define TRDT								0x0A
#define TOCAL								0x00


//DCFG
#define NZLSOHSK							0x02
#define DSPD								0x00
#define DAD									0x04

//DOEPTSIZ
#define PKTCNT								19
#define XFRSIZ								0x00
#define STUPCNT								29



//DIEPINTx
#define TXFE								0x07
#define TOC									0x03
#define EPDISD								0x01
#define XFRC								0x00


//DOEPINTx
#define NAK									13
#define STSPHSRX							0x05
#define STUP								0x03





//GINTSTS
#define MMIS								0x01
#define USBSUSP								11
#define IISOIXFR							20
#define IPXFR_DEVICE_MOD					21
#define WUI									31

//GINTMSK
#define WUIM								31
#define SRQIM								30
#define IPXFRM_DEVICE_MOD					21
#define IISOIXFRM							20   /*interrupt for incomplete transfer iso mode*/
#define OEPINT								19
#define IEPINT								18
#define USBRST								12
#define ENUMDNEM							13
#define USBSUSPM							11
#define ESUSPM								10
#define SOFM								0x03
#define OTGINT								0x02
#define MMISM								0x01


//GRXSTSR
#define PKTSTS								17



//GCCFG
#define NOVBUSSENS							21
#define VBUSBSEN							19
#define VBUSASEN							18
#define PWRDWN								16

//DCTL
#define CGINAK								0x08
#define SDIS								0x01
#define RWUSIG								0x00


//DOEPINTx
#define SNAK								27
#define STALL								21

//DAINTMSK
#define IEPM0								0
#define OEPM0								16

//DIEPMSK
#define TOM									0x03
#define EPDM								0x01
#define XFRCM								0x00

//DOEPMSK
#define STUPM								0x03
#define NAKM								13
#define STSPHSRXM							0x05

#define RXFD								0x00

#define TX0FD								16
#define TX0FSA								0x00

#define STUPCNT								29

#define ENUMDNE								13

//2-bit
#define MPSIZ__OTG_FS_DIEPCTL0				0x00

//DIEPCTL
#define EPENA								31
#define SD0PID_SEVNFRM						28      //set pid for data0 (in bulk/interrupt mode), set even frame (in isochronous mode)
#define TXFNUM								22
#define EPTYP								18
#define USBAEP								15
//11-bit(in byte format)
#define MPSIZ__OTG_FS_DIEPCTLx				0x00

//GINTMSK
#define NPTXFEM								0x05
#define RXFLVLM								0x04

//HNPTXFSIZ__DIEPTXF0
#define NPTXFD__TX0FD						16
#define NPTXFSA__TX0FSA						0x00

//PCGCCTL
#define GATEHCLK							0x01
#define STPPCLK								0x00

//DAINTMSK
#define INEPM0								0x00
#define OUTEPM0								16


//DOEPCTL
#define EPENA								31
#define CNAK								26

//OTG_FS_DIEPTSIZ0
#define PKTCNT								19
#define XFRSIZ								0x00

//DSTS
#define ENUMSPD								0x01


//void pinConfig(void);

//void coreUsbInit(void);

//void deviceUsbInit(void);

void usbInterruptHandler(void);

void initUsb(void);

#endif /* INC_NEWUSB_H_ */
