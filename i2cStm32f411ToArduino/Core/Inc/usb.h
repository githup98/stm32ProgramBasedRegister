/*
 * usb.h
 *
 *  Created on: Jul 22, 2023
 *      Author: hello
 */

#ifndef INC_USB_H_
#define INC_USB_H_

#include <stdint.h>

#define RCC_BASE_ADDR 			    0x40023800

#define OTGFSEN__RCC_AHB2ENR        0x07

#define GPIOAEN__RCC_AHB1ENR        0x00

#define GPIOA_BASE_ADDR 			0x40020000

#define OTG_FS_BASE_ADDR            0x50000000

#define MODER11						22
#define MODER12 					24

#define OT11						11
#define OT12 						12

#define OSPEEDR11 					22
#define OSPEEDR12  					24

#define PUPDR11 					22
#define PUPDR12  					24

#define AFRH11						12
#define AFRH12 						16

#define GINTMSK                     0x00

#define RXFLVL                      0x04

#define HNPCAP                      0x09
#define SRPCAP                      0x08
#define TRDT                        0x0A
#define TOCAL                       0x00

#define OTGINT                      0x02
#define MMISM                       0x01

#define NZLSOHSK                    0x02
#define DSPD                        0x00

#define USBRST                      12
#define ENUMDNEM                    13
#define USBSUSPM                    11
#define ESUSPM                      10
#define SOFM                        0x03

#define VBUSBSEN                    19

#define SNAK                        27

#define IEPM0                       0x00
#define OEPM0                       16

#define TOM                         0x03
#define XFRCM                       0x00

#define STUPM                       0x03

#define RXFD                        0x00

#define TX0FD                       16
#define TX0FSA                      0x00

#define STUPCNT                     29

#define ENUMDNE						13

//2-bit
#define MPSIZ__OTG_FS_DIEPCTL0      0x00

#define DAD                         0x04

#define EPENA                       31
#define TXFNUM                      22
#define EPTYP                       18
#define USBAEP                      15
//11-bit(in byte format)
#define MPSIZ__OTG_FS_DIEPCTLx      0x00


#define NPTXFEM                     0x05
#define RXFLVLM                     0x04


void pinConfig(void);

void coreUsbInit(void);

void deviceUsbInit(void);


#endif /* INC_USB_H_ */
