/*
 * usb.c
 *
 *  Created on: Jul 22, 2023
 *      Author: hello
 */
#include "usb.h"

void pinConfig(void)
{
	//DP PA12, DM PA11
	//enable GPIOA registers
	uint32_t *pRCC_AHB1ENR = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCC_AHB1ENR |= (1 << GPIOAEN__RCC_AHB1ENR);

	//MODE: alter function mode
	uint32_t *pGPIOA__GPIOx_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*pGPIOA__GPIOx_MODER &= ~(3 << MODER11);
	*pGPIOA__GPIOx_MODER |= (2 << MODER11);
	*pGPIOA__GPIOx_MODER &= ~(3 << MODER12);
	*pGPIOA__GPIOx_MODER |= (2 << MODER12);

	//otyper push-pull
	uint32_t *pGPIOA__GPIOx_OTYPER = (uint32_t*)(GPIOA_BASE_ADDR + 0x04);
	*pGPIOA__GPIOx_OTYPER &= ~(1 << OT11);
	*pGPIOA__GPIOx_OTYPER &= ~(1 << OT12);

	//speed: fast
	uint32_t *pGPIOA__GPIOx_OSPEEDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x08);
	*pGPIOA__GPIOx_OSPEEDR &= ~(3 << OSPEEDR11);
	*pGPIOA__GPIOx_OSPEEDR |= (2 << OSPEEDR11);
	*pGPIOA__GPIOx_OSPEEDR &= ~(3 << OSPEEDR12);
	*pGPIOA__GPIOx_OSPEEDR |= (2 << OSPEEDR12);

	//pull-up/down: no
	uint32_t *pGPIOA__GPIOx_PUPDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x0C);
	*pGPIOA__GPIOx_PUPDR &= ~(3 << PUPDR11);
	*pGPIOA__GPIOx_PUPDR &= ~(3 << PUPDR12);

	//assign OTG_FS function
	uint32_t *pGPIOA__GPIOx_AFRH = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOA__GPIOx_AFRH &= ~(0x0f << AFRH11);
	*pGPIOA__GPIOx_AFRH |= (0x0A << AFRH11);
	*pGPIOA__GPIOx_AFRH &= ~(0x0f << AFRH12);
	*pGPIOA__GPIOx_AFRH |= (0x0A << AFRH12);
}

void coreUsbInit(void)
{
	/*
	1.Program the following fields in the OTG_FS_GAHBCFG register:
		– Global interrupt mask bit GINTMSK = 1
		– RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
		– Periodic TxFIFO empty level
	2.Program the following fields in the OTG_FS_GUSBCFG register:
		– HNP capable bit
		– SRP capable bit
		– FS timeout calibration field
		– USB turnaround time field
	3.The software must unmask the following bits in the OTG_FS_GINTMSK register:
	OTG interrupt mask
	Mode mismatch interrupt mask
	4.
	The software can read the CMOD bit in OTG_FS_GINTSTS to determine whether the
	OTG_FS controller is operating in host or device mode.
	*/

	//enable usb registers
	uint32_t *pRCC_AHB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x34);
	*pRCC_AHB2ENR |= (1 << OTGFSEN__RCC_AHB2ENR);

	//enable global interrupt (for OTG_FS)
	uint32_t *pOTG_FS_GAHBCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x008);
	*pOTG_FS_GAHBCFG |= (1 << GINTMSK);

	//set RX FIFO non-empty (read only bit)
	//uint32_t *pOTG_FS_GINTSTS = (uint32_t*)(OTOTG_FS_BASE_ADDR + 0x014);

	uint32_t *pOTG_FS_GUSBCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x00C);
	*pOTG_FS_GUSBCFG &= ~(1 << HNPCAP); //disable HNP due to device mode only
	*pOTG_FS_GUSBCFG |= (1 << SRPCAP); //enable SRP due to device mode only

	//FS timeout calibration field
	*pOTG_FS_GUSBCFG &= ~(7 << TOCAL);
	*pOTG_FS_GUSBCFG |= (5 << TOCAL); //randomly select

	//USB turn-around time field: 0x06 due to AHB clock equal to 32 MHZ
	*pOTG_FS_GUSBCFG &= ~(0x0f << TRDT);
	*pOTG_FS_GUSBCFG |= (6 << TRDT);

	//software must un-mask: OTG interrupt mask,	Mode mismatch interrupt mask (miss mode: access register in host mode while running in device mode vice versa)
	uint32_t *pOTG_FS_GINTMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x018);
	*pOTG_FS_GINTMSK |= (1 << OTGINT);
	*pOTG_FS_GINTMSK |= (1 << MMISM);

}


void ePInitWhenEnumerateComplete(void)
{
	//Endpoint initialization on enumeration completion
	/*
	1.On the Enumeration Done interrupt (ENUMDNE in OTG_FS_GINTSTS), read the
	OTG_FS_DSTS register to determine the enumeration speed.
	2. Program the MPSIZ field in OTG_FS_DIEPCTL0 to set the maximum packet size. This
	step configures control endpoint 0. The maximum packet size for a control endpoint
	depends on the enumeration speed.
	*/

	uint32_t *pOTG_FS_GINTSTS = (uint32_t*)(OTG_FS_BASE_ADDR + 0x014);
	while(! ((*pOTG_FS_GINTSTS >> ENUMDNE) & 0x01));  //can use interrput handler instead of while loop
	uint32_t *pOTG_FS_DSTS = (uint32_t*)(OTG_FS_BASE_ADDR + 0x808); //read this register to get enumerate speed (which is used in FS timeout calibration field)

	//max size for
	uint32_t *pOTG_FS_DIEPCTL0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0x900);
	*pOTG_FS_DIEPCTL0 &= ~(3 << MPSIZ__OTG_FS_DIEPCTL0);
	*pOTG_FS_DIEPCTL0 |= (3 << MPSIZ__OTG_FS_DIEPCTL0); //maximum packet size of usb full speed is 64 bytes (Usb in a nutshell)
}

void ePInitWhenSetAddressCommand(void)
{
	//Endpoint initialization on SetAddress command
	/*
	This section describes what the application must do when it receives a SetAddress
	command in a SETUP packet.
	1. Program the OTG_FS_DCFG register with the device address received in the
	SetAddress command
	2. Program the core to send out a status IN packet
	*/
	uint32_t *pOTG_FS_DCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x800);
	*pOTG_FS_DCFG &= ~(0x7f << DAD);
	//*pOTG_FS_DCFG &= ~(0x7f << DAD); //wait for address from host

	//Program the core to send out a status IN packet
}

void ePInitWhenSetConfigurationCommand(void)
{
	/*
	1. When a SetConfiguration command is received, the application must program the
	endpoint registers to configure them with the characteristics of the valid endpoints in
	the new configuration.
	2. When a SetInterface command is received, the application must program the endpoint
	registers of the endpoints affected by this command.
	3. Some endpoints that were active in the prior configuration or alternate setting are not
	valid in the new configuration or alternate setting. These invalid endpoints must be
	deactivated.
	4. Unmask the interrupt for each active endpoint and mask the interrupts for all inactive
	endpoints in the OTG_FS_DAINTMSK register.
	5. Set up the Data FIFO RAM for each FIFO.
	6. After all required endpoints are configured; the application must program the core to
	send a status IN packet.
	*/


}


void ePActivation(void)
{
	/*This section describes the steps required to activate a device endpoint or to configure an
	existing device endpoint to a new type.

	1.Program the characteristics of the required endpoint into the following fields of the
	OTG_FS_DIEPCTLx register (for IN or bidirectional endpoints) or the
	OTG_FS_DOEPCTLx register (for OUT or bidirectional endpoints).
	– Maximum packet size
	– USB active endpoint = 1
	– Endpoint start data toggle (for interrupt and bulk endpoints)
	– Endpoint type
	– TxFIFO number

	2.Once the endpoint is activated, the core starts decoding the tokens addressed to that
	endpoint and sends out a valid handshake for each valid token received for the
	endpoint.
	*/

	//uint32_t *pOTG_FS_DIEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + );
	//*pOTG_FS_DIEPCTLx &= ~(0x7ff << MPSIZ__OTG_FS_DIEPCTLx);
	//*pOTG_FS_DIEPCTLx |= ( << MPSIZ__OTG_FS_DIEPCTLx);
	//*pOTG_FS_DIEPCTLx |= (1 << USBAEP);
	//*pOTG_FS_DIEPCTLx |= (1 << EPENA);
	//*pOTG_FS_DIEPCTLx &= ~(3 << EPTYP);
	//*pOTG_FS_DIEPCTLx |= ( << EPTYP); //control or interrupt or bulk
	//*pOTG_FS_DIEPCTLx &= ~(0x0f << TXFNUM);
	//*pOTG_FS_DIEPCTLx |= ( << TXFNUM);   // only for IN endpoint


}

void ePDeactivation(void)
{
	/*
	This section describes the steps required to deactivate an existing endpoint.
	1. In the endpoint to be deactivated, clear the USB active endpoint bit in the
	OTG_FS_DIEPCTLx register (for IN or bidirectional endpoints) or the
	OTG_FS_DOEPCTLx register (for OUT or bidirectional endpoints).
	2. Once the endpoint is deactivated, the core ignores tokens addressed to that endpoint,
	which results in a timeout on the USB.
	Note: The application must meet the following conditions to set up the device core to handle
	traffic:
	NPTXFEM and RXFLVLM in the OTG_FS_GINTMSK register must be cleared.
	*/

	//set up the device core to handletraffic
	///uint32_t *pOTG_FS_GINTMSK = (uint32_t*)(OTG_FS_BASE_ADDR + );
	//*pOTG_FS_GINTMSK &= ~(1 << NPTXFEM);
	//*pOTG_FS_GINTMSK &= ~(1 << RXFLVLM);

	//uint32_t *pOTG_FS_DIEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + );
	//*pOTG_FS_DIEPCTLx &= ~(1 << USBAEP);

}


void deviceUsbInitStep2(void)
{
	//Endpoint initialization on USB reset (user manual)
	/*
		SNAK = 1 in OTG_FS_DOEPCTLx (for all OUT endpoints)

	2	Unmask the following interrupt bits
– INEP0 = 1 in OTG_FS_DAINTMSK (control 0 IN endpoint)
– OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
– STUP = 1 in DOEPMSK
– XFRC = 1 in DOEPMSK
– XFRC = 1 in DIEPMSK
– TOC = 1 in DIEPMSK

3 Set up the Data FIFO RAM for each of the FIFOs
– Program the OTG_FS_GRXFSIZ register, to be able to receive control OUT data
and setup data. If thresholding is not enabled, at a minimum, this must be equal to
1 max packet size of control endpoint 0 + 2 words (for the status of the control
OUT data packet) + 10 words (for setup packets).
– Program the OTG_FS_TX0FSIZ register (depending on the FIFO number chosen)
to be able to transmit control IN data. At a minimum, this must be equal to 1 max
packet size of control endpoint 0.

4 Program the following fields in the endpoint-specific registers for control OUT endpoint
0 to receive a SETUP packet
–STUPCNT = 3 in OTG_FS_DOEPTSIZ0 (to receive up to 3 back-to-back SETUP
packets)
	 */
	uint32_t *pOTG_FS_DOEPCTL0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB00); //for END Point 0 (special end point)
	*pOTG_FS_DOEPCTL0 |= (1 << SNAK); //set not acknowledge

	uint32_t *pOTG_FS_DOEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB00 + 0x20 * 1); //for END Point x
	*pOTG_FS_DOEPCTLx |= (1 << SNAK); //set not acknowledge
	uint32_t *pOTG_FS_DOEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB00 + 0x20 * 2); //for END Point x
	*pOTG_FS_DOEPCTLx |= (1 << SNAK); //set not acknowledge
	uint32_t *pOTG_FS_DOEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB00 + 0x20 * 3); //for END Point x
	*pOTG_FS_DOEPCTLx |= (1 << SNAK); //set not acknowledge

	//Unmask the following interrupt bits: IEPM0, OEPM0, TOM, XFRCM (OTG_FS_DIEPMSK), STUPM, XFRCM (OTG_FS_DOEPMSK)
	uint32_t *pOTG_FS_DAINTMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x81C);
	*pOTG_FS_DAINTMSK |= (1 << IEPM0);
	*pOTG_FS_DAINTMSK |= (1 << OEPM0);
	uint32_t *pOTG_FS_DIEPMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x810);
	*pOTG_FS_DIEPMSK |= (1 << TOM);
	*pOTG_FS_DIEPMSK |= (1 << XFRCM);
	uint32_t *pOTG_FS_DOEPMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x814);
	*pOTG_FS_DOEPMSK |= (1 << STUPM);
	*pOTG_FS_DOEPMSK |= (1 << XFRCM);

	//setup data FIFO RAM
	uint32_t *pOTG_FS_GRXFSIZ = (uint32_t*)(OTG_FS_BASE_ADDR + 0x024);
	*pOTG_FS_GRXFSIZ &= ~(0xff << RXFD);
	*pOTG_FS_GRXFSIZ |= (128 << RXFD);  //128 bytes

	//Program the OTG_FS_TX0FSIZ register,  see table 127, user manual
	uint32_t *pOTG_FS_DIEPTXF0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0x028);
	*pOTG_FS_DIEPTXF0 &= ~(0xff << TX0FSA);
	*pOTG_FS_DIEPTXF0 |= (128 << TX0FSA);
	*pOTG_FS_DIEPTXF0 &= ~(0xff << TX0FD);
	*pOTG_FS_DIEPTXF0 |= (64 << TX0FD);

	//receive a SETUP packet
	uint32_t *pOTG_FS_DOEPTSIZ0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB10);
	*pOTG_FS_DOEPTSIZ0 &= ~(3 << STUPCNT);
	*pOTG_FS_DOEPTSIZ0 |= (3 << STUPCNT);

	initEPAfterEnumCom();
}


void deviceUsbInit(void)
{
	/*
	1.Program the following fields in the OTG_FS_DCFG register:
	– Device speed
	– Non-zero-length status OUT handshake
	2.Program the OTG_FS_GINTMSK register to unmask the following interrupts:
	– USB reset
	– Enumeration done
	– Early suspend
	– USB suspend
	– SOF
	3. Program the VBUSBSEN bit in the OTG_FS_GCCFG register to enable V BUS sensing
	in “B” device mode and supply the 5 volts across the pull-up resistor on the DP line.
	4. Wait for the USBRST interrupt in OTG_FS_GINTSTS. It indicates that a reset has been
	detected on the USB that lasts for about 10 ms on receiving this interrupt.
	*/

	//speed and OUT Handshake
	uint32_t *pOTG_FS_DCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x800);
	*pOTG_FS_DCFG &= ~(1 << NZLSOHSK); //STALL but still send received OUT packet
	*pOTG_FS_DCFG &= ~(3 << DSPD);
	*pOTG_FS_DCFG |= (3 << DSPD); //full speed

	//un-mask USB reset ... SOF
	uint32_t *pOTG_FS_GINTMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x018);
	*pOTG_FS_GINTMSK |= (1 << USBRST);
	*pOTG_FS_GINTMSK |= (1 << ENUMDNEM);
	*pOTG_FS_GINTMSK |= (1 << USBSUSPM);
	*pOTG_FS_GINTMSK |= (1 << ESUSPM);
	*pOTG_FS_GINTMSK |= (1 << SOFM);

	//enable vbus B device
	uint32_t *pOTG_FS_GCCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x038);
	*pOTG_FS_GCCFG |= (1 << VBUSBSEN);

	//wait for usb reset to init endpoint (device programming model)
}

