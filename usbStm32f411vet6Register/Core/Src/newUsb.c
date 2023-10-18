#include "newUsb.h"
#include "stm32f4xx_hal.h"

uint32_t *pOTG_FS_GINTMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x018);
uint32_t *pOTG_FS_GINTSTS = (uint32_t*)(OTG_FS_BASE_ADDR + 0x014);
uint32_t *pOTG_FS_GRSTCTL = (uint32_t*)(OTG_FS_BASE_ADDR + 0x010);
uint32_t *pOTG_FS_GRXSTSR = (uint32_t*)(OTG_FS_BASE_ADDR + 0x01C);
uint32_t *pOTG_FS_GRXSTSP = (uint32_t*)(OTG_FS_BASE_ADDR + 0x020);
uint32_t *pOTG_FS_GUSBCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x00C);
uint32_t *pOTG_FS_GAHBCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x008);

uint32_t *pOTG_FS_DAINT = (uint32_t*)(OTG_FS_BASE_ADDR + 0x818);
uint32_t *pOTG_FS_DCTL = (uint32_t*)(OTG_FS_BASE_ADDR + 0x804);
uint32_t *pOTG_FS_DSTS = (uint32_t*)(OTG_FS_BASE_ADDR + 0x808);
uint32_t *pOTG_FS_DCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x800);
uint32_t *pOTG_FS_GCCFG = (uint32_t*)(OTG_FS_BASE_ADDR + 0x038);
uint32_t *pOTG_FS_PCGCCTL = (uint32_t*)(OTG_FS_BASE_ADDR + 0xE00);
uint32_t *pOTG_FS_DIEPMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x810);
uint32_t *pOTG_FS_DOEPMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x814);
uint32_t *pOTG_FS_DAINTMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x81C);

uint32_t *pOTG_FS_DIEPEMPMSK = (uint32_t*)(OTG_FS_BASE_ADDR + 0x834);

uint32_t *pOTG_FS_GRXFSIZ = (uint32_t*)(OTG_FS_BASE_ADDR + 0x024);
uint32_t *pOTG_FS_HNPTXFSIZ__DIEPTXF0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0x028);
uint32_t *pOTG_FS_DIEPTXF1_3 = (uint32_t*)(OTG_FS_BASE_ADDR + 0x104 + 0x04);

////uint32_t *pOTG_FS_DIEPINTx = (uint32_t*)(OTG_FS_BASE_ADDR + 0x908 + 0x20*0);
////uint32_t *pOTG_FS_DOEPINTx = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB08 + 0x20*0);
////uint32_t *pOTG_FS_DOEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB00);
////uint32_t *pOTG_FS_DIEPCTLx = (uint32_t*)(OTG_FS_BASE_ADDR + 0x900);
uint32_t *pOTG_FS_DOEPTSIZ0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0xB10);
uint32_t *pOTG_FS_DIEPTSIZ0 = (uint32_t*)(OTG_FS_BASE_ADDR + 0x910 + 0x20 * 0);

uint32_t *pOTG_FS_DTXFSTS = (uint32_t*)(OTG_FS_BASE_ADDR + 0x918 + 0x20 * 0);





//create struct with packed and aligned 1 for write to fifo

#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT  struct __attribute__((packed, aligned(1)))
#endif

#ifndef __UNALIGNED_32_READ_DATA  //Read mean that fifo will read data into fifo == write data to fifo
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
//#pragma GCC diagnostic ignored "-Wattributes"
__PACKED_STRUCT T_UINT32_READ_DATA{uint32_t v;};
#pragma GCC diagnostic pop
#define __UNALIGNED_32_READ_DATA(addr)  (((const struct T_UINT32_READ_DATA*)(const void*)addr)->v)
#endif

uint8_t DeviceDescriptor[18] __attribute__((aligned(4))) =
{
	0x12, //length = 18 bytes
	0x01, //bDescriptorType = device
	0x00, //bcdUSB = usb 2.0, due to LBS first -> 0x00 -> 0x02
	0x02,
	0x02, //bDeviceClass = CDC (0x02), or 0x00 (defined later in interface desc)
	0x02, //bDeviceSubClass don't care
	0x00, //bDeviceProtocolClass don't care
	0x40, //bMaxPAcketSize0 , why previous is 0x08
	0x83, //idVendor 0x483 (1155 in Dec)
	0x04,
	0x40, //idVendor 0x5740 (22336 in Dec), why previous uis 22288 ?
	0x57,
	0x00, //bcdDevice 1.0
	0x01,
	0x00, //three strings are no need,  iManufacturer
	0x00, //iProduct
	0x00, //iSerialNumber
	0x01, //bNumConfigurations

};





//configuration for USB_FS, cpy from: ../../testUSB\ simple/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

#if USB_CDC_CONFIG_DESC_SIZ // //ttemp is nott used
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                       /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength */
  0x00,
  0x02,                                       /* bNumInterfaces: 2 interfaces */
  0x01,                                       /* bConfigurationValue: Configuration value */
  0x00,                                       /* iConfiguration: Index of string descriptor
                                                 describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                       /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                       /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                             /* MaxPower (mA) */

  /*---------------------------------------------------------------------------*/

  /* Interface Descriptor */
  0x09,                                       /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,                                       /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x01,                                       /* bNumEndpoints: One endpoint used */
  0x02,                                       /* bInterfaceClass: Communication Interface Class */
  0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
  0x01,                                       /* bInterfaceProtocol: Common AT commands */
  0x00,                                       /* iInterface */

  /* Header Functional Descriptor */
  0x05,                                       /* bLength: Endpoint Descriptor size */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x00,                                       /* bDescriptorSubtype: Header Func Desc */
  0x10,                                       /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
  0x00,                                       /* bmCapabilities: D0+D1 */
  0x01,                                       /* bDataInterface */

  /* ACM Functional Descriptor */
  0x04,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,                                       /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,                                       /* bFunctionLength */
  0x24,                                       /* bDescriptorType: CS_INTERFACE */
  0x06,                                       /* bDescriptorSubtype: Union func desc */
  0x00,                                       /* bMasterInterface: Communication class interface */
  0x01,                                       /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                                 /* bEndpointAddress */
  0x03,                                       /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  CDC_FS_BINTERVAL,                           /* bInterval */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
  0x01,                                       /* bInterfaceNumber: Number of Interface */
  0x00,                                       /* bAlternateSetting: Alternate setting */
  0x02,                                       /* bNumEndpoints: Two endpoints used */
  0x0A,                                       /* bInterfaceClass: CDC */
  0x00,                                       /* bInterfaceSubClass */
  0x00,                                       /* bInterfaceProtocol */
  0x00,                                       /* iInterface */

  /* Endpoint OUT Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                                 /* bEndpointAddress */
  0x02,                                       /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                                       /* bInterval */

  /* Endpoint IN Descriptor */
  0x07,                                       /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                                  /* bEndpointAddress */
  0x02,                                       /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                                        /* bInterval */
};
#endif //if CDC




volatile uint8_t totalLengthEp0 = 0;
volatile uint8_t remainLengthEp0 = 0;
uint8_t xferInLen = 0;
uint8_t xferOutLen = 0;

uint8_t UserTxBuff[Tx_BUFFER_SIZE];
uint8_t UserRxBuff[Rx_BUFFER_SIZE];

////volatile void *Descriptor = NULL;  //using volatile to update Descriptor in interrupt context
void *Descriptor = NULL;  //using volatile to update Descriptor in interrupt context
						  //using int or using pointer to function for multi desc

//for debug
#define LIMIT 20
#define LIMIT_1 21
uint8_t indexCheckOderRun = 0;
uint8_t checkOderRun[20] = {0};
uint8_t startStoreOder = 0;

//////

//uint32_t dataFromHost[20] = {0};

struct epStatus
{
	uint8_t In;
	uint8_t Out;
	uint8_t Stall;
	uint8_t type;
};

struct epStatus epStt = 
{
	.In = 0,   //"." mean init not follow order of struct member, in this case it's yes/no is still ok
	.Out = 0,
	.Stall = 0,
	.type = 0,
};



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
//	*pGPIOA__GPIOx_OTYPER &= ~(1 << OT09);
//	*pGPIOA__GPIOx_OTYPER &= ~(1 << OT10);
	*pGPIOA__GPIOx_OTYPER &= ~(1 << OT11);
	*pGPIOA__GPIOx_OTYPER &= ~(1 << OT12);

	//speed: high speed
	uint32_t *pGPIOA__GPIOx_OSPEEDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x08);
	*pGPIOA__GPIOx_OSPEEDR &= ~(3 << OSPEEDR11);
	*pGPIOA__GPIOx_OSPEEDR |= (3 << OSPEEDR11);
	*pGPIOA__GPIOx_OSPEEDR &= ~(3 << OSPEEDR12);
	*pGPIOA__GPIOx_OSPEEDR |= (3 << OSPEEDR12);

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


uint8_t resetCore(void)
{
	//wait for AHB master enter idle state
	__IO uint32_t count = 0U;
	do
	{
		count++;
		HAL_Delay(1U);
		if(count > 20000U)
		{
			return AHB_IDEL_FAIL;
		}
	}while(((*pOTG_FS_GRSTCTL) & 0x80000000) == 0U); //AHBIDL bit is 0 <=> AHB master is running

	*pOTG_FS_GRSTCTL |= (1 << 0); //reset core

	count = 0;
	do
	{
		count++;
		HAL_Delay(1U);
		if(count > 20000U)
		{
			return SOFT_RESET_FAIL;
		}
	}while(((*pOTG_FS_GRSTCTL) & 0x1) == 1U); //AHBIDL bit is 0 <=> AHB master is running
	
	return SOFT_RESET_OK;

}


uint8_t setMode(uint8_t mode)
{
	*pOTG_FS_GUSBCFG &= ~((1 << FDMOD)|(1 << FHMOD));
	*pOTG_FS_GUSBCFG |= (1 << mode);
	HAL_Delay(50);
	if((*pOTG_FS_GINTSTS & 0x1) != (!(mode - 29))) //0: Device, 1: Host
	{
		return SET_MODE_FAIL;
	}
	return SET_MODE_OK;

}

uint8_t setSpeed(uint8_t speed)
{
	//stm32f411 is only support "full speed" mode
	if(speed != 3)
	{
		return 1; //fail
	}
	
	*pOTG_FS_DCFG &= ~(3 << DSPD);
	*pOTG_FS_DCFG |= (speed << DSPD);
	
	return 0; //ok
}


uint8_t devInit(void)
{
	//setup Vbus sens (this will active pullup resistor)
	*pOTG_FS_DCTL |= (1 << SDIS); //disable pull up resistor
	*pOTG_FS_GCCFG |= (1 << NOVBUSSENS);
	*pOTG_FS_GCCFG &= ~(1 << VBUSBSEN);
	*pOTG_FS_GCCFG &= ~(1 << VBUSASEN);

	//reset PHY clock
	*pOTG_FS_PCGCCTL &= ~(0x1F); 

	//set device speed
	if(setSpeed(3) != 0)
	{
		return 1; //set speed fail
	}

	//flush fifo

	//clear all device previous interupt mask setting 
	*pOTG_FS_DIEPMSK &= 0x00;
	*pOTG_FS_DOEPMSK &= 0x00;
	*pOTG_FS_DAINTMSK &= 0x00;
	
	//global interrupt
	*pOTG_FS_GINTMSK &= 0x00;
	*pOTG_FS_GINTSTS &= 0xBFFFFFFF; //except SRQINT

	//set up interrupt
	//enable receive fifo non-empty interrupt, when dma is disable
	*pOTG_FS_GINTMSK |= (1 << RXFLVLM); //unmask

	//enable interrupt for device mode only 
	*pOTG_FS_GINTMSK |= ((1 << USBSUSPM) | (1 << USBRST) | (1 << ENUMDNEM) \
	| (1 << OEPINT) | (1 << IEPINT) | (1 << IISOIXFRM) | (1 << IPXFRM_DEVICE_MOD) \
	| (1 << WUIM));


	//enble sof (for i2s or isoch mode) -> disable in device mode
	// *pOTG_FS_GINTMSK |= (1 << SOFM);

	//enable vbus sense (recommend for switch mode) -> disable in device mode
	// *pOTG_FS_GINTMSK |= ((1 << SRQIM) | (1 << OTGINT));

	return 0; //OK
}

void setRxSizeFifo(uint8_t size)
{
	*pOTG_FS_GRXFSIZ &= 0x0000;
	*pOTG_FS_GRXFSIZ |= size;
}
void setTxSizeFifo(uint8_t numTx, uint8_t size)
{
	uint16_t TxOffset = *pOTG_FS_GRXFSIZ; //get size of Rx due to struct of FIFO
										 //see user manual (FIFO architecture)
	//set for Tx0
	if(numTx == 0)
	{
	*pOTG_FS_HNPTXFSIZ__DIEPTXF0 &= 0x00;
	*pOTG_FS_HNPTXFSIZ__DIEPTXF0 |= ((uint16_t)size << NPTXFD__TX0FD | TxOffset);
	}
	else
	{
		TxOffset += *pOTG_FS_HNPTXFSIZ__DIEPTXF0 >> 16; //size of Tx0
		int i = 0;
		for(i = 0; i < (numTx - 1U); i++)
		{
			TxOffset += (*(pOTG_FS_DIEPTXF1_3 + 0x04*i)) >> 16; //get size of Txx
		}
		*(pOTG_FS_DIEPTXF1_3 + 0x04*i) = (uint16_t)size << 16 | TxOffset;
	}

}


void startDev(void)
{
	//enable global interrupt
	*pOTG_FS_GAHBCFG |= (1 << GINTMSK_);

	//ensure clock gating is open
	*pOTG_FS_PCGCCTL &= ~((1 << STPPCLK) | (1 << GATEHCLK));

	//start connecting
	*pOTG_FS_DCTL &= ~(1 << SDIS);

}





uint32_t epReceivedData[12] = {0};
void USB_ReadPacket(void)
{
	//read 8 byte, it is request device description package from host
	for(int i = 0; i < 2; i++)
	{
		epReceivedData[i] = *(uint32_t*)(OTG_FS_BASE_ADDR + OTG_FIFO_BASE);
	}

}

uint8_t DeviceDescriptor[18];
void *getFSDeviceDescriptor(uint8_t *lenDes)
{
	*lenDes = sizeof(DeviceDescriptor);
	return (void*)DeviceDescriptor;
}

void *getFsConfigurationDesc(uint8_t *lenDes)
{
	*lenDes = sizeof(USBD_CDC_CfgDesc);
	return (void*)USBD_CDC_CfgDesc;
}

uint8_t ep0SendData(uint8_t *pBuff, uint8_t epAddr, uint8_t *lengthData)
{

	if(epStt.In == 1)
	{

		if(*lengthData == 0)
		{
			*pOTG_FS_DIEPTSIZ0 &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DIEPTSIZ0 &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DIEPTSIZ0 |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
		}

		if(*lengthData > MAX_PACKET_SIZE_EP0)
		{
			*lengthData = MAX_PACKET_SIZE_EP0;
		}
		if(*lengthData != 0)
		{


			*pOTG_FS_DIEPTSIZ0 &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DIEPTSIZ0 &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DIEPTSIZ0 |= ((0x3FF << PKTCNT) & (1 << PKTCNT));

			*pOTG_FS_DIEPTSIZ0 |= (*lengthData << XFRSIZ);
		}


		//enable EP (start transmit)
		*pOTG_FS_DIEPCTL(0) |= ((1 << CNAK) | (1 << EPENA)); //respond DATA is send  (show packet in wireshark)

		//enable Tx empty fifo interrupt
		if(*lengthData > 0)
		{
			//for enum 0
			*pOTG_FS_DIEPEMPMSK |= (1 << 0);
		}
	}
	else
	{
		if(*lengthData > 0) //due to if it have data need to transfer, ep will sent 64byte for each transfer 
		{
			*lengthData = MAX_PACKET_SIZE_EP0;
		}
		//if(lenRequired != 0)
		//{
			*pOTG_FS_DOEPTSIZ0 &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DOEPTSIZ0 &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DOEPTSIZ0 |= ((0x3FF << PKTCNT) & (1 << PKTCNT));

			*pOTG_FS_DOEPTSIZ0 |= (*lengthData << XFRSIZ);
			
			//enable EP (start transmit)
			*pOTG_FS_DOEPCTL(0) |= ((1 << CNAK) | (1 << EPENA));

		//}
	}

	return 0;
}



uint8_t epXSendData(uint8_t* pBuff, uint8_t epAddr,  uint8_t *lengthData)
{
	if(epStt.In == 1)
	{

		if(*lengthData == 0)
		{
			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
		}
		if(*lengthData > MAX_PACKET_SIZE_EPx)
		{
			*lengthData = MAX_PACKET_SIZE_EPx;
		}
		if(*lengthData != 0)
		{

			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
			*pOTG_FS_DIEPTSIZ(epAddr & 0x0f) |= (*lengthData << XFRSIZ);
		}


		//enable EP (start transmit)
		*pOTG_FS_DIEPCTL(epAddr & 0x0f) |= ((1 << CNAK) | (1 << EPENA)); //respond DATA is send  (show packet in wireshark)

		//enable Tx empty fifo interrupt
		if(*lengthData > 0)
		{
			//for enum 0
			*pOTG_FS_DIEPEMPMSK |= (1 << 0);
		}
	}
	else
	{
		if(*lengthData > MAX_PACKET_SIZE_EPx)
		{
			*lengthData = MAX_PACKET_SIZE_EPx;
		}
		//if(requiredLength != 0)
		//{
			*pOTG_FS_DOEPTSIZ(epAddr & 0x0f) &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DOEPTSIZ(epAddr & 0x0f) &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DOEPTSIZ(epAddr & 0x0f) |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
			*pOTG_FS_DOEPTSIZ(epAddr & 0x0f) |= (*lengthData << XFRSIZ);

			//enable EP (start transmit)
			*pOTG_FS_DOEPCTL(epAddr & 0x0f) |= ((1 << CNAK) | (1 << EPENA));

		//}
	}

	return 0;
}



void setAddressEp(uint32_t *setupPacket)
{
	uint32_t address = (uint32_t)(setupPacket[0] >> 16);
	if(((setupPacket[1] >> 16) == 0) && ((setupPacket[1] & 0xffff) == 0) && (address < 128))
	{
		*pOTG_FS_DCFG &= ~(0x7f << DAD);
		*pOTG_FS_DCFG |= (address << DAD);
	}

}




#if 1 //move to new file

void usbdActivateEndpoint(uint8_t epNum, uint8_t epType, uint8_t epMaxPacketSize)
{
	if(epStt.In == 1)
	{
		//unmask interrupt for partical end point
		*pOTG_FS_DAINTMSK |= ((0xffff << 0) & (1 << epNum)); //equivalant 2 command: clear previous value, assign new value
		if((*pOTG_FS_DIEPCTL(epNum) >> EPENA) == 0x01) //due to EPENA is 31th bit -> no need to & 0x01
		{
			*pOTG_FS_DIEPCTL(epNum) |= ((epMaxPacketSize << MPSIZ__OTG_FS_DIEPCTL0) | (epType << 18) | (epNum << 22)\
			| (1 << SD0PID_SEVNFRM) | (1 << USBAEP));
		}
	}	
	else
	{
		//unmask interrupt for partical end point
		*pOTG_FS_DAINTMSK |= ((0xffff << 16) & ((1 << epNum) << 16)); //equivalant 2 command: clear previous value, assign new value
		if((*pOTG_FS_DOEPCTL(epNum) >> EPENA) == 0x01) //due to EPENA is 31th bit -> no need to & 0x01
		{
			*pOTG_FS_DOEPCTL(epNum) |= ((epMaxPacketSize << MPSIZ__OTG_FS_DIEPCTL0) | (epType << 18) | (epNum << 22)\
			| (1 << SD0PID_SEVNFRM) | (1 << USBAEP));
		}
	}
}

void openEp(uint8_t epAddress, uint8_t epType, uint8_t epMaxPacketSize)
{
	if((epAddress & 0x80) == 0x80)
	{
		epStt.In = 1;
	}
	else
	{
		epStt.In = 0;
	}
	
	// temp no need
	//ep->num = ep_addr & EP_ADDR_MSK;
	//ep->maxpacket = ep_mps;
	//ep->type = ep_type;

	if(epStt.In != 0)
	{
		//set txfifo
		//ep->tx_fifo_num = ep->num;
	}
	
	// init data PID
	if(epType == 2) //bulk type
	{
		//ep->data_pid_start = 0U;
	}

	//epNum = epAddress & 0x0f;
	usbdActivateEndpoint((epAddress & 0x0f), epType, epMaxPacketSize);

}


void usbdOpenEp(uint8_t epAddress, uint8_t epType, uint8_t epMaxPacketSize)
{
	openEp(epAddress, epType, epMaxPacketSize);
}

#endif //if 0 move to new file


/*usbSend and usbReceive , both are use ep0SendData and epXSendData*/
void usbSend(uint8_t* pBuff, uint8_t epAddr, uint8_t lenData)
{
	////epNum != epAddr
	xferInLen = lenData;

	epStt.In = 1;
	
	if((epAddr & 0x0f) == 0)
	{
		ep0SendData(NULL, epAddr, &xferInLen);

	}
	else
	{
		if(epStt.type == EP_TYPE_ISOC) // = 0x01
		{	
			epXSendData(pBuff, epAddr, &xferInLen);//pBuff due to it contain writePacket func for ISOC type 
		}
		else
		{
			
			epXSendData(NULL, epAddr, &xferInLen); 
		}
	}
}


void usbReceive(uint8_t *pBuff, uint8_t epAddr, uint8_t lenData)
{
	
	xferOutLen = lenData;
	
	epStt.In = 0;	
	if((epAddr & 0x0f) == 0)
	{
		ep0SendData(NULL, epAddr, &xferOutLen);  //<==> EP0StartXfer in HAL library
	}
	else
	{
		if(epStt.type == EP_TYPE_ISOC) // == 0x01	
		{
			epXSendData(pBuff, epAddr, &xferOutLen);
		}
		else
		{
			epXSendData(NULL, epAddr, &xferOutLen);
		}
	}
	
}

void usbdCdcClassInit(uint8_t speed)
{
	if(speed == 0) //usb high speed
	{
		//add some code for this device type
	}
	else if(speed == 3) //usb full speed
	
	{
		//open IN, OUT ep
		//usbdOpenEp(addressm type, size);
		usbdOpenEp(CDC_IN_EP_ADDR, USBD_EP_TYPE_BULK, CDC_DATA_FS_IN_PACKET_SIZE);
		
		//change state of ep 1: is used (no need in tthis period)
		//

		usbdOpenEp(CDC_OUT_EP_ADDR, USBD_EP_TYPE_BULK, CDC_DATA_FS_OUT_PACKET_SIZE);
		
	}

	//open CDC_CMD EP
	usbdOpenEp(CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
	//change state of ep 2: is used (no need in tthis period),
	//

	//init physical interface components (skipped in low level programing)

	//prepair receive next packet
	if(speed == 0)
	{
		//setting for high speed device
	}
	else if(speed == 3)
	{
		//epReceived();
	}

}



void usbdSetClass(uint8_t speed) //parse type usb class
{
	usbdCdcClassInit(speed);
}

uint8_t min(uint8_t len1, uint16_t len2)
{
	return (uint8_t)(len1 < len2 ? len1 : len2);
}


uint8_t checkProblem = 0;

void stdDev(uint32_t *pSetup)
{
	uint16_t lenFromSetupCommand = (pSetup[1] >> 16);
	uint8_t bRequest = (pSetup[0] >> 8) & 0xff;
	uint8_t lenDesc = 0;
	uint8_t speedEnumDone = (*pOTG_FS_DSTS >> ENUMSPD) & 0x03;		

	switch (bRequest)
	{
	case 5:
		setAddressEp(pSetup);
		usbSend(NULL, 0x00, 0);
		break;

	case 6:
		switch(pSetup[0] >> 24)
		{
			case 0x01: //get device descriptor

				Descriptor = (void*)getFSDeviceDescriptor(&lenDesc);
				remainLengthEp0 = min(lenDesc, lenFromSetupCommand);
				totalLengthEp0 = remainLengthEp0;

				usbSend(NULL, 0x80, remainLengthEp0);
				break;

			case 0x02: //get config

				epStt.In = 1;
				//we must create function to change maxpacketsize between HS and FS, check HAL code
				//but now we can skip it due to value default in config des is for FS

				//attention due to size of config desc is larger 64 byte
				
				Descriptor = (void*)getFsConfigurationDesc(&lenDesc);
				remainLengthEp0 = min(lenDesc, lenFromSetupCommand);
				totalLengthEp0 = remainLengthEp0;
				
				usbSend(NULL, 0x80, remainLengthEp0);
				break;

			case 0x06: //get device_qualifer (for device support both HS and FS speed)
				if(speedEnumDone == 0x00) //High speed
				{
					//do some things
				}
				else if(speedEnumDone == 0x03) //full speed
				{
					usbSend(NULL, 0x80, 0); //no send data to avoid waiting from host.
				}
				break;

			default:
				break;
		}

		break;

	case 0x09: //set config
		usbdSetClass(speedEnumDone); //class:cdc or msc or dfu ... and speed param
		break;
	default:
		break;
	}
}

void extractSetupPacket(uint32_t *pSetup)
{
	uint8_t bmRequestType = pSetup[0] & 0xff;
	//if(bmRequestType == 0x80)//get phase transfer direction
	//{
		switch ((bmRequestType >> 5) & 0x03) //get Type
		{
		case 0:
			switch(bmRequestType & 0x1f)     //get Recepient
			{
			case 0:
				stdDev(pSetup);
				break;
			default:
				break;
			}
			break;
		default:
			break;

		}
	//}
	//else
	//{
		////temptly do nothing
	//}
}



void writePacket(uint8_t *dataSrc, uint8_t len)
{
	uint8_t count32 = (len + 3) / 4 ; //+3 'cause of len = 5, 9... bytes
	uint8_t *pSrc = dataSrc;
	for(int i = 0; i < count32; i++)
	{
		*(uint32_t*)(OTG_FS_BASE_ADDR + OTG_FIFO_BASE) = __UNALIGNED_32_READ_DATA(pSrc);
		pSrc++;
		pSrc++;
		pSrc++;
		pSrc++;
	}
}


static void  writeEmpFifo(uint8_t **data, uint8_t len)
{
	uint8_t lenLocal = 0;
	uint8_t len32 = 0;
	uint8_t count = 0;

	lenLocal = len - count;

	if(lenLocal > MAX_PACKET_SIZE_EP0)
	{
		lenLocal = MAX_PACKET_SIZE_EP0; //64 is max packet size, this value should be read from register, this line is aim for testing
	}

	len32 = (lenLocal + 3) / 4U;
	//check available space in fifo (unit in word)
	while((*pOTG_FS_DTXFSTS >= len32) && (len != 0U) && (count < len)) //this line must be modify later
	{
		lenLocal = len - count;
		if(lenLocal > MAX_PACKET_SIZE_EP0)
		{
			lenLocal = MAX_PACKET_SIZE_EP0;
		}

		len32 = (len + 3) / 4U;
		writePacket(*data, len); //

		(*data) += lenLocal; //descTrans += lenLocal
		count += lenLocal;
	}

	// remove tx empty fifo intererrupt flag for end point 0
	if(lenLocal <= count)
	{
		*pOTG_FS_DIEPEMPMSK &= ~(1 << 0);
	}
	
	return; 	
}


void usbEpSetStall(uint8_t epNum)
{
	if(epStt.In == 1)
	{
		if((((*pOTG_FS_DIEPCTL(epNum) >> 31) & 0x01) == 0) && (epNum != 0)) //need to modify for all CTL register
		{
			*pOTG_FS_DIEPCTL(epNum) |= ~(1 << 30); //this line will clear EPDIS bit (rs bit), mean EPDIS interrupt will be trigger
		}
		*pOTG_FS_DIEPCTL(epNum) |= (1 << STALL);
	}
	else
	{
		if((((*pOTG_FS_DOEPCTL(epNum) >> 31) & 0x01) == 0) && (epNum != 0)) //need to modify for all CTL register
		{
			*pOTG_FS_DOEPCTL(epNum) |= ~(1 << 30); //this line will clear EPDIS bit (rs bit), mean EPDIS interrupt will be trigger
		}
		*pOTG_FS_DOEPCTL(epNum) |= (1 << STALL);
	}
}



void ep0OutStart(void)
{
	//need modify for all DOEPTSIZ register
	
	//if(USB_OTG_CORE_ID_300A)
	//{
	//
	//}


	*pOTG_FS_DOEPTSIZ0 = 0U;
	*pOTG_FS_DOEPTSIZ0 |= (1 << PKTCNT);
	*pOTG_FS_DOEPTSIZ0 |= (3 * 8);
	*pOTG_FS_DOEPTSIZ0 |= (3 << STUPCNT);
	
	//if(dma == 1)
	//{
	//
	//}
}


void stallEP(uint8_t epAddress)
{
	if((epAddress & 0x80) == 0x80)
	{
		epStt.In = 1;
		//epNum =
	}
	else
	{
		epStt.Out = 1;
		//epNum = 
	}
	epStt.Stall = 1;

	usbEpSetStall(epAddress & 0x0f);

	if((epAddress & 0x0f) == 0)
	{
		ep0OutStart(); //reset DOEPTSIZ register
	}
}


//void prepairReceive()
//{
//
//}


uint8_t timeWrite = 0;



void dataInStageCallback(uint32_t *pSetup)///need to add epNum param
{
	
	if(1)//if enum == 0, change if condition for overall
	{
		if(1) //check status: USBD_EP0_DATA_IN, please complete the code for the overview
		{

			if(remainLengthEp0 > MAX_PACKET_SIZE_EP0)
			{
				remainLengthEp0 -= MAX_PACKET_SIZE_EP0;
				usbSend(NULL, 0x80, remainLengthEp0);
				usbReceive(NULL, 0x80, 0);
			}
			else
			{
				stallEP(0x80);
				usbReceive(NULL, 0x80, 0);
			}
		}
	}
}



void epOutComplete(uint8_t eNum)
{
	//if(dma == 1)
	//{
	//
	//}
	
	ep0OutStart();

	//dataOutStage() // in setup phase this function can be ignored
}



void suspendCallback(void)
{
	//if(lowPowerEnable)
	//{
	//
	//}
	*pOTG_FS_PCGCCTL |= (1 << STPPCLK); //stop PHY clock source
	*pOTG_FS_GINTSTS |= (1 << USBSUSP); //clear usususp interrupt flag 
}


void resetUSBSignal(void)
{
	//disable remote wakeup signal
	*pOTG_FS_DCTL &= ~(1 << RWUSIG);

	//init 4 end point (f411 has 4 end point)
	for(int i = 0; i < 4; i++)
	{
		//clear status of interrupt
		*pOTG_FS_DIEPINT(i) = 0xFB7F;
		*pOTG_FS_DOEPINT(i) = 0xFB7F;

		//clear STALL signal
		*pOTG_FS_DIEPCTL(i) &= ~(1 << STALL);
		*pOTG_FS_DOEPCTL(i) &= ~(1 << STALL);

		//enable SNAK for OUT end point
		*pOTG_FS_DOEPCTL(i) |= (1 << SNAK);
	}

	//enable interrupt for end point IN 0//OUT 0
	*pOTG_FS_DAINTMSK |= ((1 << INEPM0) | (1 << OUTEPM0));

	//set interrupt when using endpoint 1 (this is skipped temporary)
	*pOTG_FS_DIEPMSK |= ((1 << TOM) | (1 << XFRCM) | (1 << EPDM));
	*pOTG_FS_DOEPMSK |= ((1 << NAKM) | (1 << STSPHSRXM) | (1 << STUPM) \
	| (1 << XFRCM) | (1 << EPDM));
	
	//set address (address = 0)
	*pOTG_FS_DCFG &= ~(1 << DAD);	

	//setup EP0 for first time receive setup packet
	ep0OutStart();

}


void usbActiveSetup(void)
{
	//set maxpacket size
	*pOTG_FS_DIEPCTL(0) &= ~(1 << MPSIZ__OTG_FS_DIEPCTLx); //00 mean maxpacket size ==  64 bytes (maximum)

	//clear global IN NAK CGINAK bit in DCTL (w bit)
	*pOTG_FS_DCTL |= (1 << CGINAK);

}

uint8_t usbGetSpeed(void)
{
	uint8_t speed;
	if(((*pOTG_FS_DSTS >> 1) & 0x03) == 3) //OTG_FS
	{
		speed = 2; 
	}
	else if(((*pOTG_FS_DSTS >> 1) & 0x03) == 0) //OTG_HS
	{
		speed  = 0;
	}
	else
	{
		speed = 0x0f;
	}
	
	return speed;

}

void usbSetTRDTValue(uint8_t hclk, uint8_t speed)
{
	uint8_t usbTrd;
	if(speed == 2) //OTG_FS
	{
		if ((hclk >= 14200000U) && (hclk < 15000000U))
		{
		  /* hclk Clock Range between 14.2-15 MHz */
		  usbTrd = 0xFU;
		}
		else if ((hclk >= 15000000U) && (hclk < 16000000U))
		{
		  /* hclk Clock Range between 15-16 MHz */
		  usbTrd = 0xEU;
		}
		else if ((hclk >= 16000000U) && (hclk < 17200000U))
		{
		  /* hclk Clock Range between 16-17.2 MHz */
		  usbTrd = 0xDU;
		}
		else if ((hclk >= 17200000U) && (hclk < 18500000U))
		{
			//hclk Clock Range between 17.2-18.5 MHz
			usbTrd = 0xCU;
		}
		else if ((hclk >= 18500000U) && (hclk < 20000000U))
		{
		  /* hclk Clock Range between 18.5-20 MHz */
		  usbTrd = 0xBU;
		}
		else if ((hclk >= 20000000U) && (hclk < 21800000U))
		{
			//hclk Clock Range between 20-21.8 MHz
			usbTrd = 0xAU;
		}
		else if ((hclk >= 21800000U) && (hclk < 24000000U))
		{
		  /* hclk Clock Range between 21.8-24 MHz */
		  usbTrd = 0x9U;
		}
		else if ((hclk >= 24000000U) && (hclk < 27700000U))
		{
		  /* hclk Clock Range between 24-27.7 MHz */
		  usbTrd = 0x8U;
		}
		else if ((hclk >= 27700000U) && (hclk < 32000000U))
		{
		  /* hclk Clock Range between 27.7-32 MHz */
		  usbTrd = 0x7U;
		}
		else /* if(hclk >= 32000000) */
		{
		  /* hclk Clock Range between 32-200 MHz */
		  usbTrd = 0x6U;
		}	
	}
	else if(speed == 0) //OTG_FS
	{
		usbTrd = 9;
	}
	else  //default case
	{
		usbTrd = 9;
	}

	//clear value
	*pOTG_FS_GUSBCFG &= ~(0x0f << TRDT);  //4 bit 10 -> 13
	*pOTG_FS_GUSBCFG |= (usbTrd << TRDT);

}

//void enumSetSpeed(void)
//{
//
//}



void enumReset(void)
{
	//#ifdef USE_USBD_COMPOSITE
	//#endif

	//for OUT EP
	usbdOpenEp(0x00, 2, 64); //2: bulk type
	
	//pdev->ep_out[0x00U & 0xFU].is_used = 1U;
	//pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;
	
	//for IN EP
	usbdOpenEp(0x80, 2, 64);
	
	//pdev->ep_out[0x00U & 0xFU].is_used = 1U;
	//pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;
}


void enumdoneResetCallback(void)
{
	//set speed, this func is used in qualify config
	//enumSetSpeed(); // program based on register isn't consider temply
	
	//reset
	enumReset();
}


uint16_t readOutEpIntrStatus(uint8_t epNum)
{
	return (uint16_t)(*pOTG_FS_DOEPINT(epNum) & *pOTG_FS_DOEPMSK);
}

uint16_t readInEpIntrStatus(uint8_t epNum)
{
	//get TXFE bit for DIEPINT, use tmp due to 7th bit always = 0, if use register it will never change
	uint32_t tmp = *pOTG_FS_DIEPMSK | (((*pOTG_FS_DIEPEMPMSK >> epNum) & 0x01) << TXFE); 	
	return (uint16_t)(tmp &= *pOTG_FS_DIEPINT(epNum));
}


void usbInterruptHandler(void)
{

// handler for usb suspend, reset enumdone, read, get descriptor, transfer data into FIFO, IN done, OUT done


	uint32_t matchedGlobalIntr = ((*pOTG_FS_GINTSTS) & (*pOTG_FS_GINTMSK));


	//check mismatch interrupt
	if(((matchedGlobalIntr >> MMIS) & 1) == 1)
	{
		//clear mismode interrupt bit
		*pOTG_FS_GINTSTS |= (1 << MMIS);
		return;
	}


	//receive data from host
	if(((matchedGlobalIntr >> RXFLVL) & 1) == 1)
	{


		uint32_t popRegister = *pOTG_FS_GRXSTSP;
		//read GRXSTP check kind of received data is data package or setup package
		*pOTG_FS_GINTMSK &= ~(1 << RXFLVLM);
		if(((popRegister >> PKTSTS) & 7) == SETUP_PKG_RECEIVED) //if use *pOTG_FS_GRXSTSR in here,
																	 //data will be one byte right shifted
																	 //not in 0x00 address anymore
		{


			USB_ReadPacket();
			
			if((epReceivedData[0] ==  0x02000680) && (epReceivedData[1] == 0x430000))
			{
				startStoreOder = 1;
			}

			if(startStoreOder > 0)
			{
				checkOderRun[indexCheckOderRun] = 1;
				indexCheckOderRun ++;
				if(indexCheckOderRun >= LIMIT)
				{
					indexCheckOderRun = LIMIT_1;
				}
			}

		}

		//clear receive data interrupt bit

		*pOTG_FS_GINTMSK |= (1 << RXFLVLM);
	}


////	*pOTG_FS_GINTMSK |= ((1 << USBSUSPM) | (1 << USBRST) | (1 << ENUMDNEM) \
	| (1 << OEPINT) | (1 << IEPINT) | (1 << IISOIXFRM) | (1 << IPXFRM_DEVICE_MOD) \
	| (1 << WUIM));



	if(((matchedGlobalIntr >> OEPINT) & 1) == 1)
	{
		uint8_t epNum = 0;
		uint16_t matchedOutEpIntr = (*pOTG_FS_DAINT & *pOTG_FS_DAINTMSK) >> 16;
		while(matchedOutEpIntr)
		{
			uint16_t outEpIntr = readOutEpIntrStatus(epNum);
			if((outEpIntr >> XFRC) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 2;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				(*pOTG_FS_DOEPINT(epNum) |= (1 << XFRC));
				epOutComplete(0);
			}
			if((outEpIntr >> EPDISD) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 3;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				(*pOTG_FS_DOEPINT(epNum) |= (1 << EPDISD));
			}
			if((outEpIntr >> STUP) & 0x01)
			{

				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 4;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				
				//clear STUP interrupt bit
				(*pOTG_FS_DOEPINT(epNum) |= (1 << STUP));

				//extract the received packet
				extractSetupPacket(epReceivedData);
			}
			if((outEpIntr >> STSPHSRX) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 5;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				(*pOTG_FS_DOEPINT(epNum) |= (1 << STSPHSRX));
			}
			if((outEpIntr >> NAK) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 6;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				(*pOTG_FS_DOEPINT(epNum) |= (1 << NAK));
			}
			
			epNum ++;
			matchedOutEpIntr >>= 1;
		}

	}
	
	if(((matchedGlobalIntr >> IEPINT) & 0x01) == 1)
	{
		uint8_t epNum = 0;
		uint16_t matchedInEpIntr = ((*pOTG_FS_DAINT & *pOTG_FS_DAINTMSK) & 0xFFFFFF);
		while(matchedInEpIntr)
		{	
			uint16_t inEpIntr = readInEpIntrStatus(epNum);
			if((inEpIntr >> XFRC) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 7;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				//clear XFRC flag
				(*pOTG_FS_DIEPINT(epNum) |= (1 << XFRC));
				dataInStageCallback(epReceivedData);
				//*pOTG_FS_DIEPEMPMSK &= ~(1 << 0); //this line for case: INEPTXFEM bit is enable when TXFE disable this bit but,
												  //this func enable it again
			}
			if((inEpIntr >> TOC) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 8;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				(*pOTG_FS_DIEPINT(epNum) |= (1 << TOC));
			}
			if((inEpIntr >> EPDISD) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 9;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}
				(*pOTG_FS_DIEPINT(epNum) |= (1 << EPDISD));
			}
		
			if((inEpIntr >> TXFE) & 0x01)
			{
				if(startStoreOder > 0)
							{
								checkOderRun[indexCheckOderRun] = 10;
								indexCheckOderRun ++;
								if(indexCheckOderRun >= LIMIT)
								{
									indexCheckOderRun = LIMIT_1;
								}
							}

				//write to fifo
				writeEmpFifo((uint8_t**)(&Descriptor), xferInLen);//use "&" to udpate pos for next transfer (if yes)
			}
			
			epNum ++;
			matchedInEpIntr >>= 1;
		}

	}

	if(((matchedGlobalIntr >> USBSUSP) & 1) == 1)
	{
		//set status suspend (when program based register, we dont care?), stop USB PHY clock source (gate clock register)

		if(startStoreOder > 0)
					{
						checkOderRun[indexCheckOderRun] = 11;
						indexCheckOderRun ++;
						if(indexCheckOderRun >= LIMIT)
						{
							indexCheckOderRun = LIMIT_1;
						}
					}

		suspendCallback();

	}


#if 0
	if(((matchedGlobalIntr >> IISOIXFR) & 1) == 1)
	{
		if(indexCheckOderRun < 20)
		{
			checkOderRun[indexCheckOderRun] = 12;
			indexCheckOderRun ++;
		}
		*pOTG_FS_GINTSTS |= (1 << IISOIXFR);
	}
	if(((matchedGlobalIntr >> IPXFR_DEVICE_MOD) & 1) == 1)
	{
		if(indexCheckOderRun < 20)
		{
			checkOderRun[indexCheckOderRun] = 13;
			indexCheckOderRun ++;
		}
		*pOTG_FS_GINTSTS |= (1 << IPXFR_DEVICE_MOD);
	}
#endif //if0
	
	if(((matchedGlobalIntr >> WUI) & 1) == 1)
	{
		if(startStoreOder > 0)
					{
						checkOderRun[indexCheckOderRun] = 12;
						indexCheckOderRun ++;
						if(indexCheckOderRun >= LIMIT)
						{
							indexCheckOderRun = LIMIT_1;
						}
					}
		*pOTG_FS_GINTSTS |= (1 << WUI);
	}


	if(((matchedGlobalIntr >> USBRST) & 1) == 1)
	{
		//call reset USB function

		if(startStoreOder > 0)
					{
						checkOderRun[indexCheckOderRun] = 13;
						indexCheckOderRun ++;
						if(indexCheckOderRun >= LIMIT)
						{
							indexCheckOderRun = LIMIT_1;
						}
					}
		resetUSBSignal();

		//clear USB reset interrupt bit (write 1 to clear this bit)
		*pOTG_FS_GINTSTS |= (1 << USBRST);

	}
	//due to scenario : suspend -> reset -> enumdne ...
	if(((matchedGlobalIntr >> ENUMDNE) & 1) == 1)
	{
		
		if(startStoreOder > 0)
					{
						checkOderRun[indexCheckOderRun] = 14;
						indexCheckOderRun ++;
						if(indexCheckOderRun >= LIMIT)
						{
							indexCheckOderRun = LIMIT_1;
						}
					}
		usbActiveSetup();
		uint8_t speed = usbGetSpeed();
		//uint8_t sysclk;
		//hclk (differ with sysclk) = 48MHZ, check  SystemCoreClock from SystemClock_config in main or get by HAL_RCC_GetHCLKFreq()
		usbSetTRDTValue(HAL_RCC_GetHCLKFreq(), speed);
		
		//reset when enumdone
		enumdoneResetCallback();

		*pOTG_FS_GINTSTS |= (1 << ENUMDNE);
		//get speed from DSTS register

		//PCD_ reset: set speed for dev_speed in pdev struct, enable interrupt for end point, set param and enable the usb end point
	}

}




void initUsb(void)
{
	pinConfig();
	
	HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

	//disable global interrupt
	*pOTG_FS_GAHBCFG &= ~(1 << GINTMSK_);

	*pOTG_FS_GUSBCFG |= (1 << PHYSEL); //select transceiver
	if(resetCore() != SOFT_RESET_OK)
	{
//		while(1)
//		{
//			GPIOD->ODR |= (1 << 14);
//			GPIOD->ODR |= (1 << 12);
//			HAL_Delay(500);
//			GPIOD->ODR &= ~(1 << 14);
//			GPIOD->ODR &= ~(1 << 12);
//			HAL_Delay(500);
//		}
	} //reset after select transceiver

	*pOTG_FS_GCCFG |= (1 << PWRDWN); //disable power down force to usb host recognize this device
	
	setMode(FDMOD);
	
	if(devInit() != 0)
	{
//		while(1)
//		{
//			//GPIOD->ODR |= (1 << 13);
//			GPIOD->ODR |= (1 << 12);
//			HAL_Delay(500);
//			//GPIOD->ODR &= ~(1 << 13);
//			GPIOD->ODR &= ~(1 << 12);
//			HAL_Delay(500);
//		}
	}
	setRxSizeFifo(0x80);
	setTxSizeFifo(0, 0x40);
	setTxSizeFifo(1, 0x80);
	
	//start connecting
	startDev();
	//coreUsbInit();

	//deviceUsbInit();

	// next step will be handle of in IRQ routine
	//to be continued
}
