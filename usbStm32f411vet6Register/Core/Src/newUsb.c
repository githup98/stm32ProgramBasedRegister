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



//#ifndef   __UNALIGNED_UINT32_WRITE
//  #pragma GCC diagnostic push
//  #pragma GCC diagnostic ignored "-Wpacked"
//  #pragma GCC diagnostic ignored "-Wattributes"
//  __PACKED_STRUCT T_UINT32_WRITE { uint32_t v; };
//  #pragma GCC diagnostic pop
//  #define __UNALIGNED_UINT32_WRITE(addr, val)    (void)((((struct T_UINT32_WRITE *)(void *)(addr))->v) = (val))
//#endif


typedef struct T32
{
	uint32_t v;
}__attribute__((packed, aligned(1))) T32; //when use align for struct or member struct just increase size, so must use "packed" together




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



//larger size should be first declare
uint32_t epReceivedData[12] = {0};
uint32_t xferCount = 0;

uint32_t data[CDC_DATA_FS_OUT_PACKET_SIZE/4]={0}; //for word alignment
uint8_t cmdLength;
uint8_t cmdOpCode;  //.stack


//request from host --> should be use a struct
uint8_t bmRequestType;
uint8_t bRequest;
uint16_t wValue;
uint16_t wIndex;
uint16_t wLength;

//state of usb device
uint8_t stateFsUsbDevice = 0;

volatile uint8_t totalLengthEp0In = 0;
volatile uint8_t totalLengthEp0Out = 0;
volatile uint8_t remainLengthEp0In = 0;
volatile uint8_t remainLengthEp0Out = 0;


uint8_t xferInLen = 0;
uint8_t xferOutLen = 0;


uint8_t *xferBuffIn = NULL;
uint8_t *xferBuffOut = NULL;




uint8_t setAddressSuccess = 0;

uint8_t UserTxBuff[Tx_BUFFER_SIZE];
uint8_t UserRxBuff[Rx_BUFFER_SIZE];
uint8_t *RxBuff = NULL;
uint8_t *TxBuff = NULL;
uint8_t TxState = 0; //transmitting
uint8_t RxState = 0; //receiving
//padding 2byte
uint32_t RxLength = 0;
uint32_t TxLength = 0;
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
	uint8_t Type;
	uint8_t State;
};

struct epStatus epStt = 
{
	.In = 0,   //"." mean init not follow order of struct member, in this case it's yes/no is still ok
	.Out = 0,
	.Stall = 0,
	.Type = 0,
	.State = 0,
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
	*pGPIOA__GPIOx_OSPEEDR &= ~(3 << OSPEEDR11);//usbSen
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





void USB_ReadPacket(uint8_t *pData, uint8_t length)
{
	//read 8 byte, it is request device description package from host

	uint8_t count32Byte = (uint32_t)length >> 2;  //alignment, may be use >> 2
	uint8_t remainByte = length % 4; //maybe use len - count32Byte* 4 (depend on instruction is generated)

	if(remainByte != 0)
	{
		count32Byte += 1;
	}

	for(int i = 0; i < count32Byte; i++)
	{
		((T32*)(void*)pData)->v = *(uint32_t*)(OTG_FS_BASE_ADDR + OTG_FIFO_BASE);
		pData += 4;
		//__UNALIGNED_UINT32_WRITE(epReceivedData, *(uint32_t*)(OTG_FS_BASE_ADDR + OTG_FIFO_BASE));
	}

#ifdef SHIFT_PDATA
	pData -= (4 - remainByte);
#endif

	//__UNALIGNED_UINT32_WRITE

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



uint8_t ep0SendData(uint8_t *pBuff, uint8_t epAddr, uint8_t lengthData)
{

	if(epStt.In == 1)
	{
		*pOTG_FS_DIEPTSIZ0 &= ~(0x3FF << PKTCNT);
		*pOTG_FS_DIEPTSIZ0 &= ~(0x7FFFF << XFRSIZ);
		if(lengthData == 0)
		{

			*pOTG_FS_DIEPTSIZ0 |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
		}

		else
		{
			if(lengthData > MAX_PACKET_SIZE_EP0)
			{
				lengthData = MAX_PACKET_SIZE_EP0;
			}
			
			*pOTG_FS_DIEPTSIZ0 |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
			*pOTG_FS_DIEPTSIZ0 |= (lengthData << XFRSIZ);
		}


		if(0)  //check dma enable or not (dma just support for HS device only)
		{
		
		}
		else
		{
			//enable EP (start transmit)
			*pOTG_FS_DIEPCTL(0) |= ((1 << CNAK) | (1 << EPENA)); //respond DATA is send  (show packet in wireshark)

			//enable Tx empty fifo interrupt
			if(lengthData > 0)
			{
				//for enum 0
				*pOTG_FS_DIEPEMPMSK |= (1 << 0);
			}
		}
		
		
		//update remaining data length
		xferInLen = lengthData; 
	}
	else
	{
		if(lengthData > 0) //due to if it have data need to transfer, ep will sent 64byte for each transfer 
		{
			lengthData = MAX_PACKET_SIZE_EP0;
		}
		//if(lenRequired != 0)
		//{
			*pOTG_FS_DOEPTSIZ0 &= ~(0x3FF << PKTCNT);
			*pOTG_FS_DOEPTSIZ0 &= ~(0x7FFFF << XFRSIZ);

			*pOTG_FS_DOEPTSIZ0 |= ((0x3FF << PKTCNT) & (1 << PKTCNT));

			*pOTG_FS_DOEPTSIZ0 |= (lengthData << XFRSIZ);
			
			//enable EP (start transmit)
			*pOTG_FS_DOEPCTL(0) |= ((1 << CNAK) | (1 << EPENA));

		//}

		xferOutLen = lengthData;
	}

	return 0;
}

uint8_t checkEPXSEND = 0;

uint8_t epXSendData(uint8_t* pBuff, uint8_t epAddr,  uint8_t lengthData)
{
	uint8_t epNum = epAddr & 0x0f;
	uint16_t packetCount = 0;
	if(epStt.In == 1)
	{

		*pOTG_FS_DIEPTSIZ(epNum) &= ~(0x3FF << PKTCNT);
		*pOTG_FS_DIEPTSIZ(epNum) &= ~(0x7FFFF << XFRSIZ);
		if(lengthData == 0)
		{

			*pOTG_FS_DIEPTSIZ(epNum) |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
		}
		
		//if(lengthData > MAX_PACKET_SIZE_EPx)
		//{
		//	lengthData = MAX_PACKET_SIZE_EPx;
		//}
		else
		{
			// -1 is due to usb will send 0 length  packet to notify finish transfer, in 64 + 64, follow below express: PCKCNT = 2
			// it mean usb have remain 1 packet sending, this is not correct -> must minus 1 in express

			packetCount = ((lengthData + MAX_PACKET_SIZE_EPx - 1) / MAX_PACKET_SIZE_EPx);
			*pOTG_FS_DIEPTSIZ(epNum) |= ((0x3FF << PKTCNT) & (packetCount << PKTCNT));
			*pOTG_FS_DIEPTSIZ(epNum) |= (lengthData << XFRSIZ);
			

			if(epStt.Type == EP_TYPE_ISOC)
			{	
				//stm32f411 not implement this bit in this register
				//*pOTG_FS_DIEPTSIZ(epNum) &= ~(3 << 29);
				//*pOTG_FS_DIEPTSIZ(epNum) |= (1 << 29);
			}
		}

	
		if(0) //check dma is enable or not
		{
			//temporary planting empty //tam thoi de trong
		}
		else
		{
			//enable EP (start transmit)
			*pOTG_FS_DIEPCTL(epNum) |= ((1 << CNAK) | (1 << EPENA)); //respond DATA is send  (show packet in wireshark)

	
			if(epStt.Type != EP_TYPE_ISOC)
			{
				//enable Tx empty fifo interrupt
				if(lengthData > 0)
				{
					//for enum 0
					*pOTG_FS_DIEPEMPMSK |= (1 << epNum);
				}
			}
			else
			{
			 	*pOTG_FS_DIEPCTL(epNum) &= ~(3 << SD0PID_SEVNFRM); //clear ODD/EVEN frame bit /
				if((*pOTG_FS_DSTS >> 8) & 1) //FNSOF != 0 -> set even frame
				{
					*pOTG_FS_DIEPCTL(epNum) |= (1 << SD0PID_SEVNFRM); //set even frame
				}
				else
				{
					*pOTG_FS_DIEPCTL(epNum) |= (1 << SODDFRM);
				}
				writeEmpFifo(&xferBuffOut, lengthData);
			}
		}
	
		
		//update remaining data length
		xferInLen = lengthData; 
	}
	else
	{
		checkEPXSEND ++;
		*pOTG_FS_DIEPTSIZ(epNum) &= ~(0x3FF << PKTCNT);
		*pOTG_FS_DIEPTSIZ(epNum) &= ~(0x7FFFF << XFRSIZ);
		
		if(lengthData == 0)
		{
			*pOTG_FS_DOEPTSIZ(epNum) |= ((0x3FF << PKTCNT) & (1 << PKTCNT));
			*pOTG_FS_DIEPTSIZ(epNum) |= ((0x7FFFF << XFRSIZ) & (MAX_PACKET_SIZE_EPx << XFRSIZ));
		}
		else
		{
			
			packetCount = ((lengthData + MAX_PACKET_SIZE_EPx -1) / MAX_PACKET_SIZE_EPx);
			*pOTG_FS_DOEPTSIZ(epNum) |= ((0x3FF << PKTCNT) & (packetCount << PKTCNT));
			*pOTG_FS_DOEPTSIZ(epNum) |= ((MAX_PACKET_SIZE_EPx * packetCount) << XFRSIZ);
		}
		
		if(0) //check dma
		{
			//temporary planting empty
		}
		else
		{
			if(epStt.Type == EP_TYPE_ISOC)
			{
				*pOTG_FS_DOEPCTL(epNum) &= ~(3 << SD0PID_SEVNFRM); //clear value
				if((*pOTG_FS_DSTS >> 8) & 1) //set Odd /even frame
				{
					*pOTG_FS_DOEPCTL(epNum) |= (1 << SD0PID_SEVNFRM); //set even (chan)
				}
				else
				{
					*pOTG_FS_DOEPCTL(epNum) |= (1 << SODDFRM);
				}
			}
		}
		//enable EP (start transmit)
		*pOTG_FS_DOEPCTL(epNum) |= ((1 << CNAK) | (1 << EPENA));
		xferOutLen = lengthData;
	}

	return 0;
}



void setAddressEp(void)
{
	if((wLength == 0) && (wIndex == 0) && (wValue < 128))
	{
		*pOTG_FS_DCFG &= ~(0x7f << DAD);
		*pOTG_FS_DCFG |= (wValue << DAD);
	}
	stateFsUsbDevice = USBD_ADDRESSED_STATE;
}




#if 1 //move to new file

void usbdActivateEndpoint(uint8_t epNum, uint8_t epType, uint8_t epMaxPacketSize)
{
	if(epStt.In == 1)
	{
		//unmask interrupt for partical end point
		*pOTG_FS_DAINTMSK |= ((0xffff << 0) & (1 << epNum)); //equivalant 2 command: clear previous value, assign new value
		if((*pOTG_FS_DIEPCTL(epNum) & (1 << USBAEP)) == 0x00) //due to EPENA is 31th bit -> no need to & 0x01
		{
			*pOTG_FS_DIEPCTL(epNum) |= ((epMaxPacketSize << MPSIZ__OTG_FS_DIEPCTL0) | (epType << 18) | (epNum << 22)\
			| (1 << SD0PID_SEVNFRM) | (1 << USBAEP));
		}
	}	
	else
	{
		//unmask interrupt for partical end point
		*pOTG_FS_DAINTMSK |= ((0xffff << 16) & ((1 << epNum) << 16)); //equivalant 2 command: clear previous value, assign new value
		if((*pOTG_FS_DOEPCTL(epNum) & (1 << USBAEP)) == 0x00) //due to EPENA is 31th bit -> no need to & 0x01
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


void stallEp(uint8_t epAddress)
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




#endif //if 0 move to new file


/*usbSend and usbReceive , both are use ep0SendData and epXSendData*/

//this func will rise a DIEP_TXFC interrupt(overall: send data to host will rise this interrupt)
void usbSend(uint8_t* pBuff, uint8_t epAddr, uint8_t lenData)
{
	////epNum != epAddr

	epStt.State = USBD_EP0_DATA_IN;
	epStt.In = 1;
	
	if((epAddr & 0x0f) == 0)
	{
		ep0SendData(pBuff, epAddr, lenData);

	}
	else
	{
		epXSendData(pBuff, epAddr, lenData);//pBuff due to it contain writePacket func for ISOC type 
	}
}


void usbReceive(uint8_t *pBuff, uint8_t epAddr, uint8_t lenData) //re-check this func, check how to pBuff is passed
{
	
	xferBuffOut = pBuff;
	xferCount = 0; //reset xferCount

	epStt.In = 0;	
	if((epAddr & 0x0f) == 0)
	{
		ep0SendData(pBuff, epAddr, lenData);  //<==> EP0StartXfer in HAL library
	}
	else
	{
		epXSendData(pBuff, epAddr, lenData);
	}
	
}


void usbSendStatus(uint8_t* pBuff, uint8_t epAddr, uint8_t lenData)
{
	usbSend(pBuff, epAddr, lenData);
}


void usbSendData(uint8_t *pBuff, uint8_t epAddr, uint8_t lenData)
{
	usbSend(pBuff, epAddr, remainLengthEp0In);
}

void usbSendError(void)
{
	//set stall bit in IN/OUT endpoint
	stallEp(0x80);//in ep
	stallEp(0x00);//out ep
}


//new file
void usbdContinousRx(uint8_t*pBuff, uint8_t length)
{
 	usbReceive(pBuff, 0x00, length);
}

void cdcClassSetTxBuffer(uint8_t* pBuffer, uint8_t length)
{
	TxBuff = pBuffer;
	TxLength = length;
}
void cdcClassSetRxBuffer(uint8_t* pBuffer)
{
	RxBuff = pBuffer;
}


void cdcClassReceivePacket(uint8_t speed, uint8_t* pBuff)
{
	if(speed == 0) //HS
	{
		//temporary planting empty
	}
	else if(speed == 3)
	{
		usbReceive(pBuff, CDC_OUT_EP_ADDR, CDC_DATA_FS_OUT_PACKET_SIZE);
	}
}


uint8_t usbdCdcClassInit(uint8_t speed)
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
	
	cdcClassSetTxBuffer(UserTxBuff, 0);
	cdcClassSetRxBuffer(UserRxBuff);

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
		usbReceive((uint8_t*)Rx_BUFFER_SIZE, CDC_OUT_EP_ADDR, CDC_DATA_FS_OUT_PACKET_SIZE);
	}
	return USBD_OK; //return 0
}



uint8_t usbdSetClass(uint8_t speed) //parse type usb class
{
	uint8_t ret = usbdCdcClassInit(speed);
	return ret;
}


void usbdSetConfig(uint16_t configNum, uint8_t speed)
{
	//should check set address is success or not!!!!!
	
	uint8_t ret;

	if(configNum != 0)
	{
		ret = usbdSetClass(speed); //class:cdc or msc or dfu ... and speed param
		if(ret != USBD_OK)
		{
			usbSendError();
		}
		else
		{
			usbSendStatus(NULL, 0x00, 0); //this func will rise a DIEP_TXFC interrupt(overall: send data to host will rise this interrupt)
			stateFsUsbDevice = USBD_CONFIGURED_STATE;
		}
	}
}

uint16_t min(uint16_t len1, uint16_t len2)
{
	return (len1 < len2 ? len1 : len2);
}


uint8_t checkProblem = 0;

void stdDev(void)
{
	uint8_t lenDesc = 0;
	uint8_t speedEnumDone = (*pOTG_FS_DSTS >> ENUMSPD) & 0x03;		

	switch (bRequest)
	{
	case 5:
		setAddressEp();
		usbSendStatus(NULL, 0x00, 0);
		break;

	case 6:
		switch(wValue >> 8)
		{
			case 0x01: //get device descriptor

				Descriptor = (void*)getFSDeviceDescriptor(&lenDesc);

				remainLengthEp0In = (uint8_t)min(lenDesc, wLength);
				totalLengthEp0In = lenDesc;
				
				usbSendData(NULL, 0x80, lenDesc);
				break;

			case 0x02: //get config

				//we must create function to change maxpacketsize between HS and FS, check HAL code
				//but now we can skip it due to value default in config des is for FS

				//attention due to size of config desc is larger 64 byte
				
				Descriptor = (void*)getFsConfigurationDesc(&lenDesc);
				remainLengthEp0In = (uint8_t)min(lenDesc, wLength);
				totalLengthEp0In = lenDesc;

				usbSendData(NULL, 0x80, lenDesc);
				break;

			case 0x06: //get device_qualifer (for device support both HS and FS speed)
				if(speedEnumDone == 0x00) //High speed
				{
					//do some things
				}
				else if(speedEnumDone == 0x03) //full speed
				{
					usbSendStatus(NULL, 0x80, 0); //no send data to avoid waiting from host.
				}
				break;

			default:
				break;
		}

		break;

	case 0x09: //set config
		usbdSetConfig(wValue, speedEnumDone);
		break;
	default:
		break;
	}
}




uint8_t usbdCoreFindIF(uint8_t requireIndex)
{
#ifdef USBD_USED_COMPOSITE
	//We do not use usb composite in this case so, do nothing in this #preprocess
	
#else

	return 0;

#endif
}



uint8_t usbdCoreFindEP(uint8_t requireIndex)
{
#ifdef USBD_USED_COMPOSITE
	//We do not use usb composite in this case so, do nothing in this #preprocess
	
#else

	return 0;

#endif
}



void cdcClassControl(uint8_t cmdOpCode, uint8_t* pBuff, uint8_t length)
{
switch(cmdOpCode)
	{
	case CDC_SEND_ENCAPSULATED_COMMAND:

		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:

		break;

	case CDC_SET_COMM_FEATURE:

		break;

	case CDC_GET_COMM_FEATURE:

		break;

	case CDC_CLEAR_COMM_FEATURE:

		break;

	/*******************************************************************************/
	/* Line Coding Structure                                                       */
	/*-----------------------------------------------------------------------------*/
	/* Offset | Field       | Size | Value  | Description                          */
	/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
	/* 4      | bCharFormat |   1  | Number | Stop bits                            */
	/*                                        0 - 1 Stop bit                       */
	/*                                        1 - 1.5 Stop bits                    */
	/*                                        2 - 2 Stop bits                      */
	/* 5      | bParityType |  1   | Number | Parity                               */
	/*                                        0 - None                             */
	/*                                        1 - Odd                              */
	/*                                        2 - Even                             */
	/*                                        3 - Mark                             */
	/*                                        4 - Space                            */
	/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
	/*******************************************************************************/
	case CDC_SET_LINE_CODING:

		break;

	case CDC_GET_LINE_CODING:

		break;

	case CDC_SET_CONTROL_LINE_STATE:

		break;

	case CDC_SEND_BREAK:

		break;

	default:
		break;
	}	
}


void usbdPrepairRx(uint32_t *pBuff, uint8_t len)
{
	epStt.State = USBD_EP0_DATA_OUT;
	totalLengthEp0Out = len;
	remainLengthEp0Out = len; //if USBD_AVOID_PACKET_SPLIT_MPS is not used
	usbReceive((uint8_t*)pBuff, 0, len);  //pBuff is used for DMA, or ISO mode
}

void cdcClassSetup(void)
{

	uint8_t len;
	switch(bmRequestType & bm_REQUEST_TYPE_MASK)
	{
		case bm_REQUEST_TYPE_CLASS:
			if(wLength != 0)
			{
				if((bmRequestType & 0x80) == (DEVICE_TO_HOST)) //prepair for send or receive data from host, in this case is received data
				{
					cdcClassControl(cmdOpCode, (uint8_t*)data, cmdLength);
					
					len = min(CDC_REQ_MAX_DATA_SIZE, wLength);
					usbSendData((uint8_t*)data, 0x80, len); //endpoint 0?????
				}
				else
				{
					cmdOpCode = bRequest;
					cmdLength = (uint8_t)min(wLength, MAX_PACKET_SIZE_EP0);
					
					//prepairRx
					usbdPrepairRx(data, cmdLength);
				}
			}
			else
			{
				cdcClassControl(bRequest, NULL, 0);
			}
		
			break;

		case bm_REQUEST_TYPE_STANDARD:
			break;

		default:
			usbSendError();
			break;
	}
}




void stdIF(void)
{

	uint8_t indexInterface = 0;
	switch(bmRequestType & bm_REQUEST_TYPE_MASK)
	{
		case bm_REQUEST_TYPE_STANDARD:
		case bm_REQUEST_TYPE_CLASS:
		case bm_REQUEST_TYPE_VENDOR:
			if((stateFsUsbDevice == USBD_ADDRESSED_STATE) || (stateFsUsbDevice == USBD_CONFIGURED_STATE)\
			|| (stateFsUsbDevice == USBD_DEFAULT_STATE))//set address successfully
			{
				if((wIndex & 0xff) <= USBD_MAX_NUM_INTERFACES)  // <  1
				{

#ifdef USBD_USED_COMPOSITE	
indexInterface = usbdCoreFindIF(wIndex); //due to not use USBD_USED_COMPOSITE so indexInterface = 0;
#endif			
			
					if((indexInterface != 0xff) && (indexInterface < USBD_MAX_SUPPORTED_CLASS))
					{
						//setup for CDC class
						cdcClassSetup();
					
					}
					if(wLength == 0)
					{
						usbSendStatus(NULL, 0, 0); //send status
					}
				}
				else
				{
					//send error for both IN and OUT endpoint
					usbSendError();
					
				}
			}

			break;

		default:
			break;
	}	
		
}


void extractSetupPacket(uint32_t *pSetup)
{
	bmRequestType = pSetup[0] & 0xff;
	bRequest = (pSetup[0] >> 8) & 0xff;
	wIndex = pSetup[1] & 0xFFFF;
	wValue = pSetup[0] >> 16;
	wLength = pSetup[1] >> 16;

	//if(bmRequestType == 0x80)//get phase transfer direction
	//{

		//switch ((bmRequestType >> 5) & 0x03) //get Type
		//{
		//case 0:
			switch(bmRequestType & 0x1f)     //get Recepient
			{
			case 0:
				stdDev();
				break;
			case 1:
				stdIF();
				break;
			default:
				break;
			}
		//	break;
		//default:
		//	break;

		//}
	//}
	//else
	//{
		////temptly do nothing
	//}
}






//void prepairReceive()
//{
//
//}


uint8_t timeWrite = 0;

void cdcClassTransmit(void)
{

}

void cdcClassDataIn(uint8_t epNum)
{
	if((totalLengthEp0In > 0) && (totalLengthEp0In % MAX_PACKET_SIZE_EP0 == 0))
	{
		totalLengthEp0In = 0;
		usbSendStatus(NULL, epNum, 0);// send 0 length data packet
	}
	else
	{
		TxState = 0;
		//cdcClassSetTxBuffer(UserTxBuff, );
		////cdcClassTransmit(); re-check this point
	}

}


void dataInStageCallback(uint8_t epNum)///need to add epNum param
{
	uint8_t index;
	if(epNum == 0)//if enum == 0, change if condition for overall
	{
		if(epStt.State == USBD_EP0_DATA_IN) //check status: USBD_EP0_DATA_IN, please complete the code for the overview
		{

			if(remainLengthEp0In > MAX_PACKET_SIZE_EP0)
			{
				remainLengthEp0In -= MAX_PACKET_SIZE_EP0;
				usbSendData(NULL, 0x80, remainLengthEp0In);
				usbReceive(NULL, 0x80, 0);
			}
			else
			{
				stallEp(0x80);
				usbReceive(NULL, 0x80, 0);
			}
		}
	}
	else
	{
		index = usbdCoreFindEP(wIndex);
		if((index != 0xFF) & (index < USBD_MAX_SUPPORTED_CLASS))
		{
			if(stateFsUsbDevice == USBD_CONFIGURED_STATE)
			{
				cdcClassDataIn(epNum);
			}
		}
	}
}



void ep0Ready(void)
{
	//cmdOpCode = 0xFF;
	if(cmdOpCode != 0xFF)
	{
		cdcClassControl(cmdOpCode, (uint8_t*)data, cmdLength);
		cmdOpCode = 0xFF;
	}
}




void cdcClassReceiveFs(uint8_t speed, uint8_t* pRxBuff)
{
	cdcClassSetRxBuffer(&UserRxBuff[0]);//reset buffer
	cdcClassReceivePacket(speed, pRxBuff);
}

uint32_t getRxDataSize(uint8_t epNum)
{
	return xferCount;
}

void cdcClassDataOut(uint8_t epNum)
{
	////uint32_t RxLength = getRxDataSize(epNum);
	uint8_t speedEnumDone = (*pOTG_FS_DSTS >> ENUMSPD) & 0x03;		
	cdcClassReceiveFs(speedEnumDone, RxBuff);
}


void dataOutStage(uint8_t* pBuff, uint8_t epNum)
{
	uint8_t index = 0;
	if(epNum == 0)
	{
		if(epStt.State == USBD_EP0_DATA_OUT)
		{
			if(remainLengthEp0Out > MAX_PACKET_SIZE_EP0)
			{
				remainLengthEp0Out -= MAX_PACKET_SIZE_EP0;
				usbdContinousRx(pBuff, min(remainLengthEp0Out, MAX_PACKET_SIZE_EP0));
			}
			else
			{
				//Find class relative current request
				switch(bmRequestType & 0x1F)
				{
					case REQUEST_RECEPIENT_DEVICE:
						index = 0;
						break;

					case REQUEST_RECEPIENT_INTERFACE:
						index = usbdCoreFindIF(wIndex);					
						break;

					case REQUEST_RECEPIENT_ENDPOINT:
						index = usbdCoreFindEP(wIndex);					
						break;
					
					default:
						index = 0;
				}

				if(index <= USBD_MAX_SUPPORTED_CLASS)
				{
					if(stateFsUsbDevice == USBD_CONFIGURED_STATE) //check cocnfigured state?
					{
						//if(cmdOpCode != 0xFF)
						ep0Ready();//set cmdOpCode == 0xff	
					}
					
					//send status
					usbSendStatus(NULL, 0, 0);
				}
			}
		}
	}
	else
	{
		index = usbdCoreFindEP(epNum);
		if((index != 0xFF) & (index < USBD_MAX_SUPPORTED_CLASS))
		{
			if(stateFsUsbDevice == USBD_CONFIGURED_STATE) //check for USBD_STATE_CONFIGURED
			{
				cdcClassDataOut(epNum);
			}
		}
	}
}

void epOutComplete(uint8_t epNum)
{
	//if(dma == 1)
	//{
	//
	//}
	
	if((epNum == 0) && (xferOutLen == 0))  //consider ep 0 and 1 using same xferOutLen
	{
		ep0OutStart();
	}

	// in setup phase this function can be ignored
	//but when SET LINE CODING it be will active due to ep_state change to USBD_EP0_DATA_OUT
	
	dataOutStage((uint8_t*)epReceivedData, epNum);
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


uint8_t checkStuckNAK = 0;

void usbInterruptHandler(void)
{

// handler for usb suspend, reset enumdone, read, get descriptor, transfer data into FIFO, IN done, OUT done


	uint32_t matchedGlobalIntr = ((*pOTG_FS_GINTSTS) & (*pOTG_FS_GINTMSK));
	uint32_t popRegister = 0;
	uint16_t byteCount = 0;
	
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

		
		popRegister = *pOTG_FS_GRXSTSP;
		byteCount = popRegister >> BCNT & 0x7FF;

		//read GRXSTP check kind of received data is data package or setup package
		*pOTG_FS_GINTMSK &= ~(1 << RXFLVLM);
		if(checkStuckNAK == 1)
		{
			uint8_t w =0;
		}
		if(((popRegister >> PKTSTS) & 7) == 0x02)
		{
			if(startStoreOder > 0)
						{
							checkOderRun[indexCheckOderRun] = 15;
							indexCheckOderRun ++;
							if(indexCheckOderRun >= LIMIT)
							{
								indexCheckOderRun = LIMIT_1;
							}
						}

			if(byteCount != 0)
			{
				USB_ReadPacket((uint8_t*)epReceivedData, byteCount);
			}
			xferCount += byteCount;
		}

		else if(((popRegister >> PKTSTS) & 7) == SETUP_PKG_RECEIVED) //if use *pOTG_FS_GRXSTSR in here,
																	 //data will be one byte right shifted
																	 //not in 0x00 address anymore
		{


			USB_ReadPacket((uint8_t*)epReceivedData, 8);
			
			xferCount += byteCount;


			//if((epReceivedData[0] ==  0x02000680) && (epReceivedData[1] == 0x430000))
			if((epReceivedData[0] ==  0x2021))
			{
				startStoreOder = 1;
				checkStuckNAK = 1;
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
				epOutComplete(epNum);
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
				dataInStageCallback(epNum);
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
