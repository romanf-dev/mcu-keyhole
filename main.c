/** 
  ******************************************************************************
  *  @file   main.c
  *  @brief  Entry point and main loop.
  ******************************************************************************
  *  License: Public domain.
  *****************************************************************************/

#include <stdint.h>
#include <stdalign.h>
#include "stm32f1xx_hal_pcd.h"
#include "usbd_cdc.h"

static USBD_HandleTypeDef hUsbDeviceFS;
static PCD_HandleTypeDef hpcd_USB_FS;

size_t cmd_get_response(char* buf, size_t len);

//
// Errors and exceptions turn onboard LED on.
//
static void Error_Handler(void)
{
    __disable_irq();
    GPIOC->BSRR = GPIO_BSRR_BR13;
    for(;;);
}

//
// Override hardware exceptions.
//
void HardFault_Handler(void)
{
    Error_Handler();
}

//
// USB interrupt handler.
//
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

//
// Synchronous delay without interrupts.
// Note that systick counter register is 24 bit, so, ms cannot be larger than
// 0xffffff / 72000 = ~233 ms.
//
static inline void Delay(unsigned int ms)
{
    SysTick->LOAD  = 72000U * ms - 1U;
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
      ;
    
    SysTick->CTRL = 0;
}

//
// Write byte stream to USB CDC interface.
// Lowspeed USB has only 8 bytes of payload, so, the byte stream is splitted
// to 8 bytes chunks.
//
static inline void USBCDC_WriteString(
    USBD_HandleTypeDef* pdev, 
    const uint8_t* buf,
    size_t sz)
{
    while (sz)
    {
        const size_t len = (sz > 8) ? 8 : sz;

        while (CDC_Transmit(pdev, buf, len) == USBD_BUSY)
            ;

        sz -= len;
        buf += len;
    }
}

//
// Try read byte stream from USB CDC interface.
// Receiving stops when either next chunk of data exceeds buffer length or
// last symbol in the current chunk is CR or LF.
//
static inline size_t USBCDC_ReadString(
    USBD_HandleTypeDef* pdev, 
    uint8_t* buf, 
    size_t len)
{
    size_t i = 0;

    for (;;)
    {
        const size_t avail = CDC_GetRxBufferBytesAvailable(pdev);

        if (avail > 0)
        {
            if (i + avail >= len)
            {
                break;
            }

            if (CDC_ReadRxBuffer(pdev, buf + i, avail) == USB_CDC_RX_BUFFER_OK)
            {
                i += avail;
                
                if (buf[i - 1] == '\n')
                {
                    break;
                }
            }
        }
    }

    buf[i] = 0;
    return i;
}

#define USBD_VID                1155
#define USBD_LANGID_STRING      1033
#define USBD_PID_FS             22336

static const alignas(sizeof(uint32_t)) uint8_t 
USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] =
{
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING)
};

static const alignas(sizeof(uint32_t)) uint8_t 
USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] =
{
    0x12,                       /*bLength */
    USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x02,                       /*bDeviceClass*/
    0x02,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID_FS),        /*idProduct*/
    HIBYTE(USBD_PID_FS),        /*idProduct*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    0x01                        /*bNumConfigurations*/
};

enum
{
    ManufacturerStrSize = 19*2,
    ProductStrSize = 20*2,
    SerialStrSize = 5*2,
    ConfigStrSize = 11*2,
    InterfaceStrSize = 14*2,
};

static const alignas(sizeof(uint32_t)) uint16_t manufacturerStr[] = { 
    (USB_DESC_TYPE_STRING << 8) | ManufacturerStrSize,
    'S','T','M','i','c','r','o','e','l','e','c','t','r','o','n','i','c','s' 
};

static const alignas(sizeof(uint32_t)) uint16_t productStr[] = { 
    (USB_DESC_TYPE_STRING << 8) | ProductStrSize, 
    'B','l','u','e','p','i','l','l',' ','C','D','C',' ','d','e','v','i','c','e' 
};

static const alignas(sizeof(uint32_t)) uint16_t serialStr[] = { 
    (USB_DESC_TYPE_STRING << 8) | SerialStrSize, '1','2','3','4' 
};

static const alignas(sizeof(uint32_t)) uint16_t configStr[] = { 
    (USB_DESC_TYPE_STRING << 8) | ConfigStrSize, 
    'C','D','C',' ','C','o','n','f','i','g' 
};

static const alignas(sizeof(uint32_t)) uint16_t interfaceStr[] = { 
    (USB_DESC_TYPE_STRING << 8) | InterfaceStrSize, 
    'C','D','C',' ','I','n','t','e','r','f','a','c','e' 
};

_Static_assert(ManufacturerStrSize == sizeof(manufacturerStr), "assert1");
_Static_assert(ProductStrSize == sizeof(productStr), "assert2");
_Static_assert(SerialStrSize == sizeof(serialStr), "assert3");
_Static_assert(ConfigStrSize == sizeof(configStr), "assert4");
_Static_assert(InterfaceStrSize == sizeof(interfaceStr), "assert5");

static const USBD_DescriptorsTypeDef FS_Desc =
{
    { USBD_FS_DeviceDesc, sizeof(USBD_FS_DeviceDesc) },
    { USBD_LangIDDesc, sizeof(USBD_LangIDDesc) },
    { manufacturerStr, sizeof(manufacturerStr) },
    { productStr, sizeof(productStr) },
    { serialStr, sizeof(serialStr) },
    { configStr, sizeof(configStr) },
    { interfaceStr, sizeof(interfaceStr) }
};

//
// Entry point.
//
int main(void)
{
    //
    // Enable HSE and wait until it is ready.
    //
    RCC->CR |= RCC_CR_HSEON;            
    while(!(RCC->CR & RCC_CR_HSERDY))
        ;
    
    //
    // Configure latency (0b010 should be used if sysclk > 48Mhz).
    //
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;

    //
    // Switch to HSE, configure PLL multiplier and set HSE as PLL source.
    //
    RCC->CFGR |= RCC_CFGR_SW_HSE;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_PLLSRC;

    //
    // Enable PLL and wait until it is ready.
    //
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY))
        ;
    
    //
    // Set PLL as clock source and wait until it switches.
    //
    RCC->CFGR = (RCC->CFGR | RCC_CFGR_SW_PLL) & ~RCC_CFGR_SW_HSE;
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))
        ;

    //
    // The CPU is now running at 72MHz frequency.
    // It is safe to disable HSI.
    //
    RCC->CR &= ~RCC_CR_HSION;

    //
    // Enable Debug availability even in sleep modes.
    //
    DBGMCU->CR = 
        DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;

    //
    // Initialize NVIC.
    //
    NVIC_SetPriorityGrouping(3);

    //
    // Enable clocks for GPIOA, GPIOC and for USB.
    //
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    
    //
    // Turn off onboard LED.
    //
    GPIOC->CRH |= GPIO_CRH_CNF13_0 | GPIO_CRH_MODE13_1;
    GPIOC->BSRR = GPIO_BSRR_BS13;

    //
    // USB resistor is connected to PA12. 
    // Set it to low to enforce USB enumeration.
    //
    GPIOA->CRH |= GPIO_CRH_CNF12_0 | GPIO_CRH_MODE12_1;
    GPIOA->BSRR = GPIO_BSRR_BR12; 
    Delay(100);

    hpcd_USB_FS.Instance = USB;
    hpcd_USB_FS.Init.dev_endpoints = 8;
    hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;

    //
    // Initialize USB and create CDC endpoint.
    //
    if (USBD_Init(&hUsbDeviceFS, &hpcd_USB_FS, &FS_Desc, &USBD_CDC) != USBD_OK)
    {
        Error_Handler();
    }

    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00 , PCD_SNG_BUF, 0x18);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80 , PCD_SNG_BUF, 0x58);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_IN_EP, PCD_SNG_BUF, 0xC0);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_OUT_EP, PCD_SNG_BUF, 0x110);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x100);

    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);    
  
    __enable_irq();

    if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
    {
        Error_Handler();
    }

    //
    // Blink onboard LED to indicate succesful USB initialization.
    //
    GPIOC->BSRR = GPIO_BSRR_BR13;
    Delay(100);
    GPIOC->BSRR = GPIO_BSRR_BS13;

    //
    // Main loop...
    //
    for (;;)
    {
        static uint8_t buffer[1024];

        USBCDC_ReadString(&hUsbDeviceFS, buffer, sizeof(buffer));

        const size_t response = cmd_get_response(
            (char*)buffer, 
            sizeof(buffer)
        );

        if (response > 0)
        {
            USBCDC_WriteString(&hUsbDeviceFS, buffer, response);
        }
    }

    return 0; /* make compiler happy. */
}
