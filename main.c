/** 
  ******************************************************************************
  *  @file   main.c
  *  @brief  Entry point and main loop.
  ******************************************************************************
  *  License: Public domain.
  *****************************************************************************/

#include <stdint.h>
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceFS;

size_t cmd_get_response(char* buf, size_t len);

//
// Errors and exceptions turn onboard LED on.
//
void Error_Handler(void)
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
    extern PCD_HandleTypeDef hpcd_USB_FS;
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
static inline void USBCDC_WriteString(char* buf, size_t sz)
{
    while (sz)
    {
        const size_t len = (sz > 8) ? 8 : sz;

        while (CDC_Transmit_FS((uint8_t*) buf, len) == USBD_BUSY)
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
static inline size_t USBCDC_ReadString(char* buf, size_t len)
{
    size_t i = 0;

    for (;;)
    {
        const size_t avail = CDC_GetRxBufferBytesAvailable_FS();

        if (avail > 0)
        {
            if (i + avail >= len)
            {
                break;
            }

            if (CDC_ReadRxBuffer_FS((uint8_t*) buf + i, avail) == USB_CDC_RX_BUFFER_OK)
            {
                i += avail;
                
                if (buf[i - 1] == '\r' || buf[i - 1] == '\n')
                {
                    break;
                }
            }
        }
    }

    buf[i] = 0;
    return i;
}

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

    //
    // Initialize USB and create CDC endpoint.
    //
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
    {
        Error_Handler();
    }

    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
    {
        Error_Handler();
    }

    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
    {
        Error_Handler();
    }
    
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
        static char buffer[50];

        USBCDC_ReadString(buffer, sizeof(buffer));

        const size_t response_bytes = cmd_get_response(buffer, sizeof(buffer) - 2);

        if (response_bytes > 0)
        {
            buffer[response_bytes] = '\r';
            buffer[response_bytes + 1] = '\n';

            USBCDC_WriteString(buffer, response_bytes + 2);
        }
    }

    return 0; /* make compiler happy. */
}
