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
#include "magnesium.h"
#include <assert.h>

_Static_assert(MG_NVIC_PRIO_BITS == __NVIC_PRIO_BITS, "NVIC prio mismatch");

#define USBD_VID                1155
#define USBD_LANGID_STRING      1033
#define USBD_PID_FS             22336

static USBD_HandleTypeDef g_usb_device;
static PCD_HandleTypeDef g_usbfs_driver;
struct mg_context_t g_mg_context;

static struct mg_message_t g_datain_ack_mem[50];

static struct usb_packet_t
{
    struct mg_message_t header;
    size_t len;
    uint8_t payload[128];
} 
g_usb_data_mem[50];

static struct cmd_buffer_t
{
    struct mg_message_t header;
    size_t len;
    char string[1000];
} 
g_cmdline_mem[5];

static struct mg_message_pool_t g_datain_ack_msgs;
static struct mg_message_pool_t g_usb_data_msgs;
static struct mg_message_pool_t g_cmdline_msgs;

static struct mg_queue_t g_datain_ack;
static struct mg_queue_t g_replies;
static struct mg_queue_t g_usb_packets;
static struct mg_queue_t g_requests;

static struct mg_actor_t g_sender;
static struct mg_actor_t g_receiver;
static struct mg_actor_t g_executor;

#define msg_pool_init_helper(pool, array) \
    mg_message_pool_init((pool), (array), sizeof(array), sizeof((array)[0]))

size_t cmd_get_response(char* buf, size_t len);

//
// Errors and exceptions turn onboard LED on.
//
static void _Noreturn Error_Handler(void)
{
    __disable_irq();

    GPIOC->BSRR = GPIO_BSRR_BR13;
    for (;;);
}

//
// Override hardware exceptions.
//
void HardFault_Handler(void)
{
    Error_Handler();
}

//
// Library's dependency.
//
void __assert_func(const char *file, int ln, const char *func, const char *expr)
{
    Error_Handler();
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
// USB interrupt handler. It eventually calls USB stack callbacks containing
// DataIn and DataOut functions.
//
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&g_usbfs_driver);
}

//
// High-priority actor executor.
//
void WWDG_IRQHandler(void)
{
    mg_context_schedule(WWDG_IRQn);
}

//
// Low-priority actor executor.
//
void PVD_IRQHandler(void)
{
    mg_context_schedule(PVD_IRQn);
}

//
// Host transmission acknowledge. It must also send Zero-Length-Packets in case
// when total data length is a multiple of packet size.
//
static uint8_t CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    const struct _PCD_HandleTypeDef* hpcd = pdev->pData;

    if ((pdev->ep_in[epnum].total_length > 0) && 
        (pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0)
    {
        CDC_Transmit(pdev, NULL, 0); // ZLP
    }
    else
    {
        struct mg_message_t* ack = mg_message_alloc(&g_datain_ack_msgs);
        assert(ack != 0);
        mg_queue_push(&g_datain_ack, ack);
    }

    return USBD_OK;
}

//
// Incoming data notifier. It it also called just after endpoint creation in 
// order to start receiving. When it cannot allocate new packet it re-uses
// existing one (discarding all the data it holds).
//
static uint8_t CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    struct usb_packet_t* const packet = mg_message_alloc(&g_usb_data_msgs);
    static struct usb_packet_t* receiving_msg = 0;
    
    if (receiving_msg != 0)
    {
        if (packet != 0)
        {
            receiving_msg->len = USBD_GetRxCount(pdev, epnum);
            mg_queue_push(&g_usb_packets, &receiving_msg->header);
            receiving_msg = packet;
        }
    }
    else
    {
        assert(packet != 0);
        receiving_msg = packet;
    }

    CDC_Receive(pdev, receiving_msg->payload, sizeof(receiving_msg->payload));

    return USBD_OK;
}

//
// Receiver accepts USB packets, concatenate them into contiguous string and
// also monitors for CR/LF pattern. After the commandline string is populated
// it is sent to command processor actor.
//
static struct mg_queue_t* Actor_Receiver(
    struct mg_actor_t* self, 
    struct mg_message_t* msg)
{
    static struct cmd_buffer_t* request = 0;
    static size_t pointer = 0;
    struct usb_packet_t* data = (struct usb_packet_t*) msg;

    for (size_t i = 0; i < data->len; ++i)
    {
        if (request == 0)
        {
            request = mg_message_alloc(&g_cmdline_msgs);
            pointer = 0;

            if (request == 0)
            {
                break;
            }
        }

        request->string[pointer++] = data->payload[i];

        if ((data->payload[i] == '\n') || 
            (pointer == sizeof(request->string) - 1))
        {
            request->string[pointer] = '\0';
            request->len = pointer;
            mg_queue_push(&g_requests, &request->header);
            request = 0;
        }
    }

    mg_message_free(msg);

    return &g_usb_packets;
}

//
// Command executing actor. The response is populated in-place into the same 
// buffer.
//
static struct mg_queue_t* Actor_Executor(
    struct mg_actor_t* self, 
    struct mg_message_t* msg)
{
    struct cmd_buffer_t* buffer = (struct cmd_buffer_t*) msg;
    buffer->len = cmd_get_response(buffer->string, sizeof(buffer->string));
    mg_queue_push(&g_replies, msg);
    return &g_requests;
}

//
// Data sending actor. It processes the next outgoing message only after host
// acknowledgement of the previous one.
//
static struct mg_queue_t* Actor_Sender(
    struct mg_actor_t* self, 
    struct mg_message_t* msg)
{
    static struct mg_message_t* msg_being_sent = 0;

    if (msg_being_sent == 0)
    {
        struct cmd_buffer_t* reply = (struct cmd_buffer_t*) msg;
        CDC_Transmit(&g_usb_device, reply->string, reply->len);
        msg_being_sent = msg;
    }
    else
    {
        mg_message_free(msg_being_sent);
        mg_message_free(msg);
        msg_being_sent = 0;
    }

    return (msg_being_sent != 0) ? &g_datain_ack : &g_replies;
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
    // Setup parameters for low-level USB driver.
    //
    g_usbfs_driver.Instance = USB;
    g_usbfs_driver.Init.dev_endpoints = 8;
    g_usbfs_driver.Init.speed = PCD_SPEED_FULL;
    g_usbfs_driver.Init.low_power_enable = DISABLE;
    g_usbfs_driver.Init.lpm_enable = DISABLE;
    g_usbfs_driver.Init.battery_charging_enable = DISABLE;

    //
    // Initialize CDC class and hook DataIn/DataOut callbacks.
    //
    const USBD_ClassTypeDef* const cdcBase = CDC_Init();
    static USBD_ClassTypeDef cdc;

    cdc.Init = cdcBase->Init;
    cdc.DeInit = cdcBase->DeInit;
    cdc.Setup = cdcBase->Setup;
    cdc.EP0_TxSent = cdcBase->EP0_TxSent;
    cdc.EP0_RxReady = cdcBase->EP0_RxReady;
    cdc.DataIn = CDC_DataIn;
    cdc.DataOut = CDC_DataOut;
    cdc.HSConfigDescriptor = cdcBase->HSConfigDescriptor;
    cdc.FSConfigDescriptor = cdcBase->FSConfigDescriptor;
    cdc.OtherSpeedConfigDescriptor = cdcBase->OtherSpeedConfigDescriptor;
    cdc.DeviceQualifierDescriptor = cdcBase->DeviceQualifierDescriptor;

    static const alignas(sizeof(uint32_t)) uint8_t 
        desc_lang_id[USB_LEN_LANGID_STR_DESC] =
    {
        USB_LEN_LANGID_STR_DESC,
        USB_DESC_TYPE_STRING,
        LOBYTE(USBD_LANGID_STRING),
        HIBYTE(USBD_LANGID_STRING)
    };

    static const alignas(sizeof(uint32_t)) uint8_t 
        desc_device[USB_LEN_DEV_DESC] =
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
        ManufacturerStrSize = 19 * sizeof(uint16_t),
        ProductStrSize = 11 * sizeof(uint16_t),
        SerialStrSize = 5 * sizeof(uint16_t),
        ConfigStrSize = 11 * sizeof(uint16_t),
        InterfaceStrSize = 14 * sizeof(uint16_t),
    };

    static const alignas(sizeof(uint32_t)) uint16_t manufacturer_str[] = { 
        (USB_DESC_TYPE_STRING << 8) | ManufacturerStrSize,
        'S','T','M','i','c','r','o','e','l','e','c','t','r','o','n','i','c','s' 
    };

    static const alignas(sizeof(uint32_t)) uint16_t product_str[] = { 
        (USB_DESC_TYPE_STRING << 8) | ProductStrSize, 
        'C','D','C',' ','d','e','v','i','c','e' 
    };

    static const alignas(sizeof(uint32_t)) uint16_t serial_str[] = { 
        (USB_DESC_TYPE_STRING << 8) | SerialStrSize, '1','2','3','4' 
    };

    static const alignas(sizeof(uint32_t)) uint16_t config_str[] = { 
        (USB_DESC_TYPE_STRING << 8) | ConfigStrSize, 
        'C','D','C',' ','C','o','n','f','i','g' 
    };

    static const alignas(sizeof(uint32_t)) uint16_t interface_str[] = { 
        (USB_DESC_TYPE_STRING << 8) | InterfaceStrSize, 
        'C','D','C',' ','I','n','t','e','r','f','a','c','e' 
    };

    _Static_assert(ManufacturerStrSize == sizeof(manufacturer_str), "assert1");
    _Static_assert(ProductStrSize == sizeof(product_str), "assert2");
    _Static_assert(SerialStrSize == sizeof(serial_str), "assert3");
    _Static_assert(ConfigStrSize == sizeof(config_str), "assert4");
    _Static_assert(InterfaceStrSize == sizeof(interface_str), "assert5");

    static const USBD_DescriptorsTypeDef desc_array =
    {
        { desc_device, sizeof(desc_device) },
        { desc_lang_id, sizeof(desc_lang_id) },
        { manufacturer_str, sizeof(manufacturer_str) },
        { product_str, sizeof(product_str) },
        { serial_str, sizeof(serial_str) },
        { config_str, sizeof(config_str) },
        { interface_str, sizeof(interface_str) }
    };

    //
    // Initialize USB and create CDC endpoint.
    //
    if (USBD_Init(&g_usb_device, &g_usbfs_driver, &desc_array, &cdc) != USBD_OK)
    {
        Error_Handler();
    }

    HAL_PCDEx_PMAConfig(&g_usbfs_driver, 0x00 , PCD_SNG_BUF, 0x18);
    HAL_PCDEx_PMAConfig(&g_usbfs_driver, 0x80 , PCD_SNG_BUF, 0x58);
    HAL_PCDEx_PMAConfig(&g_usbfs_driver, CDC_IN_EP, PCD_SNG_BUF, 0xC0);
    HAL_PCDEx_PMAConfig(&g_usbfs_driver, CDC_OUT_EP, PCD_SNG_BUF, 0x110);
    HAL_PCDEx_PMAConfig(&g_usbfs_driver, CDC_CMD_EP, PCD_SNG_BUF, 0x100);

    //
    // Three vectors will be used: USB interrupt and two extra vectors for 
    // high and low priority actors respectively.
    //
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0);
    NVIC_SetPriority(WWDG_IRQn, 1);
    NVIC_SetPriority(PVD_IRQn, 2);

    //
    // Enable interrupts globally, but interrupts cannot be activated since
    // they are masked on NVIC.
    //
    __enable_irq();

    //
    // Initialize magnesium stuff...
    // 
    mg_context_init();

    msg_pool_init_helper(&g_datain_ack_msgs, g_datain_ack_mem);
    msg_pool_init_helper(&g_usb_data_msgs, g_usb_data_mem);
    msg_pool_init_helper(&g_cmdline_msgs, g_cmdline_mem);

    mg_queue_init(&g_datain_ack);
    mg_queue_init(&g_replies);
    mg_queue_init(&g_usb_packets);
    mg_queue_init(&g_requests);

    mg_actor_init(&g_receiver, Actor_Receiver, WWDG_IRQn, &g_usb_packets);
    mg_actor_init(&g_executor, Actor_Executor, PVD_IRQn, &g_requests);
    mg_actor_init(&g_sender, Actor_Sender, PVD_IRQn, &g_replies);

    //
    // Enable corresponding vectors and activate USB endpoints.
    //
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);    
    NVIC_EnableIRQ(WWDG_IRQn);
    NVIC_EnableIRQ(PVD_IRQn);

    if (USBD_Start(&g_usb_device) != USBD_OK)
    {
        Error_Handler();
    }

    //
    // Blink onboard LED to indicate succesful USB initialization.
    //
    GPIOC->BSRR = GPIO_BSRR_BR13;
    Delay(100);
    GPIOC->BSRR = GPIO_BSRR_BS13;   

    for (;;) __WFI();

    return 0; /* make compiler happy. */
}

