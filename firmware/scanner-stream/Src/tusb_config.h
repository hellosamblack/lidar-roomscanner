#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU              OPT_MCU_STM32H5
#define CFG_TUSB_OS               OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE

#define BOARD_TUD_RHPORT          0

#define CFG_TUD_ENABLED           1
#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_CDC               1
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    2048
#define CFG_TUD_CDC_EP_BUFSIZE    64

#endif
