#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/gpio.h"

#define LED_GPIO           GPIO_NUM_8
#define UWB_SPI_SCLK_GPIO  GPIO_NUM_6
#define UWB_SPI_MISO_GPIO  GPIO_NUM_9
#define UWB_SPI_MOSI_GPIO  GPIO_NUM_16
#define UWB_SPI_CS_GPIO    GPIO_NUM_15
#define UWB_SPI_IRQ_GPIO   GPIO_NUM_10
#define UWB_RST_GPIO       GPIO_NUM_19
#define UWB_WAKEUP_GPIO    GPIO_NUM_24

#endif // PIN_CONFIG_H
