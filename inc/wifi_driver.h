/*
 * wifi_driver.h
 *
 *  Created on: 20 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef WIFI_DRIVER_H_
#define WIFI_DRIVER_H_

#include "kl_lib_f100.h"

#define WIFI_UART           USART2
#define WIFI_Clock_EN()     rccEnableUSART2(FALSE);
#define WIFI_TX_DMA         STM32_DMA1_STREAM7
#define WIFI_IRQ_Handler    USART2_IRQHandler
#define WIFI_IRQ            USART2_IRQn


#define WIFI_GPIO           GPIOA
#define WIFI_TX_PIN         2
#define WIFI_RX_PIN         3
#define WIFI_BAUDRATE       115200


class wifi_driver_t {
private:
public:
    void Init();
    void IRQ_Handler();
};

extern wifi_driver_t WiFi;

#endif /* WIFI_DRIVER_H_ */
