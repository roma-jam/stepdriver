/*
 * wifi_driver.h
 *
 *  Created on: 20 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef WIFI_DRIVER_H_
#define WIFI_DRIVER_H_

#include "kl_lib_f100.h"
#include "round_buf.h"

/* INFO
 * <cr><lf>+WIND:<number>:<descriptive string><cr><lf> asynchronius command
 *
 * AT<cmd><cr>
 * <zero or more response lines><cr><lf>
 * <responsecode><cr><lf>
 */


#define WIFI_UART           USART2
#define WIFI_Clock_EN()     rccEnableUSART2(FALSE);
#define WIFI_TX_DMA         STM32_DMA1_STREAM7
#define WIFI_IRQ_Handler    USART2_IRQHandler
#define WIFI_IRQ            USART2_IRQn

#define WIFI_GPIO           GPIOA
#define WIFI_TX_PIN         2
#define WIFI_RX_PIN         3
#define WIFI_BAUDRATE       115200

#define WIFI_CMD_BUF_SZ     127     // Maximum Command Length is 127 character (um p. 5)
#define WIFI_RX_BYTE        WIFI_UART->DR

#define WIFI_STR_CR         0x0D // mean <cr>
#define WIFI_STR_LF         0x0A // mean <lf>


class wifi_driver_t {
private:
    void IHandleByte();
public:
    round_buf_t RplBuf;
    void Init();
    void IRQ_Handler();
    void Event_CommandRdy() {}
};

extern wifi_driver_t WiFi;

#endif /* WIFI_DRIVER_H_ */
