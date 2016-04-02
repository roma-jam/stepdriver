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

#define WIFI_DMA_MODE       STM32_DMA_CR_PL(0b10)   |  /* Priority is high */  \
                            STM32_DMA_CR_MSIZE_BYTE | \
                            STM32_DMA_CR_PSIZE_BYTE | \
                            STM32_DMA_CR_MINC       |  /* Memory pointer increase*/ \
                            STM32_DMA_CR_DIR_M2P    |  /* Direction is memory to peripheral*/ \
                            STM32_DMA_CR_TCIE           // Enable Transmission Complete IRQ

#define WIFI_GPIO           GPIOA
#define WIFI_TX_PIN         2
#define WIFI_RX_PIN         3
#define WIFI_BAUDRATE       115200

#define WIFI_CMD_BUF_SZ     128     // Maximum Command Length is 127 character (um p. 5)
#define WIFI_RX_BYTE        WIFI_UART->DR

#define WIFI_PWR_GPIO       GPIOB
#define WIFI_PWR_PIN        3

class wifi_driver_t {
private:
    uint8_t HostBuf[WIFI_CMD_BUF_SZ];
    uint8_t *pRead;
    Thread *PWaitThd;
    bool WiFiDmaIsIdle;
    uint32_t ITransmitSize, SlotsFilled;
    void IHandleByte();
    void IWaitTxEnd() {
        chSysLock();
        PWaitThd = chThdSelf();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
    }
    void ITxEnd() {
        if(PWaitThd != NULL) {
            chSysLockFromIsr();
            chSchReadyI(PWaitThd);
            chSysUnlockFromIsr();
        }
    }
public:
    round_buf_t RplBuf;
    void Init();
    void CmdSend(uint8_t *PBuf, uint32_t Length);
    void CmdSendDma();
    void PowerOn()  { PinClear(WIFI_PWR_GPIO, WIFI_PWR_PIN); }
    void PowerOff() { PinSet  (WIFI_PWR_GPIO, WIFI_PWR_PIN); }
    void IRQ_RxHandler();
    void IRQ_TxHandler();
};

extern wifi_driver_t WiFi;

#endif /* WIFI_DRIVER_H_ */
