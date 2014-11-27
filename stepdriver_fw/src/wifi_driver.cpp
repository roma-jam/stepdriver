/*
 * wifi_driver.cpp
 *
 *  Created on: 20 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "wifi_driver.h"
#include "http_server.h"

wifi_driver_t WiFi;

void wifi_driver_t::Init() {
//    PinSetupAlterFuncOutput(WIFI_GPIO, WIFI_TX_PIN, omPushPull);
    PinSetupIn(WIFI_GPIO, WIFI_RX_PIN, pudNone);
    // ==== USART configuration ====
    WIFI_Clock_EN();
    WIFI_UART->BRR = Clk.APB2FreqHz / WIFI_BAUDRATE;
    WIFI_UART->CR2 = 0;
    WIFI_UART->CR1 = USART_CR1_TE | USART_CR1_RE;        // TX & RX enable
    WIFI_UART->CR1 |= USART_CR1_RXNEIE; // RxNotEmptu IRQ

    WIFI_UART->CR3 = USART_CR3_DMAT;    // Enable DMA at TX & RX

    nvicEnableVector(WIFI_IRQ, CORTEX_PRIORITY_MASK(IRQ_PRIO_MEDIUM));
    // ==== DMA ====
    // Here only the unchanged parameters of the DMA are configured.
    dmaStreamAllocate     (WIFI_TX_DMA, 1, NULL, NULL);
    dmaStreamSetPeripheral(WIFI_TX_DMA, &WIFI_UART->DR);
    dmaStreamSetMode      (WIFI_TX_DMA,
            STM32_DMA_CR_PL(0b10)   |     // Priority is high
            STM32_DMA_CR_MSIZE_BYTE |
            STM32_DMA_CR_PSIZE_BYTE |
            STM32_DMA_CR_MINC       |         // Memory pointer increase
            STM32_DMA_CR_DIR_M2P    |      // Direction is memory to peripheral
            STM32_DMA_CR_TCIE           // Enable Transmission Complete IRQ
             );
    WIFI_UART->CR1 |= USART_CR1_UE;        // Enable USART
    Uart.Printf("WiFi Init\r");
}

void wifi_driver_t::IHandleByte() {
    uint8_t Byte = WIFI_RX_BYTE;
    RplBuf.Write(Byte);
//    Uart.Printf("%c", Byte);
    if(Byte == WIFI_STR_LF) HttpServer.Wakeup();
}

void wifi_driver_t::IRQ_Handler() {
    IHandleByte();
    WIFI_UART->SR &= ~USART_SR_RXNE;
}

extern "C" {
CH_IRQ_HANDLER(WIFI_IRQ_Handler) {
    CH_IRQ_PROLOGUE();
    chSysLockFromIsr();
    WiFi.IRQ_Handler();
    chSysUnlockFromIsr();
    CH_IRQ_EPILOGUE();
}
}
