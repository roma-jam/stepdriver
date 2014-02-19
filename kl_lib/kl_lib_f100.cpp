/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "kl_lib_f100.h"
#include <stdarg.h>
#include <string.h>

// ============================== UART command =================================
#ifdef DBG_UART_ENABLED
DbgUart_t Uart;

static inline void FPutChar(char c) { Uart.IPutChar(c); }

void DbgUart_t::IPutChar(char c) {
    *PWrite++ = c;
    if(PWrite >= &TXBuf[UART_TXBUF_SIZE]) PWrite = TXBuf;   // Circulate buffer
}

void DbgUart_t::Printf(const char *format, ...) {
    uint32_t MaxLength = (PWrite < PRead)? (PRead - PWrite) : ((UART_TXBUF_SIZE + PRead) - PWrite);
    va_list args;
    va_start(args, format);
    IFullSlotsCount += kl_vsprintf(FPutChar, MaxLength, format, args);
    va_end(args);

    // Start transmission if Idle
    if(IDmaIsIdle) {
        IDmaIsIdle = false;
        dmaStreamSetMemory0(UART_TX_DMA, PRead);
        uint32_t PartSz = (TXBuf + UART_TXBUF_SIZE) - PRead;    // Char count from PRead to buffer end
        ITransSize = (IFullSlotsCount > PartSz)? PartSz : IFullSlotsCount;  // How many to transmit now
        dmaStreamSetTransactionSize(UART_TX_DMA, ITransSize);
        dmaStreamEnable(UART_TX_DMA);
    }
}

// ==== Init & DMA ====
// Wrapper for IRQ
extern "C" {
void DbgUartIrq(void *p, uint32_t flags) { Uart.IRQDmaTxHandler(); }
}
void DbgUart_t::Init(uint32_t ABaudrate) {
    PWrite = TXBuf;
    PRead = TXBuf;
    IFullSlotsCount = 0;
    IDmaIsIdle = true;
    PinSetupAlterFuncOutput(GPIOA, 9, omPushPull);      // TX1

    // ==== USART configuration ====
    rccEnableUSART1(FALSE);     // UART clock
    USART1->BRR = Clk.APB2FreqHz / ABaudrate;
    USART1->CR2 = 0;
    USART1->CR3 = USART_CR3_DMAT;   // Enable DMA at transmitter
    USART1->CR1 = USART_CR1_TE;     // Transmitter enabled

    // ==== DMA ====
    // Here only the unchanged parameters of the DMA are configured.
    dmaStreamAllocate     (UART_TX_DMA, 1, DbgUartIrq, NULL);
    dmaStreamSetPeripheral(UART_TX_DMA, &USART1->DR);
    dmaStreamSetMode      (UART_TX_DMA,
            STM32_DMA_CR_PL(0b10) |     // Priority is high
            STM32_DMA_CR_MSIZE_BYTE |
            STM32_DMA_CR_PSIZE_BYTE |
            STM32_DMA_CR_MINC |         // Memory pointer increase
            STM32_DMA_CR_DIR_M2P |      // Direction is memory to peripheral
            STM32_DMA_CR_TCIE           // Enable Transmission Complete IRQ
             );
    USART1->CR1 |= USART_CR1_UE;        // Enable USART
}


// ==== IRQs ====
void DbgUart_t::IRQDmaTxHandler() {
    dmaStreamDisable(UART_TX_DMA);    // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SIZE)) PRead = TXBuf; // Circulate pointer

    if(IFullSlotsCount == 0) IDmaIsIdle = true; // Nothing left to send
    else {  // There is something to transmit more
        dmaStreamSetMemory0(UART_TX_DMA, PRead);
        uint32_t PartSz = (TXBuf + UART_TXBUF_SIZE) - PRead;
        ITransSize = (IFullSlotsCount > PartSz)? PartSz : IFullSlotsCount;
        dmaStreamSetTransactionSize(UART_TX_DMA, ITransSize);
        dmaStreamEnable(UART_TX_DMA);    // Restart DMA
    }
}
#endif

