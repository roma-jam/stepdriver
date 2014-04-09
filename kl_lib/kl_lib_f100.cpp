/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "kl_lib_f100.h"
#include <stdarg.h>
#include <string.h>


#if 1 // ============================= Timer ===================================
void Timer_t::Init(TIM_TypeDef* Tmr) {
    ITmr = Tmr;
    if(ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
    // Clock src
    if(ANY_OF_3(ITmr, TIM2, TIM3, TIM4)) PClk = &Clk.APB1FreqHz;
    else PClk = &Clk.APB2FreqHz;
}

void Timer_t::InitPwm(GPIO_TypeDef *GPIO, uint16_t N, uint8_t Chnl, Inverted_t Inverted) {
    // GPIO
    PinSetupAlterFuncOutput(GPIO, N, omPushPull);

    // Output
    uint16_t tmp = (Inverted == invInverted)? 0b111 : 0b110; // PWM mode 1 or 2
    switch(Chnl) {
        case 1:
            PCCR = &ITmr->CCR1;
            ITmr->CCMR1 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC1E;
            break;

        case 2:
            PCCR = &ITmr->CCR2;
            ITmr->CCMR1 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC2E;
            break;

        case 3:
            PCCR = &ITmr->CCR3;
            ITmr->CCMR2 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC3E;
            break;

        case 4:
            PCCR = &ITmr->CCR4;
            ITmr->CCMR2 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC4E;
            break;

        default: break;
    }
}

#endif


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

