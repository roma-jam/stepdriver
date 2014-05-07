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


#if 1 // ============================= I2C =====================================
void i2cDmaIrqHandler(void *p, uint32_t flags) {
    chSysLockFromIsr();
    //Uart.Printf("===T===");
    Thread *PThd = ((i2c_t*)p)->PRequestingThread;
    if (PThd != NULL) {
        ((i2c_t*)p)->PRequestingThread = NULL;
        chSchReadyI(PThd);
    }
    chSysUnlockFromIsr();
}

void i2c_t::Init(
        I2C_TypeDef *pi2c,
        GPIO_TypeDef *PGpio,
        uint16_t SclPin,
        uint16_t SdaPin,
        uint32_t BitrateHz,
        const stm32_dma_stream_t *APDmaTx,
        const stm32_dma_stream_t *APDmaRx
    ) {
    ii2c = pi2c;
    IPGpio = PGpio;
    ISclPin = SclPin;
    ISdaPin = SdaPin;
    IBitrateHz = BitrateHz;
    Standby();
    Resume();

    // ==== DMA ====
    // Here only unchanged parameters of the DMA are configured.
    // Setup Dma TX
    PDmaTx = APDmaTx;
    dmaStreamAllocate(PDmaTx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaTx, &ii2c->DR);
    // Setup Dma RX
    PDmaRx = APDmaRx;
    dmaStreamAllocate(PDmaRx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaRx, &ii2c->DR);
}

void i2c_t::Standby() {
    if(ii2c == I2C1) { rccResetI2C1(); rccDisableI2C1(FALSE); }
    else             { rccResetI2C2(); rccDisableI2C2(FALSE); }
    // Disable GPIOs
    PinSetupAnalog(IPGpio, ISclPin);
    PinSetupAnalog(IPGpio, ISdaPin);
}

void i2c_t::Resume() {
    Error = false;
    // ==== GPIOs ====
    PinSetupAlterFuncOutput(IPGpio, ISclPin, omOpenDrain, ps50MHz);
    PinSetupAlterFuncOutput(IPGpio, ISdaPin, omOpenDrain, ps50MHz);
    // ==== Clock and reset ====
    if(ii2c == I2C1) { rccEnableI2C1(FALSE); rccResetI2C1(); }
    else             { rccEnableI2C2(FALSE); rccResetI2C2(); }
    // Minimum clock is 2 MHz
    uint32_t ClkMhz = Clk.APB1FreqHz / 1000000;
    uint16_t tmpreg = ii2c->CR2;
    tmpreg &= (uint16_t)~I2C_CR2_FREQ;
    if(ClkMhz < 2)  ClkMhz = 2;
    if(ClkMhz > 32) ClkMhz = 32;
    tmpreg |= ClkMhz;
    ii2c->CR2 = tmpreg;
    ii2c->CR1 &= (uint16_t)~I2C_CR1_PE; // Disable i2c to setup TRise & CCR
    ii2c->TRISE = (uint16_t)(((ClkMhz * 300) / 1000) + 1);
    // 16/9
    tmpreg = (uint16_t)(Clk.APB1FreqHz / (IBitrateHz * 25));
    if(tmpreg == 0) tmpreg = 1; // minimum allowed value
    tmpreg |= I2C_CCR_FS | I2C_CCR_DUTY;
    ii2c->CCR = tmpreg;
    ii2c->CR1 |= I2C_CR1_PE;    // Enable i2c back
    // ==== DMA ====
    ii2c->CR2 |= I2C_CR2_DMAEN;
}

void i2c_t::Reset() {
    Standby();
    Resume();
}

uint8_t i2c_t::CmdWriteRead(uint8_t Addr,
        uint8_t *WPtr, uint8_t WLength,
        uint8_t *RPtr, uint8_t RLength) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength);
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    // Read if needed
    if(RLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        // Send repeated start
        SendStart();
        if(WaitEv5() != OK) return FAILURE;
        SendAddrWithRead(Addr);
        if(WaitEv6() != OK) { SendStop(); return FAILURE; }
        // If single byte is to be received, disable ACK before clearing ADDR flag
        if(RLength == 1) AckDisable();
        else AckEnable();
        ClearAddrFlag();
        dmaStreamSetMemory0(PDmaRx, RPtr);
        dmaStreamSetMode   (PDmaRx, I2C_DMARX_MODE);
        dmaStreamSetTransactionSize(PDmaRx, RLength);
        DmaLastTransferSet(); // Inform DMA that this is last transfer => do not ACK last byte
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaRx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaRx);
    } // if != 0
    else WaitBTF(); // if nothing to read, just stop
    SendStop();
    return OK;
}

uint8_t i2c_t::CmdWriteWrite(uint8_t Addr,
        uint8_t *WPtr1, uint8_t WLength1,
        uint8_t *WPtr2, uint8_t WLength2) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr1);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength1);
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    if(WLength2 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr2);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength2);
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    WaitBTF();
    SendStop();
    return OK;
}

// ==== Flag operations ====
// Busy flag
uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(ii2c->SR2 & I2C_SR2_BUSY)) return OK;
        chThdSleepMilliseconds(1);
    }
    Error = true;
    return TIMEOUT;
}

// BUSY, MSL & SB flags
uint8_t i2c_t::WaitEv5() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--) {
        uint16_t Flag1 = ii2c->SR1;
        uint16_t Flag2 = ii2c->SR2;
        if((Flag1 & I2C_SR1_SB) and (Flag2 & (I2C_SR2_MSL | I2C_SR2_BUSY))) return OK;
    }
    Error = true;
    return FAILURE;
}

uint8_t i2c_t::WaitEv6() {
    uint32_t RetryCnt = 45;
    uint16_t Flag1;
    do {
        Flag1 = ii2c->SR1;
        if((RetryCnt-- == 0) or (Flag1 & I2C_SR1_AF)) return FAILURE;   // Fail if timeout or NACK
    } while(!(Flag1 & I2C_SR1_ADDR)); // ADDR set when Address is sent and ACK received
    return OK;
}

uint8_t i2c_t::WaitEv8() {
    uint32_t RetryCnt = 45;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_TXE) return OK;
    Error = true;
    return TIMEOUT;
}

uint8_t i2c_t::WaitRx() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_RXNE) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitStop() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->CR1 & I2C_CR1_STOP) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitBTF() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_BTF) return OK;
    return TIMEOUT;
}
#endif
