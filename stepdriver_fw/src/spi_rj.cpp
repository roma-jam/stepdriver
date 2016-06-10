/*
 * spi_rj.cpp
 *
 *  Created on: 10 џэт. 2014 у.
 *      Author: r.leonov
 */


#include "spi_rj.h"

Spi_t Spi;

void Spi_t::Init() {
    // GPIOs
    PinSetupOut(GPIO, NSS, omPushPull, ps50MHz); // CS
    PinSetupAlterFuncOutput(GPIO, SCK, omPushPull, ps50MHz);
    PinSetupAlterFuncOutput(GPIO, MISO, omPushPull, ps50MHz);
    PinSetupAlterFuncOutput(GPIO, MOSI, omPushPull, ps50MHz);

    // SPI
    rccEnableSPI1(FALSE);
    Mode(SPI_MODE);
    ClearCR2();                 // Disable all interrupts
    ClearSR();                  // Clear Status Register
    SetBaud(brDiv4);            // Set baudrate: Fpclk/16 = 48 MHz/16 = 3 Mhz NOTE: max freq 5MHz
    Enable();
//    nvicEnableVector(SPI1_IRQn, CORTEX_PRIORITY_MASK(IRQ_PRIO_MEDIUM));

    State = spiIdle;
}


//void Spi_t::DaisyTxRxData(uint8_t id, uint8_t *Ptr, uint8_t Length, uint8_t *ToPtr) {
//    for(uint8_t i = 0; i < Length; i++) {
//        *ToPtr++ = DaisyTxRxByte(id, *Ptr++);
//    }
//}
//
//void Spi_t::DaisyRxData(uint8_t id, uint8_t Length, uint8_t *ToPtr) {
//    for(uint8_t i = 0; i < Length; i++) {
//        *ToPtr++ = DaisyTxRxByte(id, 0x00);
////        Uart.Printf("<-%X\r", *ToPtr);
//    }
//}
//
//uint8_t Spi_t::DaisyTxRxByte(uint8_t id, uint8_t AByte) {
////    if (State == spiBusy) return BUSY;
//    if (id > SPI_SLAVE_CNT)
//        return CMD_ERROR;
//
//    uint8_t rpl;
//	uint8_t need_to_write, need_to_read;
//	need_to_write = id;
//	need_to_read = (SPI_SLAVE_CNT-1) - id;
//	NssLo();
//	while(need_to_read-- > 0) ReadByte();
//	rpl = WriteReadByte(AByte);
//	while(need_to_write-- > 0) ReadByte();
//	NssHi();
//    return rpl;
//}

uint8_t Spi_t::WriteReadByte(uint8_t Abyte)
{
    uint8_t read_byte;
    NssLo();
    read_byte = byte_exchange(Abyte);
    NssHi();
    return read_byte;
}


uint8_t Spi_t::byte_exchange(uint8_t AByte) {
    SPI->DR = AByte;
#if (APP_SPI_DEBUG_IO)
    Uart.Printf("->%X\r", AByte);
#endif
    while (!(SPI->SR & SPI_SR_TXE));
    while (SPI->SR & SPI_SR_BSY);
    while (!(SPI->SR & SPI_SR_RXNE));
#if (APP_SPI_DEBUG_IO)
    Uart.Printf("<-%X\r", SPI->DR);
#endif
    return SPI->DR;
}

void Spi_t::IrqHandler() {
    if(SPI->SR & SPI_SR_TXE) {       // Interrupt by TxEmpty Buffer
//        sr &= ~SPI_SR_TXE;
//        State = spiTxEmpty;
        Uart.Printf("1 TxE, CR2=%X\r", SPI->CR2);
        TxEIrqDisable();
        Uart.Printf("2 TxE, CR2=%X\r", SPI->CR2);
    } // If TXE

    if(SPI->SR & SPI_SR_BSY) {
//        sr &= ~SPI_SR_BSY;
        Uart.Printf("Bsy\r");
        State = spiBusy;
    }

    if(SPI->SR & SPI_SR_RXNE) {
//        sr &= ~SPI_SR_RXNE;
        State = spiRxNotEmpty;
        Uart.Printf("1 RxNE, CR2=%X\r", SPI->CR2);
        RxNEIrqDisable();
        Uart.Printf("2 RxNE, CR2=%X\r", SPI->CR2);
    }
    ClearSR();
}

extern "C" {
CH_IRQ_HANDLER(SPI1_IRQHandler) {
    CH_IRQ_PROLOGUE();
    chSysLockFromIsr();
    Spi.IrqHandler();
    chSysUnlockFromIsr();
    CH_IRQ_EPILOGUE();
}
}
