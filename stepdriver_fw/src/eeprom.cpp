/*
 * eeprom.cpp
 *
 *  Created on: 03 апр. 2015 г.
 *      Author: RomaJam
 */


#include "eeprom.h"

eeprom_t EE;

void eeprom_t::Init() {
    eeprom_spi.Init(); // Init SPI driver

    PinSetupOut(EEPROM_GPIO, EEPROM_WRITE, omPushPull, ps50MHz); // WRITE
    PinSetupOut(EEPROM_GPIO, EEPROM_HOLD, omPushPull, ps50MHz); // HOLD
    holdDisable();
    writeEnable();

    uint8_t a;
    eeprom_spi.NssLo();
    eeprom_spi.WriteReadByte(0x05);
    a = eeprom_spi.ReadByte();
    Uart.Printf("%X\r", a);
    eeprom_spi.NssHi();
    Uart.Printf("EE init\r");
}


#if 1 // ==================== LOW LEVEL ========================================

void eeprom_spi_t::Init() {
    PinSetupOut(EEPROM_GPIO, EEPROM_NSS, omPushPull, ps50MHz); // CS
    NssHi();
    PinSetupAlterFuncOutput(EEPROM_GPIO, EEPROM_SCK, omPushPull, ps50MHz);
    PinSetupAlterFuncOutput(EEPROM_GPIO, EEPROM_MISO, omPushPull, ps50MHz);
    PinSetupAlterFuncOutput(EEPROM_GPIO, EEPROM_MOSI, omPushPull, ps50MHz);

    // SPI
    rccEnableSPI2(FALSE);
    Mode(EEPROM_SPI_MODE);
    ClearCR2();                 // Disable all interrupts
    ClearSR();                  // Clear Status Register
    SetBaud(brDiv32);            // Set baudrate: Fpclk/16 = 48 MHz/16 = 3 Mhz NOTE: max freq 5MHz
//    TxEIrqEnable();
    ErrIrqEnable();
    Enable();
    nvicEnableVector(SPI2_IRQn, CORTEX_PRIORITY_MASK(IRQ_PRIO_MEDIUM));
}

void eeprom_spi_t::IrqHandler() {
    Uart.Printf("i:%X\r", SPI->SR);
    // This is an interrupt
    ClearSR();
}

extern "C" {
CH_IRQ_HANDLER(SPI2_IRQHandler) {
    CH_IRQ_PROLOGUE();
    chSysLockFromIsr();
    EE.eeprom_spi.IrqHandler();
    chSysUnlockFromIsr();
    CH_IRQ_EPILOGUE();
}
}
#endif // =====================================================================
