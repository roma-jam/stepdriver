/*
 * eeprom.h
 *
 *  Created on: 03 апр. 2015 г.
 *      Author: RomaJam
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "kl_lib_f100.h"
#include "spi_rj.h"

#define EEPROM_SPI   SPI2
#define EEPROM_GPIO  GPIOB
#define EEPROM_HOLD  11
#define EEPROM_WRITE 10
#define EEPROM_NSS   12
#define EEPROM_SCK   13
#define EEPROM_MISO  14
#define EEPROM_MOSI  15

// ==== SPI related ====
#define EEPROM_SPI_MODE    SPI_CR1_SSM |       /* NSS Software Managment Enable */    \
                    SPI_CR1_SSI |       /* Internal slave select */            \
                    SPI_CR1_MSTR        /* Master Configuration */

class eeprom_spi_t {
private:
    inline void RxNEIrqEnable()         { EEPROM_SPI->CR2 |=  SPI_CR2_RXNEIE;   }
    inline void RxNEIrqDisable()        { EEPROM_SPI->CR2 &= ~SPI_CR2_RXNEIE;   }
    inline void TxEIrqEnable()          { EEPROM_SPI->CR2 |=  SPI_CR2_TXEIE;    }
    inline void TxEIrqDisable()         { EEPROM_SPI->CR2 &= ~SPI_CR2_TXEIE;    }
    inline void ErrIrqEnable()          { EEPROM_SPI->CR2 |=  SPI_CR2_ERRIE;    }
    inline void ErrIrqDisable()         { EEPROM_SPI->CR2 &= ~SPI_CR2_ERRIE;    }

    inline void Disable()               { EEPROM_SPI->CR1 &= ~SPI_CR1_SPE;      }
    inline void Enable()                { EEPROM_SPI->CR1 |=  SPI_CR1_SPE;      }
    inline void ClearCR2()              { EEPROM_SPI->CR2  = 0;                 }
    inline void ClearSR()               { EEPROM_SPI->SR   = 0x02;              }
    inline void SetBaud(brDiv_t brDiv)  {
        EEPROM_SPI->CR1 &= ~SPI_CR1_BR; // clear baudrate
        EEPROM_SPI->CR1 |= (brDiv << 3);
    }
    inline void Mode(uint16_t AMode)    { EEPROM_SPI->CR1  = AMode;             }
public:
    void NssLo()  { PinClear(EEPROM_GPIO, EEPROM_NSS); }
    void NssHi()  { PinSet  (EEPROM_GPIO, EEPROM_NSS); }
    uint8_t WriteReadByte(uint8_t AByte) {
        EEPROM_SPI->DR = AByte;
        while (!(EEPROM_SPI->SR & SPI_SR_TXE));
        while (EEPROM_SPI->SR & SPI_SR_BSY);
        while (!(EEPROM_SPI->SR & SPI_SR_RXNE));
        return EEPROM_SPI->DR;
    }
    uint8_t ReadByte() { return WriteReadByte(0x00); }
    void Init();
    void IrqHandler();
};

class eeprom_t {
private:
    void writeEnable()  { PinSet  (EEPROM_GPIO, EEPROM_WRITE); }
    void writeDisable() { PinClear(EEPROM_GPIO, EEPROM_WRITE); }
    void holdEnable()   { PinClear(EEPROM_GPIO, EEPROM_HOLD);  }
    void holdDisable()  { PinSet  (EEPROM_GPIO, EEPROM_HOLD);  }

    uint8_t readbyte();
    uint32_t readU32();
public:
    eeprom_spi_t eeprom_spi;
    void Init();
};

extern eeprom_t EE;

#endif /* INC_EEPROM_H_ */
