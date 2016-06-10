/*
 * spi_rj.h
 *
 *  Created on: 10 џэт. 2014 у.
 *      Author: r.leonov
 */

#ifndef SPI_RJ_H_
#define SPI_RJ_H_

#include "kl_lib_f100.h"
#include "config.h"

#define SPI   SPI1
#define GPIO  GPIOA
#define NSS   4
#define SCK   5
#define MISO  6
#define MOSI  7

/* WARNING! SPI_SLAVE_CNT couldn't be 0 */
#define SPI_SLAVE_CNT  1 // 1 - one Slave, 2 - two Slaves, 3 - three Slaver e.t.c


// ==== SPI related ====
#define SPI_MODE  	SPI_CR1_SSM |       /* NSS Software Managment Enable */    \
					SPI_CR1_SSI |       /* Internal slave select */            \
					SPI_CR1_CPOL |      /* Polaruty High */                    \
					SPI_CR1_CPHA |      /* 2 Edge */                           \
					SPI_CR1_MSTR        /* Master Configuration */



// Baudrate division
enum brDiv_t {
    brDiv2   = 0b000,
    brDiv4   = 0b001,
    brDiv8   = 0b010,
    brDiv16  = 0b011,
    brDiv32  = 0b100,
    brDiv64  = 0b101,
    brDiv128 = 0b110,
    brDiv256 = 0b111
};

enum SpiState_t {
    spiTxEmpty, spiBusy, spiRxNotEmpty, spiIdle
};

class Spi_t {
private:
    SpiState_t State;

    inline void Disable()               { SPI->CR1 &= ~SPI_CR1_SPE;      }
    inline void Enable()                { SPI->CR1 |=  SPI_CR1_SPE;      }
    inline void ClearCR2()              { SPI->CR2  = 0;                 }
    inline void ClearSR()               { SPI->SR   = 0x02;              }
    inline void SetBaud(brDiv_t brDiv)  { SPI->CR1 |= brDiv << 3;        }
    inline void Mode(uint16_t AMode)    { SPI->CR1  = AMode;             }
    // IRQs
    inline void RxNEIrqEnable()         { SPI->CR2 |=  SPI_CR2_RXNEIE;   }
    inline void RxNEIrqDisable()        { SPI->CR2 &= ~SPI_CR2_RXNEIE;   }
    inline void TxEIrqEnable()          { SPI->CR2 |=  SPI_CR2_TXEIE;    }
    inline void TxEIrqDisable()         { SPI->CR2 &= ~SPI_CR2_TXEIE;    }
    // Dma
    inline void TxDMAEnable()           { SPI->CR2 |=  SPI_CR2_TXDMAEN;  }
    inline void TxDMADisable()          { SPI->CR2 &= ~SPI_CR2_TXDMAEN;  }
    inline void RxDMAEnable()           { SPI->CR2 |=  SPI_CR2_RXDMAEN;  }
    inline void RxDMADisable()          { SPI->CR2 &= ~SPI_CR2_RXDMAEN;  }

    // Cmd Line
    void NssLo()  { PinClear(GPIO, NSS); }//Uart.Printf("lo\r"); }
    void NssHi()  { PinSet  (GPIO, NSS); }//Uart.Printf("hi\r"); }

    uint8_t byte_exchange(uint8_t Abyte);
public:
    void Init();
//    uint8_t DaisyTxRxByte(uint8_t id, uint8_t AByte);
//    void DaisyTxRxData(uint8_t id, uint8_t *Ptr, uint8_t Length, uint8_t *ToPtr);
//    void DaisyRxData(uint8_t id, uint8_t Length, uint8_t *ToPtr);

    uint8_t WriteReadByte(uint8_t AByte);
    uint8_t ReadByte() { return WriteReadByte(0x00); }

    void IrqHandler();
};

extern Spi_t Spi;

#endif /* SPI_RJ_H_ */
