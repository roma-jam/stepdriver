/*
 * kl_lib_f0xx.h
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#ifndef KL_LIB_F100_H_
#define KL_LIB_F100_H_

#include "stm32f10x.h"
#include "ch.h"
#include "hal.h"
#include "clocking_f100.h"
#include "kl_sprintf.h"

extern "C" {
//void _init(void);   // Need to calm linker
void __attribute__ ((weak)) _init(void)  {}

}

// Return values
#define OK              0
#define NO_ERROR        0
#define FAILURE         1
#define TIMEOUT         2
#define BUSY            3
#define NEW             4
#define IN_PROGRESS     5
#define LAST            6
#define CMD_ERROR       7
#define WRITE_PROTECT   8
#define LENGTH_ERROR    9

// IRQ priorities
#define IRQ_PRIO_LOW            15  // Minimum
#define IRQ_PRIO_MEDIUM         9
#define IRQ_PRIO_HIGH           7
#define IRQ_PRIO_VERYHIGH       4 // Higher than systick

// DMA
#define DMA_PRIORITY_LOW        STM32_DMA_CR_PL(0b00)
#define DMA_PRIORITY_MEDIUM     STM32_DMA_CR_PL(0b01)
#define DMA_PRIORITY_HIGH       STM32_DMA_CR_PL(0b10)
#define DMA_PRIORITY_VERYHIGH   STM32_DMA_CR_PL(0b11)

typedef void (*ftVoidVoid)(void);

// =============================== General =====================================
#define PACKED __attribute__ ((__packed__))
#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

// Simple pseudofunctions
#define TRIM_VALUE(v, Max)  { if(v > Max) v = Max; }
#define IS_LIKE(v, precise, deviation)  (((precise - deviation) < v) and (v < (precise + deviation)))

#define MIN(a, b)                       (((a) < (b))? (a) : (b))
#define MAX(a, b)                       (((a) > (b))? (a) : (b))

#define ANY_OF_2(a, b1, b2)             (((a)==(b1)) or ((a)==(b2)))
#define ANY_OF_3(a, b1, b2, b3)         (((a)==(b1)) or ((a)==(b2)) or ((a)==(b3)))
#define ANY_OF_4(a, b1, b2, b3, b4)     (((a)==(b1)) or ((a)==(b2)) or ((a)==(b3)) or ((a)==(b4)))
#define ANY_OF_5(a, b1, b2, b3, b4, b5) (((a)==(b1)) or ((a)==(b2)) or ((a)==(b3)) or ((a)==(b4)) or ((a)==(b5)))


// ============================ Simple delay ===================================
static inline void DelayLoop(volatile uint32_t ACounter) { while(ACounter--); }
static inline void Delay_ms(uint32_t Ams) {
    volatile uint32_t __ticks = (Clk.AHBFreqHz / 4000) * Ams;
    DelayLoop(__ticks);
}

// ===================== Single pin manipulations ==============================
enum PinOutMode_t {
    omPushPull  = 0,
    omOpenDrain = 1
};
enum PinPullUpDown_t {
    pudNone = 0b00,
    pudPullUp = 0b01,
    pudPullDown = 0b10
};
enum PinSpeed_t {
    ps2MHz  = 0b10,
    ps10MHz = 0b01,
    ps50MHz = 0b11
};

// Set/clear
static inline void PinSet    (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { PGpioPort->BSRR = (uint32_t)(1<<APinNumber); }
static inline void PinClear  (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { PGpioPort->BRR  = (uint32_t)(1<<APinNumber); }
static inline void PinToggle (GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { PGpioPort->ODR ^= (uint32_t)(1<<APinNumber); }
// Check state
static inline bool PinIsSet(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) { return (PGpioPort->IDR & (uint32_t)(1<<APinNumber)); }
// Setup
static inline void PinClockEnable(GPIO_TypeDef *PGpioPort) {
    if     (PGpioPort == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    else if(PGpioPort == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    else if(PGpioPort == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    else if(PGpioPort == GPIOD) RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
}
static inline void PinSetupOut(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const PinOutMode_t PinOutMode,
        const PinSpeed_t ASpeed = ps50MHz
        ) {
    // Clock
    PinClockEnable(PGpioPort);
    // Setup
    uint32_t CnfMode = ((uint32_t)PinOutMode << 2) | (uint32_t)ASpeed;
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRL |= CnfMode << Offset;
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRH |= CnfMode << Offset;
    }
}
static inline void PinSetupIn(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber, const PinPullUpDown_t APullUpDown) {
    // Clock
    PinClockEnable(PGpioPort);
    // Setup
    uint32_t CnfMode;
    if(APullUpDown == pudNone) CnfMode = 0b0100;
    else {
        CnfMode = 0b1000;
        if(APullUpDown == pudPullDown) PGpioPort->ODR &= ~((uint32_t)(1<<APinNumber));
        else PGpioPort->ODR |= (uint32_t)(1<<APinNumber);
    }
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRL |= CnfMode << Offset;
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRH |= CnfMode << Offset;
    }
}
static inline void PinSetupAnalog(GPIO_TypeDef *PGpioPort, const uint16_t APinNumber) {
    // Clock
    PinClockEnable(PGpioPort);
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
    }
}
static inline void PinSetupAlterFuncOutput(
        GPIO_TypeDef *PGpioPort,
        const uint16_t APinNumber,
        const PinOutMode_t PinOutMode,
        const PinSpeed_t ASpeed = ps50MHz
        ) {
    // Clock
    PinClockEnable(PGpioPort);
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;     // Enable AFIO clock
    // Setup
    uint32_t CnfMode = ((uint32_t)PinOutMode << 2) | 0b1000 | (uint32_t)ASpeed;
    if(APinNumber < 8) {
        uint8_t Offset = APinNumber*4;
        PGpioPort->CRL &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRL |= CnfMode << Offset;
    }
    else {
        uint8_t Offset = (APinNumber - 8) * 4;
        PGpioPort->CRH &= ~((uint32_t)(0b1111 << Offset));  // Clear both mode and cnf
        PGpioPort->CRH |= CnfMode << Offset;
    }
}

// Disable JTAG, leaving SWD
static inline void JtagDisable() {
    bool AfioWasEnabled = (RCC->APB2ENR & RCC_APB2ENR_AFIOEN);
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;     // Enable AFIO
    uint32_t tmp = AFIO->MAPR;
    tmp &= ~0x07000000;
    tmp |= 0x02000000;
    AFIO->MAPR = tmp;
    if (!AfioWasEnabled) RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN;
}

// ================================= IWDG ======================================
enum IwdgPre_t {
    iwdgPre4 = 0x00,
    iwdgPre8 = 0x01,
    iwdgPre16 = 0x02,
    iwdgPre32 = 0x03,
    iwdgPre64 = 0x04,
    iwdgPre128 = 0x05,
    iwdgPre256 = 0x06
};

class IWDG_t {
private:
    void EnableAccess() { IWDG->KR = 0x5555; }
    void SetPrescaler(IwdgPre_t Prescaler) { IWDG->PR = (uint32_t)Prescaler; }
    void SetReload(uint16_t Reload) { IWDG->RLR = Reload; }
public:
    void Reload() { IWDG->KR = 0xAAAA; }
    void Enable() { IWDG->KR = 0xCCCC; }
    void SetTimeout(uint32_t ms) {
        EnableAccess();
        SetPrescaler(iwdgPre256);
        uint32_t Count = (ms * (LSI_FREQ_HZ/1000)) / 256;
        TRIM_VALUE(Count, 0xFFF);
        SetReload(Count);
        Reload();
    }
    bool ResetOccured() {
        if(RCC->CSR & RCC_CSR_IWDGRSTF) {
            RCC->CSR |= RCC_CSR_RMVF;   // Clear flags
            return true;
        }
        else return false;
    }
};


#if 1 // ============================== Timers =================================
enum TmrTrigInput_t {tiITR0=0x00, tiITR1=0x10, tiITR2=0x20, tiITR3=0x30, tiTIED=0x40, tiTI1FP1=0x50, tiTI2FP2=0x60, tiETRF=0x70};
enum TmrMasterMode_t {mmReset=0x00, mmEnable=0x10, mmUpdate=0x20, mmComparePulse=0x30, mmCompare1=0x40, mmCompare2=0x50, mmCompare3=0x60, mmCompare4=0x70};
enum TmrSlaveMode_t {smDisable=0, smEncoder1=1, smEncoder2=2, smEncoder3=3, smReset=4, smGated=5, smTrigger=6, smExternal=7};
enum Inverted_t {invNotInverted, invInverted};
enum ExtTrigPol_t {etpInverted=0x8000, etpNotInverted=0x0000};
enum ExtTrigPsc_t {etpOff=0x0000, etpDiv2=0x1000, etpDiv4=0x2000, etpDiv8=0x30000};

class Timer_t {
private:
    TIM_TypeDef* ITmr;
    uint32_t *PClk;
public:
    volatile uint16_t *PCCR;    // Made public to allow DMA
    // Common
    void Init(TIM_TypeDef* Tmr);
    void Deinit();
    void Enable()  { ITmr->CR1 |=  TIM_CR1_CEN; }
    void Disable() { ITmr->CR1 &= ~TIM_CR1_CEN; }
    void SetTopValue(uint16_t Value) { ITmr->ARR = Value; }
    void SetCounter(uint16_t Value) { ITmr->CNT = Value; }
    uint16_t GetCounter() { return ITmr->CNT; }
    uint16_t GetTopValue() { return ITmr->ARR; }
    void SetPrescaler(uint16_t Value) { ITmr->PSC = Value; }
    void SetUpdateFreq(uint32_t FreqHz) {
        uint32_t ClkCnt = *PClk / (ITmr->PSC + 1);
        SetTopValue((ClkCnt / FreqHz) - 1);
    }
    void SetCounterFreq(uint32_t CounterFreqHz) { ITmr->PSC = (*PClk / CounterFreqHz) - 1; }
    // Master/Slave
    void SetTriggerInput(TmrTrigInput_t TrgInput) {
        uint16_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_TS;   // Clear bits
        tmp |= (uint16_t)TrgInput;
        ITmr->SMCR = tmp;
    }
    void SelectMasterMode(TmrMasterMode_t MasterMode) {
        uint16_t tmp = ITmr->CR2;
        tmp &= ~TIM_CR2_MMS;
        tmp |= (uint16_t)MasterMode;
        ITmr->CR2 = tmp;
    }
    void SelectSlaveMode(TmrSlaveMode_t SlaveMode) {
        uint16_t tmp = ITmr->SMCR;
        tmp &= ~TIM_SMCR_SMS;
        tmp |= (uint16_t)SlaveMode;
        ITmr->SMCR = tmp;
    }
    void EnableExternalClk(ExtTrigPol_t ExtTrigPol = etpNotInverted, ExtTrigPsc_t ExtTrigPsc = etpOff) { ITmr->SMCR = (uint16_t)ExtTrigPol | (uint16_t)ExtTrigPsc | TIM_SMCR_ECE; }
    // DMA, Irq, Evt
    void EnableDmaOnTrigger() { ITmr->DIER |= TIM_DIER_TDE; }
    void GenerateUpdateEvt()  { ITmr->EGR = TIM_EGR_UG; }
    void EnableIrqOnUpdate()  { ITmr->DIER |= TIM_DIER_UIE; }
    void ClearIrqBits()       { ITmr->SR = 0; }
    // PWM
    void InitPwm(GPIO_TypeDef *GPIO, uint16_t N, uint8_t Chnl, Inverted_t Inverted);
    void SetPwm(uint16_t Value) { *PCCR = Value; }
};
#endif


// ============================== UART command =================================
#define DBG_UART_ENABLED

#ifdef DBG_UART_ENABLED
#define UART_TXBUF_SIZE     1024
#define UART_TX_DMA         STM32_DMA1_STREAM4

class DbgUart_t {
private:
    uint8_t TXBuf[UART_TXBUF_SIZE];
    uint8_t *PWrite, *PRead;
    uint32_t IFullSlotsCount, ITransSize;
    bool IDmaIsIdle;
public:
    void Printf(const char *S, ...);
//    void FlushTx() { while(!IDmaIsIdle); }  // wait DMA
    void Init(uint32_t ABaudrate);
    void IRQDmaTxHandler();

    void IPutChar(char c);
};

extern DbgUart_t Uart;
#endif

#endif /* KL_LIB_F100_H_ */
