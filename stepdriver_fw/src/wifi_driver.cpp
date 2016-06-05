/*
 * wifi_driver.cpp
 *
 *  Created on: 20 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "wifi_driver.h"
#include "http_server.h"

wifi_driver_t WiFi;

// Wrapper for IRQ
extern "C"
{
void WiFiTxIrq(void *p, uint32_t flags) { WiFi.IRQ_TxHandler(); }
}

void wifi_driver_t::Init()
{
    PinSetupOut(WIFI_PWR_GPIO, WIFI_PWR_PIN, omPushPull, ps50MHz);
    PowerOff();

    WiFiDmaIsIdle = true;
    SlotsFilled = 0;
    PinSetupAlterFuncOutput(WIFI_GPIO, WIFI_TX_PIN, omPushPull);
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
    dmaStreamAllocate     (WIFI_TX_DMA, IRQ_PRIO_HIGH, WiFiTxIrq, NULL);
    dmaStreamSetPeripheral(WIFI_TX_DMA, &WIFI_UART->DR);
    dmaStreamSetMode      (WIFI_TX_DMA,  WIFI_DMA_MODE);
    WIFI_UART->CR1 |= USART_CR1_UE;        // Enable USART
    Uart.Printf("WiFi Init\r");
}

#if 1 // ======================= TX Part ======================================
void wifi_driver_t::CmdSend(uint8_t *PBuf, uint32_t Length)
{
#if (APP_WIFI_DEBUG)
    Uart.Printf("s:%u\r", Length);
#endif
	uint32_t LenToSend = Length;
	uint8_t *Ptr = PBuf;
	if(LenToSend > HTTP_REQUEST_SIZE)
	{
		do
		{
			dmaStreamSetMemory0(WIFI_TX_DMA, Ptr);
			dmaStreamSetTransactionSize(WIFI_TX_DMA, HTTP_REQUEST_SIZE);
			dmaStreamSetMode(WIFI_TX_DMA, WIFI_DMA_MODE);
			dmaStreamEnable(WIFI_TX_DMA);
			IWaitTxEnd();
			LenToSend -= HTTP_REQUEST_SIZE;
			Ptr += HTTP_REQUEST_SIZE;
			chThdSleepMilliseconds(351);
		} while(LenToSend > HTTP_REQUEST_SIZE);
	}
	dmaStreamSetMemory0(WIFI_TX_DMA, Ptr);
	dmaStreamSetTransactionSize(WIFI_TX_DMA, LenToSend);
	dmaStreamSetMode(WIFI_TX_DMA, WIFI_DMA_MODE);
	dmaStreamEnable(WIFI_TX_DMA);
	IWaitTxEnd();
}

//void wifi_driver_t::CmdSendDma() {
//    uint32_t PartSz = (HostBuf + UART_TXBUF_SIZE) - pRead; // Cnt from PRead to end of buf
//    ITransmitSize = MIN(SlotsFilled, PartSz);
//    if(ITransmitSize != 0) {
//        WiFiDmaIsIdle = false;
//        dmaStreamSetMemory0(WIFI_TX_DMA, pRead);
//        dmaStreamSetTransactionSize(WIFI_TX_DMA, ITransmitSize);
//        dmaStreamSetMode(WIFI_TX_DMA, WIFI_DMA_MODE);
//        dmaStreamEnable(WIFI_TX_DMA);
//    }
//}

void wifi_driver_t::IRQ_TxHandler()
{
    ITxEnd();
    dmaStreamDisable(WIFI_TX_DMA);    // Registers may be changed only when stream is disabled

//    SlotsFilled -= ITransmitSize;
//    pRead += ITransmitSize;
//    if(pRead >= (HostBuf + UART_TXBUF_SIZE)) pRead = HostBuf; // Circulate pointer
//
//    if(SlotsFilled == 0) WiFiDmaIsIdle = true; // Nothing left to send
//    else CmdSendDma();
}
#endif // =====================================================================


#if 1 // =========================== RX Part ==================================
void wifi_driver_t::IHandleByte()
{
    uint8_t Byte = WIFI_RX_BYTE;
    RplBuf.Write(Byte);
    if(Byte == WIFI_STR_LF) HttpServer.Wakeup();
}

void wifi_driver_t::IRQ_RxHandler()
{
    IHandleByte();
    WIFI_UART->SR &= ~USART_SR_RXNE;
}

extern "C"
{
CH_IRQ_HANDLER(WIFI_IRQ_Handler)
        {
    CH_IRQ_PROLOGUE();
    chSysLockFromIsr();
    WiFi.IRQ_RxHandler();
    chSysUnlockFromIsr();
    CH_IRQ_EPILOGUE();
}
}
#endif // ======================================================================
