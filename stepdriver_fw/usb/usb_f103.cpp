/*
 * usb_f103.cpp
 *
 *  Created on: 29 но€б. 2013 г.
 *      Author: r.leonov
 */



#include "usb_f103.h"

#include "vcp.h"

struct BTableReg_t {
  volatile uint32_t TxAddr;     // TX buffer offset register
  volatile uint16_t TxCnt0;     // TX counter register 0
  volatile uint16_t TxCnt1;     // TX counter register 1
  volatile uint32_t RxAddr;     // RX buffer offset register
  volatile uint16_t RxCnt0;     // RX counter register 0
  volatile uint16_t RxCnt1;     // RX counter register 1
};

#define GET_BTB_REG(id)     ((BTableReg_t*)((uint32_t)STM32_USBRAM_BASE + \
                            (uint32_t)STM32_USB->BTABLE * 2 + \
                            (uint32_t)(id) * sizeof(BTableReg_t)))

#define BTB_ADDR2PTR(addr)  ((uint32_t *)((addr) * 2 + STM32_USBRAM_BASE))

const uint8_t cZero[2] = {0, 0};
const uint8_t cOne[2]  = {1, 0};
Usb_t Usb;

void Usb_t::Init() {
    IsReady = false;
    // Init Gpios
    PinSetupOut(USB_GPIO, USB_PULLUP, omPushPull, ps50MHz);
    // Init Eps' pointers
    PEpInterrupt = &Ep[1];
    PEpBulkOut = &Ep[2];    // Out endpoint
    PEpBulkIn  = &Ep[3];    // In endpoint
    for(uint8_t i=0; i<EP_CNT; i++) Ep[i].Indx = i;
    Disconnect();

    ClockDividerUpdate();
    rccEnableUSB(FALSE);
    STM32_USB->CNTR = CNTR_FRES;
    STM32_USB->ISTR = 0;
    nvicEnableVector(STM32_USB1_LP_NUMBER, CORTEX_PRIORITY_MASK(STM32_USB_USB1_LP_IRQ_PRIORITY));
    STM32_USB->CNTR = 0;
    IReset();
}

void Usb_t::IReset() {
    IsReady = false;
    if(chVTIsArmedI(&IWupSuspTmr)) chVTResetI(&IWupSuspTmr);    // Reset timer if enabled

    PktMemReset();
    STM32_USB->BTABLE = 0;          // Clear table address
    STM32_USB->DADDR = DADDR_EF;    // Enable USB device with zero Addr
    STM32_USB->CNTR =   CNTR_RESETM |
#ifdef SUP_WUP_ENABLE
                        CNTR_SUSPM  |
                        CNTR_WKUPM  |
#endif
                        CNTR_CTRM;  // Enable Reset and ControlTransfer irqs
    // Init EPs
    for(uint8_t i=0; i<EP_CNT; i++) Ep[i].Init(&EpCfg[i]);
    Ep[0].State = esSetup;
}

void Usb_t::PktMemReset() {
    PktMemNext = 4 * sizeof(BTableReg_t);
}
uint16_t Usb_t::PktMemAlloc(uint16_t Sz) {
    uint16_t ret = PktMemNext;
    PktMemNext += Sz;
    return ret;
}

void Usb_t::IIrqHandler() {
    uint32_t istr = STM32_USB->ISTR;
//    Uart.Printf("\ri=%X\r", istr);
    if(istr & ISTR_RESET) {
//        Uart.Printf("Rst\r");
        IReset();
        STM32_USB->ISTR = ~ISTR_RESET;
    }
#ifdef SUP_WUP_ENABLE
    if(istr & ISTR_SUSP) {
        Uart.Printf("Sup\r");
        ISuspend();
        STM32_USB->ISTR = ~ISTR_SUSP;
    }
    if(istr & ISTR_WKUP) {
        Uart.Printf("Wup\r");
        IWakeup();
        STM32_USB->ISTR = ~ISTR_WKUP;
    }
#endif

    while(istr & ISTR_CTR) {
        uint16_t EpID = istr & ISTR_EP_ID_MASK;
//        Uart.Printf("Ctr\r");
        uint16_t epr = STM32_USB->EPR[EpID];
        if(epr & EPR_CTR_TX) ICtrHandlerIN(EpID);
        if(epr & EPR_CTR_RX) ICtrHandlerOUT(EpID, epr);
        istr = STM32_USB->ISTR;
    }
    STM32_USB->ISTR = 0;
}

// IN transaction
void Usb_t::ICtrHandlerIN(uint16_t EpID) {
//    Uart.Printf("in.ep%u\r", EpID);
    EPR_CLEAR_CTR_TX(EpID);
    Ep_t *ep = &Ep[EpID];
    if(EpID == 0) {
        switch(ep->State) {
            case esInData:
                if((ep->LengthIn != 0) or ep->TransmitFinalZeroPkt) {
                    ep->TransmitDataChunk(); // Transfer not completed
                }
                else {  // Buf len == 0
                    ep->State = esInStatus;
                    ep->ReceiveZeroPkt();
                }
                break;

            case esOutStatus:
                if(Events.OnTransactionEnd[0] != nullptr) Events.OnTransactionEnd[0]();
                Events.OnTransactionEnd[0] = nullptr;
                ep->State = esSetup;
                break;

            default:
                Uart.Printf("e\r");
                ep->State = esError;
                ep->SetStallIn();
                ep->SetStallOut();
                break;
        } // switch
    } // if(EpID == 0)
    else {
        if((ep->LengthIn != 0) or ep->TransmitFinalZeroPkt) {
            ep->TransmitDataChunk(); // Transfer not completed
        }
        else if(ep->PInQueue != nullptr) ep->WriteFromQueue(ep->PInQueue);
        else {  // Buf len == 0
            ep->State = esIdle;
            ep->ResumeWaitingThd(RDY_OK);
        }
    }
}

// OUT transaction
/* Beware: if Rx Length is read before setup pkt read out, SETUP bit may reset sometimes.
 * Workaround: read EPR first.
 */
void Usb_t::ICtrHandlerOUT(uint16_t EpID, uint16_t Epr) {
//    Uart.Printf("out.ep%u\r", EpID);
    EPR_CLEAR_CTR_RX(EpID);
    Ep_t *ep = &Ep[EpID];
    uint16_t Len = ep->GetRxDataLength();
    if(EpID == 0) {
        if(Epr & EPR_SETUP) {
            ep->State = esSetup; // In case of setup pkt, reset stage to Setup one
            ep->ReadToBuf(Ep0OutBuf, 8);
            SetupPktHandler();
        }
        else {
            if(Len > 0) {   // Otherwise, do nothing: this is zero pkt from host
                if(ep->PtrOut != nullptr) {
                    uint16_t CntToRead = MIN(Len, ep->LengthOut);
                    ep->ReadToBuf(ep->PtrOut, CntToRead);
                    ep->LengthOut -= CntToRead;
                    if(ep->LengthOut == 0) {    // Transfer completed
                        ep->PtrOut = nullptr;
                        ep->State = esOutStatus;
                        ep->TransmitZeroPkt();
                    }
                    else {
                        ep->PtrOut += CntToRead;
                        ep->StartOutTransaction();
                    }
                }
                else ep->FlushRx(Len);
            } // if Len>0
        } // if setup
    } // if(EpID == 0)
    else {
//        Uart.Printf("Len=%u\r", Len);
        if(ep->POutQueue != nullptr) ep->ReadToQueue(Len);
        else ep->FlushRx(Len);
        ep->StartOutTransaction();
        // WakeUp VcpThd
        if(Vcp.PThread != nullptr) {
            Vcp.BytesToRead = Len;
            chSysLockFromIsr();
            chSchReadyI(Vcp.PThread);
            chSysUnlockFromIsr();
        }
    }
}

void Ep_t::TransmitZeroPkt() {
    GET_BTB_REG(Indx)->TxCnt0 = 0;
    StartInTransaction();
}
void Ep_t::ReceiveZeroPkt() {
//    Uart.Printf("RxZero\r");
    StartOutTransaction();
}

void Usb_t::SetupPktHandler() {
    //Uart.Printf("Setup\r");
    Uart.Printf("%A\r", Ep0OutBuf, 8, ' ');
    // Try to handle request
    uint8_t *FPtr;
    uint32_t FLength;
    Ep[0].State = DefaultReqHandler(&FPtr, &FLength);
    // If standard request handler failed, try handle it in application
    if(Ep[0].State == esError) {
        if(Events.OnCtrlPkt != nullptr) Ep[0].State = Events.OnCtrlPkt(&FPtr, &FLength);
    }
    // Prepare to next transaction
    switch(Ep[0].State) {
        case esInData:
            Ep[0].PtrIn = FPtr;
            Ep[0].LengthIn = FLength;
            Ep[0].TransmitDataChunk();
            break;
        case esOutData:
            Ep[0].PtrOut = FPtr;
            Ep[0].LengthOut = FLength;
            Ep[0].StartOutTransaction();
            break;
        case esOutStatus:
            Ep[0].TransmitZeroPkt();
            break;
        default:
            Ep[0].SetStallIn();
            Ep[0].SetStallOut();
            break;
    } // switch
}

// Ep0 callbacks
static void SetAddress() {
    uint32_t Addr = Usb.SetupReq.wValue;
//    Uart.Printf("SetAddr %u\r", Addr);
    STM32_USB->DADDR = Addr | DADDR_EF;
}


EpState_t Usb_t::DefaultReqHandler(uint8_t **PPtr, uint32_t *PLen) {
    uint8_t Recipient = SetupReq.bmRequestType & USB_REQTYPE_RECIPIENT_MASK;
    uint8_t EpID;
    if(Recipient == USB_REQTYPE_RECIPIENT_DEVICE) {
        //Uart.Printf("Dev\r\n");
        switch(SetupReq.bRequest) {
            case USB_REQ_GET_STATUS:    // Just return the current status word
//                Uart.Printf("GetStatus\r");
                *PPtr = (uint8_t*)cZero;    // Remote wakeup = 0, selfpowered = 0
                *PLen = 2;
                return esInData;
                break;
            case USB_REQ_SET_ADDRESS:
//                Uart.Printf("SetAddr %u\r", SetupReq.wValue);
                *PLen = 0;
                Events.OnTransactionEnd[0] = SetAddress;
                return esOutStatus;
                break;
            case USB_REQ_GET_DESCRIPTOR:
//                Uart.Printf("GetDesc t=%u i=%u\r", SetupReq.Type, SetupReq.Indx);
                GetDescriptor(SetupReq.Type, SetupReq.Indx, PPtr, PLen);
                // Trim descriptor if needed, as host can request part of descriptor.
                TRIM_VALUE(*PLen, SetupReq.wLength);
//                Uart.Printf("DescLen=%u\r", PBuf->Len);
                if(*PLen != 0) return esInData;
                break;
            case USB_REQ_GET_CONFIGURATION:
//                Uart.Printf("GetCnf\r");
                *PPtr = &Configuration;
                *PLen = 1;
                return esInData;
                break;
            case USB_REQ_SET_CONFIGURATION:
//                Uart.Printf("SetCnf %u\r", SetupReq.wValue);
                Configuration = (uint8_t)(SetupReq.wValue & 0xFF);
                *PLen = 0;
                IsReady = true;
                if(Events.OnReady != nullptr) Events.OnReady();
//                if(PThread != nullptr) {
//                    chSysLockFromIsr();
//                    chEvtSignalI(PThread, EVTMASK_USB_READY);
//                    chSysUnlockFromIsr();
//                }
                return esOutStatus;
                break;
            default: break;
        } // switch
    }
//    else if(Recipient == USB_RTYPE_RECIPIENT_INTERFACE) {
//        if(SetupReq.bRequest == USB_REQ_GET_STATUS) {
//            Uart.Printf("InterfGetSta\r");
//            *Ptr = (uint8_t*)ZeroStatus;
//            *PLen = 2;
//            return OK;
//        }
//    }
    else if(Recipient == USB_REQTYPE_RECIPIENT_ENDPOINT) {
//        EP0_PRINT("Ep\r");
        switch(SetupReq.bRequest) {
            case USB_REQ_SYNCH_FRAME:
                *PPtr = (uint8_t*)cZero;
                *PLen = 2;
                return esInData;
                break;
            case USB_REQ_SET_FEATURE:
                // Only ENDPOINT_HALT is handled as feature
                if(SetupReq.wValue != 0) return esError;
                // Handle only non-control eps
                EpID = SetupReq.wIndex & 0x0F;
                if(EpID != 0) {
                    if(SetupReq.wIndex & 0x80) Ep[EpID].SetStallIn();
                    else Ep[EpID].SetStallOut();
                    Ep[EpID].ResumeWaitingThd(OK);
                }
                return esOutStatus;
            case USB_REQ_CLEAR_FEATURE:
                // Only ENDPOINT_HALT is handled as feature
                if(SetupReq.wValue != 0) return esError;
                // Handle only non-control eps
                EpID = SetupReq.wIndex & 0x0F;
                if(EpID != 0) {
                    // Clear stall
                    if(SetupReq.wIndex & 0x80) EPR_SET_STAT_TX(EpID, EPR_STAT_TX_NAK);
                    else EPR_SET_STAT_RX(EpID, EPR_STAT_RX_NAK);
                    Ep[EpID].ResumeWaitingThd(OK);
                }
                return esOutStatus;
                break;
            case USB_REQ_GET_STATUS:
                EpID = SetupReq.wIndex & 0x0F;
                if(SetupReq.wIndex & 0x80) *PPtr = Ep[EpID].IsStalledIn()?  (uint8_t*)cOne : (uint8_t*)cZero;
                else                       *PPtr = Ep[EpID].IsStalledOut()? (uint8_t*)cOne : (uint8_t*)cZero;
                *PLen = 2;
                return esInData;
                break;
        } // switch bRequest
    } // if Ep
    return esError;
}

extern "C" {
CH_IRQ_HANDLER(STM32_USB1_LP_HANDLER) {
    CH_IRQ_PROLOGUE();
    Usb.IIrqHandler();
    CH_IRQ_EPILOGUE();
}
} // extern C

// ================================= Ep ========================================
void Ep_t::Init(const EpCfg_t *PCfg) {
    // Process waiting thread if any
    ResumeWaitingThd(RDY_RESET);
    PInQueue = nullptr; // Reset queue if any
    LengthIn = 0;
    // Set EP type, NAK both directions
    uint16_t epr;
    epr = PCfg->Type;
    epr |= EPR_STAT_TX_NAK | EPR_STAT_RX_NAK;
    // EPxR register setup
    EPR_SET(Indx, epr | Indx);
    EPR_TOGGLE(Indx, epr);
    // ==== Endpoint size and address initialization ====
    uint16_t BlSz, NBlocks;
    if(PCfg->OutMaxsize <= 62) {
        BlSz = 0;
        NBlocks = PCfg->OutMaxsize / 2;
    }
    else {
        BlSz = 0x8000;
        NBlocks = (PCfg->OutMaxsize / 32) - 1;
    }
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    PReg->TxCnt0 = 0;
    PReg->RxCnt0 = BlSz | (NBlocks << 10);
    PReg->TxAddr = Usb.PktMemAlloc(PCfg->InMaxsize);
    PReg->RxAddr = Usb.PktMemAlloc(PCfg->OutMaxsize);
}

void Ep_t::ResumeWaitingThd(msg_t ReadyMsg) {
    if(PThread != NULL) {
        chSysLockFromIsr();
        if(PThread->p_state == THD_STATE_SUSPENDED) {
            PThread->p_u.rdymsg = ReadyMsg;
            chSchReadyI(PThread);
        }
        chSysUnlockFromIsr();
        PThread = NULL;
    }
}


// Inner
uint16_t Ep_t::GetRxDataLength() {
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    uint16_t Len = PReg->RxCnt0 & RXCOUNT_COUNT_MASK;
//    Uart.Printf("UL=%u\r", Len);
    return Len;
}

void Ep_t::ReadToBuf(uint8_t *PDstBuf, uint16_t Len) {
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    uint32_t *PSrc = BTB_ADDR2PTR(PReg->RxAddr);
//    Uart.Printf("%X\r", PSrc);
    Uart.Printf("%A\r", (uint8_t*)PSrc, 32, ' ');

    Len = (Len + 1) / 2;
    for(uint8_t i=0; i < Len; i++) {
        *(uint16_t*)PDstBuf = (uint16_t)*PSrc++;
        PDstBuf += 2;
    }
}

void Ep_t::FlushRx(uint16_t Len) {
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    uint32_t *PSrc = BTB_ADDR2PTR(PReg->RxAddr);
    Len = (Len + 1) / 2;
    for(uint8_t i=0; i < Len; i++) (void)*PSrc++;
}

void Ep_t::ReadToQueue(uint16_t Len) {
//    Uart.Printf("R2Q %u\r", Len);
    uint16_t w;
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    uint32_t *PSrc = BTB_ADDR2PTR(PReg->RxAddr);
    bool OddLength = Len & 0x01;
    Len = Len / 2;
    chSysLockFromIsr();
    while(Len--) {
        w = (uint16_t)*PSrc++;
        chIQPutI(POutQueue, (uint8_t)(w & 0xFF));
        chIQPutI(POutQueue, (uint8_t)(w >> 8));
    }
    // Last byte for odd lengths
    if(OddLength) chIQPutI(POutQueue, (uint8_t)(*PSrc & 0xFF));
    chSysUnlockFromIsr();
}

void Ep_t::WriteFromQueue(OutputQueue *PQ) {
    //if(PQ == nullptr) return;
    // Count of bytes to write to USB memory
    chSysLock();
    uint32_t n = chOQGetFullI(PQ);
    chSysUnlock();
    if(n == 0) return;
//    Uart.Printf("n=%u\r", n);
    if(n > EpCfg[Indx].InMaxsize) {
        n = EpCfg[Indx].InMaxsize;
        PInQueue = PQ;  // Remember q
    }
    else {
        PInQueue = nullptr; // Forget the queue
        TransmitFinalZeroPkt = (n == EpCfg[Indx].InMaxsize);
    }
    // Write bytes
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    PReg->TxCnt0 = n;
    uint32_t *p = USB_ADDR2PTR(PReg->TxAddr);
    uint16_t b1, b2;
    uint16_t w;
    n = (n + 1) / 2;
    chSysLock();
    while(n > 0) {
        b1 = chOQGetI(PQ);
        b2 = chOQGetI(PQ);
        w = (b2 << 8) | b1;
        *p++ = w;
        n--;
    }
    chSysUnlock();
    StartInTransaction();
}

// Fill USB memory with BufIn's data
void Ep_t::FillInBuf() {
    // Count of bytes to write to USB memory
    uint16_t n = (LengthIn > EpCfg[Indx].InMaxsize)? EpCfg[Indx].InMaxsize : LengthIn;
    // If last data chunk size equals Ep size, additional zero pkt must be transmitted
    TransmitFinalZeroPkt = (n == EpCfg[Indx].InMaxsize);
    LengthIn -= n;
    BTableReg_t *PReg = GET_BTB_REG(Indx);
    PReg->TxCnt0 = n;
    uint32_t *p = USB_ADDR2PTR(PReg->TxAddr);
    n = (n + 1) / 2;
    while(n > 0) {
        *p++ = *(uint16_t*)PtrIn;
        PtrIn += 2;
        n--;
    }
}


void Usb_t::ClockDividerUpdate() {
	switch (Clk.AHBFreqHz) {
		case 48000000:
			RCC->CFGR |= RCC_CFGR_USBPRE;
			break;
		case 72000000:
			RCC->CFGR &= ~RCC_CFGR_USBPRE;
			break;
		default:
			Uart.Printf("UsbClkErr\r");
			break;
	}
    // if 48 = return, if 72 - div by 1.5
}


