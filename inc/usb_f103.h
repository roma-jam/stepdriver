/*
 * usb_f103.h
 *
 *  Created on: 29 но€б. 2013 г.
 *      Author: r.leonov
 */

#ifndef USB_F103_H_
#define USB_F103_H_

#include "kl_lib_f100.h"
#include "stm32_usb.h"
#include "descriptors.h"
#include "buf_sizes.h"

// USB PINS
#define USB_GPIO        GPIOA
#define USB_PULLUP      15
#define USB_DM
#define USB_DM

enum EpState_t {esIdle, esSetup, esInData, esOutData, esInStatus, esOutStatus, esError};

// Functional type for unhandled ctrl pkt
typedef EpState_t (*ftCtrlPkt)(uint8_t **PPtr, uint32_t *PLen);

struct UsbSetupReq_t {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    union {
        uint16_t wValue;
        struct {
            uint8_t Indx;
            uint8_t Type;
        };
    };
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__ ((__packed__));

struct UsbEvents_t {
    ftVoidVoid OnReady;
    ftVoidVoid OnWakeup;
    ftVoidVoid OnSuspend;
    ftCtrlPkt  OnCtrlPkt;
    ftVoidVoid OnTransactionEnd[EP_CNT];
};

class Ep_t {
private:
    uint8_t Indx;
    EpState_t State;
    Thread *PThread;
    uint8_t *PtrIn, *PtrOut;
    uint32_t LengthIn, LengthOut;
    OutputQueue *PInQueue;
    bool TransmitFinalZeroPkt;
    void Init(const EpCfg_t *PCfg);
    void FillInBuf();
    void SetStallIn()   { EPR_SET_STAT_TX(Indx, EPR_STAT_TX_STALL); }
    void SetStallOut()  { EPR_SET_STAT_RX(Indx, EPR_STAT_RX_STALL); }
    bool IsStalledIn()  { return (STM32_USB->EPR[Indx] & EPR_STAT_TX_MASK) == EPR_STAT_TX_STALL; }
    bool IsStalledOut() { return (STM32_USB->EPR[Indx] & EPR_STAT_RX_MASK) == EPR_STAT_RX_STALL; }
    void StartInTransaction()  { EPR_SET_STAT_TX(Indx, EPR_STAT_TX_VALID); }
    void TransmitDataChunk() { FillInBuf(); StartInTransaction(); }
    void TransmitZeroPkt();
    void ReceiveZeroPkt();
    uint16_t GetRxDataLength();
    void FlushRx(uint16_t Len);
    inline void ResumeWaitingThd(msg_t ReadyMsg);
public:
    InputQueue *POutQueue;  // Queue
    void StartOutTransaction() { EPR_SET_STAT_RX(Indx, EPR_STAT_RX_VALID); }
    void WriteFromQueue(OutputQueue *PQ);
    void ReadToBuf(uint8_t *PDstBuf, uint16_t Len);
    void ReadToQueue(uint16_t Len);
    friend class Usb_t;
};

enum UsbState_t {usDisconnected, usConnected, usConfigured};

class Usb_t {
private:
    Ep_t Ep[EP_CNT];
    void ClockDividerUpdate();
    uint16_t PktMemNext;
    void PktMemReset();
    uint16_t PktMemAlloc(uint16_t Sz);
    void IReset();
    void ICtrHandlerIN(uint16_t EpID);
    void ICtrHandlerOUT(uint16_t EpID, uint16_t Epr);
    void SetupPktHandler();
    EpState_t DefaultReqHandler(uint8_t **PPtr, uint32_t *PLen);
    uint8_t Configuration;
public:
    bool IsReady;
    UsbState_t State;
    UsbEvents_t Events;
    VirtualTimer IWupSuspTmr;
    union {
        uint8_t Ep0OutBuf[EP0_SZ];
        UsbSetupReq_t SetupReq;
    };
    void Init();
    void Connect()      { PinSet(USB_GPIO, USB_PULLUP);   State = usConnected;    };
    void Disconnect()   { PinClear(USB_GPIO, USB_PULLUP); State = usDisconnected; };

    Ep_t *PEpBulkOut, *PEpBulkIn, *PEpInterrupt;
    void IIrqHandler();
    friend class Ep_t;
};

extern Usb_t Usb;

#if 1 // =========================== Constants =================================
#define USB_REQ_GET_STATUS                  0
#define USB_REQ_CLEAR_FEATURE               1
#define USB_REQ_SET_FEATURE                 3
#define USB_REQ_SET_ADDRESS                 5
#define USB_REQ_GET_DESCRIPTOR              6
#define USB_REQ_SET_DESCRIPTOR              7
#define USB_REQ_GET_CONFIGURATION           8
#define USB_REQ_SET_CONFIGURATION           9
#define USB_REQ_GET_INTERFACE               10
#define USB_REQ_SET_INTERFACE               11
#define USB_REQ_SYNCH_FRAME                 12

#define USB_FEATURE_ENDPOINT_HALT           0
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP    1
#define USB_FEATURE_TEST_MODE               2

// Setup request type (bmRequestType)
#define USB_REQTYPE_DEV2HOST                (1<<7)
#define USB_REQTYPE_HOST2DEV                (0<<7)

#define USB_REQTYPE_TYPEMASK                0x60
#define USB_REQTYPE_STANDARD                (0<<5)
#define USB_REQTYPE_CLASS                   (1<<5)
#define USB_REQTYPE_VENDOR                  (2<<5)
#define USB_REQTYPE_RECIPIENT_MASK          0x1F
#define USB_REQTYPE_RECIPIENT_DEVICE        0x00
#define USB_REQTYPE_RECIPIENT_INTERFACE     0x01
#define USB_REQTYPE_RECIPIENT_ENDPOINT      0x02
#define USB_REQTYPE_RECIPIENT_OTHER         0x03
#endif


#endif /* USB_F103_H_ */
