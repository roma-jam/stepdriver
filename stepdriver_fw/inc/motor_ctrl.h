/*
 * motor_ctrl.h
 *
 *  Created on: 19 дек. 2013 г.
 *      Author: r.leonov
 */

#ifndef MOTOR_CTRL_H_
#define MOTOR_CTRL_H_

#include "kl_lib_f100.h"
#include "L6470_lib.h"
#include "spi_rj.h"
#include "string.h"

#define Rslt_t              uint8_t
#define DRIVER_INIT_TIMEOUT 20


#define CMD_BUF_SZ          99
#define ACK_BUF_SZ          7
#define ACK_BUF_ERR_SZ      3

enum MotorState_t {
	msIdle, msInit, msOff, msReset, msSleep, msTimeLapse, msGoHome
};

struct Params_t {
    union {
        uint8_t ParamsBuf[37];
        struct {
            uint8_t  dir;
            uint32_t curr_pos;
            uint32_t el_pos;
            uint32_t mark_pos;
            uint32_t speed;
            uint32_t acc;
            uint32_t dec;
            uint32_t max_speed;
            uint32_t min_speed;
            uint32_t adc;
            uint32_t status;
        } __attribute__ ((__packed__));
    };
} __attribute__ ((__packed__));


class Motor_t {
private:
    uint8_t id;
    uint8_t TxBuf[4], RxBuf[4];
    uint8_t *PTxBuf, *PRxBuf;
    bool isPoweredOn;
public:
    MotorState_t State, NewState;
    Params_t Prm;
    void PoweredOn() { isPoweredOn = true; }
    bool isPowered() { return isPoweredOn; }
    void Init(uint8_t AssignId);

    void SetState(MotorState_t AState) { NewState = AState; }
    void DoInit()   { SetState(msInit);  }
    void DoSleep()  { SetState(msSleep); }
    void DoIdle()   { SetState(msIdle);  }
    void DoReset()  { SetState(msReset); }
    void DoGoHome() { SetState(msGoHome);}
    void UpdatePrm();
    uint32_t GetPosition() {
        uint32_t Pos;
        GetParams(ADDR_ABS_POS, &Pos);
        return Pos;
    }
    uint8_t NOP();
    void SetParamBuf(uint8_t Addr, uint8_t *PBuf, uint8_t ALength);
    void SetParam(uint8_t Addr, uint32_t Value);
    void GetParams(uint8_t Addr, uint32_t* PValue);
    void Run(uint8_t Dir, uint32_t Speed);
    void StepClock(uint8_t Dir);
    void Move(uint8_t Dir, uint32_t Step);
    void GoTo(uint32_t Position);
    void GoTo_Dir(uint8_t Dir, uint32_t Position);
    void GoUntil(uint8_t Act, uint8_t Dir, uint32_t Speed);
    void ReleaseSW(uint8_t Act, uint8_t Dir);
    void GoHome();
    void GoMark();
    void ResetPos();
    void ResetDevice();
    void SoftStop();
    void HardStop();
    void SoftHiZ();
    void HardHiZ();
    void GetStatus(uint32_t *PValue);
};

struct CmdBuf_t {
    union {
        struct {
            uint8_t MtrID;
            uint8_t ID;
            uint8_t Data[CMD_BUF_SZ-2];
        };
        uint8_t Buf[CMD_BUF_SZ];
    };
};

struct AckBuf_t {
    union {
        struct {
            uint8_t MtrID;
            uint8_t CmdID;
            uint8_t Len;
            uint8_t Data[ACK_BUF_SZ-2];
        };
        uint8_t Buf[ACK_BUF_SZ];
    };
};


class Driver_t {
private:
    uint8_t Cmd[CMD_BUF_SZ];
    bool _IsInit;
//    CmdBuf_t Cmd;
    AckBuf_t Ack;
    Thread *PThread;
public:
    uint8_t *PCmdBuf, *PAckBuf;
    uint8_t CmdLength;
    Motor_t Motor[SPI_SLAVE_CNT];
    uint8_t NumberOfMotors;
    void PutToBuf(uint8_t AByte) { if(PCmdBuf >= (Cmd + CMD_BUF_SZ)) PCmdBuf = Cmd; *PCmdBuf++ = AByte; CmdLength++; }
    Rslt_t Init();
    bool IsInit() { return _IsInit; }
    void CmdHandle();
    void Task();
};

extern Driver_t Driver;
//class Motor_t {
//private:
//    uint8_t TxBuf[4], RxBuf[4];
//    uint8_t *PTxBuf, *PRxBuf;
//    Thread *PThread;
//    MotorState_t State, NewState;
//    uint32_t Step;
//public:
//
//    uint8_t *PBuf, *PIByte;
//    Params_t prm;
//    uint8_t number;
//    void Init();
//    void Task();
//    void SetState(MotorState_t AState) { NewState = AState; }
//
//    uint8_t NOP(uint8_t id);
//    void SetParam(uint8_t id, uint8_t Addr, uint32_t Value);
//    void GetParams(uint8_t id, uint8_t Addr, uint32_t* PValue);
//    void Run(uint8_t id, uint8_t Dir, uint32_t Speed);
//    void StepClock(uint8_t id, uint8_t Dir);
//    void Move(uint8_t id, uint8_t Dir, uint32_t Step);
//    void GoTo(uint8_t id, uint32_t Position);
//    void GoTo_Dir(uint8_t id, uint8_t Dir, uint32_t Position);
//    void GoUntil(uint8_t id, uint8_t Act, uint8_t Dir, uint32_t Speed);
//    void ReleaseSW(uint8_t id, uint8_t Act, uint8_t Dir);
//    void GoHome(uint8_t id);
//    void GoMark(uint8_t id);
//    void ResetPos(uint8_t id);
//    void ResetDevice(uint8_t id);
//    void SoftStop(uint8_t id);
//    void HardStop(uint8_t id);
//    void SoftHiZ(uint8_t id);
//    void HardHiZ(uint8_t id);
//    void GetStatus(uint8_t id, uint32_t *PValue);
//    // Irq
//    void IrqSpiHandler();
//};


#endif /* MOTOR_CTRL_H_ */
