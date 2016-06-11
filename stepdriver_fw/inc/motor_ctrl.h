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
#include "cmd_list.h"
#include "config.h"

#define Rslt_t              uint8_t

#define CMD_BUF_SZ          99
#define ACK_BUF_SZ          7
#define ACK_BUF_ERR_SZ      3

enum MotorState_t {
	msIdle = 0,
	msOff,
	msPowerUp,
	msBusy,
	msCalibrate
};

#pragma pack(push, 1)
struct params_t {
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
};
struct driver_config_t {
    uint32_t magic;
    uint32_t acc;
    uint32_t dec;
    uint32_t max_speed;
    uint32_t min_speed;
    uint8_t step_mode;
};
#pragma pack(pop)


class Motor_t {
private:
    bool isPoweredOn;
public:
    MotorState_t State, NewState;
    params_t param;
    uint8_t forward;
    uint8_t backward;

    // Power Functions
    void PoweredOn() { isPoweredOn = true; }
    bool isPowered() { return isPoweredOn; }
    void Init();

    // State Fuctions
    void SetState(MotorState_t AState) { NewState = AState; }

    // Low Level config functions
    void UpdatePrm();

    uint32_t GetParam(uint8_t Addr);
    uint32_t GetStatus();
    uint32_t GetPosition() { return GetParam(L6470_ADDR_ABS_POS); }

    uint8_t NOP();
    void SetParam(uint8_t Addr, uint32_t Value);
    void Run(uint8_t Dir, uint32_t Speed);
    void Stop()  { HardStop(); }
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
    AckBuf_t Ack;

    Thread *PThread;
public:
    uint8_t *PCmdBuf, *PAckBuf;
    uint8_t CmdLength;
    uint32_t MaxPos;

    void PutToBuf(uint8_t AByte)
    {
        if(PCmdBuf >= (Cmd + CMD_BUF_SZ))
            PCmdBuf = Cmd;
        *PCmdBuf++ = AByte;
        CmdLength++;
    }

    driver_config_t config;
    Motor_t Motor;
    bool isInit() { return Motor.isPowered(); }

    cmdType get_cmd_type(char* S);
    void cmd_handle();

    Rslt_t Init();
    void Task();
    void StartCalibration();
    void BackwardEndStop();
    void ForwardEndStop();

    // EEPROM config functions
    Rslt_t write_config(driver_config_t *config);
    Rslt_t read_config(driver_config_t* config);
};

extern Driver_t Driver;

#endif /* MOTOR_CTRL_H_ */
