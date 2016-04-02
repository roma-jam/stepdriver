/*
 * cmd_list.h
 *
 *  Created on: 08 ��� 2014 �.
 *      Author: r.leonov
 */

#ifndef CMD_LIST_H_
#define CMD_LIST_H_


#define DEFAULT_ID              0

#define VCP_PING                "#Ping"
#define VCP_DRIVER_INIT         "#DriverInit"

#define VCP_SET_PARAM_STRING    "#SetParam"
#define VCP_GET_PARAM_STRING           "#GetParam"
#define VCP_MOVE_STRING                "#Move"
#define VCP_GOTO_STRING                "#GoTo"
#define VCP_GOTODIR_STRING             "#GoToDir"
#define VCP_GOUNTIL_STRING             "#GoUntil"
#define VCP_RELEASE_STRING             "#ReleaseSW"
#define VCP_GO_HOME_STRING             "#GoHome"
#define VCP_GO_MARK_STRING             "#GoMark"
#define VCP_RESET_POS_STRING           "#ResetPos"
#define VCP_SOFT_HiZ_STRING            "#SoftHiZ"
#define VCP_HARD_HiZ_STRING            "#HardHiZ"
#define VCP_RESET_DEVICE_STRING        "#ResetDevice"
#define VCP_RUN_STRING                 "#Run"
#define VCP_STOP_STRING                "#Stop"
#define VCP_STEP_CLOCK_STRING          "#StepClock"
#define VCP_SOFT_STOP_STRING           "#SoftStop"
#define VCP_HARD_STOP_STRING           "#HardStop"
#define VCP_GET_STATUS_STRING          "#GetStatus"
#define VCP_UPDATE_PARAM_STRING        "#UpdateParam"

// Logic command
#define VCP_GLIDETRACK_SIZE_SET_STRING "#SetSize"
#define VCP_TIMELAPSE_PARAM_STRING     "#TLParam"
#define VCP_TIMELAPSE_START_STRING     "#TLStart"
#define VCP_TIMELAPSE_STOP_STRING      "#TLStop"

enum cmdType {
    Err = 0,
    Ping,
    DriverInit,
    SetParam,
    GetParam,
    Move,
    cmdGoTo,
    cmdGoToDir,
    GoUntil,
    ReleaseSW,
    GoHome,
    GoMark,
    ResetPos,
    SoftHiZ,
    HardHiZ,
    ResetDevice,
    Run,
    Stop,
    StepClock,
    SoftStop,
    HardStop,
    GetStaus,
    UpdateParam,
    SetSize,
    TLParam,
    TLStart,
    TLStop
};


#define VCP_RPL_OK              0
#define VCP_RPL_FAILURE         1
#define VCP_RPL_CMD_ERROR       2
#define VCP_RPL_CMD_UNKNOWN     3
#define VCP_RPL_INIT_TIMEOUT    4
#define VCP_RPL_NOT_INIT        5

#endif /* CMD_LIST_H_ */
