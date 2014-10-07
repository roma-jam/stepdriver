/*
 * cmd_list.h
 *
 *  Created on: 08 мая 2014 г.
 *      Author: r.leonov
 */

#ifndef CMD_LIST_H_
#define CMD_LIST_H_


#define DEFAULT_ID              0

#define VCP_PING                "#Ping"
#define VCP_DRIVER_INIT         "#DriverInit"

#define VCP_SET_PARAM           "#SetParam"
#define VCP_GET_PARAM           "#GetParam"
#define VCP_MOVE                "#Move"
#define VCP_GOTO                "#GoTo"
#define VCP_GOTODIR             "#GoToDir"
#define VCP_GOUNTIL             "#GoUntil"
#define VCP_RELEASE             "#ReleaseSW"
#define VCP_GO_HOME             "#GoHome"
#define VCP_GO_MARK             "#GoMark"
#define VCP_RESET_POS           "#ResetPos"
#define VCP_SOFT_HiZ            "#SoftHiZ"
#define VCP_HARD_HiZ            "#HardHiZ"
#define VCP_RESET_DEVICE        "#ResetDevice"
#define VCP_RUN                 "#Run"
#define VCP_STOP                "#Stop"
#define VCP_STEP_CLOCK          "#StepClock"
#define VCP_SOFT_STOP           "#SoftStop"
#define VCP_HARD_STOP           "#HardStop"
#define VCP_GET_STATUS          "#GetStatus"
#define VCP_UPDATE_PARAM        "#UpdateParam"

#define VCP_RPL_OK              0
#define VCP_RPL_FAILURE         1
#define VCP_RPL_CMD_ERROR       2
#define VCP_RPL_CMD_UNKNOWN     3

#endif /* CMD_LIST_H_ */
