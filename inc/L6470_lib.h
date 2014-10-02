/*
 * L6470_lib.h
 *
 *  Created on: 20 дек. 2013 г.
 *      Author: r.leonov
 */

#ifndef L6470_LIB_H_
#define L6470_LIB_H_

// RJ

// Addr Register
#define ADDR_ABS_POS         0x01
#define ADDR_EL_POS          0x02
#define ADDR_MARK            0x03
#define ADDR_SPEED           0x04
#define ADDR_ACC             0x05
#define ADDR_DEC             0x06
#define ADDR_MAX_SPEED       0x07
#define ADDR_MIN_SPEED       0x08
#define ADDR_FS_SPD          0x15
#define ADDR_KVAL_HOLD       0x09
#define ADDR_KVAL_RUN        0x0A
#define ADDR_KVAL_ACC        0x0B
#define ADDR_KVAL_DEC        0x0C
#define ADDR_INT_SPEED       0x0D
#define ADDR_ST_SLP          0x0E
#define ADDR_FN_SLP_ACC      0x0F
#define ADDR_FN_SLP_DEC      0x10
#define ADDR_K_THERM         0x11
#define ADDR_ADC_OUT         0x12
#define ADDR_OCD_TH          0x13
#define ADDR_STALL_TH        0x14
#define ADDR_STEP_MODE       0x16
#define ADDR_ALARM_EN        0x17
#define ADDR_CONFIG          0x18
#define ADDR_STATUS          0x19



#define SET_PARAM           0x00
#define GET_PARAM           0x20
#define MOVE			    0x40
#define GOTO			    0x60
#define GOTODIR			    0x68
#define GOUNTIL			    0x82
#define RELEASE			    0x92
#define GO_HOME			    0x70
#define GO_MARK			    0x78
#define RESET_POS		    0xD8
#define SOFT_HiZ		    0xA0
#define HARD_HiZ		    0xA8
#define RESET_DEVICE        0xC0
#define RUN                 0x50
#define STEP_CLOCK          0x58
#define SOFT_STOP           0xB0
#define HARD_STOP           0xB8
#define GET_STATUS		    0xD0


#endif /* L6470_LIB_H_ */
