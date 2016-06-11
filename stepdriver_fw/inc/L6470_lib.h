/*
 * L6470_lib.h
 *
 *  Created on: 20 дек. 2013 г.
 *      Author: r.leonov
 */

#ifndef L6470_LIB_H_
#define L6470_LIB_H_

// RJ
#define STATUS_OFFS_HIZ                     0
#define STATUS_OFFS_BUSY                    1
#define STATUS_OFFS_SW_F                    2
#define STATUS_OFFS_SW_EVN                  3
#define STATUS_OFFS_DIR                     4
#define STATUS_OFFS_MOTOR_STATUS            5
#define STATUS_OFFS_NOTPERF_CMD             7
#define STATUS_OFFS_WRONG_CMD               8
#define STATUS_OFFS_UVLO                    9
#define STATUS_OFFS_TH_WRN                  10
#define STATUS_OFFS_TH_SD                   11
#define STATUS_OFFS_OCD                     12
#define STATUS_OFFS_STEP_LOSS_A             13
#define STATUS_OFFS_STEP_LOSS_B             14
#define STATUS_OFFS_SCK_MOD                 15

#define STATUS_FLAG_HIZ                     (1 << STATUS_OFFS_HIZ)
// FLAG IDLE: Here low - busy, high - idle. p.55 L6470 user manual.
#define STATUS_FLAG_IDLE                    (1 << STATUS_OFFS_BUSY)
#define STATUS_FLAG_SW_F                    (1 << STATUS_OFFS_SW_F)
#define STATUS_FLAG_SW_EVN                  (1 << STATUS_OFFS_SW_EVN)
#define STATUS_FLAG_DIR                     (1 << STATUS_OFFS_DIR)
#define STATUS_FLAG_NOTPERF_CMD             (1 << STATUS_OFFS_NOTPERF_CMD)
#define STATUS_FLAG_WRONG_CMD               (1 << STATUS_OFFS_WRONG_CMD)
#define STATUS_FLAG_UVLO                    (1 << STATUS_OFFS_UVLO)
#define STATUS_FLAG_TH_WRN                  (1 << STATUS_OFFS_TH_WRN)
#define STATUS_FLAG_TH_SD                   (1 << STATUS_OFFS_TH_SD)
#define STATUS_FLAG_OCD                     (1 << STATUS_OFFS_OCD)
#define STATUS_FLAG_STEP_LOSS_A             (1 << STATUS_OFFS_STEP_LOSS_A)
#define STATUS_FLAG_STEP_LOSS_B             (1 << STATUS_OFFS_STEP_LOSS_B)
#define STATUS_FLAG_SCK_MOD                 (1 << STATUS_OFFS_SCK_MOD)

// Addr Register
#define L6470_ADDR_ABS_POS                  0x01
#define L6470_ADDR_EL_POS                   0x02
#define L6470_ADDR_MARK                     0x03
#define L6470_ADDR_SPEED                    0x04
#define L6470_ADDR_ACC                      0x05
#define L6470_ADDR_DEC                      0x06
#define L6470_ADDR_MAX_SPEED                0x07
#define L6470_ADDR_MIN_SPEED                0x08
#define L6470_ADDR_FS_SPD                   0x15
#define L6470_ADDR_KVAL_HOLD                0x09
#define L6470_ADDR_KVAL_RUN                 0x0A
#define L6470_ADDR_KVAL_ACC                 0x0B
#define L6470_ADDR_KVAL_DEC                 0x0C
#define L6470_ADDR_INT_SPEED                0x0D
#define L6470_ADDR_ST_SLP                   0x0E
#define L6470_ADDR_FN_SLP_ACC               0x0F
#define L6470_ADDR_FN_SLP_DEC               0x10
#define L6470_ADDR_K_THERM                  0x11
#define L6470_ADDR_ADC_OUT                  0x12
#define L6470_ADDR_OCD_TH                   0x13
#define L6470_ADDR_STALL_TH                 0x14
#define L6470_ADDR_STEP_MODE                0x16
#define L6470_ADDR_ALARM_EN                 0x17
#define L6470_ADDR_CONFIG                   0x18
#define L6470_ADDR_STATUS                   0x19

#define L6470_CMD_SET_PARAM                 0x00
#define L6470_CMD_GET_PARAM                 0x20
#define L6470_CMD_MOVE			            0x40
#define L6470_CMD_GOTO			            0x60
#define L6470_CMD_GOTODIR			        0x68
#define L6470_CMD_GOUNTIL			        0x82
#define L6470_CMD_RELEASE			        0x92
#define L6470_CMD_GO_HOME			        0x70
#define L6470_CMD_GO_MARK			        0x78
#define L6470_CMD_RESET_POS		            0xD8
#define L6470_CMD_SOFT_HiZ		            0xA0
#define L6470_CMD_HARD_HiZ		            0xA8
#define L6470_CMD_RESET_DEVICE              0xC0
#define L6470_CMD_RUN                       0x50
#define L6470_CMD_STEP_CLOCK                0x58
#define L6470_CMD_SOFT_STOP                 0xB0
#define L6470_CMD_HARD_STOP                 0xB8
#define L6470_CMD_GET_STATUS		        0xD0

#endif /* L6470_LIB_H_ */
