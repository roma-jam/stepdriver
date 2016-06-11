/*
 * config.h
 *
 *  Created on: 5 θώνÿ 2016 γ.
 *      Author: RomaJam
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_


// ============================ EEPROM & CONFIG ================================
#define EEPROM_READ_WRITE_TEST              0

#if (EEPROM_READ_WRITE_TEST)
#define EEPROM_TEST_BYTE                    0x5DADFACE
#define EEPROM_TEST_ADDR                    0x00
#endif

#define APP_EEPROM_CONFIG_ADDR              0
#define APP_EEPROM_MOTOR_GONFIG_ADDR        128

// ================================= USB =======================================
#define APP_VCP_DEBUG                       1
#define APP_VCP_DEBUG_IO                    0
#define APP_VCP_DEBUG_REQUEST               0

// ================================ SPI LOW LEVEL ==============================
#define APP_SPI_DEBUG_IO                    0


// ================================= ENDSTOPS ==================================
#define APP_ENDSTOP_FORWARD                 0x00
#define APP_ENDSTOP_BACKWARD                0x01

// ================================ MOTOR DRIVER ===============================
#define APP_MOTOR_DRIVER_CONFIG_MAGIC       0x4D6F746F
#define APP_MOTOR_DRIVER_INIT_TIMEOUT_S     2

#define APP_MOTOR_BUSY_STATE_CHECK_MS       149 // 49
#define APP_MOTOR_CMD_CONVERSATION_SYSTEM   16
#define APP_MOTOR_BACKWARD_DIR              0x00
#define APP_MOTOR_FORWARD_DIR               0x01
#define APP_MOTOR_CALIBRATE_SPEED           0x4000


#define APP_MOTOR_CALIBRATE_AT_POWER_UP     1
#define APP_MOTOR_DEBUG_IO                  1
#define APP_MOTOR_DEBUG_INFO                0

// ================================ DEBUG  =====================================

#define APP_EEPROM_DEBUG                    1
#define APP_EEPROM_DEBUG_IO                 1
#define APP_ENDSTOPS_DEBUG                  0
#define APP_WIFI_DEBUG                      0
#define APP_WIFI_LINE_DEBUG                 0
#define APP_HTTP_SERVER_DEBUG               1
#define APP_MOTOR_DRIVER_DEBUG              1
#define APP_MOTOR_DEBUG                     1

#endif /* INC_CONFIG_H_ */
