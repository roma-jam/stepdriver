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


// ================================ DEBUG  =====================================

#define APP_EEPROM_DEBUG                    0
#define APP_ENDSTOPS_DEBUG                  0
#endif /* INC_CONFIG_H_ */
