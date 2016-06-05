/*
 * eeprom.cpp
 *
 *  Created on: 03 апр. 2015 г.
 *      Author: RomaJam
 */


#include "eeprom.h"

eeprom_t EE;

void eeprom_t::Init() {
#if (APP_EEPROM_DEBUG)
    Uart.Printf("EE init\r\n");
#endif
    _IsInit = false;
    InitGpios();

    HOLD_Off();
    CS_Hi();
    SCLK_Lo();
    chThdSleepMilliseconds(21);
    WR_Enable();
    WriteEnable();

#if (EEPROM_READ_WRITE_TEST)
    if(!Test())
    {
        Uart.Printf("EE Test FAILURE\r\n");
        return;
    }
#endif
    _IsInit = true;

}

#if (EEPROM_READ_WRITE_TEST)
bool eeprom_t::Test()
{
    uint32_t TestByte = EEPROM_TEST_BYTE;
    writeU32(EEPROM_TEST_ADDR, TestByte);
    readU32(EEPROM_TEST_ADDR, &TestByte);
#if (APP_EEPROM_DEBUG)
    Uart.Printf("ReadByte: %X\r\n", TestByte);
#endif

    if(TestByte != EEPROM_TEST_BYTE)
        return false;

    Uart.Printf("EE Test OK\r\n");
    return true;
}
#endif

Rslt_t eeprom_t::WriteConf(uint16_t Addr, uint32_t *Conf)
{
    uint32_t WriteConf = *Conf;
    writeU32(Addr, WriteConf);
    return OK;
}

Rslt_t eeprom_t::ReadConf(uint16_t Addr, uint32_t *Conf)
{
    uint32_t ReadConf;
    readU32(Addr, &ReadConf);

#if (APP_EEPROM_DEBUG)
    Uart.Printf("EE: Read Conf addr=%X : 0x%X\r", Addr, ReadConf);
#endif

    *Conf = ReadConf;
    return OK;
}

void eeprom_t::writeU32(uint16_t Addr, uint32_t AByte)
{
    for(uint8_t i = 0; i < sizeof(uint32_t); i++)
    {
        WriteEnable();
        CS_Lo();
        WriteReadByte(EEPROM_CMD_WRITE); // Ins + Address MSB
        WriteReadByte(Addr + i);
        WriteReadByte((uint8_t)(AByte >> (i << 3)));
        CS_Hi();
        chThdSleepMilliseconds(21); // wait EEPROM
    }
}

void eeprom_t::readU32(uint16_t Addr, uint32_t *pByte)
{
    *pByte = 0;
    for(uint8_t i = 0; i < sizeof(uint32_t); i++)
    {
        CS_Lo();
        WriteReadByte(EEPROM_CMD_READ); // Ins + Address MSB
        WriteReadByte(Addr + i);
        *pByte |= (ReadByte() << (i << 3));
        CS_Hi();
    }
}

uint8_t eeprom_t::WriteReadByte(uint8_t AByte) {
    uint8_t res = 0;
    for (uint8_t i=0; i<8; i++) {
        if (AByte & 0x80)
            MOSI_Hi();
        else
            MOSI_Lo();
        AByte <<= 1;
        chThdSleepMilliseconds(1);
        SCLK_Hi();
        chThdSleepMilliseconds(1);
        res <<= 1;
        if(isMISO_Hi())
            res |= 0x01;
        SCLK_Lo();
    }
    return res;
}
