// Copyright (C) 2016 Krister W. <kisse66@hobbylabs.org>
//
// Original SPI flash driver this is based on:
// Copyright (c) 2013-2015 by Felix Rusu, LowPowerLab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code

#include "I2CEeprom.h"
I2CEeprom::I2CEeprom(void)
{
	
}

/// setup
bool I2CEeprom::init()
{
    MX_I2C1_Init();
    return true;
}

/// read 1 byte
uint8_t I2CEeprom::readByte(uint16_t addr)
{
	uint8_t buf[3];
    buf[0] = (addr >> 8);
    buf[1] = (addr & 0xff);
    HAL_I2C_Master_Transmit  (&hi2c1,I2CEE_ADD, buf, 2, 100);
    HAL_I2C_Master_Receive  (&hi2c1,I2CEE_ADD, &buf[2], 1, 100);
    return (buf[2]);
}

/// read multiple bytes
void I2CEeprom::readBytes(uint16_t addr, void * buf, uint16_t len)
{
    uint8_t *buff = (uint8_t *)buf;
    HAL_I2C_Mem_Read(&hi2c1, I2CEE_ADD, addr, I2C_MEMADD_SIZE_16BIT, buff, len, 15);
}

/// check if the chip is busy
bool I2CEeprom::busy()
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, I2CEE_ADD, 100, 15) != HAL_OK) return true;    // busy
    return false;
}

/// Write 1 byte
void I2CEeprom::writeByte(uint16_t addr, uint8_t byt)
{
    uint8_t buf[3];
    buf[0] = (addr >> 8);
    buf[1] = (addr & 0xff);
    buf[2] = byt;
    HAL_I2C_Master_Transmit  (&hi2c1,I2CEE_ADD, buf, 3, 100);
}

/// write multiple bytes
void I2CEeprom::writeBytes(uint16_t addr, const void* buf, uint16_t len)
{
    uint8_t *buff = (uint8_t *)buf;
    uint32_t frst_page_bcnt = (uint32_t)(I2CEE_PGBSZ-(addr%I2CEE_PGBSZ));   // number of bytes to write to first page 
    uint32_t full_page_bcnt = (uint32_t)(len/I2CEE_PGBSZ);                       // number of full pages to write
    uint32_t last_page_bcnt = (uint32_t)((addr+len)%I2CEE_PGBSZ);          // number of bytes to write to last page
    uint32_t eaddr = addr;
    /* write to first page till page boundary*/
    if (frst_page_bcnt > len) frst_page_bcnt = len;
    HAL_I2C_Mem_Write(&hi2c1, I2CEE_ADD, eaddr, I2C_MEMADD_SIZE_16BIT, buff, frst_page_bcnt, 15);
    if (HAL_I2C_IsDeviceReady(&hi2c1, I2CEE_ADD, 100, 15) != HAL_OK)    Error_Handler();
    len -= frst_page_bcnt;
    if (len == 0) return;
    /*  write full pages */
    buff += frst_page_bcnt; // data pointer offset
    eaddr += frst_page_bcnt; // write address offset
    while(full_page_bcnt)
    {
        HAL_I2C_Mem_Write(&hi2c1, I2CEE_ADD, eaddr, I2C_MEMADD_SIZE_16BIT, buff, I2CEE_PGBSZ, 15);
        if (HAL_I2C_IsDeviceReady(&hi2c1, I2CEE_ADD, 100, 15)!=HAL_OK)  Error_Handler();
        eaddr += I2CEE_PGBSZ; // write address offset
        buff += I2CEE_PGBSZ; // data pointer offset
        --full_page_bcnt;
        len -= I2CEE_PGBSZ;
        if (len == 0) return;
    }
    /* write to last page */                                  
    HAL_I2C_Mem_Write(&hi2c1, I2CEE_ADD, eaddr, I2C_MEMADD_SIZE_16BIT, buff, last_page_bcnt, 15);
    if (HAL_I2C_IsDeviceReady(&hi2c1, I2CEE_ADD, 100, 15)!=HAL_OK) Error_Handler();
}
