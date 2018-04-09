/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>, Adafruit <info@adafruit.com>
 * All rights reserved.
 *
 * This is mostly a re-mix of Alex Taradovs excellent Free-DAP code - we just put both halves into one library and wrapped it up in Arduino compatibility
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*- Includes ----------------------------------------------------------------*/
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Adafruit_DAP.h"
#include "dap.h"

/*- Definitions -------------------------------------------------------------*/
#define FLASH_START            0
#define FLASH_ROW_SIZE         512

#define DHCSR                  0xe000edf0
#define DEMCR                  0xe000edfc
#define AIRCR                  0xe000ed0c

#define DAP_DSU_CTRL_STATUS        0x41002100
#define DAP_DSU_DID                0x41002118
#define DAP_DSU_ADDR               0x41002104
#define DAP_DSU_DATA               0x4100210C
#define DAP_DSU_LENGTH             0x41002108

#define DAP_DSU_CTRL_CRC           0x00000004
#define DAP_DSU_STATUSA_DONE       0x00000100
#define DAP_DSU_STATUSA_BERR       0x00000400

#define NVMCTRL_CTRLA          0x41004000
#define NVMCTRL_CTRLB          0x41004004
#define NVMCTRL_PARAM          0x41004008
#define NVMCTRL_INTFLAG        0x41004010
#define NVMCTRL_STATUS         0x41004012
#define NVMCTRL_ADDR           0x41004014
#define NVMCTRL_RUNLOCK        0x41004018

#define NVMCTRL_CMD_EP         0xa500    /* Erase Page */
#define NVMCTRL_CMD_EB         0xa501    /* Erase Block */
#define NVMCTRL_CMD_WP         0xa503    /* Write Page */
#define NVMCTRL_CMD_WQW        0xa504    /* Write 128 bit word */
#define NVMCTRL_CMD_LR         0xa511    /* Lock Region */
#define NVMCTRL_CMD_UR         0xa512    /* Unlock Region */
#define NVMCTRL_CMD_SPRM       0xa513    /* Set Power Reduction Mode */
#define NVMCTRL_CMD_CPRM       0xa514    /* Clear Power Reduction Mode */
#define NVMCTRL_CMD_PBC        0xa515    /* Page Buffer Clear */
#define NVMCTRL_CMD_SSB        0xa516    /* Set Security Bit */

#define USER_ROW_ADDR          0x00804000
#define USER_ROW_SIZE          32

/*- Variables ---------------------------------------------------------------*/
device_t Adafruit_DAP_SAMx5::devices[] =
{
  { 0x60060000, (char *)"SAMD51P20A",          1024*1024, 2048  },
  { 0x60060001, (char *)"SAMD51P19A",          512*1024,  1024  },
  { 0x60060002, (char *)"SAMD51N20A",          1024*1024, 2048  },
  { 0x60060003, (char *)"SAMD51N19A",          512*1024,  1024  },
  { 0x60060004, (char *)"SAMD51J20A",          1024*1024, 2048  },
  { 0x60060005, (char *)"SAMD51J19A",          512*1024,  1024  },
  { 0x60060006, (char *)"SAMD51J18A",          256*1024,  512   },
  { 0x60060007, (char *)"SAMD51G19A",          512*1024,  1024  },
  { 0x60060008, (char *)"SAMD51G18A",          256*1024,  512   },
  { 0 },
};

bool Adafruit_DAP_SAMx5::select(uint32_t *found_id)
{
  uint32_t DAP_DSU_did;

  // Stop the core
  dap_write_word(DHCSR, 0xa05f0003);
  dap_write_word(DEMCR, 0x00100501);
  dap_write_word(AIRCR, 0x05fa0004);

  DAP_DSU_did = dap_read_word(DAP_DSU_DID);
  *found_id = DAP_DSU_did;

  for (device_t *device = devices; device->dsu_did > 0; device++)
  {
    if (device->dsu_did == DAP_DSU_did)
    {
      target_device = *device;

      return true;
    }
  }

  return false;
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAMx5::erase(void)
{
  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00001f00); // Clear flags
  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00000010); // Chip erase
  delay(100);
  while (0 == (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00000100));
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAMx5::lock(void)
{
  dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_SSB); // Set Security Bit
}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP_SAMx5::program_start(uint32_t offset)
{

  if (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000)
    perror_exit("device is locked, perform a chip erase before programming");

  dap_write_word(NVMCTRL_CTRLA, 0x04); // manual write

  dap_setup_clock(0);

  return FLASH_START + offset;
}

void Adafruit_DAP_SAMx5::programBlock(uint32_t addr, const uint8_t *buf, uint16_t size)
{

    uint32_t status = 0;

    dap_write_block(addr, buf, size);

    uint32_t timeout = 100;

    while(!(status & 0x01)){ //not ready
      status = dap_read_word(NVMCTRL_STATUS) >> 16;
      delay(1);
      timeout--;

      if(timeout == 0){
        perror_exit("timeout while writing page");
      }
    }

    dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_WP);
    while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAMx5::readBlock(uint32_t addr, uint8_t *buf)
{
  if (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000)
    perror_exit("device is locked, unable to read");

  dap_read_block(addr, buf, FLASH_ROW_SIZE);
}


bool Adafruit_DAP_SAMx5::readCRC(uint32_t length, uint32_t *crc)
{
   /* to verify CRC, compare (dap_read_word(DAP_DSU_DATA) ^ 0xFFFFFFFF) to output of crc32 program on linux */
   dap_write_word(DAP_DSU_DATA, 0xFFFFFFFF);
   dap_write_word(DAP_DSU_ADDR, 0);
   dap_write_word(DAP_DSU_LENGTH, length);

   dap_write_word(DAP_DSU_CTRL_STATUS, 0x00001f00); // Clear flags
   dap_write_word(DAP_DSU_CTRL_STATUS, DAP_DSU_CTRL_CRC); //start CRC

   uint32_t status = 0;
   while(0 == (status & DAP_DSU_STATUSA_DONE) ){
      status = dap_read_word(DAP_DSU_CTRL_STATUS);
      if( (status & DAP_DSU_STATUSA_BERR) > 0){
	//Serial.println(status, BIN);
	error_message = (char *)"bus read error during verify!";
	return false;
      }
   }
   *crc = dap_read_word(DAP_DSU_DATA);
   return true;
}

void Adafruit_DAP_SAMx5::fuseRead(){
  uint8_t buf[USER_ROW_SIZE];
  dap_read_block(USER_ROW_ADDR, buf, USER_ROW_SIZE);

  memcpy(_USER_ROW.reg, buf, USER_ROW_SIZE);
}

void Adafruit_DAP_SAMx5::fuseWrite()
{
  dap_write_word(NVMCTRL_CTRLA, 0x04); // manual write
  dap_write_word(NVMCTRL_ADDR, USER_ROW_ADDR);
  dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_EP);
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

  uint32_t status = 0;
  uint32_t timeout = 100;

  while(!(status & 0x01)){ //not ready
    status = dap_read_word(NVMCTRL_STATUS) >> 16;
    delay(1);
    timeout--;

    if(timeout == 0){
      perror_exit("timeout while writing page");
    }
  }

  for(int i=0; i<USER_ROW_SIZE; i+=16){
    dap_write_block(USER_ROW_ADDR + i, ((uint8_t *)&_USER_ROW) + i, 16);

    dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_WQW);
    while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));
  }
}
