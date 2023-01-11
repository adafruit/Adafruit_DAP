/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>, Adafruit
 * <info@adafruit.com>
 * All rights reserved.
 *
 * This is mostly a re-mix of Alex Taradovs excellent Free-DAP code - we just
 * put both halves into one library and wrapped it up in Arduino compatibility
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
#include "Adafruit_DAP.h"
#include "dap.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*- Definitions -------------------------------------------------------------*/
#define DAP_FLASH_START 0
#define DAP_FLASH_ROW_SIZE 256
#define DAP_FLASH_PAGE_SIZE 64

#define DHCSR                   0xe000edf0
#define DEMCR                   0xe000edfc
#define AIRCR                   0xe000ed0c

#define DAP_DSU_CTRL_STATUS     0x41002100 // Used for accessing both CTRL, STATUSA and STATUSB from external debugger
#define DAP_DSU_DID             0x41002118
#define DAP_DSU_ADDR            0x41002104
#define DAP_DSU_DATA            0x4100210C
#define DAP_DSU_LENGTH          0x41002108

#define DAP_DSU_CTRL_CRC        0x00000004
#define DAP_DSU_STATUSA_DONE    0x00000100
#define DAP_DSU_STATUSA_CRSTEXT 0x00000200
#define DAP_DSU_STATUSA_BERR    0x00000400

#define NVMCTRL_CTRLA           0x41004000
#define NVMCTRL_CTRLB           0x41004004
#define NVMCTRL_PARAM           0x41004008
#define NVMCTRL_INTFLAG         0x41004014
#define NVMCTRL_STATUS          0x41004018
#define NVMCTRL_ADDR            0x4100401c

#define NVMCTRL_CMD_ER          0xa502
#define NVMCTRL_CMD_WP          0xa504
#define NVMCTRL_CMD_EAR         0xa505
#define NVMCTRL_CMD_WAP         0xa506
#define NVMCTRL_CMD_WL          0xa50f
#define NVMCTRL_CMD_UR          0xa541
#define NVMCTRL_CMD_PBC         0xa544
#define NVMCTRL_CMD_SSB         0xa545

#define USER_ROW_ADDR           0x00804000

/*- Variables ---------------------------------------------------------------*/
device_t Adafruit_DAP_SAM::devices[] = {
    {0x10040100, (char *)"SAM D09D14A", 16 * 1024, 256},
    {0x10040107, (char *)"SAM D09C13A", 8 * 1024, 128},
    {0x10020100, (char *)"SAM D10D14AM", 16 * 1024, 256},
    {0x10030100, (char *)"SAM D11D14A", 16 * 1024, 256},
    {0x10030000, (char *)"SAM D11D14AM", 16 * 1024, 256},
    {0x10030003, (char *)"SAM D11D14AS", 16 * 1024, 256},
    {0x10030103, (char *)"SAM D11D14AS (Rev B)", 16 * 1024, 256},
    {0x10030006, (char *)"SAM D11C14A", 16 * 1024, 256},
    {0x10030106, (char *)"SAM D11C14A (Rev B)", 16 * 1024, 256},
    {0x1000120d, (char *)"SAM D20E15A", 32 * 1024, 512},
    {0x1000140a, (char *)"SAM D20E18A", 256 * 1024, 4096},
    {0x10001100, (char *)"SAM D20J18A", 256 * 1024, 4096},
    {0x10001200, (char *)"SAM D20J18A (Rev C)", 256 * 1024, 4096},
    {0x10010100, (char *)"SAM D21J18A", 256 * 1024, 4096},
    {0x10010200, (char *)"SAM D21J18A (Rev C)", 256 * 1024, 4096},
    {0x10010300, (char *)"SAM D21J18A (Rev D)", 256 * 1024, 4096},
    {0x1001020d, (char *)"SAM D21E15A (Rev C)", 32 * 1024, 512},
    {0x1001030a, (char *)"SAM D21E18A", 256 * 1024, 4096},
    {0x10010205, (char *)"SAM D21G18A", 256 * 1024, 4096},
    {0x10010305, (char *)"SAM D21G18A (Rev D)", 256 * 1024, 4096},
    {0x10010019, (char *)"SAM R21G18 ES", 256 * 1024, 4096},
    {0x10010119, (char *)"SAM R21G18", 256 * 1024, 4096},
    {0x10010219, (char *)"SAM R21G18A (Rev C)", 256 * 1024, 4096},
    {0x10010319, (char *)"SAM R21G18A (Rev D)", 256 * 1024, 4096},
    {0x11010100, (char *)"SAM C21J18A ES", 256 * 1024, 4096},
    {0x10810219, (char *)"SAM L21E18B", 256 * 1024, 4096},
    {0x10810000, (char *)"SAM L21J18A", 256 * 1024, 4096},
    {0x1081010f, (char *)"SAM L21J18B (Rev B)", 256 * 1024, 4096},
    {0x1081020f, (char *)"SAM L21J18B (Rev C)", 256 * 1024, 4096},
    {0x1081021e, (char *)"SAM R30G18A", 256 * 1024, 4096},
    {0x1081021f, (char *)"SAM R30E18A", 256 * 1024, 4096},
    {0, NULL, 0, 0},
};

//--------------------------------------------------------------------+
// API for both SAMD21 and SAMD51
//--------------------------------------------------------------------+

bool Adafruit_DAP_SAM::programFlash(uint32_t flashOffset, const uint8_t * data, uint32_t datalen, bool doVerify) {
  size_t const bufSize = pageSize();

  Serial.println("\nProgramming...");
  uint32_t startAddr = program_start(flashOffset);

  for(uint32_t count = 0; count < datalen; count += bufSize) {
    // Serial.print(" 0x");
    // Serial.print(startAddr + count, HEX);
    programBlock(startAddr + count, data + count, bufSize);
  }

  if (doVerify) {
    uint8_t verifyBuf[bufSize];

    bool success = true;
    Serial.println("\nVerifying...");

    for(uint32_t count = 0; count < datalen; count += bufSize) {
      Serial.print("Block: 0x");
      Serial.print(startAddr + count, HEX);
      Serial.print(" ");

      readBlock(startAddr + count, verifyBuf);

      if (memcmp(verifyBuf, data + count, bufSize) == 0) {
        Serial.println("OK");
      }
      else {
        Serial.println("Failed compare!");
        success = false;
      }
    }

    if (!success) {
      perror_exit("Error validating ");
    }

    return success;
  }

  return true;
}

void Adafruit_DAP_SAM::resetWithExtension(void)
{
  Serial.println("Enter Reset with Extension mode... ");

  // Enter reset extension mode
  dap_reset_target_hw(0);
  delay(10);

  if ( !targetConnect() ) {
    perror_exit(error_message);
    return; // false
  }

  Serial.println("Target prepare...");
  dap_target_prepare();
}

//--------------------------------------------------------------------+
// API for SAMD21 only
//--------------------------------------------------------------------+

bool Adafruit_DAP_SAM::select(uint32_t *found_id) {
  uint32_t DAP_DSU_did;

  // Stopping the core fails on locked SAM D21/51, when not doing an Extended reset first.
  // As per the ataradov/edbg source code, for SAMD MCUs which is locked by having the Security Bit set,
  // one must enter Reset Extension mode before issuing reads through DAP. Write mostly requires the Security Bit
  // to be cleared by executing a Chip Erase, before entering Reset Extension mode once more, followed by a SWD reconnect.

  resetWithExtension();

  DAP_DSU_did = dap_read_word(DAP_DSU_DID);
  *found_id = DAP_DSU_did;

  for (device_t *device = devices; device->dsu_did > 0; device++) {
    if (device->dsu_did == DAP_DSU_did) {
      target_device = *device;

      locked = dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000;
      if (locked) {
        Serial.println("Device is locked, must be unlocked first!");
      }
      else {
        // Stop the core
        finishReset();
      }
      return true;
    }
  }

  return false;
}

void Adafruit_DAP_SAM::finishReset() {
  // Stop the core
  dap_write_word(DHCSR, 0xa05f0003);
  dap_write_word(DEMCR, 0x00000001);
  dap_write_word(AIRCR, 0x05fa0004);

  // Release the reset
  dap_write_word(DAP_DSU_CTRL_STATUS, DAP_DSU_STATUSA_CRSTEXT);
}

void Adafruit_DAP_SAM::resetProtectionFuses(bool resetBootloaderProtection, bool resetRegionLocks) {
  bool doFuseWrite = false;

  fuseRead();

  if (resetBootloaderProtection && _USER_ROW.bit.BOOTPROT != 7) {
    Serial.print("Resetting BOOTPROT... ");
    _USER_ROW.bit.BOOTPROT = 7;
    doFuseWrite = true;
  }

  if (resetRegionLocks && _USER_ROW.bit.LOCK != 0xffffu) {
    Serial.print(" Resetting NVM region LOCK... ");
    _USER_ROW.bit.LOCK = 0xffffu;
    doFuseWrite = true;
  }

  if (doFuseWrite) {
    fuseWrite();
  }
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAM::deselect(void) {
  dap_write_word(DEMCR, 0x00000000);
  dap_write_word(AIRCR, 0x05fa0004);
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAM::erase(void) {
  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00001f00); // Clear flags
  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00000010); // Chip erase
  delay(100);
  while (0 == (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00000100))
    ;
  if (locked) {
    resetWithExtension();
    finishReset();
  }
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAM::lock(void) {
  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_SSB); // Set Security Bit
}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP_SAM::program_start(uint32_t offset, uint32_t size) {
  (void) size; // not used

  // DSU.STATUSB.PROT
  if (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000) {
    perror_exit("device is locked, perform a chip erase before programming");
}

  resetProtectionFuses(true, false);

  dap_write_word(NVMCTRL_CTRLB, 0); // Enable automatic write

  dap_setup_clock(0);

  return DAP_FLASH_START + offset;
}

void Adafruit_DAP_SAM::programBlock(uint32_t addr, const uint8_t *buf,
                                    uint32_t size) {
  /* DM: this is actually unnecessary after a chip erase
  dap_write_word(NVMCTRL_ADDR, addr >> 1);

  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_ER); // Erase Row
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));
  */

  // Even after a chip erase, unlocking flash regions still might be necessary, since region locks is not cleared by Chip Erase.
  dap_write_word(NVMCTRL_ADDR, addr);
  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_UR); // Unlock Region temporary
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

  dap_write_block(addr, buf, size);
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAM::readBlock(uint32_t addr, uint8_t *buf) {
  if (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000)
    perror_exit("device is locked, unable to read");

  dap_read_block(addr, buf, PAGESIZE);
}

bool Adafruit_DAP_SAM::readCRC(uint32_t length, uint32_t *crc) {
  /* to verify CRC, compare (dap_read_word(DAP_DSU_DATA) ^ 0xFFFFFFFF) to output
   * of crc32 program on linux */
  dap_write_word(DAP_DSU_DATA, 0xFFFFFFFF);
  dap_write_word(DAP_DSU_ADDR, 0);
  dap_write_word(DAP_DSU_LENGTH, length);

  dap_write_word(DAP_DSU_CTRL_STATUS, 0x00001f00);       // Clear flags
  dap_write_word(DAP_DSU_CTRL_STATUS, DAP_DSU_CTRL_CRC); // start CRC

  uint32_t status = 0;
  while (0 == (status & DAP_DSU_STATUSA_DONE)) {
    status = dap_read_word(DAP_DSU_CTRL_STATUS);
    if ((status & DAP_DSU_STATUSA_BERR) > 0) {
      // Serial.println(status, BIN);
      error_message = (char *)"bus read error during verify!";
      return false;
    }
  }
  *crc = dap_read_word(DAP_DSU_DATA);
  return true;
}

void Adafruit_DAP_SAM::fuseRead() {
  dap_read_block(USER_ROW_ADDR, _USER_ROW.reg, sizeof(_USER_ROW.reg));
}

void Adafruit_DAP_SAM::fuseWrite() {
  if (_USER_ROW.fuses == 0ULL || _USER_ROW.fuses == ~0ULL) {
    // Entire User Row fuses shold never be 0 or all 1s
    perror_exit("Will never write User Row fuses as all0 or all 1s!");
  }

  dap_write_word(NVMCTRL_CTRLB, 0);
  dap_write_word(NVMCTRL_ADDR, USER_ROW_ADDR >> 1);
  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_EAR);
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1))
    ;

  dap_write_block(USER_ROW_ADDR, _USER_ROW.reg, sizeof(_USER_ROW.reg));

  // Needs to reset the MCU, for it to reread the fuses
  delay(100);
  resetWithExtension();
  finishReset();
}

bool Adafruit_DAP_SAM::protectBoot(void) {
  fuseRead();
  _USER_ROW.bit.BOOTPROT = 0x02;
  fuseWrite();

  return true;
}

bool Adafruit_DAP_SAM::unprotectBoot(void) {
  fuseRead();

  // if locked then unlock
  if (_USER_ROW.bit.BOOTPROT != 0x07) {
    _USER_ROW.bit.BOOTPROT = 0x07;
    fuseWrite();
  }

  return true;
}


