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
#define FLASH_START 0
#define FLASH_ROW_SIZE 512

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
#define NVMCTRL_INTFLAG         0x41004010
#define NVMCTRL_STATUS          0x41004012
#define NVMCTRL_ADDR            0x41004014
#define NVMCTRL_RUNLOCK         0x41004018

#define NVMCTRL_CMD_EP          0xa500   /* Erase Page */
#define NVMCTRL_CMD_EB          0xa501   /* Erase Block */
#define NVMCTRL_CMD_WP          0xa503   /* Write Page */
#define NVMCTRL_CMD_WQW         0xa504  /* Write 128 bit word */
#define NVMCTRL_CMD_LR          0xa511   /* Lock Region */
#define NVMCTRL_CMD_UR          0xa512   /* Unlock Region */
#define NVMCTRL_CMD_SPRM        0xa513 /* Set Power Reduction Mode */
#define NVMCTRL_CMD_CPRM        0xa514 /* Clear Power Reduction Mode */
#define NVMCTRL_CMD_PBC         0xa515  /* Page Buffer Clear */
#define NVMCTRL_CMD_SSB         0xa516  /* Set Security Bit */
#define NVMCTRL_CMD_CELCK       0xa518  /* Chip Erase Lock - DSU.CTRL.CE command is not available */
#define NVMCTRL_CMD_CEULCK      0xa519  /* Chip Erase Unlock - DSU.CTRL.CE command is available */
#define NVMCTRL_CMD_SBPDIS      0xa51a  /* Sets STATUS.BPDIS, Boot loader protection is off until CBPDIS is issued or next start-up sequence. Page 628 */
#define NVMCTRL_CMD_CBPDIS      0xa51b  /* Clears STATUS.BPDIS, Boot loader protection is not off */

#define USER_ROW_ADDR           0x00804000

/*- Variables ---------------------------------------------------------------*/
device_t Adafruit_DAP_SAMx5::devices[] = {
    {0x60060000, "SAMD51P20A", 1024 * 1024, 2048},
    {0x60060300, "SAMD51P20A", 1024 * 1024, 2048},
    {0x60060001, "SAMD51P19A", 512 * 1024, 1024},
    {0x60060002, "SAMD51N20A", 1024 * 1024, 2048},
    {0x60060003, "SAMD51N19A", 512 * 1024, 1024},
    {0x60060004, "SAMD51J20A", 1024 * 1024, 2048},
    {0x60060304, "SAMD51J20A", 1024 * 1024, 2048},
    {0x60060305, "SAMD51J19A", 512 * 1024, 1024},
    {0x60060005, "SAMD51J19A", 512 * 1024, 1024},
    {0x60060006, "SAMD51J18A", 256 * 1024, 512},
    {0x60060007, "SAMD51G19A", 512 * 1024, 1024},
    {0x60060307, "SAMD51G19A", 512 * 1024, 1024},
    {0x60060008, "SAMD51G18A", 256 * 1024, 512},
    {0x61810002, "SAME51J19A", 512 * 1024, 1024},
    {0x61810302, "SAME51J19A", 512 * 1024, 1024},
    {0, NULL, 0, 0},
};

//-----------------------------------------------------------------------------

bool Adafruit_DAP_SAMx5::select(uint32_t *found_id) {
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

void Adafruit_DAP_SAMx5::resetProtectionFuses(bool resetBootloaderProtection, bool resetRegionLocks) {
  bool doFuseWrite = false;

  fuseRead();

  if (resetBootloaderProtection && _USER_ROW.bit.NVM_BOOT != 0xf) {
    Serial.println("Resetting NVM BOOT... ");
    _USER_ROW.bit.NVM_BOOT = 0xf;
    doFuseWrite = true;
  }

  if (resetRegionLocks && _USER_ROW.bit.NVM_LOCKS != 0xffffffffu) {
    Serial.println(" Resetting NVM region LOCK... ");
    _USER_ROW.bit.NVM_LOCKS = 0xffffffffu;
    doFuseWrite = true;
  }

  if (doFuseWrite) {
    fuseWrite();
  }
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAMx5::erase(void) {
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
void Adafruit_DAP_SAMx5::lock(void) {
  dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_SSB); // Set Security Bit
}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP_SAMx5::program_start(uint32_t offset, uint32_t size) {
  (void) size;

  if (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000) {
    perror_exit("device is locked, perform a chip erase before programming");
  }

  resetProtectionFuses(true, false);

  // Temporarily turn off bootloader protection:
  dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_SBPDIS);
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1))
    ;

  dap_write_word(NVMCTRL_CTRLA, 0x04); // manual write

  dap_setup_clock(0);

  return FLASH_START + offset;
}

void Adafruit_DAP_SAMx5::programBlock(uint32_t addr, const uint8_t *buf,
                                      uint32_t size) {

  // Even after a chip erase with reset, a temporary Unlock region might be necessary
  dap_write_word(NVMCTRL_ADDR, addr);
  dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_UR); // Unlock Region temporary
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

  uint32_t status = 0;

  dap_write_block(addr, buf, size);

  uint32_t timeout = 100;

  while (!(status & 0x01)) { // not ready
    status = dap_read_word(NVMCTRL_STATUS) >> 16;
    delay(1);
    timeout--;

    if (timeout == 0) {
      perror_exit("timeout while writing page");
    }
  }

  dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_WP);
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1))
    ;
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_SAMx5::readBlock(uint32_t addr, uint8_t *buf) {
  if (dap_read_word(DAP_DSU_CTRL_STATUS) & 0x00010000)
    perror_exit("device is locked, unable to read");

  dap_read_block(addr, buf, PAGESIZE);
}

bool Adafruit_DAP_SAMx5::readCRC(uint32_t length, uint32_t *crc) {
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

void Adafruit_DAP_SAMx5::fuseRead() {
  dap_read_block(USER_ROW_ADDR, _USER_ROW.reg, USER_ROW_SIZE);
}

void Adafruit_DAP_SAMx5::fuseWrite() {
  dap_write_word(NVMCTRL_CTRLA, 0x04); // manual write
  dap_write_word(NVMCTRL_ADDR, USER_ROW_ADDR);
  dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_EP);
  while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1))
    ;

  uint32_t status = 0;
  uint32_t timeout = 100;

  while (!(status & 0x01)) { // not ready
    status = dap_read_word(NVMCTRL_STATUS) >> 16;
    delay(1);
    timeout--;

    if (timeout == 0) {
      perror_exit("timeout while writing fuse");
    }
  }

  for (size_t i = 0; i < USER_ROW_SIZE; i += 16) {
    dap_write_block(USER_ROW_ADDR + i, ((uint8_t *)&_USER_ROW) + i, 16);

    dap_write_word(NVMCTRL_CTRLB, NVMCTRL_CMD_WQW);
    while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1))
      ;
  }

  // Needs to reset the MCU, for it to reread the fuses
  delay(100);
  resetWithExtension();
  finishReset();
}

bool Adafruit_DAP_SAMx5::protectBoot(void) {
  fuseRead();
  _USER_ROW.bit.NVM_BOOT = 0x0D;
  fuseWrite();

  return 0x0D == _USER_ROW.bit.NVM_BOOT;
}

bool Adafruit_DAP_SAMx5::unprotectBoot(void) {
  fuseRead();

  // if locked then unlock
  if (_USER_ROW.bit.NVM_BOOT != 0x0F) {
    _USER_ROW.bit.NVM_BOOT = 0x0F;
    fuseWrite();

    fuseRead();
    return 0x0F == _USER_ROW.bit.NVM_BOOT;
  }

  return true;
}
