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

/* The number of bytes to write at one time in program(). */
#define CHUNK_SIZE (1024)

/* Enable to verify written data (slow!) */
#define VERIFY_DATA (1)

/*- Definitions -------------------------------------------------------------*/
#define NRF5X_FLASH_START 0
#define NRF5X_FLASH_ROW_SIZE 256
#define NRF5X_FLASH_PAGE_SIZE 64

#define NRF5X_DHCSR 0xe000edf0
#define NRF5X_DEMCR 0xe000edfc
#define NRF5X_AIRCR 0xe000ed0c

#define NRF5X_FICR_CODEPAGESIZE 0x10000010 // Code memory page size
#define NRF5X_FICR_CODESIZE 0x10000014     // Code size (in pages)
#define NRF5X_FICR_HWID 0x10000100         // Part Code
#define NRF5X_FICR_CHIPVARIANT 0x10000104  // Part Variant
#define NRF5X_FICR_PACKAGEID 0x10000108    // Package Options
#define NRF5X_FICR_SRAM 0x1000010C         // RAM Variant
#define NRF5X_FICR_FLASHSIZE 0x10000110    // Flash Variant

// TODO: Change these from SAMD to nRF5x compatible registers!
#define NRF5X_DSU_CTRL_STATUS 0x41002100

#define NRF5X_NVMCTRL_CTRLA 0x41004000
#define NRF5X_NVMCTRL_CTRLB 0x41004004
#define NRF5X_NVMCTRL_PARAM 0x41004008
#define NRF5X_NVMCTRL_INTFLAG 0x41004014
#define NRF5X_NVMCTRL_STATUS 0x41004018
#define NRF5X_NVMCTRL_ADDR 0x4100401c

#define NRF5X_NVMCTRL_CMD_ER 0xa502
#define NRF5X_NVMCTRL_CMD_WP 0xa504
#define NRF5X_NVMCTRL_CMD_EAR 0xa505
#define NRF5X_NVMCTRL_CMD_WAP 0xa506
#define NRF5X_NVMCTRL_CMD_WL 0xa50f
#define NRF5X_NVMCTRL_CMD_UR 0xa541
#define NRF5X_NVMCTRL_CMD_PBC 0xa544
#define NRF5X_NVMCTRL_CMD_SSB 0xa545

#define NRF5X_USER_ROW_ADDR 0x00804000
#define NRF5X_USER_ROW_SIZE 256

// from nrf52.h

#ifdef __cplusplus
#define __I volatile /*!< Defines 'read only' permissions */
#else
#define __I volatile const /*!< Defines 'read only' permissions */
#endif
#define __O volatile  /*!< Defines 'write only' permissions */
#define __IO volatile /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define __IM                                                                   \
  volatile const      /*! Defines 'read only' structure member permissions */
#define __OM volatile /*! Defines 'write only' structure member permissions */
#define __IOM                                                                  \
  volatile /*! Defines 'read / write' structure member permissions */

/**
 * @brief Non Volatile Memory Controller (NVMC)
 */

typedef struct { /*!< (@ 0x4001E000) NVMC Structure */
  __IM uint32_t RESERVED[256];
  __IM uint32_t READY; /*!< (@ 0x00000400) Ready flag */
  __IM uint32_t RESERVED2[64];
  __IOM uint32_t CONFIG; /*!< (@ 0x00000504) Configuration register */

  __IOM uint32_t ERASEPAGE; /*!< (@ 0x00000508) Register for erasing a page in
                               code area                   */
  __IOM uint32_t ERASEALL;  /*!< (@ 0x0000050C) Register for erasing all
                               non-volatile user memory          */
  __IOM uint32_t
      ERASEPCR0; /*!< (@ 0x00000510) Deprecated register - Register for erasing
                    a page in code area. Equivalent to ERASEPAGE. */
  __IOM uint32_t ERASEUICR; /*!< (@ 0x00000514) Register for erasing user
                               information configuration registers */
} NRF_NVMC_Type;            /*!< Size = 1360 (0x550)            */

#define NRF_NVMC_BASE (0x4001E000UL)
#define NRF_NVMC ((NRF_NVMC_Type *)NRF_NVMC_BASE)

#define NRF52840_FLASH_START (0x00000000)
#define NRF52840_FLASHALGO_START (0X20000000)
#define NRF52840_FLASHALGO_SIZE (0x150)
#define NRF52840_FLASHALGO_INIT (0x20000021)
#define NRF52840_FLASHALGO_STATBASE (0x20000020 + NRF52840_FLASHALGO_SIZE)
#define NRF52840_FLASHALGO_STCKPNTR (0x20001000)
#define NRF52840_FLASHALGO_BRKPOINT (0x20000001)

static char _variant_name[30] = {0};

//-----------------------------------------------------------------------------
bool Adafruit_DAP_nRF5x::select(uint32_t *found_id) {
  uint32_t hwid;
  uint32_t chipvariant;
  uint32_t codepagesize;
  uint32_t codesize;
  uint32_t sram;

  dap_target_prepare();

  // Stop the core
  dap_write_word(NRF5X_DHCSR, 0xa05f0003);
  dap_write_word(NRF5X_DEMCR, 0x00000001);
  dap_write_word(NRF5X_AIRCR, 0x05fa0004);

  // Family ID
  hwid = dap_read_word(NRF5X_FICR_HWID);

  // Variant ID and swap its endian
  chipvariant = dap_read_word(NRF5X_FICR_CHIPVARIANT);
  chipvariant = __builtin_bswap32(chipvariant);

  *found_id = hwid;

  switch (hwid) {
  //------------- nRF51 -------------//
  // Adafruit only use XXAC (32KB 256KB) variant
  case 0x51422:
    (void)chipvariant;
    strcpy(_variant_name, "nRF51422_XXAC");
    break;

  case 0x51822:
    (void)chipvariant;
    strcpy(_variant_name, "nRF51822_XXAC");
    break;

  //------------- nRF52832 -------------//
  case 0x52832:
    strcpy(_variant_name, "nRF52832_");
    strncat(_variant_name, (char *)&chipvariant, 4);
    break;

  //------------- nRF52833 -------------//
  case 0x52833:
    strcpy(_variant_name, "nRF52833_");
    strncat(_variant_name, (char *)&chipvariant, 4);
    break;

  //------------- nRF52840 -------------//
  case 0x52840:
    strcpy(_variant_name, "nRF52840_");
    strncat(_variant_name, (char *)&chipvariant, 4);
    break;

  // No matching device ID found
  default:
    Serial.print("Unknown HWID = 0x");
    Serial.println(hwid, HEX);
    return false;
  }

  codepagesize = dap_read_word(NRF5X_FICR_CODEPAGESIZE);
  codesize = dap_read_word(NRF5X_FICR_CODESIZE);

  // Assign device details to target_device
  target_device.dsu_did = hwid;
  target_device.flash_size = codepagesize * codesize;
  target_device.n_pages = codesize;
  target_device.name = _variant_name;

  return true;
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::deselect(void) {
  dap_write_word(NRF5X_DEMCR, 0x00000000);
  dap_write_word(NRF5X_AIRCR, 0x05fa0004);
}

bool Adafruit_DAP_nRF5x::flashWaitReady(void) {
  int i;

  for (i = 0; i < 100000; i++) {
    if (dap_read_word((uint32_t)&NRF_NVMC->READY) & 1UL) {
      return true;
    }
  }
  return false;
}

bool Adafruit_DAP_nRF5x::flashReady(void) {
  return dap_read_word((uint32_t)&NRF_NVMC->READY) & 1UL;
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::erase(void) {
  dap_write_word((uint32_t)&NRF_NVMC->CONFIG, 2);   // Erase Enable
  dap_write_word((uint32_t)&NRF_NVMC->ERASEALL, 1); // Erase All

  while (!flashReady()) {
  }

  dap_write_word((uint32_t)&NRF_NVMC->CONFIG, 0); // Disable Erase
}

void Adafruit_DAP_nRF5x::erasePage(uint32_t page) {}

void Adafruit_DAP_nRF5x::eraseUICR(void) {}

void Adafruit_DAP_nRF5x::eraseFICR(void) {}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP_nRF5x::program_start(uint32_t offset) {
  //  if (dap_read_word(NRF5X_DSU_CTRL_STATUS) & 0x00010000)
  //    perror_exit("device is locked, perform a chip erase before
  //    programming");

  // TODO: convert to slow/fast clock mode
  dap_setup_clock(0);

  return NRF5X_FLASH_START + offset;
}

bool Adafruit_DAP_nRF5x::program(uint32_t addr, const uint8_t *buf,
                                 uint32_t count) {
  // address must be word-aligned
  if (addr & 0x03)
    return false;

  dap_write_word((uint32_t)&NRF_NVMC->CONFIG, 1); // Write Enable

  while (count) {
    uint8_t data[CHUNK_SIZE];
    uint32_t bytes = min(count, (uint32_t) CHUNK_SIZE);
    bool hasdata = false;

    memset(data, 0xFF, CHUNK_SIZE);
    memcpy(data, buf, bytes);

    /* If data is all 0x00, this chunk is empty. Don't bother writing it since
     * we've already erased the flash memory. */
    for (uint32_t i = 0; i < CHUNK_SIZE; i++) {
      if (data[i] != 0xFF) {
        hasdata = true;
        break;
      }
    }

    if (hasdata) {
      dap_write_block(addr, data, (int)bytes);
    }

    /* Optionally verify the written data */
    if (hasdata && VERIFY_DATA) {
      uint8_t read_buf[CHUNK_SIZE] = {0};
      dap_read_block(addr, read_buf, (int)bytes);
      int equal = memcmp(read_buf, data, bytes);
      if (equal != 0) {
        /* Verify failed! */
        return false;
      }
    }

    addr += bytes;
    buf += bytes;
    count -= bytes;

    if (!flashWaitReady()) {
      // Flash timed out before being ready!
      return false;
    };
  }

  dap_write_word((uint32_t)&NRF_NVMC->CONFIG, 0); // Write Disable

  return true;
}

void Adafruit_DAP_nRF5x::programUICR(uint32_t addr, uint32_t value) {
  dap_write_word((uint32_t)&NRF_NVMC->CONFIG, 1); // Write Enable
  dap_write_word(addr, value);
  while (!flashReady()) {
  }
  dap_write_word((uint32_t)&NRF_NVMC->CONFIG, 0); // Write Disable
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::lock(void) {
  dap_write_word(NRF5X_NVMCTRL_CTRLA,
                 NRF5X_NVMCTRL_CMD_SSB); // Set Security Bit
}

void Adafruit_DAP_nRF5x::programBlock(uint32_t addr, uint8_t *buf) {
  dap_write_word(NRF5X_NVMCTRL_ADDR, addr >> 1);

  dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_UR); // Unlock Region
  while (0 == (dap_read_word(NRF5X_NVMCTRL_INTFLAG) & 1))
    ;

  dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_ER); // Erase Row
  while (0 == (dap_read_word(NRF5X_NVMCTRL_INTFLAG) & 1))
    ;
  dap_write_block(addr, buf, NRF5X_FLASH_ROW_SIZE);
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::readBlock(uint32_t addr, uint8_t *buf) {
  if (dap_read_word(NRF5X_DSU_CTRL_STATUS) & 0x00010000)
    perror_exit("device is locked, unable to read");

  dap_read_block(addr, buf, NRF5X_FLASH_ROW_SIZE);
}

void Adafruit_DAP_nRF5x::fuseRead() {
  uint8_t buf[NRF5X_USER_ROW_SIZE];
  dap_read_block(NRF5X_USER_ROW_ADDR, buf, NRF5X_USER_ROW_SIZE);

  uint64_t fuses = ((uint64_t)buf[7] << 56) | ((uint64_t)buf[6] << 48) |
                   ((uint64_t)buf[5] << 40) | ((uint64_t)buf[4] << 32) |
                   ((uint64_t)buf[3] << 24) | ((uint64_t)buf[2] << 16) |
                   ((uint64_t)buf[1] << 8) | (uint64_t)buf[0];

  _USER_ROW.set(fuses);
}

void Adafruit_DAP_nRF5x::fuseWrite() {
  uint64_t fuses = _USER_ROW.get();
  uint8_t buf[NRF5X_USER_ROW_SIZE] = {
      (uint8_t)fuses,         (uint8_t)(fuses >> 8),  (uint8_t)(fuses >> 16),
      (uint8_t)(fuses >> 24), (uint8_t)(fuses >> 32), (uint8_t)(fuses >> 40),
      (uint8_t)(fuses >> 48), (uint8_t)(fuses >> 56)};

  dap_write_word(NRF5X_NVMCTRL_CTRLB, 0);
  dap_write_word(NRF5X_NVMCTRL_ADDR, NRF5X_USER_ROW_ADDR >> 1);
  dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_EAR);
  while (0 == (dap_read_word(NRF5X_NVMCTRL_INTFLAG) & 1))
    ;

  dap_write_block(NRF5X_USER_ROW_ADDR, buf, NRF5X_USER_ROW_SIZE);
}
