/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>, Ha Thach for
 * Adafruit <info@adafruit.com>
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

#include "Adafruit_DAP.h"
#include "dap.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "Arduino.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define DHCSR 0xe000edf0
#define DEMCR 0xe000edfc
#define AIRCR 0xe000ed0c

#define DAP_DBGMCU_IDCODE 0xE0042000
#define STM32_FLASHSIZE 0x1FFF7A22

// Flash Interface Registers
#define FLASH_R_BASE 0x40023C00UL
#define FLASH_ACR (FLASH_R_BASE + 0x00)
#define FLASH_KEYR (FLASH_R_BASE + 0x04)
#define FLASH_OPTKEYR (FLASH_R_BASE + 0x08)
#define FLASH_SR (FLASH_R_BASE + 0x0C)
#define FLASH_CR (FLASH_R_BASE + 0x10)
#define FLASH_OPTCR (FLASH_R_BASE + 0x14)
#define FLASH_OPTCR1 (FLASH_R_BASE + 0x18)

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos (0U)
#define FLASH_SR_EOP_Msk (0x1UL << FLASH_SR_EOP_Pos) /*!< 0x00000001 */
#define FLASH_SR_EOP FLASH_SR_EOP_Msk
#define FLASH_SR_SOP_Pos (1U)
#define FLASH_SR_SOP_Msk (0x1UL << FLASH_SR_SOP_Pos) /*!< 0x00000002 */
#define FLASH_SR_SOP FLASH_SR_SOP_Msk
#define FLASH_SR_WRPERR_Pos (4U)
#define FLASH_SR_WRPERR_Msk (0x1UL << FLASH_SR_WRPERR_Pos) /*!< 0x00000010 */
#define FLASH_SR_WRPERR FLASH_SR_WRPERR_Msk
#define FLASH_SR_PGAERR_Pos (5U)
#define FLASH_SR_PGAERR_Msk (0x1UL << FLASH_SR_PGAERR_Pos) /*!< 0x00000020 */
#define FLASH_SR_PGAERR FLASH_SR_PGAERR_Msk
#define FLASH_SR_PGPERR_Pos (6U)
#define FLASH_SR_PGPERR_Msk (0x1UL << FLASH_SR_PGPERR_Pos) /*!< 0x00000040 */
#define FLASH_SR_PGPERR FLASH_SR_PGPERR_Msk
#define FLASH_SR_PGSERR_Pos (7U)
#define FLASH_SR_PGSERR_Msk (0x1UL << FLASH_SR_PGSERR_Pos) /*!< 0x00000080 */
#define FLASH_SR_PGSERR FLASH_SR_PGSERR_Msk
#define FLASH_SR_BSY_Pos (16U)
#define FLASH_SR_BSY_Msk (0x1UL << FLASH_SR_BSY_Pos) /*!< 0x00010000 */
#define FLASH_SR_BSY FLASH_SR_BSY_Msk

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG_Pos (0U)
#define FLASH_CR_PG_Msk (0x1UL << FLASH_CR_PG_Pos) /*!< 0x00000001 */
#define FLASH_CR_PG FLASH_CR_PG_Msk
#define FLASH_CR_SER_Pos (1U)
#define FLASH_CR_SER_Msk (0x1UL << FLASH_CR_SER_Pos) /*!< 0x00000002 */
#define FLASH_CR_SER FLASH_CR_SER_Msk
#define FLASH_CR_MER_Pos (2U)
#define FLASH_CR_MER_Msk (0x1UL << FLASH_CR_MER_Pos) /*!< 0x00000004 */
#define FLASH_CR_MER FLASH_CR_MER_Msk
#define FLASH_CR_SNB_Pos (3U)
#define FLASH_CR_SNB_Msk (0x1FUL << FLASH_CR_SNB_Pos) /*!< 0x000000F8 */
#define FLASH_CR_SNB FLASH_CR_SNB_Msk
#define FLASH_CR_SNB_0 (0x01UL << FLASH_CR_SNB_Pos) /*!< 0x00000008 */
#define FLASH_CR_SNB_1 (0x02UL << FLASH_CR_SNB_Pos) /*!< 0x00000010 */
#define FLASH_CR_SNB_2 (0x04UL << FLASH_CR_SNB_Pos) /*!< 0x00000020 */
#define FLASH_CR_SNB_3 (0x08UL << FLASH_CR_SNB_Pos) /*!< 0x00000040 */
#define FLASH_CR_SNB_4 (0x10UL << FLASH_CR_SNB_Pos) /*!< 0x00000080 */
#define FLASH_CR_PSIZE_Pos (8U)
#define FLASH_CR_PSIZE_Msk (0x3UL << FLASH_CR_PSIZE_Pos) /*!< 0x00000300 */
#define FLASH_CR_PSIZE FLASH_CR_PSIZE_Msk
#define FLASH_CR_PSIZE_0 (0x1UL << FLASH_CR_PSIZE_Pos) /*!< 0x00000100 */
#define FLASH_CR_PSIZE_1 (0x2UL << FLASH_CR_PSIZE_Pos) /*!< 0x00000200 */
#define FLASH_CR_STRT_Pos (16U)
#define FLASH_CR_STRT_Msk (0x1UL << FLASH_CR_STRT_Pos) /*!< 0x00010000 */
#define FLASH_CR_STRT FLASH_CR_STRT_Msk
#define FLASH_CR_EOPIE_Pos (24U)
#define FLASH_CR_EOPIE_Msk (0x1UL << FLASH_CR_EOPIE_Pos) /*!< 0x01000000 */
#define FLASH_CR_EOPIE FLASH_CR_EOPIE_Msk
#define FLASH_CR_LOCK_Pos (31U)
#define FLASH_CR_LOCK_Msk (0x1UL << FLASH_CR_LOCK_Pos) /*!< 0x80000000 */
#define FLASH_CR_LOCK FLASH_CR_LOCK_Msk

#define FLASH_CR_PSIZE_WORD (2UL << FLASH_CR_PSIZE_Pos)
// Look up table for MCU ID
struct {
  uint32_t mcuid;
  const char *name;
} _stm32_devices[] = {
    {0x413, "STM32F405xx/07xx and STM32F415xx/17xx"},
    {0x419, "STM32F42xxx and STM32F43xxx"},
    {0x431, "STM32F411xC/E"},
    {0x441, "STM32F412"},
};

enum {
  STM32_DEVICES_COUNT = sizeof(_stm32_devices) / sizeof(_stm32_devices[0])
};

// Flash Layout Organization
// STM32 auto map 0x00 to FLASH_START_ADDR
#define FLASH_START_ADDR 0x08000000

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+

Adafruit_DAP_STM32::Adafruit_DAP_STM32(void) {
  memset(&target_device, 0, sizeof(target_device));
}

bool Adafruit_DAP_STM32::select(uint32_t *found_id) {
  uint32_t mcuid;

  dap_target_prepare();

  // Stop the core
  dap_write_word(DHCSR, 0xa05f0003);
  dap_write_word(DEMCR, 0x00000001);
  dap_write_word(AIRCR, 0x05fa0004);

  target_device.flash_size = (dap_read_word(STM32_FLASHSIZE) >> 16) * 1024;

  *found_id = mcuid = (dap_read_word(DAP_DBGMCU_IDCODE) & 0xFFFUL);
  for (int i = 0; i < STM32_DEVICES_COUNT; i++) {
    if (mcuid == _stm32_devices[i].mcuid) {
      target_device.name = _stm32_devices[i].name;
      break;
    }
  }

  if (target_device.name == NULL)
    return false;

  return true;
}

void Adafruit_DAP_STM32::flash_unlock(void) {
  dap_write_word(FLASH_KEYR, 0x45670123); // key 1
  dap_write_word(FLASH_KEYR, 0xCDEF89AB); // key 2
}

void Adafruit_DAP_STM32::flash_lock(void) {
  dap_write_word(FLASH_CR, FLASH_CR_LOCK);
}

bool Adafruit_DAP_STM32::flash_busy(void) {
  return dap_read_word(FLASH_SR) & FLASH_SR_BSY;
}

void Adafruit_DAP_STM32::erase(void) {
  flash_unlock();

  while (flash_busy()) {
    yield();
  }

  // Mass erase with FLASH_VOLTAGE_RANGE_3 (32-bit operation)
  // Set MER bit ( and MER1 if STM32F42xxx and STM32F43xxx)
  // Set STRT bit
  dap_write_word(FLASH_CR, FLASH_CR_MER | FLASH_CR_STRT | FLASH_CR_PSIZE_WORD);

  // Mass erase take ~8-9 seconds
  while (flash_busy()) {
    delay(100);
  }

  flash_lock();
}

uint32_t Adafruit_DAP_STM32::program_start(uint32_t addr, uint32_t size) {
  if (addr >= FLASH_START_ADDR) {
    addr -= FLASH_START_ADDR;
  }

  // in unit of KBs
  const uint32_t flash_sectors[] = {
      // Bank 0 : sector 0-3 : 16 KB, sector 4: 64 KB, sector 5-11: 128 KB
      16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128,

      // Bank 1 : sector 0-3 : 16 KB, sector 4: 64 KB, sector 5-11: 128 KB
      16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128,

      0};

  const uint32_t sector_count =
      sizeof(flash_sectors) / sizeof(flash_sectors[0]);

  uint32_t sec_start = 0;
  uint32_t sec_end = 0;
  uint32_t tempaddr = 0;

  for (uint32_t i = 0; i < sector_count; i++) {
    if (tempaddr <= addr) {
      sec_start = i;
    }

    if ((addr + size) <= tempaddr) {
      sec_end = i - 1;
      break;
    }

    tempaddr += flash_sectors[i] * 1024;
  }

  flash_unlock();

  for (uint32_t i = sec_start; i <= sec_end; i++) {
    while (flash_busy()) {
      yield();
    }

    // Set SER, Sector Number, and PSize and Start bit
    dap_write_word(FLASH_CR, FLASH_CR_SER | FLASH_CR_STRT |
                                 (i << FLASH_CR_SNB_Pos) | FLASH_CR_PSIZE_WORD);
  }

  while (flash_busy()) {
    yield();
  }

  flash_lock();

  return addr;
}

void Adafruit_DAP_STM32::programBlock(uint32_t addr, const uint8_t *buf,
                                      uint32_t size) {
  if (!size) {
    return;
  }

  flash_unlock();

  while (flash_busy()) {
    yield();
  }

  // Enable programming bit: PG
  dap_write_word(FLASH_CR, FLASH_CR_PG | FLASH_CR_PSIZE_WORD);

  // Program the whole buffer
  dap_write_block(addr, buf, size);

  while (flash_busy()) {
    yield();
  }

  flash_lock();
}

bool Adafruit_DAP_STM32::verifyFlash(uint32_t addr, const uint8_t *data,
                                     uint32_t size) {
  uint8_t buf[4 * 1024];

  while (size) {
    uint32_t const count = min(size, sizeof(buf));

    dap_read_block(addr, buf, count);

    if (memcmp(data, buf, count)) {
      return false;
    }

    data += count;
    addr += count;
    size -= count;
  }

  return true;
}

bool Adafruit_DAP_STM32::programFlash(uint32_t addr, const uint8_t *buf, uint32_t count, bool do_verify) {
  programBlock(addr, buf, count);

  if (do_verify) {
    return verifyFlash(addr, buf, count);
  }

  return true;
}

void Adafruit_DAP_STM32::deselect(void) {
  flash_lock();

  dap_write_word(DEMCR, 0x00000000);
  dap_write_word(AIRCR, 0x05fa0004);
}

bool Adafruit_DAP_STM32::protectBoot(void) {
  return true;
}

bool Adafruit_DAP_STM32::unprotectBoot(void) {
  return true;
}

