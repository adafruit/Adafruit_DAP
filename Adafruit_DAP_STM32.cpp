/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>, Ha Thach for Adafruit <info@adafruit.com>
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

#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Adafruit_DAP.h"
#include "dap.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define DHCSR             0xe000edf0
#define DEMCR             0xe000edfc
#define AIRCR             0xe000ed0c

#define DAP_DBGMCU_IDCODE 0xE0042000
#define STM32_FLASHSIZE   0x1FFF7A22

// Flash interface registers
#define FLASH_R_BASE      0x40023C00UL
#define FLASH_ACR         (FLASH_R_BASE + 0x00)
#define FLASH_KEYR        (FLASH_R_BASE + 0x04)
#define FLASH_OPTKEYR     (FLASH_R_BASE + 0x08)
#define FLASH_SR          (FLASH_R_BASE + 0x0C)
#define FLASH_CR          (FLASH_R_BASE + 0x10)
#define FLASH_OPTCR       (FLASH_R_BASE + 0x14)
#define FLASH_OPTCR1      (FLASH_R_BASE + 0x18)

// Look up table for MCU ID
struct
{
  uint32_t    mcuid;
  const char* name;
} _stm32_devices[] =
{
  { 0x413, "STM32F405xx/07xx and STM32F415xx/17xx"},
  { 0x419, "STM32F42xxx and STM32F43xxx"},
  { 0x431, "STM32F411xC/E"},
  { 0x441, "STM32F412" },
};

enum { STM32_DEVICES_COUNT = sizeof(_stm32_devices)/sizeof(_stm32_devices[0]) };

Adafruit_DAP_STM32::Adafruit_DAP_STM32(void)
{
  memset(&target_device, 0, sizeof(target_device));
}

bool Adafruit_DAP_STM32::select(uint32_t *found_id)
{
  uint32_t mcuid;

  // Stop the core
  dap_write_word(DHCSR, 0xa05f0003);
  dap_write_word(DEMCR, 0x00000001);
  dap_write_word(AIRCR, 0x05fa0004);

  target_device.flash_size = (dap_read_word(STM32_FLASHSIZE) >> 16) * 1024;

  *found_id = mcuid = (dap_read_word(DAP_DBGMCU_IDCODE) & 0xFFFUL);
  for(int i=0; i<STM32_DEVICES_COUNT; i++)
  {
    if ( mcuid == _stm32_devices[i].mcuid )
    {
      target_device.name = _stm32_devices[i].name;
      break;
    }
  }

  if (target_device.name == NULL) return false;

  // unlock FLASH_CS access
  flash_unlock();

  return true;
}

bool Adafruit_DAP_STM32::flash_unlock(void)
{
  dap_write_word(FLASH_KEYR, 0x45670123); // key 1
  dap_write_word(FLASH_KEYR, 0xCDEF89AB); // key 2

  return 0 == (dap_read_word(FLASH_CR) & (1UL << 31));
}

bool Adafruit_DAP_STM32::flash_busy(void)
{
  return (dap_read_word(FLASH_SR) >> 16) & 0x01UL;
}

void Adafruit_DAP_STM32::erase(void)
{
  while ( flash_busy() ) delay(1);

  // Mass erase with FLASH_VOLTAGE_RANGE_3 (32-bit operation)
  // Set MER bit ( and MER1 if STM32F42xxx and STM32F43xxx)
  // Set STRT bit
  dap_write_word(FLASH_CR, 0x10004 | (2UL << 8));

  while ( flash_busy() ) delay(100);
}

void Adafruit_DAP_STM32::deselect(void)
{
  dap_write_word(DEMCR, 0x00000000);
  dap_write_word(AIRCR, 0x05fa0004);
}
