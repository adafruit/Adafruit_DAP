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

/* The number of bytes to write at one time in program(). */
#define CHUNK_SIZE  (1024)

/*- Definitions -------------------------------------------------------------*/
#define NRF5X_FLASH_START            0
#define NRF5X_FLASH_ROW_SIZE         256
#define NRF5X_FLASH_PAGE_SIZE        64

#define NRF5X_DHCSR                  0xe000edf0
#define NRF5X_DEMCR                  0xe000edfc
#define NRF5X_AIRCR                  0xe000ed0c

#define NRF5X_FICR_CODEPAGESIZE      0x10000010     // Code memory page size
#define NRF5X_FICR_CODESIZE          0x10000014     // Code size (in pages)
#define NRF5X_FICR_HWID              0x10000100     // Part Code
#define NRF5X_FICR_CHIPVARIANT       0x10000104     // Part Variant
#define NRF5X_FICR_PACKAGEID         0x10000108     // Package Options
#define NRF5X_FICR_SRAM              0x1000010C     // RAM Variant
#define NRF5X_FICR_FLASHSIZE         0x10000110     // Flash Variant

// TODO: Change these from SAMD to nRF5x compatible registers!
#define NRF5X_DSU_CTRL_STATUS        0x41002100

#define NRF5X_NVMCTRL_CTRLA          0x41004000
#define NRF5X_NVMCTRL_CTRLB          0x41004004
#define NRF5X_NVMCTRL_PARAM          0x41004008
#define NRF5X_NVMCTRL_INTFLAG        0x41004014
#define NRF5X_NVMCTRL_STATUS         0x41004018
#define NRF5X_NVMCTRL_ADDR           0x4100401c

#define NRF5X_NVMCTRL_CMD_ER         0xa502
#define NRF5X_NVMCTRL_CMD_WP         0xa504
#define NRF5X_NVMCTRL_CMD_EAR        0xa505
#define NRF5X_NVMCTRL_CMD_WAP        0xa506
#define NRF5X_NVMCTRL_CMD_WL         0xa50f
#define NRF5X_NVMCTRL_CMD_UR         0xa541
#define NRF5X_NVMCTRL_CMD_PBC        0xa544
#define NRF5X_NVMCTRL_CMD_SSB        0xa545

#define NRF5X_USER_ROW_ADDR          0x00804000
#define NRF5X_USER_ROW_SIZE          256


// from nrf52.h

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

/**
  * @brief Non Volatile Memory Controller (NVMC)
  */

typedef struct {                                    /*!< NVMC Structure                                                        */
  __I  uint32_t  RESERVED0[256];
  __I  uint32_t  READY;                             /*!< Ready flag                                                            */
  __I  uint32_t  RESERVED1[64];
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */

  union {
    __IO uint32_t  ERASEPCR1;                       /*!< Deprecated register - Register for erasing a page in Code area.
                                                         Equivalent to ERASEPAGE.                                              */
    __IO uint32_t  ERASEPAGE;                       /*!< Register for erasing a page in Code area                              */
  };
  __IO uint32_t  ERASEALL;                          /*!< Register for erasing all non-volatile user memory                     */
  __IO uint32_t  ERASEPCR0;                         /*!< Deprecated register - Register for erasing a page in Code area.
                                                         Equivalent to ERASEPAGE.                                              */
  __IO uint32_t  ERASEUICR;                         /*!< Register for erasing User Information Configuration Registers         */
  __I  uint32_t  RESERVED2[10];
  __IO uint32_t  ICACHECNF;                         /*!< I-Code cache configuration register.                                  */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  IHIT;                              /*!< I-Code cache hit counter.                                             */
  __IO uint32_t  IMISS;                             /*!< I-Code cache miss counter.                                            */
} NRF_NVMC_Type;

#define NRF_NVMC_BASE                   0x4001E000UL
#define NRF_NVMC                        ((NRF_NVMC_Type           *) NRF_NVMC_BASE)

/**
 * Flash algorithm below source: ARM DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 */
static const uint32_t nRF52832AA_FLM[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x47702000, 0x47702000, 0x4c2bb570, 0x60202002, 0x20014929, 0x60083108, 0x68284d28, 0xd00207c0,
    0x60202000, 0xf000bd70, 0xe7f6f833, 0x4c22b570, 0x60212102, 0x2f10f1b0, 0x491fd303, 0x31102001,
    0x491de001, 0x60081d09, 0xf0004d1c, 0x6828f821, 0xd0fa07c0, 0x60202000, 0xe92dbd70, 0xf8df41f0,
    0x088e8058, 0x46142101, 0xf8c84605, 0x4f131000, 0xc501cc01, 0x07c06838, 0x1e76d007, 0x2100d1f8,
    0x1000f8c8, 0xe8bd4608, 0xf00081f0, 0xe7f1f801, 0x6800480b, 0x00fff010, 0x490ad00c, 0x29006809,
    0x4908d008, 0x31fc4a08, 0xd00007c3, 0x1d09600a, 0xd1f90840, 0x00004770, 0x4001e504, 0x4001e400,
    0x40010404, 0x40010504, 0x6e524635, 0x00000000,
};

#define NRF52840_FLASH_START        (0x00000000)
#define NRF52840_FLASHALGO_START    (0X20000000)
#define NRF52840_FLASHALGO_SIZE     (0x150)
#define NRF52840_FLASHALGO_INIT     (0x20000021)
#define NRF52840_FLASHALGO_STATBASE (0x20000020 + NRF52840_FLASHALGO_SIZE)
#define NRF52840_FLASHALGO_STCKPNTR (0x20001000)
#define NRF52840_FLASHALGO_BRKPOINT (0x20000001)

//-----------------------------------------------------------------------------
bool Adafruit_DAP_nRF5x::select(uint32_t *found_id)
{
  uint32_t hwid;
  uint32_t chipvariant;
  uint32_t codepagesize;
  uint32_t codesize;
  uint32_t sram;

  // Stop the core
  dap_write_word(NRF5X_DHCSR, 0xa05f0003);
  dap_write_word(NRF5X_DEMCR, 0x00000001);
  dap_write_word(NRF5X_AIRCR, 0x05fa0004);

  hwid = dap_read_word(NRF5X_FICR_HWID);

  *found_id = hwid;

  if (hwid == 0x52832)
  {
      // Read other relevant registers
      chipvariant = dap_read_word(NRF5X_FICR_CHIPVARIANT);
      codepagesize = dap_read_word(NRF5X_FICR_CODEPAGESIZE);
      codesize = dap_read_word(NRF5X_FICR_CODESIZE);
      //sram = dap_read_word(NRF5X_FICR_SRAM);

      // Assign device details to target_device
      target_device.dsu_did = hwid;
      target_device.flash_size = codepagesize * codesize;
      target_device.n_pages = codesize;
      switch (chipvariant)
      {
          case 0x41414141:
            target_device.name = "nRF52832_AAAA";
            break;
          case 0x41414142:
            target_device.name = "nRF52832_AAAB";
            break;
          case 0x41414241:
            target_device.name = "nRF52832_AABA";
            break;
          case 0x41414242:
            target_device.name = "nRF52832_AABB";
            break;
          case 0x41414230:
            target_device.name = "nRF52832_AAB0";
            break;
          default:
            target_device.name = "nRF52832_????";
            break;
      }
  }
  else if (hwid == 0x52840)
  {
      // TODO: Add nRF52840 support!
      // Read other relevant registers
      chipvariant = dap_read_word(NRF5X_FICR_CHIPVARIANT);
      codepagesize = dap_read_word(NRF5X_FICR_CODEPAGESIZE);
      codesize = dap_read_word(NRF5X_FICR_CODESIZE);
      //sram = dap_read_word(NRF5X_FICR_SRAM);

      // Assign device details to target_device
      target_device.dsu_did = hwid;
      target_device.flash_size = codepagesize * codesize;
      target_device.n_pages = codesize;
      switch (chipvariant)
      {
          case 0x41414141:
            target_device.name = "nRF52840_AAAA";
            break;
          case 0x42414141:
            target_device.name = "nRF52840_BAAA";
            break;
          case 0x41314141:
            target_device.name = "nRF52840_CAAA";
            break;
          case 0x41414241:
            target_device.name = "nRF52840_AABA";
            break;
          case 0x41414242:
            target_device.name = "nRF52840_AABB";
            break;
          case 0x41414341:
            target_device.name = "nRF52840_AACA";
            break;
          case 0x41414142:
            target_device.name = "nRF52840_AAAB";
            break;
          case 0x41414330:
            target_device.name = "nRF52840_AAC0";
            break;
          default:
            target_device.name = "nRF52840_????";
            break;
      }
  }
  else
  {
      // No matching device ID found
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::deselect(void)
{
  dap_write_word(NRF5X_DEMCR, 0x00000000);
  dap_write_word(NRF5X_AIRCR, 0x05fa0004);
}

bool Adafruit_DAP_nRF5x::flashWaitReady(void)
{
  int i;

  for (i = 0; i < 100000; i++) {
      if (dap_read_word((uint32_t)&NRF_NVMC->READY) & 1UL) {
          return true;
      }
  }
  return false;
}

bool Adafruit_DAP_nRF5x::flashReady(void)
{
  return dap_read_word((uint32_t)&NRF_NVMC->READY) & 1UL;
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::erase(void)
{
  dap_write_word( (uint32_t) &NRF_NVMC->CONFIG, 2); // Erase Enable
  dap_write_word( (uint32_t) &NRF_NVMC->ERASEALL, 1); // Erase All

  while ( !flashReady() ) { }

  dap_write_word( (uint32_t) &NRF_NVMC->CONFIG, 0);   // Disable Erase
}

void Adafruit_DAP_nRF5x::erasePage(uint32_t page)
{

}

void Adafruit_DAP_nRF5x::eraseUICR(void)
{

}

void Adafruit_DAP_nRF5x::eraseFICR(void)
{

}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP_nRF5x::program_start(uint32_t offset)
{
//  if (dap_read_word(NRF5X_DSU_CTRL_STATUS) & 0x00010000)
//    perror_exit("device is locked, perform a chip erase before programming");

  //TODO: comvert to slow/fast clock mode
  dap_setup_clock(0);

  return NRF5X_FLASH_START + offset;
}

// typedef struct {
//     uint32_t r[16];
//     uint32_t xpsr;
// } DEBUG_STATE;
//
// /**
//  * swd_write_debug_state source: ARM DAPLink Interface Firmware
//  * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
//  * SPDX-License-Identifier: Apache-2.0
//  */
// static uint8_t Adafruit_DAP_nRF5x::swd_write_debug_state(DEBUG_STATE *state)
// {
//     uint32_t i, status;
//
//     if (!swd_write_dp(DP_SELECT, 0)) {
//         return 0;
//     }
//
//     // R0, R1, R2, R3
//     for (i = 0; i < 4; i++) {
//         if (!swd_write_core_register(i, state->r[i])) {
//             return 0;
//         }
//     }
//
//     // R9
//     if (!swd_write_core_register(9, state->r[9])) {
//         return 0;
//     }
//
//     // R13, R14, R15
//     for (i = 13; i < 16; i++) {
//         if (!swd_write_core_register(i, state->r[i])) {
//             return 0;
//         }
//     }
//
//     // xPSR
//     if (!swd_write_core_register(16, state->xpsr)) {
//         return 0;
//     }
//
//     if (!swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
//         return 0;
//     }
//
//     // check status
//     if (!swd_read_dp(DP_CTRL_STAT, &status)) {
//         return 0;
//     }
//
//     if (status & (STICKYERR | WDATAERR)) {
//         return 0;
//     }
//
//     return 1;
// }
//
// /**
//  * swd_flash_syscall_exec source: ARM DAPLink Interface Firmware
//  * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
//  * SPDX-License-Identifier: Apache-2.0
//  */
// static bool Adafruit_DAP_nRF5x::swd_flash_syscall_exec(uint32_t entry,
//     uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
// {
//     DEBUG_STATE state = {{0}, 0};
//     // Call flash algorithm function on target and wait for result.
//     // NOTE: ARM devices store the first four function args in R0..3
//     state.r[0]     = arg1;                          // R0: Argument 1
//     state.r[1]     = arg2;                          // R1: Argument 2
//     state.r[2]     = arg3;                          // R2: Argument 3
//     state.r[3]     = arg4;                          // R3: Argument 4
//     state.r[9]     = NRF52840_FLASHALGO_STATBASE;   // SB: Static Base
//     state.r[13]    = NRF52840_FLASHALGO_STCKPNTR;   // SP: Stack Pointer
//     state.r[14]    = NRF52840_FLASHALGO_BRKPOINT;   // LR: Exit Point
//     state.r[15]    = entry;                         // PC: Entry Point
//     state.xpsr     = 0x01000000;                    // xPSR: T = 1, ISR = 0
//
//     if (!swd_write_debug_state(&state)) {
//         return false;
//     }
//
//     if (!swd_wait_until_halted()) {
//         return false;
//     }
//
//     if (!swd_read_core_register(0, &state.r[0])) {
//         return false;
//     }
//
//     // Flash functions return 0 if successful.
//     if (state.r[0] != 0) {
//         return false;
//     }
//
//     return true;
// }

bool Adafruit_DAP_nRF5x::program(uint32_t addr, const uint8_t* buf, uint32_t count)
{
  // /* Reset and write flash algorithm to SRAM */
  // Serial.println("Resetting target and writing flash algorithm to SRAM ...");
  // startMillis = millis();
  // dap_reset_target();
  // dap_write_block((uint32_t)NRF52840_FLASHALGO_START,
  //                 (uint8_t *)nRF52832AA_FLM,
  //                 (int)NRF52840_FLASHALGO_SIZE);
  // Serial.print("Done in "); Serial.print(millis() - startMillis); Serial.println(" ms");
  //
  // /* Execute the flash algorithm */
  // if (false == swd_flash_syscall_exec(NRF52840_FLASHALGO_INIT,
  //                                     NRF52840_FLASH_START,
  //                                     0, 0, 0)) {
  //   /* Flash algorithm init error! */
  //   return false;
  // }

  // address must be word-aligned
  if ( addr & 0x03 ) return false;

  dap_write_word( (uint32_t) &NRF_NVMC->CONFIG, 1); // Write Enable

  while(count)
  {
    uint8_t data[CHUNK_SIZE];
    uint32_t bytes = min(count, CHUNK_SIZE);
    uint32_t sum = 0;
    bool hasdata = false;

    memset(data, 0xFF, CHUNK_SIZE);
    memcpy(data, buf, bytes);

    /* If data is all 0x00, this chunk is empty. Don't bother writing it since
     * we've already erased the flash memory. */
    for (uint32_t i = 0; i < CHUNK_SIZE; i++) {
        if (data[i] != 0x00)
        {
            hasdata = true;
        }
    }
    if (hasdata) {
        dap_write_block(addr, data, (int)bytes);
    }

    addr  += bytes;
    buf   += bytes;
    count -= bytes;

    if (!flashWaitReady()) {
        // Flash timed out before being ready!
        return false;
    };
  }

  dap_write_word( (uint32_t) &NRF_NVMC->CONFIG, 0); // Write Disable

  return true;
}

void Adafruit_DAP_nRF5x::programUICR(uint32_t addr, uint32_t value)
{
  dap_write_word( (uint32_t) &NRF_NVMC->CONFIG, 1); // Write Enable
  dap_write_word(addr, value);
  while ( !flashReady() ) {  }
  dap_write_word( (uint32_t) &NRF_NVMC->CONFIG, 0); // Write Disable
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::lock(void)
{
  dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_SSB); // Set Security Bit
}



void Adafruit_DAP_nRF5x::programBlock(uint32_t addr, uint8_t *buf)
{
    dap_write_word(NRF5X_NVMCTRL_ADDR, addr >> 1);

    dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_UR); // Unlock Region
    while (0 == (dap_read_word(NRF5X_NVMCTRL_INTFLAG) & 1));

    dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_ER); // Erase Row
    while (0 == (dap_read_word(NRF5X_NVMCTRL_INTFLAG) & 1));
    dap_write_block(addr, buf, NRF5X_FLASH_ROW_SIZE);
}

//-----------------------------------------------------------------------------
void Adafruit_DAP_nRF5x::readBlock(uint32_t addr, uint8_t *buf)
{
  if (dap_read_word(NRF5X_DSU_CTRL_STATUS) & 0x00010000)
    perror_exit("device is locked, unable to read");

  dap_read_block(addr, buf, NRF5X_FLASH_ROW_SIZE);
}

void Adafruit_DAP_nRF5x::fuseRead(){
  uint8_t buf[NRF5X_USER_ROW_SIZE];
  dap_read_block(NRF5X_USER_ROW_ADDR, buf, NRF5X_USER_ROW_SIZE);

  uint64_t fuses = ((uint64_t)buf[7] << 56) |
          ((uint64_t)buf[6] << 48) |
          ((uint64_t)buf[5] << 40) |
          ((uint64_t)buf[4] << 32) |
          ((uint64_t)buf[3] << 24) |
          ((uint64_t)buf[2] << 16) |
          ((uint64_t)buf[1] << 8) |
          (uint64_t)buf[0];

  _USER_ROW.set(fuses);
}

void Adafruit_DAP_nRF5x::fuseWrite()
{
  uint64_t fuses = _USER_ROW.get();
  uint8_t buf[NRF5X_USER_ROW_SIZE] = {(uint8_t)fuses,
      (uint8_t)(fuses >> 8),
      (uint8_t)(fuses >> 16),
      (uint8_t)(fuses >> 24),
      (uint8_t)(fuses >> 32),
      (uint8_t)(fuses >> 40),
      (uint8_t)(fuses >> 48),
      (uint8_t)(fuses >> 56)
    };

  dap_write_word(NRF5X_NVMCTRL_CTRLB, 0);
  dap_write_word(NRF5X_NVMCTRL_ADDR, NRF5X_USER_ROW_ADDR >> 1);
  dap_write_word(NRF5X_NVMCTRL_CTRLA, NRF5X_NVMCTRL_CMD_EAR);
  while (0 == (dap_read_word(NRF5X_NVMCTRL_INTFLAG) & 1));

  dap_write_block(NRF5X_USER_ROW_ADDR, buf, NRF5X_USER_ROW_SIZE);
}
