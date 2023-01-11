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

#ifndef LIB_ADAFRUIT_DAP_H
#define LIB_ADAFRUIT_DAP_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*- DAP Types
 * -------------------------------------------------------------------*/
#define DAP_FREQ 16000000 // Hz

enum {
  ID_DAP_INFO = 0x00,
  ID_DAP_LED = 0x01,
  ID_DAP_CONNECT = 0x02,
  ID_DAP_DISCONNECT = 0x03,
  ID_DAP_TRANSFER_CONFIGURE = 0x04,
  ID_DAP_TRANSFER = 0x05,
  ID_DAP_TRANSFER_BLOCK = 0x06,
  ID_DAP_TRANSFER_ABORT = 0x07,
  ID_DAP_WRITE_ABORT = 0x08,
  ID_DAP_DELAY = 0x09,
  ID_DAP_RESET_TARGET = 0x0a,
  ID_DAP_SWJ_PINS = 0x10,
  ID_DAP_SWJ_CLOCK = 0x11,
  ID_DAP_SWJ_SEQUENCE = 0x12,
  ID_DAP_SWD_CONFIGURE = 0x13,
  ID_DAP_JTAG_SEQUENCE = 0x14,
  ID_DAP_JTAG_CONFIGURE = 0x15,
  ID_DAP_JTAG_IDCODE = 0x16,
};

enum {
  DAP_INFO_VENDOR = 0x01,
  DAP_INFO_PRODUCT = 0x02,
  DAP_INFO_SER_NUM = 0x03,
  DAP_INFO_FW_VER = 0x04,
  DAP_INFO_DEVICE_VENDOR = 0x05,
  DAP_INFO_DEVICE_NAME = 0x06,
  DAP_INFO_CAPABILITIES = 0xf0,
  DAP_INFO_PACKET_COUNT = 0xfe,
  DAP_INFO_PACKET_SIZE = 0xff,
};

enum {
  DAP_TRANSFER_APnDP = 1 << 0,
  DAP_TRANSFER_RnW = 1 << 1,
  DAP_TRANSFER_A2 = 1 << 2,
  DAP_TRANSFER_A3 = 1 << 3,
  DAP_TRANSFER_MATCH_VALUE = 1 << 4,
  DAP_TRANSFER_MATCH_MASK = 1 << 5,
};

enum {
  DAP_TRANSFER_INVALID = 0,
  DAP_TRANSFER_OK = 1 << 0,
  DAP_TRANSFER_WAIT = 1 << 1,
  DAP_TRANSFER_FAULT = 1 << 2,
  DAP_TRANSFER_ERROR = 1 << 3,
  DAP_TRANSFER_MISMATCH = 1 << 4,
  DAP_TRANSFER_NO_TARGET = 7,
};

enum {
  DAP_PORT_SWD = 1 << 0,
  DAP_PORT_JTAG = 1 << 1,
};

enum {
  DAP_SWJ_SWCLK_TCK = 1 << 0,
  DAP_SWJ_SWDIO_TMS = 1 << 1,
  DAP_SWJ_TDI = 1 << 2,
  DAP_SWJ_TDO = 1 << 3,
  DAP_SWJ_nTRST = 1 << 5,
  DAP_SWJ_nRESET = 1 << 7,
};

enum {
  DAP_OK = 0x00,
  DAP_ERROR = 0xff,
};

enum {
  SWD_DP_R_IDCODE = 0x00,
  SWD_DP_W_ABORT = 0x00,
  SWD_DP_R_CTRL_STAT = 0x04,
  SWD_DP_W_CTRL_STAT = 0x04, // When CTRLSEL == 0
  SWD_DP_W_WCR = 0x04,       // When CTRLSEL == 1
  SWD_DP_R_RESEND = 0x08,
  SWD_DP_W_SELECT = 0x08,
  SWD_DP_R_RDBUFF = 0x0c,
};

enum {
  SWD_AP_CSW = 0x00 | DAP_TRANSFER_APnDP,
  SWD_AP_TAR = 0x04 | DAP_TRANSFER_APnDP,
  SWD_AP_DRW = 0x0c | DAP_TRANSFER_APnDP,

  SWD_AP_DB0 = 0x00 | DAP_TRANSFER_APnDP, // 0x10
  SWD_AP_DB1 = 0x04 | DAP_TRANSFER_APnDP, // 0x14
  SWD_AP_DB2 = 0x08 | DAP_TRANSFER_APnDP, // 0x18
  SWD_AP_DB3 = 0x0c | DAP_TRANSFER_APnDP, // 0x1c

  SWD_AP_CFG = 0x04 | DAP_TRANSFER_APnDP,  // 0xf4
  SWD_AP_BASE = 0x08 | DAP_TRANSFER_APnDP, // 0xf8
  SWD_AP_IDR = 0x0c | DAP_TRANSFER_APnDP,  // 0xfc
};

/*- Definitions -------------------------------------------------------------*/
enum {
  FUSE_READ = (1 << 0),
  FUSE_WRITE = (1 << 1),
  FUSE_VERIFY = (1 << 2),
};

enum {
  M0_PLUS,
};

enum {
  DAP_TYPEID_GENERIC = 0,
  DAP_TYPEID_SAM, // Adafruit_DAP_SAM
  DAP_TYPEID_SAMX5, // SAM D51, E51
  DAP_TYPEID_NRF5X,
  DAP_TYPEID_STM32
};

typedef void (*ErrorHandler)(const char *error);

typedef struct {
  uint32_t dsu_did;
  const char *name;
  uint32_t flash_size;
  uint32_t n_pages;
} device_t;

// Generic DAP
class Adafruit_DAP {
public:
  // constructors
  Adafruit_DAP(void){};
  virtual ~Adafruit_DAP(void){};
  bool begin(int swclk, int swdio, int nreset, ErrorHandler perror);

  // High level methods
  bool targetConnect(uint32_t swj_clock = 50);

  // Low level methods
  bool dap_led(int index, int state);
  bool dap_connect(void);
  bool dap_disconnect(void);
  bool dap_swj_clock(uint32_t clock);
  bool dap_transfer_configure(uint8_t idle, uint16_t count, uint16_t retry);
  bool dap_swd_configure(uint8_t cfg);
  bool dap_get_debugger_info(char *debuggername);
  bool dap_reset_target(void);
  bool dap_reset_target_hw(int state);
  uint32_t dap_read_reg(uint8_t reg);
  bool dap_write_reg(uint8_t reg, uint32_t data);
  uint32_t dap_read_word(uint32_t addr);
  bool dap_write_word(uint32_t addr, uint32_t data);
  bool dap_read_block(uint32_t addr, uint8_t *data, int size);
  bool dap_write_block(uint32_t addr, const uint8_t *data, int size);
  bool dap_reset_link(void);
  bool dap_read_idcode(uint32_t *id);
  bool dap_target_prepare(void);
  void dap_set_target(uint8_t target);
  void dap_set_clock(uint32_t clock);

  char *error_message;
  device_t target_device;

  //------------- Common API -------------//
  virtual uint32_t getTypeID(void) = 0;
  virtual bool select(uint32_t *id) = 0;
  virtual void deselect(void) = 0;

  // erase all chip
  virtual void erase(void) = 0;

  // prepare to program flash
  virtual uint32_t program_start(uint32_t addr, uint32_t size) = 0;

  // program a block of flash
  virtual void programBlock(uint32_t addr, const uint8_t *buf, uint32_t size) = 0;

  // program to flash with (without erase)
  virtual bool programFlash(uint32_t addr, const uint8_t *buf, uint32_t count, bool do_verify) = 0;

  // compute flash crc32
  virtual uint32_t computeFlashCRC32(uint32_t addr, uint32_t size);

  // unlock bootloader
  virtual bool protectBoot(void) = 0;

  // lock bootloader
  virtual bool unprotectBoot(void) = 0;

protected:
  uint8_t _i2caddr;
  bool dbg_dap_cmd(uint8_t *data, int size, int rsize);
  void check(bool cond, char *fmt);

  ErrorHandler perror_exit;
};

// Simple and low code CRC calculation (copied from arduino-rp2040's PicoOTA)
class Adafruit_DAP_CRC32 {
public:
  Adafruit_DAP_CRC32() { crc = 0xffffffff; }

  ~Adafruit_DAP_CRC32() {}

  void add(const void *d, uint32_t len) {
    const uint8_t *data = (const uint8_t *)d;
    for (uint32_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 1) {
          crc = (crc >> 1) ^ 0xedb88320;
        } else {
          crc >>= 1;
        }
      }
    }
  }

  uint32_t get() { return ~crc; }

private:
  uint32_t crc;
};


#include "Adafruit_DAP_SAM.h"
#include "Adafruit_DAP_STM32.h"
#include "Adafruit_DAP_nRF5x.h"

#endif
