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

#include "Adafruit_DAP.h"
#include "dap.h"
#include "dap_config.h"

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif

static uint8_t hid_buffer[256];
#define REPORT_SIZE 64

bool Adafruit_DAP::begin(int swclk, int swdio, int nreset, ErrorHandler perr) {
  perror_exit = perr;
  return dap_init(swclk, swdio, nreset);
  // dap_init(swclk, swdio, nreset);
  // dap_clock_test(0);
  // return true;
}

bool Adafruit_DAP::targetConnect(uint32_t swj_clock) {
  // First disconnect, in case this really is e reconnect
  if (!dap_disconnect()) return false;
  if (!dap_connect()) return false;
  if (!dap_transfer_configure(0, 128, 128)) return false;
  if (!dap_swd_configure(0))                return false;
  if (!dap_swj_clock(swj_clock))            return false;
  if (!dap_reset_link())                    return false;

  return true;
}

bool Adafruit_DAP::dbg_dap_cmd(uint8_t *data, int size, int rsize) {
  (void) rsize;

  // TODO: leaving off here, we can only write 64 bytes at a time
  char cmd = data[0];

  memset(hid_buffer, 0xff, REPORT_SIZE + 1);

  noInterrupts();
  dap_process_request(data, hid_buffer);
  interrupts();

  if (hid_buffer[0] != cmd) {
    error_message = (char *)"invalid response received";
    return false;
  }

  memcpy(data, &hid_buffer[1], size);

  return true;
}

void Adafruit_DAP::check(bool cond, char *fmt) {
  if (!cond) {
    // va_list args;

    // dbg_close();

    perror_exit(fmt);
  }
}

/*- DAP Implementations
 * ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_led(int index, int state) {
  uint8_t buf[3];

  buf[0] = ID_DAP_LED;
  buf[1] = index;
  buf[2] = state;
  if (!dbg_dap_cmd(buf, sizeof(buf), 3))
    return false;

  if (DAP_OK != buf[0]) {
    error_message = (char *)"DAP_LED failed";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_connect(void) {
  uint8_t buf[2];

  buf[0] = ID_DAP_CONNECT;
  buf[1] = DAP_PORT_SWD;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;

  if (DAP_PORT_SWD != buf[0]) {
    error_message = (char *)"DAP_CONNECT failed";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_disconnect(void) {
  uint8_t buf[1];

  buf[0] = ID_DAP_DISCONNECT;
  if (!dbg_dap_cmd(buf, sizeof(buf), 1)) {
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_swj_clock(uint32_t clock) {
  uint8_t buf[5];

  buf[0] = ID_DAP_SWJ_CLOCK;
  buf[1] = clock & 0xff;
  buf[2] = (clock >> 8) & 0xff;
  buf[3] = (clock >> 16) & 0xff;
  buf[4] = (clock >> 24) & 0xff;
  if (!dbg_dap_cmd(buf, sizeof(buf), 5)) {
    return false;
  }

  if (DAP_OK != buf[0]) {
    error_message = (char *)"SWJ_CLOCK failed";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_transfer_configure(uint8_t idle, uint16_t count,
                                          uint16_t retry) {
  uint8_t buf[6];

  buf[0] = ID_DAP_TRANSFER_CONFIGURE;
  buf[1] = idle;
  buf[2] = count & 0xff;
  buf[3] = (count >> 8) & 0xff;
  buf[4] = retry & 0xff;
  buf[5] = (retry >> 8) & 0xff;
  if (!dbg_dap_cmd(buf, sizeof(buf), 6)) {
    return false;
  }

  if (DAP_OK != buf[0]) {
    error_message = (char *)"TRANSFER_CONFIGURE failed";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_swd_configure(uint8_t cfg) {
  uint8_t buf[2];

  buf[0] = ID_DAP_SWD_CONFIGURE;
  buf[1] = cfg;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;

  if (DAP_OK != buf[0]) {
    error_message = (char *)"SWD_CONFIGURE failed";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_get_debugger_info(char *str) {
  uint8_t buf[100];

  str[0] = 0;

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_VENDOR;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_PRODUCT;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_SER_NUM;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_FW_VER;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;
  strncat(str, (char *)&buf[1], buf[0]);
  strcat(str, " ");

  buf[0] = ID_DAP_INFO;
  buf[1] = DAP_INFO_CAPABILITIES;
  if (!dbg_dap_cmd(buf, sizeof(buf), 2))
    return false;

  strcat(str, "(");

  if (buf[1] & DAP_PORT_SWD)
    strcat(str, "S");

  if (buf[1] & DAP_PORT_JTAG)
    strcat(str, "J");

  strcat(str, ")");

  if (!(buf[1] & DAP_PORT_SWD)) {
    error_message = (char *)"SWD support required";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_reset_target(void) {
  uint8_t buf[1];

  buf[0] = ID_DAP_RESET_TARGET;
  if (!dbg_dap_cmd(buf, sizeof(buf), 1))
    return false;

  if (DAP_OK != buf[0]) {
    error_message = (char *)"RESET_TARGET failed";
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_reset_target_hw(int state) {
  uint8_t buf[7];
  int value = state ? (DAP_SWJ_SWCLK_TCK | DAP_SWJ_SWDIO_TMS) : 0;

  //-------------
  buf[0] = ID_DAP_SWJ_PINS;
  buf[1] = value;                                                  // Value
  buf[2] = DAP_SWJ_nRESET | DAP_SWJ_SWCLK_TCK | DAP_SWJ_SWDIO_TMS; // Select
  buf[3] = 0;                                                      // Wait
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = 0;
  if (!dbg_dap_cmd(buf, sizeof(buf), 7))
    return false;

  delay(10);

  //-------------
  buf[0] = ID_DAP_SWJ_PINS;
  buf[1] = DAP_SWJ_nRESET | value;                                 // Value
  buf[2] = DAP_SWJ_nRESET | DAP_SWJ_SWCLK_TCK | DAP_SWJ_SWDIO_TMS; // Select
  buf[3] = 0;                                                      // Wait
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = 0;
  if (!dbg_dap_cmd(buf, sizeof(buf), 7))
    return false;

  return true;
}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP::dap_read_reg(uint8_t reg) {
  uint8_t buf[8];

  buf[0] = ID_DAP_TRANSFER;
  buf[1] = 0x00; // DAP index
  buf[2] = 0x01; // Request size
  buf[3] = reg | DAP_TRANSFER_RnW;
  dbg_dap_cmd(buf, sizeof(buf), 4);

  if (1 != buf[0] || DAP_TRANSFER_OK != buf[1]) {
    // error_exit("invalid response while reading the register 0x%02x (count =
    // %d, value = %d)",
    //    reg, buf[0], buf[1]);
    Serial.print("invalid response reading reg ");
    Serial.print(reg, HEX);

    Serial.print(" (count = ");
    Serial.print(buf[0]);
    Serial.print(", value = ");
    Serial.print(buf[1]);
    perror_exit(")");
  }

  return ((uint32_t)buf[5] << 24) | ((uint32_t)buf[4] << 16) |
         ((uint32_t)buf[3] << 8) | (uint32_t)buf[2];
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_write_reg(uint8_t reg, uint32_t data) {
  uint8_t buf[8];

  buf[0] = ID_DAP_TRANSFER;
  buf[1] = 0x00; // DAP index
  buf[2] = 0x01; // Request size
  buf[3] = reg;
  buf[4] = data & 0xff;
  buf[5] = (data >> 8) & 0xff;
  buf[6] = (data >> 16) & 0xff;
  buf[7] = (data >> 24) & 0xff;
  dbg_dap_cmd(buf, sizeof(buf), 8);

  if (1 != buf[0] || DAP_TRANSFER_OK != buf[1]) {
    Serial.print("invalid response writing to reg ");
    Serial.print(reg, HEX);

    Serial.print(" (count = ");
    Serial.print(buf[0]);
    Serial.print(", value = ");
    Serial.print(buf[1]);
    perror_exit(")");
  }
  return true;
}

//-----------------------------------------------------------------------------
uint32_t Adafruit_DAP::dap_read_word(uint32_t addr) {
  dap_write_reg(SWD_AP_TAR, addr);
  return dap_read_reg(SWD_AP_DRW);
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_write_word(uint32_t addr, uint32_t data) {
  dap_write_reg(SWD_AP_TAR, addr);
  dap_write_reg(SWD_AP_DRW, data);
  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_read_block(uint32_t addr, uint8_t *data, int size) {
  int max_size = (32 - 5) & ~3;
  int offs = 0;

  while (size) {
    int align, sz;
    uint8_t buf[1024];

    align = 0x400 - (addr - (addr & ~0x3ff));
    sz = (size > max_size) ? max_size : size;
    sz = (sz > align) ? align : sz;

    dap_write_reg(SWD_AP_TAR, addr);

    buf[0] = ID_DAP_TRANSFER_BLOCK;
    buf[1] = 0x00; // DAP index
    buf[2] = (sz / 4) & 0xff;
    buf[3] = ((sz / 4) >> 8) & 0xff;
    buf[4] = SWD_AP_DRW | DAP_TRANSFER_RnW | DAP_TRANSFER_APnDP;
    dbg_dap_cmd(buf, sizeof(buf), 5);

    if (DAP_TRANSFER_OK != buf[2]) {
      // error_exit("invalid response while reading the block at 0x%08x (value =
      // %d)",
      //    addr, buf[2]);
      Serial.println("invalid response while reading the block ");
      Serial.println(buf[2]);
    }

    memcpy(&data[offs], &buf[3], sz);

    size -= sz;
    addr += sz;
    offs += sz;
  }
  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_write_block(uint32_t addr, const uint8_t *data,
                                   int size) {
  int max_size = (512 - 5) & ~3;
  int offs = 0;

  while (size) {
    int align, sz;
    uint8_t buf[1024];

    align = 0x400 - (addr - (addr & ~0x3ff));
    sz = (size > max_size) ? max_size : size;
    sz = (sz > align) ? align : sz;

    dap_write_reg(SWD_AP_TAR, addr);

    buf[0] = ID_DAP_TRANSFER_BLOCK;
    buf[1] = 0x00; // DAP index
    buf[2] = (sz / 4) & 0xff;
    buf[3] = ((sz / 4) >> 8) & 0xff;
    buf[4] = SWD_AP_DRW | DAP_TRANSFER_APnDP;
    memcpy(&buf[5], &data[offs], sz);
    dbg_dap_cmd(buf, 5 + sz, 0);

    if (DAP_TRANSFER_OK != buf[2]) {
      // error_exit("invalid response while writing the block at 0x%08x (value =
      // %d)",
      //    addr, buf[2]);
      Serial.println("invalid response while writing the block ");
      Serial.println(buf[2]);
    }

    size -= sz;
    addr += sz;
    offs += sz;
  }
  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_reset_link(void) {
  uint8_t buf[128];

  //-------------
  buf[0] = ID_DAP_SWJ_SEQUENCE;
  buf[1] = (7 + 2 + 7 + 1) * 8;
  buf[2] = 0xff;
  buf[3] = 0xff;
  buf[4] = 0xff;
  buf[5] = 0xff;
  buf[6] = 0xff;
  buf[7] = 0xff;
  buf[8] = 0xff;
  buf[9] = 0x9e;
  buf[10] = 0xe7;
  buf[11] = 0xff;
  buf[12] = 0xff;
  buf[13] = 0xff;
  buf[14] = 0xff;
  buf[15] = 0xff;
  buf[16] = 0xff;
  buf[17] = 0xff;
  buf[18] = 0x00;

  dbg_dap_cmd(buf, sizeof(buf), 19);
  check(DAP_OK == buf[0], (char *)"SWJ_SEQUENCE failed");

  //-------------
  buf[0] = ID_DAP_TRANSFER;
  buf[1] = 0; // DAP index
  buf[2] = 1; // Request size
  buf[3] = SWD_DP_R_IDCODE | DAP_TRANSFER_RnW;
  dbg_dap_cmd(buf, sizeof(buf), 4);

  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_read_idcode(uint32_t *v) {
  *v = dap_read_reg(SWD_DP_R_IDCODE);
  return true;
}

//-----------------------------------------------------------------------------
bool Adafruit_DAP::dap_target_prepare(void) {
  dap_write_reg(SWD_DP_W_ABORT, 0x00000016); // DP_ABORT_STKCMPCLR  = 2| DP_ABORT_STKERRCLR = 4 | DP_ABORT_ORUNERRCLR = 0x10
  dap_write_reg(SWD_DP_W_SELECT, 0x00000000); // DP_SELECT_APBANKSEL(0) = 0 | DP_SELECT_APSEL(0) = 0
  dap_write_reg(SWD_DP_W_CTRL_STAT, 0x50000f00); // DP_CST_CDBGPWRUPREQ = 0x10000000 | DP_CST_CSYSPWRUPREQ = 0x40000000| DP_CST_MASKLANE(0xf) = 0x0F00
  dap_write_reg(SWD_AP_CSW, 0x23000052); // AP_CSW_ADDRINC_SINGLE = 0x10 | AP_CSW_DEVICEEN = 0x40 | AP_CSW_PROT(0x23) = 0x23000000 | AP_CSW_SIZE_WORD = 0x02
  return true;
}

void Adafruit_DAP::dap_set_clock(uint32_t clock) { dap_setup_clock(clock); }

uint32_t Adafruit_DAP::computeFlashCRC32(uint32_t addr, uint32_t size) {
  Adafruit_DAP_CRC32 crc32;
  uint8_t buf[512];

  while(size) {
    uint32_t count = min(size, sizeof(buf));

    dap_read_block(addr, buf, (int) count);
    crc32.add(buf, count);

    addr += count;
    size -= count;
  }

  return crc32.get();
}
