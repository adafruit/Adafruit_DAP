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

#ifndef ADAFRUIT_DAP_NRF5X_H_
#define ADAFRUIT_DAP_NRF5X_H_

#include "Adafruit_DAP.h"

// DAP for nRF
class Adafruit_DAP_nRF5x : public Adafruit_DAP {
public:
  Adafruit_DAP_nRF5x(void){};
  ~Adafruit_DAP_nRF5x(void){};

  static device_t devices[];
  device_t target_device;

  bool select(uint32_t *id);
  void deselect(void);

  void erase(void);

  void erasePage(uint32_t page);
  void eraseUICR(void);
  void eraseFICR(void);

  bool program(uint32_t addr, const uint8_t *buf, uint32_t count);
  void programUICR(uint32_t addr, uint32_t value);

  bool flashWaitReady(void);
  bool flashReady(void);

  void lock(void);
  void programBlock(uint32_t addr, uint8_t *buf);
  void readBlock(uint32_t addr, uint8_t *buf);
  void fuse(void);
  void fuseRead();
  void fuseWrite();

  uint32_t program_start(uint32_t offset = 0);

  struct USER_ROW {

    uint8_t BOOTPROT : 3;
    uint8_t EEPROM : 3;
    uint8_t BOD33_Level : 6;
    uint8_t BOD33_Enable : 1;
    uint8_t BOD33_Action : 2;
    uint8_t WDT_Enable : 1;
    uint8_t WDT_Always_On : 1;
    uint8_t WDT_Period : 4;
    uint8_t WDT_Window : 4;
    uint8_t WDR_EWOFFSET : 4;
    uint8_t WDR_WEN : 1;
    uint8_t BOD33_Hysteresis : 1;
    uint16_t LOCK : 16;

    void set(uint64_t data) {
      BOOTPROT = data & 0x07;
      EEPROM = (data >> 4) & 0x07;
      BOD33_Level = (data >> 8) & 0x3F;
      BOD33_Enable = (data >> 14) & 0x01;
      BOD33_Action = (data >> 15) & 0x03;
      WDT_Enable = (data >> 25) & 0x01;
      WDT_Always_On = (data >> 26) & 0x01;
      WDT_Period = (data >> 27) & 0xF;
      WDT_Window = (data >> 31) & 0xF;
      WDR_EWOFFSET = (data >> 35) & 0xF;
      WDR_WEN = (data >> 39) & 0x01;
      BOD33_Hysteresis = (data >> 40) & 0x01;
      LOCK = (data >> 48) & 0xFFFF;
    }
    uint64_t get() {
      return ((uint64_t)LOCK << 48) | ((uint64_t)BOD33_Hysteresis << 40) |
             ((uint64_t)WDR_WEN << 39) | ((uint64_t)WDR_EWOFFSET << 35) |
             ((uint64_t)WDT_Window << 31) | ((uint64_t)WDT_Period << 27) |
             ((uint64_t)WDT_Always_On << 26) | ((uint64_t)WDT_Enable << 25) |
             ((uint64_t)BOD33_Action << 15) | ((uint64_t)BOD33_Enable << 14) |
             ((uint64_t)BOD33_Level << 8) | ((uint64_t)EEPROM << 4) |
             (uint64_t)BOOTPROT;
    }
  };
  USER_ROW _USER_ROW;
};

#endif /* ADAFRUIT_DAP_NRF5X_H_ */
