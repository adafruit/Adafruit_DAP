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

// DAP for nRF
class Adafruit_DAP_nRF5x : public Adafruit_DAP {
public:
  Adafruit_DAP_nRF5x(void){};
  ~Adafruit_DAP_nRF5x(void){};

  static device_t devices[];

  //------------- Common API -------------//
  virtual uint32_t getTypeID(void) {
    return DAP_TYPEID_NRF5X;
  }

  bool select(uint32_t *id);
  void deselect(void);

  void erase(void);
  uint32_t program_start(uint32_t offset = 0, uint32_t size = 0);
  void programBlock(uint32_t addr, const uint8_t *buf, uint32_t size);

  bool protectBoot(void);
  bool unprotectBoot(void);

  void programUICR(uint32_t addr, uint32_t value);
  void programUICR_AdafruitBootloader(void);

  bool programFlash(uint32_t addr, const uint8_t *buf, uint32_t count, bool do_verify = true);
  bool program(uint32_t addr, const uint8_t *buf, uint32_t count, bool verify = true) {
    return programFlash(addr, buf, count, verify);
  }

  bool flashWaitReady(void);
  bool flashReady(void);
};

#endif /* ADAFRUIT_DAP_NRF5X_H_ */
