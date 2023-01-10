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

#ifndef ADAFRUIT_DAP_SAM_H_
#define ADAFRUIT_DAP_SAM_H_

#define SAM_PAGE_SIZE   256
#define SAMx5_PAGE_SIZE 512

// DAP for SAM
class Adafruit_DAP_SAM : public Adafruit_DAP {
public:
  Adafruit_DAP_SAM(void) : locked(false) {};
  ~Adafruit_DAP_SAM(void){};

  static const size_t PAGESIZE = SAM_PAGE_SIZE;
  static const size_t USER_ROW_SIZE = 256;
  static device_t devices[];
  bool locked;

  //------------- Common API -------------//
  virtual uint32_t getTypeID(void) {
    return DAP_TYPEID_SAM;
  }
  virtual bool select(uint32_t *id);
  virtual void deselect(void);

  virtual void erase(void);
  virtual void programBlock(uint32_t addr, const uint8_t *buf, uint32_t size = PAGESIZE);
  virtual bool programFlash(uint32_t flashOffset, const uint8_t * data, uint32_t datalen, bool doVerify = true);

  virtual bool protectBoot(void);
  virtual bool unprotectBoot(void);

  //------------- API for both SAMD21 and SAMD51 -------------//
  void resetWithExtension(void);
  void finishReset(void);

  void lock(void);
  virtual size_t pageSize() { return PAGESIZE; }
  virtual void resetProtectionFuses(bool resetBootloaderProtection, bool resetRegionLocks);
  virtual void readBlock(uint32_t addr, uint8_t *buf);

  virtual bool readCRC(uint32_t length, uint32_t *crc);

  virtual uint32_t computeFlashCRC32(uint32_t addr, uint32_t size) {
    if ( addr != 0 ) {
      return 0;
    }
    uint32_t crc32;
    return readCRC(size, &crc32) ? (crc32 ^ 0xFFFFFFFFUL) : 0;
  }

  // uint32_t verifyBlock(uint32_t addr);
  void fuse(void);
  void fuseRead();
  void fuseWrite();

  virtual uint32_t program_start(uint32_t offset = 0, uint32_t size = 0);

  typedef union {
    struct __attribute__((__packed__)) {
      // The old USER_ROW erased the reserved BOD12 Voltage regulator config
      uint64_t BOOTPROT : 3;
      uint64_t _reserved1 : 1;
      uint64_t EEPROM : 3;
      uint64_t _reserved2 : 1;
      uint64_t BOD33_Level : 6;
      uint64_t BOD33_Enable : 1;
      uint64_t BOD33_Action : 2;
      uint64_t _reserved_BOD12_Config_Vreg: 8;
      uint64_t WDT_Enable : 1;
      uint64_t WDT_Always_On : 1;
      uint64_t WDT_Period : 4;
      uint64_t WDT_Window : 4;
      uint64_t WDR_EWOFFSET : 4;
      uint64_t WDR_WEN : 1;
      uint64_t BOD33_Hysteresis : 1;
      uint64_t _reserved_BOD12_Config : 1;
      uint64_t _reserved3 : 6;
      uint64_t LOCK : 16;
    } bit;
    uint64_t fuses;
    uint32_t fuseParts[2];
    uint8_t reg[USER_ROW_SIZE]; // Store a full page size
  } USER_ROW;
  USER_ROW _USER_ROW;
};

// DAP for SAMx5
class Adafruit_DAP_SAMx5 : public Adafruit_DAP_SAM {
public:
  Adafruit_DAP_SAMx5(void) : Adafruit_DAP_SAM() { };
  ~Adafruit_DAP_SAMx5(void){};

  static const size_t PAGESIZE = SAMx5_PAGE_SIZE;
  static const size_t USER_ROW_SIZE = 32;

  static device_t devices[];

  //------------- Common API -------------//
  virtual uint32_t getTypeID(void) {
    return DAP_TYPEID_SAMX5;
  }

  virtual bool select(uint32_t *id);
  virtual void erase(void);
  virtual uint32_t program_start(uint32_t offset = 0, uint32_t size = 0);
  virtual void programBlock(uint32_t addr, const uint8_t *buf, uint32_t size = PAGESIZE);

  virtual bool protectBoot(void);
  virtual bool unprotectBoot(void);

  void lock(void);
  virtual size_t pageSize() { return PAGESIZE; }
  virtual void resetProtectionFuses(bool resetBootloaderProtection, bool resetRegionLocks);
  virtual void readBlock(uint32_t addr, uint8_t *buf);

  bool readCRC(uint32_t length, uint32_t *crc);

  virtual uint32_t computeFlashCRC32(uint32_t addr, uint32_t size) {
    if ( addr != 0 ) {
      return 0;
    }
    uint32_t crc32;
    return readCRC(size, &crc32) ? (crc32 ^ 0xFFFFFFFFUL) : 0;
  }

  // uint32_t verifyBlock(uint32_t addr);

  void fuse(void);
  void fuseRead();
  void fuseWrite();


  typedef union {
    struct __attribute__((__packed__)) {
      uint8_t BOD33_Disable : 1;
      uint8_t BOD33_Level : 8;
      uint8_t BOD33_Action : 2;
      uint8_t BOD33_Hysteresis : 4;
      uint8_t : 8;
      uint8_t : 3;
      uint8_t NVM_BOOT : 4;
      uint8_t : 2;
      uint8_t SEESBLK : 4;
      uint8_t SEEPSZ : 3;
      uint8_t RAM_ECCDIS : 1;
      uint8_t : 8;
      uint8_t WDT_Enable : 1;
      uint8_t WDT_Always_On : 1;
      uint8_t WDT_Period : 4;
      uint8_t WDT_Window : 4;
      uint8_t WDT_EWOFFSET : 4;
      uint8_t WDT_WEN : 1;
      uint8_t : 1;
      uint16_t NVM_LOCKS : 32; // As per SAM D5x/E5x Family datasheet page 53
      uint32_t User_Page : 32;
    } bit;
    uint8_t reg[USER_ROW_SIZE];
  } USER_ROW;
  USER_ROW _USER_ROW;
};

#endif
