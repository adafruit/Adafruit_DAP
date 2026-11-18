// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Adafruit Industries
#ifndef ADAFRUIT_DAP_CH32_H
#define ADAFRUIT_DAP_CH32_H

#include "Adafruit_DAP.h"

// The nanosecond SWIO transport currently requires the Arduino-Pico core.
#if defined(ARDUINO_ARCH_RP2040) && !defined(__riscv)

/** Standalone CH32V407/CH32V467 programmer using WCH single-wire debug.
 * Connecting resets and halts the target. This is a flash programmer, not a
 * transparent debugger: deselect resets it again to start its firmware.
 */
class Adafruit_DAP_CH32 : public Adafruit_DAP {
public:
  static const uint32_t FLASH_BASE =
      0x08000000; ///< Physical code-flash address.
  static const uint32_t FLASH_SIZE = 512 * 1024; ///< Supported E-density flash.
  static const uint32_t ERASE_SIZE = 4096;       ///< Erase sector, bytes.
  static const uint32_t PAGESIZE = 256;          ///< Programming page, bytes.

  Adafruit_DAP_CH32();
  bool begin(uint16_t swio, int16_t nreset, ErrorHandler handler = nullptr);
  bool begin(int swclk, int swdio, int nreset, ErrorHandler handler) override;
  bool targetConnect(uint32_t unused_clock = 50) override;
  uint32_t getTypeID() override;
  bool select(uint32_t *id) override;
  void deselect() override;
  bool dap_disconnect() override;
  bool dap_reset_target() override;
  bool dap_read_block(uint32_t addr, uint8_t *data, int size) override;
  void erase() override;
  uint32_t program_start(uint32_t addr, uint32_t size) override;
  void programBlock(uint32_t addr, const uint8_t *buf, uint32_t size) override;
  bool programFlash(uint32_t addr, const uint8_t *buf, uint32_t size,
                    bool do_verify = true) override;
  bool verifyFlash(uint32_t addr, const uint8_t *buf, uint32_t size);
  uint32_t computeFlashCRC32(uint32_t addr, uint32_t size) override;
  bool protectBoot() override;
  bool unprotectBoot() override;

private:
  // RISC-V debug module / WCH extension register addresses, not memory
  // addresses.
  enum : uint8_t {
    DATA0 = 0x04,
    DMCONTROL = 0x10,
    DMSTATUS = 0x11,
    ABSTRACTCS = 0x16,
    COMMAND = 0x17,
    ABSTRACTAUTO = 0x18,
    PROGBUF0 = 0x20,
    PROGBUF1 = 0x21,
    PROGBUF2 = 0x22,
    DMCFGR = 0x7d,
    DMSHADOWCFGR = 0x7e
  };
  enum : uint32_t {
    SWIO_CONFIG = 0x5aa50400,
    SWIO_KEY_MASK = 0xffff0000,
    DM_ACTIVE = 1,
    DM_HALT = 0x80000000,
    DM_ACK_RESET = 0x10000000,
    DM_ALL_HALTED = 0x200,
    DM_AUTHENTICATED = 0x80,
    ABSTRACT_BUSY = 0x1000,
    ABSTRACT_ERROR = 0x700,
    COMMAND_READ = 0x00220000,
    COMMAND_WRITE = 0x00230000,
    COMMAND_EXECUTE = 0x00240000,
    REG_S0 = 0x1008,
    REG_S1 = 0x1009,
    // RISC-V instructions: lw s1,0(s0); sw s1,0(s0); ebreak.
    LOAD_WORD = 0x00042483,
    STORE_WORD = 0x00942023,
    ADVANCE_ADDRESS = 0x00440413, ///< addi s0,s0,4 counts one completed access.
    BREAKPOINT = 0x00100073,
    CHIP_ID = 0x1ffff704,
    CHIP_ID_MASK = 0xffffff0f,
    CHIP_407R = 0x46720002,
    CHIP_467R = 0x46750002,
    FPEC = 0x40022000,
    FLASH_KEYR = FPEC + 4,
    FLASH_STATR = FPEC + 12,
    FLASH_CTLR = FPEC + 16,
    FLASH_ADDR = FPEC + 20,
    FLASH_MODEKEYR = FPEC + 36,
    FLASH_KEY1 = 0x45670123,
    FLASH_KEY2 = 0xcdef89ab,
    FLASH_BUSY = 3,
    FLASH_BUFFER_BUSY = 2,
    FLASH_WRITE_ERROR = 0x10,
    FLASH_EOP = 0x20,
    FLASH_LOCK = 0x80,
    FLASH_FAST_LOCK = 0x8000,
    FLASH_SECTOR_ERASE = 2,
    FLASH_START = 0x40,
    FLASH_PAGE_PROGRAM = 0x10000,
    FLASH_PROGRAM_START = 0x200000,
    DEBUG_TIMEOUT_MS = 100,
    FLASH_TIMEOUT_MS = 2000,
    SYSTICK_RELOAD = 0xffffff,
    SYSTICK_ENABLED_CPU_CLOCK = 5,
    FLASH_SETTINGS_MASK = 0x07000000
  };
  struct Timing {
    uint32_t shortLow, longLow, high, half, recharge, zero, riseTimeout;
  };
  uint16_t _swio = 0;
  int16_t _reset = -1;
  uint32_t _mask = 0;
  Timing _timing = {};
  bool _begun = false;
  bool _connected = false;
  bool _selected = false;
  bool _prepared = false;
  uint32_t _next = 0;
  uint32_t _end = 0;
  uint32_t _flashSettings = 0;

  bool fail(const char *message);
  void releasePins();
  void wireWrite(uint8_t reg, uint32_t value);
  bool wireRead(uint8_t reg, uint32_t &value);
  static uint32_t sendBits(uint32_t mask, const Timing &timing, uint64_t bits,
                           int count, uint32_t deadline);
  static int readBit(uint32_t mask, const Timing &timing, uint32_t &riseLeft);
  bool readDmi(uint8_t reg, uint32_t &value);
  bool writeDmiChecked(uint8_t reg, uint32_t value);
  bool waitAbstract();
  bool readRegister(uint16_t reg, uint32_t &value);
  bool writeRegister(uint16_t reg, uint32_t value);
  bool readWord(uint32_t address, uint32_t &value);
  bool writeWord(uint32_t address, uint32_t value);
  bool flashWait(uint32_t busyMask = FLASH_BUSY);
  bool flashUnlock();
  bool flashLock();
  bool eraseRange(uint32_t address, uint32_t size);
  bool programPage(uint32_t address, const uint8_t *data);
  bool flashAddress(uint32_t &address, uint32_t size);
};
#endif
#endif
