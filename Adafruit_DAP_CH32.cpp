// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026 Adafruit Industries
// SWIO waveform derived from Open Embedded Probe v0.0.58 OepSwioPhy.cpp,
// Copyright (c) 2026 Open Embedded Probe. See license-oep.txt.
#include "Adafruit_DAP.h"

#if defined(ARDUINO_ARCH_RP2040) && !defined(__riscv)
#include <hardware/gpio.h>
#include <hardware/structs/sio.h>
#include <hardware/structs/systick.h>
#include <hardware/sync.h>
#include <string.h>

namespace {
// SysTick is a 24-bit down counter at clk_sys on Arduino-Pico. All waveform
// helpers MUST be forced inline into the RAM functions: XIP stalls break SWIO.
__force_inline uint32_t ticks() { return (0u - systick_hw->cvr) & 0xffffff; }
__force_inline void waitUntil(uint32_t deadline) {
  while ((int32_t)((ticks() - deadline) << 8) < 0) {
  }
}
__force_inline void low(uint32_t mask) { sio_hw->gpio_clr = mask; }
__force_inline void high(uint32_t mask) { sio_hw->gpio_set = mask; }
__force_inline void drive(uint32_t mask) { sio_hw->gpio_oe_set = mask; }
__force_inline void release(uint32_t mask) { sio_hw->gpio_oe_clr = mask; }
__force_inline bool sample(uint32_t mask) {
  return (sio_hw->gpio_in & mask) != 0;
}
uint32_t nanosecondsToTicks(uint32_t ns) {
  return ((uint64_t)ns * rp2040.f_cpu() + 999999999) / 1000000000;
}
} // namespace

/** Create a disconnected CH32 programmer. */
Adafruit_DAP_CH32::Adafruit_DAP_CH32() {
  error_message = nullptr;
  perror_exit = nullptr;
  target_device = {};
}

/** Initialize SWIO and an open-drain reset pin; both start released.
 * @param swio RP2040 GPIO connected to target PA13/SWIO (0..29).
 * @param nreset RP2040 GPIO connected to NRST (0..29, different from swio).
 * @param handler Optional error notification; methods still return failure.
 * @return True when both pins are valid.
 */
bool Adafruit_DAP_CH32::begin(uint16_t swio, int16_t nreset,
                              ErrorHandler handler) {
  if (_begun)
    releasePins();
  _begun = _connected = _selected = _prepared = false;
  perror_exit = handler;
  error_message = nullptr;
  if (swio > 29 || nreset < 0 || nreset > 29 || swio == nreset)
    return fail("CH32 requires separate SWIO and RESET GPIOs 0..29");
  if (systick_hw->rvr != SYSTICK_RELOAD ||
      (systick_hw->csr & SYSTICK_ENABLED_CPU_CLOCK) !=
          SYSTICK_ENABLED_CPU_CLOCK)
    return fail("SWIO requires Arduino-Pico's CPU-clock SysTick timer");
  _swio = swio;
  _reset = nreset;
  _mask = 1u << swio;
  _timing = {nanosecondsToTicks(262),   nanosecondsToTicks(862),
             nanosecondsToTicks(262),   nanosecondsToTicks(131),
             nanosecondsToTicks(30),    nanosecondsToTicks(524),
             nanosecondsToTicks(100000)};
  gpio_init(_swio);
  gpio_set_drive_strength(_swio, GPIO_DRIVE_STRENGTH_2MA);
  gpio_init(_reset);
  releasePins();
  _begun = true;
  return true;
}

/** Initialize through the common DAP API. SWCLK is unused and never driven.
 * @param swclk Unused clock pin.
 * @param swdio SWIO GPIO.
 * @param nreset Reset GPIO.
 * @param handler Optional error handler.
 * @return True on success.
 */
bool Adafruit_DAP_CH32::begin(int swclk, int swdio, int nreset,
                              ErrorHandler handler) {
  (void)swclk;
  if (swdio < 0 || swdio > 29 || nreset < 0 || nreset > 29) {
    // Route invalid inputs through begin() too, to discard any prior session
    // and report the error consistently without truncating the pin numbers.
    return begin((uint16_t)30, (int16_t)-1, handler);
  }
  return begin((uint16_t)swdio, (int16_t)nreset, handler);
}

/** Release the programmer pins without ever driving RESET high. */
void Adafruit_DAP_CH32::releasePins() {
  gpio_set_dir(_swio, GPIO_IN);
  gpio_disable_pulls(_swio);
  if (_reset >= 0) {
    gpio_set_dir(_reset, GPIO_IN);
    gpio_disable_pulls(_reset);
  }
}

/** Record a failure and notify the optional callback. */
bool Adafruit_DAP_CH32::fail(const char *message) {
  error_message = (char *)message;
  _prepared = false;
  if (perror_exit)
    perror_exit(message);
  return false;
}

// WCH SWIO frames are MSB first. A short low encodes one, a long low zero.
__force_inline uint32_t Adafruit_DAP_CH32::sendBits(uint32_t mask,
                                                    const Timing &timing,
                                                    uint64_t bits, int count,
                                                    uint32_t deadline) {
  for (uint64_t bit = (uint64_t)1 << (count - 1); bit; bit >>= 1) {
    uint32_t width = timing.longLow;
    if (bits & bit)
      width = timing.shortLow;
    waitUntil(deadline);
    low(mask);
    uint32_t fell = ticks();
    waitUntil((fell + width) & 0xffffff);
    high(mask);
    deadline = (ticks() + timing.high) & 0xffffff;
  }
  return deadline;
}

// Read turnaround includes a brief weak-drive recharge pulse, as in OEP.
__force_inline int Adafruit_DAP_CH32::readBit(uint32_t mask,
                                              const Timing &timing,
                                              uint32_t &riseLeft) {
  uint32_t deadline = ticks();
  low(mask);
  waitUntil(deadline = (deadline + timing.shortLow) & 0xffffff);
  release(mask);
  high(mask);
  waitUntil(deadline = (deadline + timing.half) & 0xffffff);
  drive(mask);
  waitUntil(deadline = (deadline + timing.recharge) & 0xffffff);
  release(mask);
  waitUntil(deadline = (deadline + timing.half) & 0xffffff);
  int value = sample(mask);
  if (!value) {
    waitUntil(deadline = (deadline + timing.zero) & 0xffffff);
    drive(mask);
    waitUntil(deadline = (deadline + timing.recharge) & 0xffffff);
    release(mask);
  }
  uint32_t started = ticks();
  do {
    if (sample(mask)) {
      drive(mask);
      uint32_t elapsed = (ticks() - started) & 0xffffff;
      if (elapsed < riseLeft)
        riseLeft -= elapsed;
      else
        riseLeft = 0;
      waitUntil((ticks() + timing.half) & 0xffffff);
      return value;
    }
  } while (((ticks() - started) & 0xffffff) < riseLeft);
  return 2;
}

/** Send one WCH DMI register write from RAM with interrupts masked. */
void __not_in_flash_func(Adafruit_DAP_CH32::wireWrite)(uint8_t reg,
                                                       uint32_t value) {
  const uint32_t mask = _mask;
  const Timing timing = _timing;
  high(mask);
  drive(mask);
  uint64_t frame = ((uint64_t)1 << 40) | ((uint64_t)(reg & 0x7f) << 33) |
                   ((uint64_t)1 << 32) | value;
  uint32_t saved = save_and_disable_interrupts();
  waitUntil(sendBits(mask, timing, frame, 41, ticks()));
  restore_interrupts(saved);
  delayMicroseconds(8);
}

/** Read one WCH DMI register; SWIO has no parity, so callers confirm reads. */
bool __not_in_flash_func(Adafruit_DAP_CH32::wireRead)(uint8_t reg,
                                                      uint32_t &value) {
  const uint32_t mask = _mask;
  const Timing timing = _timing;
  high(mask);
  drive(mask);
  uint32_t result = 0;
  uint32_t riseLeft = _timing.riseTimeout;
  uint32_t saved = save_and_disable_interrupts();
  waitUntil(
      sendBits(mask, timing, (1u << 8) | ((reg & 0x7f) << 1), 9, ticks()));
  for (int bit = 0; bit < 32; bit++) {
    int decoded = readBit(mask, timing, riseLeft);
    if (decoded == 2) {
      release(_mask);
      restore_interrupts(saved);
      delayMicroseconds(8);
      return false;
    }
    result = (result << 1) | decoded;
  }
  restore_interrupts(saved);
  delayMicroseconds(8);
  value = result;
  return true;
}

/** Confirm a DMI value twice, separated by a different register read. */
bool Adafruit_DAP_CH32::readDmi(uint8_t reg, uint32_t &value) {
  uint32_t first = 0, second = 0, separator = 0;
  uint8_t other = DMSTATUS;
  if (reg == DMSTATUS)
    other = DMCONTROL;
  // Status can change between samples while a halt or command completes.
  // Re-read, without reissuing any command or target-memory write.
  for (int attempt = 0; attempt < 3; attempt++) {
    if (wireRead(reg, first) && wireRead(other, separator) &&
        wireRead(reg, second) && first == second) {
      value = second;
      return true;
    }
  }
  return fail("SWIO read did not agree");
}

/** Write a readable debug register and confirm it before executing code. */
bool Adafruit_DAP_CH32::writeDmiChecked(uint8_t reg, uint32_t value) {
  wireWrite(reg, value);
  uint32_t back;
  if (!readDmi(reg, back) || back != value)
    return fail("SWIO write did not verify");
  return true;
}

/** Reset the target, enable SWIO and halt its core. No flash is changed.
 * @param unused_clock Ignored; SWIO uses fixed pulse widths, not an SWD clock.
 * @return True when an authenticated halted debug module answers.
 */
bool Adafruit_DAP_CH32::targetConnect(uint32_t unused_clock) {
  (void)unused_clock;
  error_message = nullptr;
  _connected = _selected = _prepared = false;
  target_device = {};
  if (!_begun)
    return fail("Call begin before connecting");
  gpio_put(_reset, 0);
  gpio_set_dir(_reset, GPIO_OUT);
  delay(20);
  gpio_set_dir(_reset, GPIO_IN);
  gpio_pull_up(_swio);
  delay(2);
  if (!gpio_get(_swio))
    return fail("SWIO is held low");
  wireWrite(DMSHADOWCFGR, SWIO_CONFIG);
  wireWrite(DMCFGR, SWIO_CONFIG);
  wireWrite(DMSHADOWCFGR, SWIO_CONFIG);
  wireWrite(DMCFGR, SWIO_CONFIG);
  wireWrite(DMCONTROL, DM_ACTIVE);
  wireWrite(DMCONTROL, DM_ACTIVE);
  uint32_t config;
  if (!readDmi(DMCFGR, config) ||
      (config & SWIO_KEY_MASK) != (SWIO_CONFIG & SWIO_KEY_MASK))
    return fail("No WCH SWIO debug module");
  wireWrite(DMCONTROL, DM_ACTIVE | DM_HALT | DM_ACK_RESET);
  uint32_t started = millis();
  do {
    uint32_t status;
    if (!readDmi(DMSTATUS, status))
      return false;
    if ((status & 15) == 2 && (status & DM_AUTHENTICATED) &&
        (status & DM_ALL_HALTED)) {
      if (!writeDmiChecked(ABSTRACTAUTO, 0))
        return false;
      wireWrite(ABSTRACTCS, ABSTRACT_ERROR);
      _connected = waitAbstract();
      return _connected;
    }
  } while (millis() - started < DEBUG_TIMEOUT_MS);
  return fail("CH32 did not halt");
}

/** Wait for an abstract command, failing on a sticky error or timeout. */
bool Adafruit_DAP_CH32::waitAbstract() {
  uint32_t started = millis();
  do {
    uint32_t status;
    if (!readDmi(ABSTRACTCS, status))
      return false;
    if (!(status & ABSTRACT_BUSY)) {
      if (!(status & ABSTRACT_ERROR))
        return true;
      wireWrite(ABSTRACTCS, ABSTRACT_ERROR);
      return fail("RISC-V abstract command failed");
    }
  } while (millis() - started < DEBUG_TIMEOUT_MS);
  return fail("RISC-V abstract command timed out");
}

/** Read a core register over two distinct DATA0 sentinels. */
bool Adafruit_DAP_CH32::readRegister(uint16_t reg, uint32_t &value) {
  uint32_t first, second;
  wireWrite(DATA0, 0);
  wireWrite(COMMAND, COMMAND_READ | reg);
  if (!waitAbstract() || !readDmi(DATA0, first))
    return false;
  wireWrite(DATA0, UINT32_MAX);
  wireWrite(COMMAND, COMMAND_READ | reg);
  if (!waitAbstract() || !readDmi(DATA0, second))
    return false;
  if (first != second)
    return fail("Core register read did not agree");
  value = second;
  return true;
}

/** Set a core register and verify it before using it as an address or value. */
bool Adafruit_DAP_CH32::writeRegister(uint16_t reg, uint32_t value) {
  if (!writeDmiChecked(DATA0, value))
    return false;
  wireWrite(COMMAND, COMMAND_WRITE | reg);
  uint32_t back;
  if (!waitAbstract() || !readRegister(reg, back))
    return false;
  if (back != value)
    return fail("Core register write did not verify");
  return true;
}

/** Read an aligned target memory word through a verified program buffer. */
bool Adafruit_DAP_CH32::readWord(uint32_t address, uint32_t &value) {
  if (address % 4)
    return fail("Unaligned memory read");
  if (!writeRegister(REG_S0, address) ||
      !writeDmiChecked(PROGBUF0, LOAD_WORD) ||
      !writeDmiChecked(PROGBUF1, ADVANCE_ADDRESS) ||
      !writeDmiChecked(PROGBUF2, BREAKPOINT))
    return false;
  wireWrite(COMMAND, COMMAND_EXECUTE);
  uint32_t next;
  if (!waitAbstract() || !readRegister(REG_S0, next))
    return false;
  // SWIO has no acknowledgement: a lost execute command must not return the
  // previous load's value as though it came from this address.
  if (next != address + 4)
    return fail("Target memory read did not execute once");
  return readRegister(REG_S1, value);
}

/** Store an aligned target word exactly once; never blindly retry MMIO writes.
 */
bool Adafruit_DAP_CH32::writeWord(uint32_t address, uint32_t value) {
  if (address % 4)
    return fail("Unaligned memory write");
  if (!writeRegister(REG_S0, address) || !writeRegister(REG_S1, value) ||
      !writeDmiChecked(PROGBUF0, STORE_WORD) ||
      !writeDmiChecked(PROGBUF1, ADVANCE_ADDRESS) ||
      !writeDmiChecked(PROGBUF2, BREAKPOINT))
    return false;
  wireWrite(COMMAND, COMMAND_EXECUTE);
  uint32_t next;
  if (!waitAbstract() || !readRegister(REG_S0, next))
    return false;
  if (next != address + 4)
    return fail("Target memory write did not execute once");
  return true;
}

/** Return this driver's common DAP type identifier. */
uint32_t Adafruit_DAP_CH32::getTypeID() { return DAP_TYPEID_CH32; }

/** Identify a supported 512 KiB Metro target before allowing flash changes.
 * @param id Receives the complete chip identifier, including revision.
 * @return True for CH32V407RET or CH32V467RET.
 */
bool Adafruit_DAP_CH32::select(uint32_t *id) {
  _selected = _prepared = false;
  if (!_connected || !id)
    return fail("Invalid selection request");
  if (!readWord(CHIP_ID, *id))
    return false;
  const char *name;
  switch (*id & CHIP_ID_MASK) {
  case CHIP_407R:
    name = "CH32V407RET";
    break;
  case CHIP_467R:
    name = "CH32V467RET";
    break;
  default:
    return fail("Unsupported CH32 chip ID");
  }
  target_device = {*id, name, FLASH_SIZE, FLASH_SIZE / PAGESIZE};
  _selected = true;
  return true;
}

/** Reset and release the target after programming (also after a failure). */
bool Adafruit_DAP_CH32::dap_reset_target() {
  if (!_begun)
    return fail("Call begin before resetting");
  _connected = _selected = _prepared = false;
  // Clear halt request before the physical reset; reset also relocks flash.
  wireWrite(ABSTRACTAUTO, 0);
  wireWrite(DMCONTROL, DM_ACTIVE);
  gpio_put(_reset, 0);
  gpio_set_dir(_reset, GPIO_OUT);
  delay(20);
  releasePins();
  return true;
}

/** Restart the target, discard the programming session, and release its pins.
 */
void Adafruit_DAP_CH32::deselect() { dap_reset_target(); }

/** Release pins. Use deselect() first to restart a halted target. */
bool Adafruit_DAP_CH32::dap_disconnect() {
  if (_begun)
    releasePins();
  _connected = _selected = _prepared = false;
  return true;
}

/** Read target memory without requiring word alignment in the caller's buffer.
 * @param addr Target memory address (physical or code alias).
 * @param data Destination buffer.
 * @param size Number of bytes.
 * @return True when every read completed.
 */
bool Adafruit_DAP_CH32::dap_read_block(uint32_t addr, uint8_t *data, int size) {
  if (!_selected || size < 0 || (!data && size) ||
      (uint32_t)size > UINT32_MAX - addr)
    return fail("Invalid CH32 memory read");
  while (size) {
    uint32_t word;
    uint32_t aligned = addr - addr % 4;
    if (!readWord(aligned, word))
      return false;
    uint32_t count = min((uint32_t)size, 4 - addr % 4);
    memcpy(data, (uint8_t *)&word + addr % 4, count);
    data += count;
    addr += count;
    size -= count;
  }
  return true;
}

/** Validate a code-flash range and normalize its zero-based alias. */
bool Adafruit_DAP_CH32::flashAddress(uint32_t &address, uint32_t size) {
  if (address < FLASH_SIZE)
    address += FLASH_BASE;
  if (!_selected || !size || address < FLASH_BASE ||
      address >= FLASH_BASE + FLASH_SIZE ||
      size > FLASH_BASE + FLASH_SIZE - address)
    return fail("Invalid CH32 flash range");
  return true;
}

/** Wait for both flash busy bits, then check write-protection errors. */
bool Adafruit_DAP_CH32::flashWait(uint32_t busyMask) {
  uint32_t started = millis();
  do {
    uint32_t status;
    if (!readWord(FLASH_STATR, status))
      return false;
    if (!(status & busyMask)) {
      if (status & FLASH_WRITE_ERROR)
        return fail("CH32 flash is write protected");
      return true;
    }
  } while (millis() - started < FLASH_TIMEOUT_MS);
  return fail("CH32 flash operation timed out");
}

/** Unlock normal and page programming; leave option bytes untouched. */
bool Adafruit_DAP_CH32::flashUnlock() {
  uint32_t control;
  if (!readWord(FLASH_CTLR, control))
    return false;
  // Preserve the flash access clock and enhanced-read setting.
  _flashSettings = control & FLASH_SETTINGS_MASK;
  if (control & FLASH_LOCK) {
    if (!writeWord(FLASH_KEYR, FLASH_KEY1) ||
        !writeWord(FLASH_KEYR, FLASH_KEY2))
      return false;
  }
  if (control & FLASH_FAST_LOCK) {
    if (!writeWord(FLASH_MODEKEYR, FLASH_KEY1) ||
        !writeWord(FLASH_MODEKEYR, FLASH_KEY2))
      return false;
  }
  if (!readWord(FLASH_CTLR, control))
    return false;
  if (control & (FLASH_LOCK | FLASH_FAST_LOCK))
    return fail("CH32 flash stayed locked");
  return writeWord(FLASH_STATR, FLASH_WRITE_ERROR | FLASH_EOP);
}

/** Relock both flash programming paths. */
bool Adafruit_DAP_CH32::flashLock() {
  return writeWord(FLASH_CTLR, _flashSettings | FLASH_LOCK | FLASH_FAST_LOCK);
}

/** Erase whole 4 KiB sectors in the explicitly requested range. */
bool Adafruit_DAP_CH32::eraseRange(uint32_t address, uint32_t size) {
  if (!flashUnlock())
    return false;
  bool ok = true;
  for (uint32_t offset = 0; offset < size && ok; offset += ERASE_SIZE) {
    ok = flashWait() &&
         writeWord(FLASH_CTLR, _flashSettings | FLASH_SECTOR_ERASE) &&
         writeWord(FLASH_ADDR, address + offset) &&
         writeWord(FLASH_CTLR,
                   _flashSettings | FLASH_SECTOR_ERASE | FLASH_START) &&
         flashWait() && writeWord(FLASH_CTLR, _flashSettings);
  }
  bool locked = flashLock();
  return ok && locked;
}

/** Erase all code flash. This does not erase or change option bytes. */
void Adafruit_DAP_CH32::erase() {
  _prepared = false;
  uint32_t address = FLASH_BASE;
  if (flashAddress(address, FLASH_SIZE))
    eraseRange(address, FLASH_SIZE);
}

/** Erase the sectors for an image and prepare sequential page writes.
 * @param addr Sector-aligned code address; zero-based alias is accepted.
 * @param size Image length. Its final sector is erased in full.
 * @return Normalized physical address, or UINT32_MAX on failure.
 */
uint32_t Adafruit_DAP_CH32::program_start(uint32_t addr, uint32_t size) {
  _prepared = false;
  if (!flashAddress(addr, size))
    return UINT32_MAX;
  if (addr % ERASE_SIZE) {
    fail("Image must start on a 4 KiB sector boundary");
    return UINT32_MAX;
  }
  uint32_t eraseSize = ((size + ERASE_SIZE - 1) / ERASE_SIZE) * ERASE_SIZE;
  if (!eraseRange(addr, eraseSize))
    return UINT32_MAX;
  _next = addr;
  // Keep the exact image limit; page padding is generated by programFlash(),
  // never accepted as additional caller data.
  _end = addr + size;
  _prepared = true;
  return addr;
}

/** Program one 256-byte page using WCH's PAGE_PG / PG_STRT sequence. */
bool Adafruit_DAP_CH32::programPage(uint32_t address, const uint8_t *data) {
  if (!flashWait() ||
      !writeWord(FLASH_CTLR, _flashSettings | FLASH_PAGE_PROGRAM) ||
      !flashWait())
    return false;
  for (uint32_t offset = 0; offset < PAGESIZE; offset += 4) {
    uint32_t word;
    memcpy(&word, data + offset, sizeof(word));
    // During page loading BSY can remain asserted until PG_STRT. WCH's
    // FLASH_ProgramPage_Fast waits only for WR_BSY between the 64 words.
    if (!writeWord(address + offset, word) || !flashWait(FLASH_BUFFER_BUSY))
      return false;
  }
  return writeWord(FLASH_CTLR,
                   _flashSettings | FLASH_PAGE_PROGRAM | FLASH_PROGRAM_START) &&
         flashWait() && writeWord(FLASH_CTLR, _flashSettings);
}

/** Program consecutive pages after program_start(), optionally read them back.
 * @param addr Next address in the prepared image range.
 * @param buf Source bytes; memory-mapped Pico flash arrays are supported.
 * @param size Byte count. Only the final page may be partial (padded with
 * 0xff).
 * @param do_verify Compare every programmed byte, including final-page padding.
 * @return True on complete success. Any failure invalidates the session.
 */
bool Adafruit_DAP_CH32::programFlash(uint32_t addr, const uint8_t *buf,
                                     uint32_t size, bool do_verify) {
  if (!flashAddress(addr, size))
    return false;
  if (!_prepared || !buf || addr != _next || size > _end - _next ||
      (size % PAGESIZE && size != _end - _next))
    return fail("Write must follow the prepared CH32 image range");
  if (!flashUnlock())
    return false;
  bool ok = true;
  uint8_t page[PAGESIZE];
  for (uint32_t offset = 0; offset < size && ok; offset += PAGESIZE) {
    uint32_t count = min(PAGESIZE, size - offset);
    memset(page, 0xff, sizeof(page));
    memcpy(page, buf + offset, count);
    ok = programPage(addr + offset, page);
    if (ok && do_verify)
      ok = verifyFlash(addr + offset, page, PAGESIZE);
  }
  bool locked = flashLock();
  if (!ok || !locked) {
    _prepared = false;
    return false;
  }
  _next += size;
  if (_next == _end)
    _prepared = false;
  return true;
}

/** Common DAP programming entry point; errors remain in error_message. */
void Adafruit_DAP_CH32::programBlock(uint32_t addr, const uint8_t *buf,
                                     uint32_t size) {
  programFlash(addr, buf, size, true);
}

/** Compare a flash range byte for byte with a source buffer.
 * @param addr Code address (physical or zero-based).
 * @param buf Expected bytes.
 * @param size Byte count.
 * @return True only when all bytes match.
 */
bool Adafruit_DAP_CH32::verifyFlash(uint32_t addr, const uint8_t *buf,
                                    uint32_t size) {
  if (!flashAddress(addr, size) || !buf)
    return false;
  uint8_t back[32];
  for (uint32_t offset = 0; offset < size; offset += sizeof(back)) {
    uint32_t count = min((uint32_t)sizeof(back), size - offset);
    if (!dap_read_block(addr + offset, back, count))
      return false;
    if (memcmp(back, buf + offset, count))
      return fail("CH32 flash readback mismatch");
  }
  return true;
}

/** Compute IEEE CRC32 of code flash, stopping immediately on a read error.
 * @param addr Code-flash address.
 * @param size Number of bytes.
 * @return CRC32 on success; zero on error (see error_message).
 */
uint32_t Adafruit_DAP_CH32::computeFlashCRC32(uint32_t addr, uint32_t size) {
  if (!flashAddress(addr, size))
    return 0;
  Adafruit_DAP_CRC32 crc;
  uint8_t buffer[32];
  while (size) {
    uint32_t count = min(size, (uint32_t)sizeof(buffer));
    if (!dap_read_block(addr, buffer, count))
      return 0;
    crc.add(buffer, count);
    addr += count;
    size -= count;
  }
  return crc.get();
}

/** Option-byte changes are intentionally unsupported. */
bool Adafruit_DAP_CH32::protectBoot() {
  return fail("CH32 option-byte changes are unsupported");
}
/** Option-byte changes are intentionally unsupported. */
bool Adafruit_DAP_CH32::unprotectBoot() {
  return fail("CH32 option-byte changes are unsupported");
}
#endif
