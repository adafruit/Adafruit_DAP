/*!
 * @file dragdrop_engine.cpp
 *
 * Drag-and-drop programming logic.
 *
 */

#include "dragdrop_engine.h"
#include <string.h>

// ---- virtual FAT16 drive ----

static void put16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v) {
  put16(p, (uint16_t)v);
  put16(p + 2, (uint16_t)(v >> 16));
}

// FAT12: entry n occupies 12 bits starting at byte n * 1.5.
static void setFat12(uint8_t *fat, uint32_t n, uint16_t v) {
  uint32_t off = n * 3 / 2;
  if (n & 1) {
    fat[off] = (uint8_t)((fat[off] & 0x0F) | ((v << 4) & 0xF0));
    fat[off + 1] = (uint8_t)(v >> 4);
  } else {
    fat[off] = (uint8_t)v;
    fat[off + 1] = (uint8_t)((fat[off + 1] & 0xF0) | ((v >> 8) & 0x0F));
  }
}

// FAT date: bits 15-9 year since 1980, 8-5 month, 4-0 day.
static const uint16_t FILE_DATE = ((2026 - 1980) << 9) | (1 << 5) | 1;

void VirtualFat::setVolumeLabel(const char *label) {
  for (uint8_t i = 0; i < 11; i++) {
    char c = *label ? *label++ : ' ';
    _label[i] = (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; // upper case
  }
}

bool VirtualFat::addFile(const char *name83, Generator gen, void *context) {
  if (_count >= MAX_FILES) {
    return false;
  }
  memcpy(_files[_count].name, name83, 11);
  _files[_count].gen = gen;
  _files[_count].context = context;
  _count++;
  return true;
}

void VirtualFat::bootSector(uint8_t *buf) {
  buf[0] = 0xEB; // jump to boot code (x86), required by some hosts
  buf[1] = 0x3C;
  buf[2] = 0x90;
  memcpy(buf + 3, "MSWIN4.1", 8);  // OEM name
  put16(buf + 11, SECTOR_SIZE);    // bytes per sector
  buf[13] = SECTORS_PER_CLUSTER;   // sectors per cluster
  put16(buf + 14, FAT_START);      // reserved sectors
  buf[16] = 1;                     // number of FATs
  put16(buf + 17, ROOT_ENTRIES);   // root directory entries
  put16(buf + 19, TOTAL_SECTORS);  // total sectors (16-bit)
  buf[21] = 0xF8;                  // media: fixed disk
  put16(buf + 22, FAT_SECTORS);    // sectors per FAT
  put16(buf + 24, 63);             // sectors per track
  put16(buf + 26, 255);            // heads
  buf[36] = 0x80;                  // drive number
  buf[38] = 0x29;                  // extended boot signature
  put32(buf + 39, 0x0DA4C032);     // volume serial number
  memcpy(buf + 43, _label, 11);    // volume label
  memcpy(buf + 54, "FAT12   ", 8); // file system type
  buf[510] = 0x55;
  buf[511] = 0xAA;
}

void VirtualFat::fatSector(uint32_t index, uint8_t *buf) {
  if (index != 0) {
    return; // all generated files fit in the first FAT sector
  }
  setFat12(buf, 0, 0xFF8); // media descriptor
  setFat12(buf, 1, 0xFFF);
  for (uint8_t i = 0; i < _count; i++) {
    setFat12(buf, 2 + i, 0xFFF); // one cluster each: end of chain
  }
}

void VirtualFat::rootSector(uint32_t index, uint8_t *buf) {
  if (index != 0) {
    return;
  }
  // Run each generator once to learn its size (using buf as scratch space).
  uint32_t sizes[MAX_FILES];
  for (uint8_t i = 0; i < _count; i++) {
    sizes[i] = _files[i].gen(_files[i].context, (char *)buf, SECTOR_SIZE);
    if (sizes[i] > SECTOR_SIZE) {
      sizes[i] = SECTOR_SIZE;
    }
  }
  memset(buf, 0, SECTOR_SIZE);

  uint8_t *e = buf;
  memcpy(e, _label, 11); // volume label entry
  e[11] = 0x08;
  put16(e + 24, FILE_DATE);
  e += 32;
  for (uint8_t i = 0; i < _count; i++, e += 32) {
    memcpy(e, _files[i].name, 11);
    e[11] = 0x01;                     // read-only
    put16(e + 16, FILE_DATE);         // created
    put16(e + 18, FILE_DATE);         // accessed
    put16(e + 24, FILE_DATE);         // modified
    put16(e + 26, (uint16_t)(2 + i)); // first cluster
    put32(e + 28, sizes[i]);          // size in bytes
  }
}

void VirtualFat::readSector(uint32_t lba, uint8_t *buf) {
  memset(buf, 0, SECTOR_SIZE);
  if (lba == 0) {
    bootSector(buf);
  } else if (lba >= FAT_START && lba < ROOT_START) {
    fatSector(lba - FAT_START, buf);
  } else if (lba >= ROOT_START && lba < DATA_START) {
    rootSector(lba - ROOT_START, buf);
  } else if (lba < TOTAL_SECTORS) {
    uint32_t rel = lba - DATA_START;
    uint32_t file = rel / SECTORS_PER_CLUSTER;
    if (file < _count && rel % SECTORS_PER_CLUSTER == 0) {
      uint32_t n =
          _files[file].gen(_files[file].context, (char *)buf, SECTOR_SIZE);
      if (n < SECTOR_SIZE) {
        memset(buf + n, 0, SECTOR_SIZE - n);
      }
    }
  }
}

// ---- drag-and-drop engine ----

static const uint32_t TARGET_FLASH = 0x08000000;    // STM32 flash
static const uint32_t DBGMCU_IDCODE = 0xE0042000;   // DEV_ID in bits 11:0
static const uint32_t FLASH_SIZE_REG = 0x1FFF7A20;  // KB in bits 31:16

// SWD clock as delay count: 4 is about 940 kHz on a CH32V203
// at 144 MHz.
static const uint32_t SWJ_CLOCK = 4;

// Idle cycles after each SWD transfer: 32 (about 34 us) lets the STM32 finish
// programming a flash word (16 us typical) before the next request, so it
// answers OK instead of WAIT.
static const uint8_t IDLE_CYCLES = 32;

// First error reported by Adafruit_DAP during the current step
static const char *dap_error = nullptr;

void dragdrop_dap_error(const char *text) {
  if (!dap_error) {
    // Adafruit_DAP prints transfer failures and then reports ")"
    dap_error = (text && text[0] && !(text[0] == ')' && !text[1]))
                    ? text
                    : "the target stopped answering over SWD";
  }
}

// The CH32V203's 96-bit unique ID as 24 hex digits
static void ch32_serial_string(char *str) {
  static const char hex[] = "0123456789ABCDEF";
  const volatile uint32_t *uid = (const volatile uint32_t *)0x1FFFF7E8;
  uint8_t n = 0;
  for (int w = 0; w < 3; w++) {
    uint32_t v = uid[w];
    for (int i = 7; i >= 0; i--) {
      str[n++] = hex[(v >> (i * 4)) & 0xFU];
    }
  }
  str[n] = '\0';
}

// Text helpers
namespace {
struct Text {
  char *buf;
  uint32_t max, len;
  void str(const char *s) {
    while (s && *s && len < max)
      buf[len++] = *s++;
  }
  void hex32(uint32_t v) {
    static const char d[] = "0123456789ABCDEF";
    str("0x");
    for (int i = 28; i >= 0; i -= 4) {
      if (len < max)
        buf[len++] = d[(v >> i) & 0xF];
    }
  }
  void dec(uint32_t v) {
    char t[11];
    int n = 0;
    do {
      t[n++] = (char)('0' + v % 10);
      v /= 10;
    } while (v);
    while (n && len < max)
      buf[len++] = t[--n];
  }
};

uint32_t get32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

// "FEATHE~1BIN" -> "FEATHE~1.BIN"
void name83(const char *n, char *out) {
  int k = 0;
  for (int i = 0; i < 8 && n[i] != ' '; i++)
    out[k++] = n[i];
  if (n[8] != ' ') {
    out[k++] = '.';
    for (int i = 8; i < 11 && n[i] != ' '; i++)
      out[k++] = n[i];
  }
  out[k] = 0;
}
} // namespace

/*!
 * @brief Identify the target (no halt or reset) and set up the drive.
 * @param dap Probe; begin() must have been called on it.
 * @param label Volume label, up to 11 characters.
 */
void DragDropEngine::begin(Adafruit_DAP_STM32 *dap, const char *label) {
  _dap = dap;
  _found = identify();
  _dap->dap_disconnect();
  _fat.setVolumeLabel(label);
  _state = IDLE;
  _result = NONE;
  rebuildFiles();
}

const char *DragDropEngine::targetName(void) const {
  return _found ? (_name ? _name : "unknown") : "none";
}

const char *DragDropEngine::nameFor(uint32_t id) {
  switch (id & 0xFFF) {
  case 0x413: return "STM32F405/407/415/417";
  case 0x419: return "STM32F42x/43x";
  case 0x423: return "STM32F401xB/C";
  case 0x433: return "STM32F401xD/E";
  case 0x431: return "STM32F411";
  case 0x441: return "STM32F412";
  case 0x421: return "STM32F446";
  default: return nullptr;
  }
}

// Flash sector containing an offset from TARGET_FLASH
bool DragDropEngine::sectorAt(uint32_t offset, uint8_t *sector,
                              uint32_t *start, uint32_t *size) {
  if (offset < 0x10000) {
    *sector = (uint8_t)(offset / 0x4000);
    *start = *sector * 0x4000U;
    *size = 0x4000;
  } else if (offset < 0x20000) {
    *sector = 4;
    *start = 0x10000;
    *size = 0x10000;
  } else if (offset < 0x100000) {
    *sector = (uint8_t)(5 + (offset - 0x20000) / 0x20000);
    *start = 0x20000 + (*sector - 5) * 0x20000U;
    *size = 0x20000;
  } else {
    return false;
  }
  return true;
}

// Connect at SWJ_CLOCK with IDLE_CYCLES.
bool DragDropEngine::connect(void) {
  dap_error = nullptr;
  if (!_dap->targetConnect(SWJ_CLOCK) ||
      !_dap->dap_transfer_configure(IDLE_CYCLES, 128, 128)) {
    dragdrop_dap_error(_dap->error_message);
  }
  return !dap_error;
}

// Identify the target without halting or resetting it.
bool DragDropEngine::identify(void) {
  _id = _kb = _dpidr = 0;
  _name = nullptr;
  if (!connect()) {
    return false;
  }
  if (!_dap->dap_read_idcode(&_dpidr)) {
    dragdrop_dap_error("no SWD target found: check the wiring and power");
    return false;
  }
  _dap->dap_target_prepare();
  _id = _dap->dap_read_word(DBGMCU_IDCODE);
  _kb = _dap->dap_read_word(FLASH_SIZE_REG) >> 16;
  _name = nameFor(_id);
  if (!dap_error && !_name) {
    dragdrop_dap_error("the target is not a supported STM32F4");
  }
  return !dap_error;
}

// Reset and halt the target before its firmware runs and identify it.
bool DragDropEngine::select(void) {
  if (!connect()) {
    return false;
  }
  uint32_t id = 0;
  bool found = _dap->select(&id);
  if (!dap_error && !found) {
    dragdrop_dap_error("the target is not a supported STM32F4");
  }
  if (!dap_error) {
    _id = id;
    _kb = _dap->target_device.flash_size / 1024;
    _name = nameFor(id);
  }
  return !dap_error;
}

// Program erased flash and read it back
bool DragDropEngine::programVerify(uint32_t addr, const uint8_t *buf,
                                   uint32_t len) {
  dap_error = nullptr;
  _dap->programBlock(addr, buf, len);
  if (!dap_error && !_dap->verifyFlash(addr, buf, len) && !dap_error) {
    dragdrop_dap_error("verify failed: the flash differs from the file");
  }
  return !dap_error;
}

void DragDropEngine::rebuildFiles(void) {
  _fat.clearFiles();
  _fat.addFile("DETAILS TXT", detailsText, this);
  if (_result == FAILED) {
    _fat.addFile("FAIL    TXT", failText, this);
  }
}

// STM32 vector table: initial SP in SRAM (or F4 CCM RAM), reset handler an
// odd address in flash.
bool DragDropEngine::looksLikeImage(const uint8_t *buf) {
  uint32_t sp = get32(buf), reset = get32(buf + 4);
  bool sp_ok = !(sp & 3) && ((sp >= 0x20000000 && sp <= 0x20080000) ||
                             (sp >= 0x10000000 && sp <= 0x10010000));
  bool reset_ok =
      (reset & 1) && reset >= TARGET_FLASH && reset < TARGET_FLASH + 0x100000;
  return sp_ok && reset_ok;
}

void DragDropEngine::parseDir(const uint8_t *buf) {
  const uint32_t first_free = _fat.firstFreeSector();
  for (uint32_t i = 0; i < VirtualFat::SECTOR_SIZE / 32; i++) {
    const uint8_t *e = buf + i * 32;
    uint8_t attr = e[11];
    if (e[0] == 0x00) {
      break; // end of directory
    }
    if (e[0] == 0xE5 || attr == 0x0F || (attr & 0x18)) {
      continue; // deleted, long-name part, volume label or directory
    }
    uint32_t cluster = (uint32_t)e[26] | ((uint32_t)e[27] << 8);
    if (cluster < 2) {
      continue; // empty file (size not written yet)
    }
    uint32_t start = VirtualFat::DATA_START +
                     (cluster - 2) * VirtualFat::SECTORS_PER_CLUSTER;
    if (start < first_free) {
      continue; // one of our generated files
    }
    // Update an existing entry or add (replacing the oldest if full).
    Seen *s = nullptr;
    for (uint8_t k = 0; k < _seen_n; k++) {
      if (_seen[k].lba == start)
        s = &_seen[k];
    }
    if (!s) {
      if (_seen_n < 4) {
        s = &_seen[_seen_n++];
      } else {
        memmove(&_seen[0], &_seen[1], sizeof(Seen) * 3);
        s = &_seen[3];
      }
    }
    s->lba = start;
    s->size = get32(e + 28);
    memcpy(s->name, e, 11);
  }
}

const DragDropEngine::Seen *DragDropEngine::seenAt(uint32_t lba) const {
  for (uint8_t k = 0; k < _seen_n; k++) {
    if (_seen[k].lba == lba)
      return &_seen[k];
  }
  return nullptr;
}

/*!
 * @brief Classify a sector written by the host.
 * @param lba Sector number.
 * @param buf Sector data; must stay valid until process() if true is
 *        returned.
 * @return True if the sector must be handed to process().
 */
bool DragDropEngine::write(uint32_t lba, const uint8_t *buf) {
  if (_state == REMOUNT) {
    return false;
  }
  if (lba >= VirtualFat::ROOT_START && lba < VirtualFat::DATA_START) {
    parseDir(buf);
    _t_dir = millis();
    if (_state == PROGRAMMING)
      _t_last = _t_dir;
    return false;
  }
  if (lba < VirtualFat::DATA_START || lba >= VirtualFat::TOTAL_SECTORS) {
    return false; // boot sector / FAT: nothing to do
  }
  if (_state == IDLE) {
    if (!looksLikeImage(buf)) {
      return false; // not the start of a firmware image
    }
    _state = PROGRAMMING;
    _start_lba = lba;
    _selected = _failed = false;
    _erased = _bytes = 0;
    _held = false;
    _error = nullptr;
    _t_start = millis();
  }
  _t_last = millis();
  if (_failed || lba < _start_lba) {
    return false; // acknowledged, ignored
  }
  _p_lba = lba;
  _p_buf = buf;
  _pending = true;
  return true;
}

void DragDropEngine::fail(const char *error) {
  if (!_failed) {
    _failed = true;
    _error = error;
  }
}

/*! @brief Program the sector handed over by write(). May take seconds. */
void DragDropEngine::process(void) {
  if (!_pending) {
    return;
  }
  _pending = false;

  // First sector: reset the target and halt it before its firmware runs.
  if (!_selected) {
    _selected = true;
    _found = select();
    if (!_found) {
      fail(dap_error);
    }
  }
  if (!_failed) {
    // Program the previously held sector
    if (_held) {
      programSector(_hold_lba, _hold);
    }
    memcpy(_hold, _p_buf, VirtualFat::SECTOR_SIZE);
    _hold_lba = _p_lba;
    _held = true;
  }
  _t_last = millis(); // after the (possibly slow) work
}

// Erase, program and verify one sector of the image.
void DragDropEngine::programSector(uint32_t lba, const uint8_t *buf) {
  if (_failed) {
    return;
  }
  uint32_t offset = (lba - _start_lba) * VirtualFat::SECTOR_SIZE;
  uint32_t len = VirtualFat::SECTOR_SIZE;
  const Seen *file = seenAt(_start_lba);
  if (file && file->size) {
    if (offset >= file->size) {
      return; // past the end of the file
    }
    if (file->size - offset < len) {
      len = file->size - offset;
    }
  }
  uint8_t sector;
  uint32_t sec_start, sec_size;
  if (offset + len > _kb * 1024U ||
      !sectorAt(offset, &sector, &sec_start, &sec_size)) {
    fail("image is larger than the target's flash");
    return;
  }
  if (!(_erased & (1UL << sector))) {
    _erased |= 1UL << sector;
    dap_error = nullptr;
    _dap->program_start(TARGET_FLASH + sec_start, sec_size); // erases
    if (dap_error) {
      fail(dap_error);
      return;
    }
  }
  // Erased flash reads 0xFF and this sector was erased
  bool blank = true;
  for (uint32_t i = 0; i < len && blank; i++) {
    blank = (buf[i] == 0xFF);
  }
  if (blank) {
    if (offset + len > _bytes) {
      _bytes = offset + len;
    }
    return;
  }
  uint32_t addr = TARGET_FLASH + offset;
  uint32_t aligned = len & ~3U;
  uint8_t tail[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(tail, buf + aligned, len & 3);
  bool ok = true;
  if (aligned) {
    ok = programVerify(addr, buf, aligned);
  }
  if (ok && (len & 3)) {
    ok = programVerify(addr + aligned, tail, 4);
  }
  if (!ok) {
    fail(dap_error);
  } else if (offset + len > _bytes) {
    _bytes = offset + len;
  }
}

void DragDropEngine::finish(uint32_t now_ms, bool unrecognised_hex) {
  const Seen *file = seenAt(_start_lba);
  if (_state == PROGRAMMING) {
    if (_held) { // the last sector
      programSector(_hold_lba, _hold);
      _held = false;
      _t_last = millis();
    }
    if (_selected) {
      dap_error = nullptr;
      _dap->deselect(); // lock the flash, reset the target to run
      _dap->dap_disconnect();
      if (dap_error) {
        fail(dap_error);
      }
    }
    _result = _failed ? FAILED : OK;
    _res_error = _error;
    _res_bytes = _bytes;
    _res_ms = _t_last - _t_start;
  } else { // a .bin/.hex that never started a session
    file = nullptr;
    for (uint8_t k = 0; k < _seen_n; k++) {
      if (_seen[k].size)
        file = &_seen[k];
    }
    _result = FAILED;
    _res_error = unrecognised_hex
                     ? "Intel HEX files are not supported yet: use a .bin"
                     : "not recognised as STM32 firmware (no valid vector "
                       "table at the start of the file)";
    _res_bytes = _res_ms = 0;
  }
  if (file) {
    name83(file->name, _res_name);
  } else {
    strcpy(_res_name, "(unknown)");
  }

  _state = REMOUNT; // drive reports "not ready", host re-reads it
  _t_remount = now_ms;
  _pending = _selected = _failed = _held = false;
  _seen_n = 0;
  rebuildFiles();
}

/*!
 * @brief End drops after a write gap and finish remounts. Call often.
 * @param now_ms Current millis().
 */
void DragDropEngine::poll(uint32_t now_ms) {
  if (_state == PROGRAMMING && !_pending && now_ms - _t_last > QUIET_MS) {
    finish(now_ms, false);
  } else if (_state == REMOUNT && now_ms - _t_remount > REMOUNT_MS) {
    _state = IDLE;
  } else if (_state == IDLE && _seen_n && now_ms - _t_dir > UNRECOGNISED_MS) {
    // A firmware-named file was written but never started a session.
    for (uint8_t k = 0; k < _seen_n; k++) {
      const char *ext = _seen[k].name + 8;
      bool bin = memcmp(ext, "BIN", 3) == 0, hex = memcmp(ext, "HEX", 3) == 0;
      if (_seen[k].size && (bin || hex)) {
        finish(now_ms, hex);
        return;
      }
    }
    _seen_n = 0; // ignore spurious files
  }
}

uint32_t DragDropEngine::detailsText(void *context, char *buf, uint32_t max) {
  DragDropEngine *self = (DragDropEngine *)context;
  Text t = {buf, max, 0};

  t.str("Adafruit_DAP drag-and-drop programmer\r\n\r\n");
  t.str("Probe     : QT Py CH32V203\r\n");
  char serial[32];
  ch32_serial_string(serial);
  t.str("Unique ID : ");
  t.str(serial);
  t.str("\r\n");
  t.str("Interfaces: this drive, USB serial bridge\r\n\r\n");
  if (self->_found) {
    t.str("Target    : ");
    t.str(self->_name ? self->_name : "unknown");
    t.str("\r\nDEV_ID    : ");
    t.hex32(self->_id & 0xFFF);
    t.str("\r\nFlash     : ");
    t.dec(self->_kb);
    t.str(" KB\r\nSWD DPIDR : ");
    t.hex32(self->_dpidr);
    t.str("\r\n\r\n");
  } else {
    t.str("Target    : none detected (check SWD wiring and target "
          "power)\r\n\r\n");
  }
  t.str("Drop a firmware .bin onto this drive to program the target.\r\n");
  t.str("Last drop : ");
  if (self->_result == NONE) {
    t.str("none yet\r\n");
  } else {
    t.str(self->_res_name);
    if (self->_result == OK) {
      t.str(": OK, ");
      t.dec(self->_res_bytes);
      t.str(" bytes in ");
      t.dec(self->_res_ms / 1000);
      t.str(".");
      t.dec((self->_res_ms % 1000) / 100);
      t.str(" s\r\n");
    } else {
      t.str(": FAILED (see FAIL.TXT)\r\n");
    }
  }
  return t.len;
}

uint32_t DragDropEngine::failText(void *context, char *buf, uint32_t max) {
  DragDropEngine *self = (DragDropEngine *)context;
  Text t = {buf, max, 0};
  t.str("Programming failed\r\n\r\nFile : ");
  t.str(self->_res_name);
  t.str("\r\nError: ");
  t.str(self->_res_error ? self->_res_error : "unknown");
  t.str("\r\n");
  return t.len;
}
