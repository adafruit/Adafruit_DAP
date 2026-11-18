// Destructive regression on a Metro CH32V407 with the bundled TinyUF2 image.
// It uses the first bootloader sector, then restores and verifies TinyUF2.
// PicoBrains GP2 = SWIO, GP27 = RESET; common GND. No Serial Monitor required.
#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>
#include <examples/ch32/flash_from_header/tinyuf2_metro407.h>

Adafruit_DAP_CH32 programmer;
uint8_t data[256];
uint8_t readback[256];
uint8_t beforeWrite[256];
const char *result = "Starting";
bool changedFlash = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print("Adafruit CH32 exact image-length regression\n\r");

  Adafruit_DAP_CRC32 crc;
  crc.add(firmware, sizeof(firmware));
  if (crc.get() != firmware_crc32) {
    finish("FAIL: restore image CRC");
    return;
  }
  if (!programmer.begin(2, 27)) {
    finish("FAIL: begin");
    return;
  }
  if (!programmer.targetConnect()) {
    finish("FAIL: connect");
    return;
  }
  uint32_t id;
  if (!programmer.select(&id) || (id & 0xffffff0f) != 0x46720002) {
    finish("FAIL: expected CH32V407RET");
    return;
  }
  if (programmer.computeFlashCRC32(0, sizeof(firmware)) != firmware_crc32) {
    finish("FAIL: target must contain bundled TinyUF2 first");
    return;
  }
  Serial.print("PASS: target and restore image checked\n\r");

  memset(data, 0xa5, sizeof(data));
  changedFlash = true;
  if (programmer.program_start(0, 1) == UINT32_MAX) {
    finish("FAIL: prepare one byte");
    return;
  }
  if (!programmer.dap_read_block(Adafruit_DAP_CH32::FLASH_BASE,
                                 beforeWrite, sizeof(beforeWrite))) {
    finish("FAIL: initial page snapshot");
    return;
  }
  if (programmer.programFlash(0, data, sizeof(data))) {
    finish("FAIL: oversized one-byte image accepted");
    return;
  }
  if (!pageUnchanged(0)) {
    finish("FAIL: rejected write changed flash");
    return;
  }
  Serial.print("PASS: 256 bytes rejected for a one-byte image; page unchanged\n\r");

  if (programmer.program_start(0, 257) == UINT32_MAX ||
      !programmer.programFlash(0, data, sizeof(data))) {
    finish("FAIL: first full page");
    return;
  }
  if (!programmer.dap_read_block(Adafruit_DAP_CH32::FLASH_BASE + 256,
                                 beforeWrite, sizeof(beforeWrite))) {
    finish("FAIL: final page snapshot");
    return;
  }
  if (programmer.programFlash(256, data, sizeof(data))) {
    finish("FAIL: oversized final chunk accepted");
    return;
  }
  if (!pageUnchanged(256)) {
    finish("FAIL: rejected final chunk changed flash");
    return;
  }
  if (programmer.programFlash(256, data, 1)) {
    finish("FAIL: rejected write left the session prepared");
    return;
  }
  Serial.print("PASS: oversized final chunk rejected and session invalidated\n\r");

  if (programmer.program_start(0, 257) == UINT32_MAX ||
      !programmer.programFlash(0, data, sizeof(data)) ||
      !programmer.programFlash(256, data, 1)) {
    finish("FAIL: exact final chunk");
    return;
  }
  if (!programmer.dap_read_block(Adafruit_DAP_CH32::FLASH_BASE + 256,
                                 readback, sizeof(readback))) {
    finish("FAIL: final-page readback");
    return;
  }
  if (readback[0] != 0xa5) {
    finish("FAIL: final image byte");
    return;
  }
  for (uint32_t i = 1; i < sizeof(readback); i++) {
    if (readback[i] != 0xff) {
      finish("FAIL: final-page padding");
      return;
    }
  }
  if (programmer.programFlash(257, data, 1)) {
    finish("FAIL: append after image completion accepted");
    return;
  }
  Serial.print("PASS: exact final byte and 255 padding bytes; append rejected\n\r");
  finish("PASS: exact image limits, rejection without writes, final padding; TinyUF2 restored");
}

void loop() {
  Serial.printf("%s\n\r", result);
  delay(1000);
}

bool pageUnchanged(uint32_t address) {
  // Compare physical flash before/after instead of assuming erased bytes are FF.
  if (!programmer.dap_read_block(Adafruit_DAP_CH32::FLASH_BASE + address,
                                 readback, sizeof(readback))) {
    return false;
  }
  return memcmp(readback, beforeWrite, sizeof(readback)) == 0;
}

void finish(const char *message) {
  result = message;
  if (changedFlash) {
    Serial.print("Restoring the complete TinyUF2 image\n\r");
    if (programmer.program_start(0, sizeof(firmware)) == UINT32_MAX ||
        !programmer.programFlash(0, firmware, sizeof(firmware), true)) {
      result = "FAIL: TinyUF2 restore failed; reload the embedded-image example";
    }
  }
  programmer.deselect();
}
