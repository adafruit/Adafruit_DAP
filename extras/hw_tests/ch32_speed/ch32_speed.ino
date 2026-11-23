// Programs the bundled Metro CH32V407 TinyUF2 image and times full verification.
// PicoBrains GP2 = SWIO, GP27 = RESET; common GND, separately powered Metro.
#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>
#include <examples/ch32/flash_from_header/tinyuf2_metro407.h>

Adafruit_DAP_CH32 programmer;
const char *result = "Starting";
uint32_t eraseTime = 0;
uint32_t programTime = 0;
uint32_t readTime = 0;
uint32_t verifyTime = 0;
bool passed = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print("Adafruit CH32 programming speed and readback check\n\r");

  Adafruit_DAP_CRC32 imageCRC;
  imageCRC.add(firmware, sizeof(firmware));
  if (imageCRC.get() != firmware_crc32) {
    finish("FAIL: source image CRC");
    return;
  }
  if (!programmer.begin(2, 27) || !programmer.targetConnect()) {
    finish("FAIL: connect");
    return;
  }
  uint32_t id;
  if (!programmer.select(&id) || (id & 0xffffff0f) != 0x46720002) {
    finish("FAIL: expected CH32V407RET");
    return;
  }
  uint32_t started = millis();
  if (programmer.program_start(0, sizeof(firmware)) == UINT32_MAX) {
    finish("FAIL: erase");
    return;
  }
  eraseTime = millis() - started;
  started = millis();
  if (!programmer.programFlash(0, firmware, sizeof(firmware), false)) {
    finish("FAIL: program/readback");
    return;
  }
  programTime = millis() - started;
  started = millis();
  if (!programmer.verifyFlash(0, firmware, sizeof(firmware))) {
    finish("FAIL: independent complete byte readback");
    return;
  }
  verifyTime = millis() - started;
  started = millis();
  if (programmer.computeFlashCRC32(0, sizeof(firmware)) != firmware_crc32) {
    finish("FAIL: complete image CRC");
    return;
  }
  readTime = millis() - started;

  // Program the expected image before checking unaligned and partial-word CRCs.
  // Keep these checks outside the measured programming and verification times.
  const uint32_t lengths[] = {1, 3, 4, 55, 56, 63, 64, 255, 256, 257};
  for (uint32_t length : lengths) {
    Adafruit_DAP_CRC32 crc;
    crc.add(firmware + 1, length);
    if (programmer.computeFlashCRC32(1, length) != crc.get()) {
      finish("FAIL: target CRC boundary");
      return;
    }
  }
  Serial.print("PASS: target CRC boundary lengths match host CRC\n\r");

  uint8_t wrong[4];
  memcpy(wrong, firmware, sizeof(wrong));
  wrong[0]++;
  if (programmer.verifyFlash(0, wrong, sizeof(wrong))) {
    finish("FAIL: incorrect expected data accepted");
    return;
  }
  // A verification failure must not prevent a later read from the same image.
  if (!programmer.verifyFlash(0, firmware, sizeof(wrong))) {
    finish("FAIL: read after mismatch");
    return;
  }
  passed = true;
  finish("PASS: complete readback, CRC and mismatch detection");
}

void loop() {
  Serial.printf("%s; erase=%lu ms, program+pageCRC=%lu ms, byte-readback=%lu ms, CRC=%lu ms\n\r",
                result, eraseTime, programTime, verifyTime, readTime);
  delay(1000);
}

void finish(const char *message) {
  result = message;
  if (!passed && programmer.error_message) {
    Serial.printf("%s\n\r", programmer.error_message);
  }
  programmer.deselect();
}
