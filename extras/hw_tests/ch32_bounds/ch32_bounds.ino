// Read-only regression against the bundled Metro CH32V407 TinyUF2 image.
// PicoBrains GP2 = SWIO, GP27 = RESET; common GND. Resets the target.
#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>
#include <examples/ch32/flash_from_header/tinyuf2_metro407.h>

Adafruit_DAP_CH32 programmer;
bool passed = false;
const char *stage = "Starting";
const uint32_t imageSize = sizeof(firmware);
const uint32_t expectedCRC = firmware_crc32;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print("Adafruit CH32 flash-range checks\n\r");

  if (!programmer.begin(2, 27)) {
    fail("begin");
    return;
  }
  if (!programmer.targetConnect()) {
    fail("connect");
    return;
  }
  uint32_t id;
  if (!programmer.select(&id)) {
    fail("identify");
    return;
  }
  if (id != 0x46720012) {
    fail("wrong fixture target");
    return;
  }
  Serial.print("PASS: target identified\n\r");

  if (programmer.computeFlashCRC32(0, imageSize) != expectedCRC) {
    fail("initial CRC");
    return;
  }
  Serial.print("PASS: complete image CRC\n\r");

  uint8_t bytes[8];
  if (!programmer.dap_read_block(0x08000001, bytes, sizeof(bytes))) {
    fail("unaligned byte read");
    return;
  }
  if (memcmp(bytes, firmware + 1, sizeof(bytes))) {
    fail("byte read mismatch");
    return;
  }
  Serial.print("PASS: unaligned byte read\n\r");

  if (programmer.program_start(1, 256) != UINT32_MAX) {
    fail("unaligned image accepted");
    return;
  }
  if (programmer.program_start(0x0807f000, 4097) != UINT32_MAX) {
    fail("past-flash range accepted");
    return;
  }
  if (programmer.program_start(0xfffff000, 8192) != UINT32_MAX) {
    fail("overflow range accepted");
    return;
  }
  Serial.print("PASS: invalid ranges rejected\n\r");

  if (programmer.programFlash(0, bytes, sizeof(bytes))) {
    fail("unprepared write accepted");
    return;
  }
  if (programmer.protectBoot() || programmer.unprotectBoot()) {
    fail("option bytes accepted");
    return;
  }
  Serial.print("PASS: unprepared writes and option changes rejected\n\r");

  if (programmer.computeFlashCRC32(0, imageSize) != expectedCRC) {
    fail("final CRC");
    return;
  }
  programmer.deselect();
  stage = "PASS: identify, full CRC, byte read, invalid ranges and write guards; firmware unchanged";
  passed = true;
}
void loop() {
  Serial.printf("%s\n\r", stage);
  delay(1000);
}

void fail(const char *message) {
  stage = message;
  if (programmer.error_message) {
    Serial.printf("%s\n\r", programmer.error_message);
  }
  programmer.deselect();
}
