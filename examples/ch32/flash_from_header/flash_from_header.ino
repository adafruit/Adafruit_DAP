// Standalone PicoBrains programmer: SWIO GP2, RESET GP27, common GND.
// Power the Metro separately through USB. SWCLK is not used.
// This embedded image is ONLY for Metro CH32V407RET6 Rev A (25 MHz crystal).
#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>
#include "tinyuf2_metro407.h"

Adafruit_DAP_CH32 programmer;
bool programmed = false;
const char *result = "Starting";

void setup() {
  Serial.begin(115200); // Optional diagnostics; no Serial Monitor is required.
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);

  Serial.print("Adafruit CH32 standalone programmer: embedded image\n\r");

  Adafruit_DAP_CRC32 imageCRC;
  imageCRC.add(firmware, sizeof(firmware));
  if (imageCRC.get() != firmware_crc32) {
    fail("Embedded image CRC mismatch");
    return;
  }
  if (!programmer.begin(2, 27)) {
    fail(programmer.error_message);
    return;
  }
  if (!programmer.targetConnect()) {
    fail(programmer.error_message);
    return;
  }
  uint32_t id;
  if (!programmer.select(&id)) {
    fail(programmer.error_message);
    return;
  }
  // Ignore the silicon revision nibble, but require the image's exact chip.
  if ((id & 0xffffff0f) != 0x46720002) {
    fail("This image requires CH32V407RET");
    return;
  }
  Serial.print("Target identified; erasing image sectors\n\r");
  if (programmer.program_start(0, sizeof(firmware)) == UINT32_MAX) {
    fail(programmer.error_message);
    return;
  }
  Serial.print("Programming and verifying TinyUF2\n\r");
  if (!programmer.programFlash(0, firmware, sizeof(firmware), true)) {
    fail(programmer.error_message);
    return;
  }
  programmer.deselect();
  result = "PASS: TinyUF2 programmed and verified from Pico flash";
  programmed = true;
}

void loop() {
  // Repeat the result for a monitor opened after standalone programming.
  Serial.printf("%s\n\r", result);
  digitalWrite(LED_BUILTIN, programmed);
  delay(1000);
}

void fail(const char *message) {
  result = message;
  programmer.deselect();
  digitalWrite(LED_BUILTIN, LOW);
}
