// Identify and read 16 bytes without erasing or programming the target.
// PicoBrains GP2 = SWIO, GP27 = RESET; common GND. Resets the target.
#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>

Adafruit_DAP_CH32 programmer;
bool passed = false;
uint32_t chipID = 0;
uint8_t firstBytes[16];

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);
  Serial.print("Adafruit CH32 read-only identification\n\r");

  if (!programmer.begin(2, 27)) {
    programmer.deselect();
    return;
  }
  if (!programmer.targetConnect()) {
    programmer.deselect();
    return;
  }
  if (!programmer.select(&chipID)) {
    programmer.deselect();
    return;
  }
  Serial.print("PASS: target identified\n\r");
  if (!programmer.dap_read_block(0x08000000, firstBytes, sizeof(firstBytes))) {
    programmer.deselect();
    return;
  }
  passed = true;
  programmer.deselect();
}

void loop() {
  Serial.printf("CH32 identify: %s, ID %08lx, %s\n\r", passed ? "PASS" : "FAIL", chipID,
                programmer.error_message ? programmer.error_message : "no error");
  if (passed) {
    for (uint8_t value : firstBytes) {
      Serial.printf("%02x ", value);
    }
    Serial.print("\n\r");
  }
  digitalWrite(LED_BUILTIN, passed);
  delay(1000);
}
