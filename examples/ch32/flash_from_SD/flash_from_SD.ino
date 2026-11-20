// Standalone PicoBrains: SWIO GP2, RESET GP27, SD CS GP17.
// Standard Pico SPI: MISO GP16, SCK GP18, MOSI GP19. Common target GND.
// Put a raw binary (not ELF or UF2) named METRO407.BIN on the card.
// This example requires the saved Metro CH32V407 TinyUF2 image below.
#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat_Adafruit_Fork.h>

Adafruit_DAP_CH32 programmer;
SdFat SD;
File32 image;
uint8_t buffer[Adafruit_DAP_CH32::PAGESIZE];
const uint32_t imageSize = 18584;
const uint32_t imageCRC32 = 0x609ef709;
bool programmed = false;
const char *result = "Starting";

void setup() {
  Serial.begin(115200); // Standalone: do not wait for the USB host or DTR.
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);

  Serial.print("Adafruit CH32 standalone programmer: microSD image\n\r");

  if (!SD.begin(17, SD_SCK_MHZ(16))) {
    fail("Cannot initialize the PicoBrains SD card");
    return;
  }
  image = SD.open("METRO407.BIN", O_RDONLY);
  if (!image || image.size() != imageSize) {
    fail("Missing METRO407.BIN or incorrect image size");
    return;
  }
  // Validate the entire input before erasing any target sectors.
  Adafruit_DAP_CRC32 crc;
  uint32_t remaining = imageSize;
  while (remaining) {
    uint32_t count = min(remaining, (uint32_t)sizeof(buffer));
    if (image.read(buffer, count) != (int)count) {
      fail("Cannot read complete SD image");
      return;
    }
    crc.add(buffer, count);
    remaining -= count;
  }
  if (crc.get() != imageCRC32 || !image.seekSet(0)) {
    fail("SD image CRC mismatch or rewind failed");
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
  if ((id & 0xffffff0f) != 0x46720002) {
    fail("This image requires CH32V407RET");
    return;
  }
  if (programmer.program_start(0, imageSize) == UINT32_MAX) {
    fail(programmer.error_message);
    return;
  }
  Serial.print("Programming and verifying TinyUF2 from microSD\n\r");
  uint32_t address = 0;
  while (address < imageSize) {
    uint32_t count = min(imageSize - address, (uint32_t)sizeof(buffer));
    if (image.read(buffer, count) != (int)count) {
      fail("SD read failed during programming");
      return;
    }
    if (!programmer.programFlash(address, buffer, count, true)) {
      fail(programmer.error_message);
      return;
    }
    address += count;
  }
  image.close();
  programmer.deselect();
  result = "PASS: TinyUF2 programmed and verified from microSD";
  programmed = true;
}

void loop() {
  Serial.printf("%s\n\r", result);
  digitalWrite(LED_BUILTIN, programmed);
  delay(1000);
}

void fail(const char *message) {
  result = message;
  image.close();
  programmer.deselect();
  digitalWrite(LED_BUILTIN, LOW);
}
