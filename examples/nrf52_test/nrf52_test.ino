#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>

//teensy only, otherwise change sd cs pin
#define SD_CS 10
#define SWDIO 11
#define SWCLK 12
#define SWRST 13

#define FILE_S132       "S132_201.BIN"
#define FILE_BOOTLOADER "FT52_050.BIN"

#define BOOTLOADER_ADDR 0x74000
#define S132_ADDR       0

#define BUFSIZE 256       //don't change!
uint8_t buf[BUFSIZE];

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_nRF5x dap;

// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1);
}


void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  while(!Serial) {
    delay(1);         // will pause the chip until it opens serial console
  }

  dap.begin(SWCLK, SWDIO, SWRST, &error);

    // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
    error("Card failed, or not present");
  }
  Serial.println("Card initialized");

  File dataFile = SD.open(FILE_S132);

  if(!dataFile){
     error("Couldn't open file");
  }

  Serial.print("Connecting...");
  if (! dap.dap_disconnect())                      error(dap.error_message);

  char debuggername[100];
  if (! dap.dap_get_debugger_info(debuggername))   error(dap.error_message);
  Serial.print(debuggername); Serial.print("\n\r");

  if (! dap.dap_connect())                         error(dap.error_message);

  if (! dap.dap_transfer_configure(0, 128, 128))   error(dap.error_message);
  if (! dap.dap_swd_configure(0))                  error(dap.error_message);
  if (! dap.dap_reset_link())                      error(dap.error_message);
  if (! dap.dap_swj_clock(50))                     error(dap.error_message);
  dap.dap_target_prepare();

  uint32_t dsu_did;
  if (! dap.select(&dsu_did)) {
    error("No nRF5x device found!");
  }

  Serial.print("Found Target: ");
  Serial.print(dap.target_device.name);
  Serial.print("\tFlash size: ");
  Serial.print(dap.target_device.flash_size);
  Serial.print("\tFlash pages: ");
  Serial.println(dap.target_device.n_pages);
  //Serial.print("Page size: "); Serial.println(dap.target_device.flash_size / dap.target_device.n_pages);

  /* Example of how to read and set fuses
  Serial.print("Fuses... ");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  dap._USER_ROW.WDT_Period = 0x0A;
  dap.fuseWrite();
  */

  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");

  Serial.print("Programming... ");
  Serial.print(millis());

  uint32_t addr = S132_ADDR;

  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      dataFile.read(buf, BUFSIZE);
      dap.program(addr, buf, BUFSIZE);
      addr += BUFSIZE;
  }
  dataFile.close();

  Serial.println("\nDone!");

  dap.deselect();
  dap.dap_disconnect();
}

void loop() {
  //blink led on the host to show we're done
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}
