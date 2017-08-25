#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>

//teensy only, otherwise change sd cs pin
#define SD_CS BUILTIN_SDCARD
#define SWDIO 10
#define SWCLK 11
#define SWRST 12

#define FILENAME "2772cipy.bin"

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

  Serial.println("DBG: dap.begin()");
  dap.begin(SWCLK, SWDIO, SWRST, &error);
  Serial.println("DBG: DONE");

/*
    // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
    error("Card failed, or not present");
  }
  Serial.println("Card initialized");

  File dataFile = SD.open(FILENAME);

  if(!dataFile){
     error("Couldn't open file");
  }
*/

  Serial.print("Connecting...");
  Serial.println("DBG: dap.dap_disconnect()");
  if (! dap.dap_disconnect())                      error(dap.error_message);
  Serial.println("DBG: DONE");

  char debuggername[100];
  Serial.println("DBG: dap.dap_get_debugger_info()");
  if (! dap.dap_get_debugger_info(debuggername))   error(dap.error_message);
  Serial.println("DBG: DONE");
  Serial.print(debuggername); Serial.print("\n\r");

  Serial.println("DBG: dap.dap_connect()");
  if (! dap.dap_connect())                         error(dap.error_message);
  Serial.println("DBG: DONE");

  Serial.println("DBG: dap.dap_transfer_configure()");
  if (! dap.dap_transfer_configure(0, 128, 128))   error(dap.error_message);
  Serial.println("DBG: DONE");
  Serial.println("DBG: dap.dap_swd_configure()");
  if (! dap.dap_swd_configure(0))                  error(dap.error_message);
  Serial.println("DBG: DONE");
  Serial.println("DBG: dap.dap_reset_link()");
  if (! dap.dap_reset_link())                      error(dap.error_message);
  Serial.println("DBG: DONE");
  Serial.println("DBG: dap.dap_swj_clock");
  if (! dap.dap_swj_clock(50))               error(dap.error_message);
  Serial.println("DBG: DONE");
  Serial.println("DBG: dap.dap_target_prepare()");
  dap.dap_target_prepare();
  Serial.println("DBG: DONE");

  uint32_t dsu_did;
  Serial.println("DBG: Starting dap.select()");
  if (! dap.select(&dsu_did)) {
    Serial.print("Unknown device found 0x"); Serial.println(dsu_did, HEX);
    error("Unknown device found");
  }
  Serial.print("DBG: Got 0x"); Serial.println(dsu_did, HEX);
  Serial.println("DBG: DONE");
  for (device_t *device = dap.devices; device->dsu_did > 0; device++) {
    if (device->dsu_did == dsu_did) {
      Serial.print("Found Target: ");
      Serial.print(device->name);
      Serial.print("\tFlash size: ");
      Serial.print(device->flash_size);
      Serial.print("\tFlash pages: ");
      Serial.println(device->n_pages);
      //Serial.print("Page size: "); Serial.println(device->flash_size / device->n_pages);
    }
  }

  /* Example of how to read and set fuses
  Serial.print("Fuses... ");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  dap._USER_ROW.WDT_Period = 0x0A;
  dap.fuseWrite();
  */

  Serial.println(" done.");

/*
  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");

  Serial.print("Programming... ");
  Serial.print(millis());
  uint32_t addr = dap.program_start();
*/

/*
  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      dataFile.read(buf, BUFSIZE);
      dap.programBlock(addr, buf);
      addr += BUFSIZE;
  }
  dataFile.close();
*/

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
