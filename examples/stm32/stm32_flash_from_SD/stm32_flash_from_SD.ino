#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SdFat.h>

#define SD_CS 10

#define SWDIO 12
#define SWCLK 11
#define SWRST 9

#define FILENAME "FIRMWARE.BIN"

// the more the better
#define BUFSIZE   (16*1024)

// buffer should be word algined for STM32
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_STM32 dap;

SdFat SD;

// STM32 auto map 0x00 to 0x08000000, use 0 for simplicity
#define FLASH_START_ADDR    0

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

  //------------- SD Card open -------------//
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
    error("Card failed, or not present");
  }
  Serial.println("Card initialized");

  File32 dataFile = SD.open(FILENAME);

  if(!dataFile){
     error("Couldn't open file");
  }

  //------------- DAP connecting -------------//
  Serial.println("Connecting...");
  if ( !dap.targetConnect() ) {
    error(dap.error_message);
  }

  char debuggername[100];
  dap.dap_get_debugger_info(debuggername);
  Serial.print(debuggername); Serial.print("\n\r");

  uint32_t dsu_did;
  if (! dap.select(&dsu_did)) {
    error("No STM32 device found!");
  }

  Serial.print("Found Target\t");
  Serial.println(dap.target_device.name);
  Serial.print("Flash size\t");
  Serial.print(dap.target_device.flash_size / 1024);
  Serial.println(" KBs");

  //------------- Preparing sectors -------------//
  uint32_t start_ms;
  Serial.print("Preparing ... ");
  start_ms = millis();

  // preparing flash sector with address = 0, size = Binary size
  dap.programPrepare(FLASH_START_ADDR, dataFile.size());  
  Serial.print(" done in "); Serial.print(millis()-start_ms); Serial.println(" ms");

  //------------- Programming -------------//
  Serial.print("Programming ");
  Serial.print(dataFile.size());
  Serial.print(" bytes ...");

  uint32_t addr = FLASH_START_ADDR;
  start_ms = millis();
  
  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      uint32_t count = dataFile.read(buf, BUFSIZE);
      dap.programBlock(addr, buf, count);
      addr += count;
  }

  Serial.print(" done in "); Serial.print(millis()-start_ms); Serial.println(" ms");

  //------------- Verifying -------------//
  Serial.print("Verifying ...");
  dataFile.seek(0); // seek to file beginning

  addr = FLASH_START_ADDR;
  start_ms = millis();
  
  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      uint32_t count = dataFile.read(buf, BUFSIZE);
      if ( !dap.verifyFlash(addr, buf, count) ) {
        error("Flash mismatched");
      }
      addr += count;
  }

  Serial.print(" done in "); Serial.print(millis()-start_ms); Serial.println(" ms");

  dataFile.close();
  
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
