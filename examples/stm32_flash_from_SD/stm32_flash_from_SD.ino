#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>

#define SD_CS 10

#define SWDIO 11
#define SWCLK 12
#define SWRST 13

#define FILENAME "FIRMWARE.BIN"

// the more the better
#define BUFSIZE   (16*1024)

// buffer should be word algined for STM32
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_STM32 dap;

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
  
    // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
    error("Card failed, or not present");
  }
  Serial.println("Card initialized");

  File dataFile = SD.open(FILENAME);

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
  if (! dap.dap_swj_clock(50))               error(dap.error_message);
  dap.dap_target_prepare();

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
  
  Serial.print(" done in ");
  Serial.print(millis()-start_ms);
  Serial.println(" ms");

  //------------- Programming -------------//
  Serial.print("Programming ");
  Serial.print(dataFile.size());
  Serial.print(" bytes ...");

  uint32_t addr = FLASH_START_ADDR;

  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      uint32_t count = dataFile.read(buf, BUFSIZE);
      dap.programBlock(addr, buf, count);
      addr += count;
  }
  dataFile.close();

  Serial.print(" done in ");
  Serial.print(millis()-start_ms);
  Serial.println(" ms");

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
