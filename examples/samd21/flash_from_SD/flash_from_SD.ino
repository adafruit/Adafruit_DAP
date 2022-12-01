#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SdFat.h>

#define SD_CS 4
#define SWDIO 12
#define SWCLK 11
#define SWRST 9

#define FILENAME "fw.bin"

const int BUFSIZE = Adafruit_DAP_SAM::PAGESIZE;
uint8_t buf[BUFSIZE];

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_SAM dap;

SdFat SD;

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

  File32 dataFile = SD.open(FILENAME);

  if(!dataFile){
     error("Couldn't open file");
  }
  
  Serial.println("Connecting...");
  if ( !dap.targetConnect() ) {
    error(dap.error_message);
  }

  char debuggername[100];
  dap.dap_get_debugger_info(debuggername);
  Serial.print(debuggername); Serial.print("\n\r");

  uint32_t dsu_did;
  if (! dap.select(&dsu_did)) {
    Serial.print("Unknown device found 0x"); Serial.print(dsu_did, HEX);
    error("Unknown device found");
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
  dap._USER_ROW.bit.WDT_Period = 0x0A;
  dap.fuseWrite();
  Serial.println(" done.");
  */

  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");
  
  Serial.print("Programming... ");
  unsigned long t = millis();
  uint32_t addr = dap.program_start();

  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      dataFile.read(buf, BUFSIZE);
      dap.programBlock(addr, buf);
      addr += BUFSIZE;
  }
  dataFile.close();
  Serial.println(millis() - t);
  Serial.println("\nDone!");
  dap.dap_set_clock(50);

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
