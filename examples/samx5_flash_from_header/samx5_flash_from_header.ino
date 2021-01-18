#include "Adafruit_DAP.h"
#include "samd51_uf2.h"

#define SWDIO 12
#define SWCLK 11
#define SWRST 9

//create a DAP for programming Atmel SAMx5 devices
Adafruit_DAP_SAMx5 dap;

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

  /* Example of how to read and set fuses
  Serial.print("Fuses... ");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  dap._USER_ROW.bit.NVM_BOOT = 0x0B;
  dap.fuseWrite();
  Serial.println(" done.");
  */

  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");
  
  unsigned long t = millis();

  dap.programFlash(0, binfile, sizeof(binfile), true);
  
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
