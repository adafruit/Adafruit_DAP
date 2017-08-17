#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
#define FILENAME "fw.bin"

#define BUFSIZE 256 //don't change

//create a seesaw with m0 DAP support
dap_m0p dap;

//create the target options
//TODO: set some actual options here
options_t g_target_options;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  //while(!Serial);

  dap.begin();
  
    // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  File dataFile = SD.open(FILENAME);
  uint8_t buf[256];

  if(dataFile){
    dap.dap_disconnect();
    dap.dap_get_debugger_info();
    dap.dap_connect();
    dap.dap_transfer_configure(0, 128, 128);
    dap.dap_swd_configure(0);
    dap.dap_reset_link();
    dap.dap_swj_clock(DAP_FREQ);
    dap.dap_target_prepare();
  
    dap.select(&g_target_options);
  
    Serial.print("Erasing... ");
    dap.erase();
    Serial.println(" done.");
  
    Serial.print("Programming... ");
    uint32_t addr = dap.program_start();

    while (dataFile.available()) {
      dataFile.read(buf, 256);
      addr = dap.program(addr, buf);
    }
    dataFile.close();
    
    Serial.println(" done.");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
    return;
  }
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
