#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>

//teensy only, otherwise change sd cs pin
#define SD_CS 10
#define SWDIO 11
#define SWCLK 12
#define SWRST 13

#define BUFSIZE   4096
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_STM32 dap;

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
    error("No STM32 device found!");
  }

  Serial.print("Found Target\t");
  Serial.println(dap.target_device.name);
  Serial.print("Flash size\t");
  Serial.print(dap.target_device.flash_size / 1024);
  Serial.println(" KBs");
  Serial.print("Flash pages\t");
  Serial.println(dap.target_device.n_pages);

  uint32_t start_ms;
  Serial.print("Erasing... ");
  start_ms = millis();
  dap.erase();
  Serial.print(" done in ");
  Serial.print(millis()-start_ms);
  Serial.println(" ms");
  
//
//  dap.program_start();
//  Serial.print("Programming 32K ... ");
//
//  uint32_t addr = 0;
//  for(int i=0; i<sizeof(buf); i++) buf[i] = i;
//
//  for(int i=0; i<8; i++)
//  {
//    dap.program(addr, buf, sizeof(buf));
//    addr += 4096;
//  }
//
//  Serial.print("\nDone in ");
//  Serial.print(millis()-start_ms);
//  Serial.println(" ms");

  dap.deselect();
  dap.dap_disconnect();
}

#if 0
void write_bin_file(const char* filename, uint32_t addr)
{
  File dataFile = SD.open(filename);
  if(!dataFile){
     error("Couldn't open file");
  }

  Serial.print("Programming... ");
  Serial.println(filename);

  while (dataFile.available()) 
  {
    memset(buf, BUFSIZE, 0xFF);  // empty it out
    uint32_t count = dataFile.read(buf, BUFSIZE);
    dap.program(addr, buf, count);
    addr += count;
  }
  
  dataFile.close();  
}
#endif 

void loop() {
  //blink led on the host to show we're done
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}
