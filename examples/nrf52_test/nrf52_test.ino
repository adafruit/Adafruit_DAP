#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>

// lv 1: only erase
// lv 2: erase and write one word 0xCafeBabe to 0x0000
// lv 3: read and write data from SD card
#define TEST_LV   3

//teensy only, otherwise change sd cs pin
#define SD_CS 10
#define SWDIO 11
#define SWCLK 12
#define SWRST 13

#define FILE_S132         "S132_201.BIN"
#define FILE_BOOTLOADER   "FT52_050.BIN"

#define S132_ADDR         0
#define BOOTLOADER_ADDR   0x74000

// Location that store bootloader address
#define UICR_BOOTLOADER     0x10001014
#define UICR_MBR_PARAM_PAGE 0x10001018

#define BUFSIZE   4096
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

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

  if ( !SD.exists(FILE_S132) )        error("Couldn't open file " FILE_S132);
  if ( !SD.exists(FILE_BOOTLOADER) )  error("Couldn't open file " FILE_BOOTLOADER);


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


  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");

  uint32_t start_ms = millis();
  
#if TEST_LV == 2
  {
    dap.program_start();
    Serial.print("Programming 32K ... ");
    
    uint32_t addr = 0;
    for(int i=0; i<sizeof(buf); i++) buf[i] = i;

    for(int i=0; i<8; i++) 
    {
      dap.program(addr, buf, sizeof(buf));
      addr += 4096;
    }
  }
#endif

#if TEST_LV >= 3
  dap.program_start();
  Serial.println();

  write_bin_file(FILE_S132, S132_ADDR);
  write_bin_file(FILE_BOOTLOADER, BOOTLOADER_ADDR);
#endif

  dap.programUCIR(UICR_BOOTLOADER, BOOTLOADER_ADDR);
  dap.programUCIR(UICR_MBR_PARAM_PAGE, 0x0007E000);

  Serial.print("\nDone in ");
  Serial.print(millis()-start_ms);
  Serial.println(" ms");

  dap.deselect();
  dap.dap_disconnect();
}

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

void loop() {
  //blink led on the host to show we're done
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}
