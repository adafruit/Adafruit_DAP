#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SdFat.h>

#define SD_CS 10  /* 10 on Atmel M0/M4/32x, 11 on nRF52832 Feather */

#define SWDIO 12
#define SWCLK 11
#define SWRST 9

#define BASE_ADDR               0

#define FILE_BOOTLOADER         "40_BOOT.bin"

/* UICR setting for bootloader */
#define UICR_BOOTLOADER         0x10001014
#define UICR_BOOTLOADER_VAL     0x000F4000
#define UICR_MBR_PARAM_PAGE     0x10001018
#define UICR_MBR_PARAM_PAGE_VAL 0x000FE000

#define BUFSIZE   4096
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

// Create a DAP instance for nRF5x devices
Adafruit_DAP_nRF5x dap;

SdFat SD;

// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
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
    error("No nRF5x device found!");
  }

  Serial.print("Found Target: ");
  Serial.print(dap.target_device.name);
  Serial.print("\tFlash size: ");
  Serial.print(dap.target_device.flash_size);
  Serial.print("\tFlash pages: ");
  Serial.println(dap.target_device.n_pages);
  //Serial.print("Page size: "); Serial.println(dap.target_device.flash_size / dap.target_device.n_pages);

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS)) {
    error("Card failed, or not present");
  }
  Serial.println("Card initialized");

  if ( !SD.exists(FILE_BOOTLOADER) )  error("Couldn't open file " FILE_BOOTLOADER);

  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");

  uint32_t start_ms = millis();

  dap.program_start();
  Serial.println();

  write_bin_file(FILE_BOOTLOADER, BASE_ADDR);

  /* Write UICR setting */
  dap.programUICR(UICR_BOOTLOADER, UICR_BOOTLOADER_VAL);
  dap.programUICR(UICR_MBR_PARAM_PAGE, UICR_MBR_PARAM_PAGE_VAL);

  Serial.print("\nDone in ");
  Serial.print(millis()-start_ms);
  Serial.println(" ms");

  dap.deselect();
  dap.dap_disconnect();
}

void write_bin_file(const char* filename, uint32_t addr)
{
  File32 dataFile = SD.open(filename);
  if(!dataFile){
     error("Couldn't open file");
  }

  Serial.print("Programming... ");
  Serial.println(filename);

  uint32_t charcount = 0;
  while (dataFile.available())
  {
    memset(buf, BUFSIZE, 0xFF);  // empty it out
    uint32_t count = dataFile.read(buf, BUFSIZE);
    bool rc = dap.programFlash(addr, buf, count);
    if (!rc) {
      Serial.print("Failed writing at 0x");
      Serial.print(addr, HEX);
      Serial.println("!");
    }
    addr += count;
    charcount++;
    if (charcount == 78) {
      Serial.println(".");
      charcount = 0;
    } else {
      Serial.print(".");
    }
  }
  Serial.println("");
  dataFile.close();
}

void loop() {
  //blink led on the host to show we're done
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
