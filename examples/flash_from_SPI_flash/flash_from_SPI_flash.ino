#include "Adafruit_DAP.h"
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#define FILENAME "fw.bin"

#define SWDIO 10
#define SWCLK 9
#define SWRST 11

#define BUFSIZE 256 //don't change
uint8_t buf[BUFSIZE];

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_SAM dap;

// Configuration of the flash chip pins and flash fatfs object.
// You don't normally need to change these if using a Feather/Metro
// M0 express board.
#if defined(__SAMD51__) || defined(NRF52840_XXAA)
  Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
#else
  #if (SPI_INTERFACES_COUNT == 1 || defined(ADAFRUIT_CIRCUITPLAYGROUND_M0))
    Adafruit_FlashTransport_SPI flashTransport(SS, &SPI);
  #else
    Adafruit_FlashTransport_SPI flashTransport(SS1, &SPI1);
  #endif
#endif

Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatFileSystem fatfs;

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
  
  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1);
  }
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Failed to mount filesystem!");
    Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
    while(1);
  }
  Serial.println("Mounted filesystem!");

  File dataFile = fatfs.open(FILENAME, FILE_READ);
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
    Serial.print("Unknown device found 0x"); Serial.print(dsu_did, HEX);
    error("Unknown device found");
  }
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
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
