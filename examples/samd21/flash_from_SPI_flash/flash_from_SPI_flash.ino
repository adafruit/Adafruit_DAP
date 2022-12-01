#include "Adafruit_DAP.h"
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#define FILENAME "fw.bin"

#define SWDIO 12
#define SWCLK 11
#define SWRST 9

const int BUFSIZE = Adafruit_DAP_SAM::PAGESIZE;
uint8_t buf[BUFSIZE];

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_SAM dap;

// Configuration of the flash chip pins and flash fatfs object.
// You don't normally need to change these if using a Feather/Metro
// M0 express board.
#if defined(ARDUINO_ARCH_ESP32)

  // ESP32 use same flash device that store code for file system.
  // SPIFlash will parse partition.cvs to detect FATFS partition to use
  Adafruit_FlashTransport_ESP32 flashTransport;

#elif defined(ARDUINO_ARCH_RP2040)

  // RP2040 use same flash device that store code for file system. Therefore we
  // only need to specify start address and size (no need SPI or SS)
  // By default (start=0, size=0), values that match file system setting in
  // 'Tools->Flash Size' menu selection will be used.
  Adafruit_FlashTransport_RP2040 flashTransport;

  // To be compatible with CircuitPython partition scheme (start_address = 1 MB,
  // size = total flash - 1 MB) use const value (CPY_START_ADDR, CPY_SIZE) or
  // subclass Adafruit_FlashTransport_RP2040_CPY. Un-comment either of the
  // following line:
  //  Adafruit_FlashTransport_RP2040
  //    flashTransport(Adafruit_FlashTransport_RP2040::CPY_START_ADDR,
  //                   Adafruit_FlashTransport_RP2040::CPY_SIZE);
  //  Adafruit_FlashTransport_RP2040_CPY flashTransport;

#elif defined(__SAMD51__) || defined(NRF52840_XXAA)
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
FatVolume fatfs;

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

  File32 dataFile = fatfs.open(FILENAME, FILE_READ);
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

  /* Example of how to read and set fuses
  Serial.print("Fuses... ");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  dap._USER_ROW.WDT_Period = 0x0A;
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
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
