#include "Adafruit_DAP.h"
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#define FILENAME    "merged.bin"
#define BASE_ADDR   0

/* UICR setting for bootloader */
#define UICR_BOOTLOADER         0x10001014
#define UICR_BOOTLOADER_VAL     0x0003C000



#define SWDIO 11
#define SWCLK 12
#define SWRST 9

#define BUFSIZE   (1024)
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a DAP for programming nRF devices
Adafruit_DAP_nRF5x dap;

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

// USB Mass Storage object

// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1);
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while(!Serial) {
    delay(1);         // will pause the chip until it opens serial console
  }

  dap.begin(SWCLK, SWDIO, SWRST, &error);

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1) delay(1);
  }
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Failed to mount filesystem!");
    Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
    while(1) delay(1);
  }
  Serial.println("Mounted filesystem!");

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

  Serial.print("Found Target: ");
  Serial.print(dap.target_device.name);
  Serial.print("\tFlash size: ");
  Serial.print(dap.target_device.flash_size);
  Serial.print("\tFlash pages: ");
  Serial.println(dap.target_device.n_pages);

  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");

  uint32_t t = millis();

  write_bin_file(FILENAME, BASE_ADDR);

  /* Write UICR setting */
  Serial.println("Programming UICR bootloader");
  dap.programUICR(UICR_BOOTLOADER, UICR_BOOTLOADER_VAL);

  Serial.print("All done in ");
  Serial.print(millis() - t);
  Serial.println(" ms");

  // all done: turn on LED
  digitalWrite(LED_BUILTIN, HIGH);

  dap.dap_set_clock(50);

  dap.deselect();
  dap.dap_disconnect();
}

void write_bin_file(const char* filename, uint32_t addr)
{
  File dataFile = fatfs.open(FILENAME, FILE_READ);

  if(!dataFile){
     error("Couldn't open file");
  }

  Serial.print("Programming... ");
  Serial.println(filename);

  uint32_t charcount = 0;
  while (dataFile.available())
  {
    digitalWrite(LED_BUILTIN, 1-digitalRead(LED_BUILTIN));

    memset(buf, BUFSIZE, 0xFF);  // empty it out
    uint32_t count = dataFile.read(buf, BUFSIZE);

    bool rc = dap.program(addr, buf, count);
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
}


static void dump_str_line(uint8_t const* buf, uint16_t count)
{
  Serial.print(" |");

  // each line is 16 bytes
  for(uint16_t j=0; j<16; j++)
  {
    const char ch = buf[j];
    Serial.print(isprint(ch) ? ch : '.');
  }
  Serial.println('|');
}

// dumping memory for verification
void print_memory(uint32_t addr, uint8_t* buffer, uint32_t bufsize)
{
  memset(buffer, 0xff, bufsize);
  dap.dap_read_block(addr, buffer, bufsize);

  for(uint32_t i=0; i < bufsize; i++)
  {
    if (i % 16 == 0)
    {
      if ( i != 0 )
      {
        dump_str_line(&buffer[i-16], 16);
      }

      uint32_t offaddr = addr+i;
      // print offset
      if ( offaddr < 0x10000 ) Serial.print("0");
      if ( offaddr < 0x1000  ) Serial.print("0");
      if ( offaddr < 0x100   ) Serial.print("0");
      if ( offaddr < 0x10    ) Serial.print("0");
      Serial.print(offaddr, HEX);
      Serial.print("  ");
    }

    if ( buffer[i] < 0x10 ) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }

  dump_str_line(&buffer[bufsize-16], 16);
}
