#include "Adafruit_DAP.h"
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "Adafruit_TinyUSB.h"

#define FILENAME    "blespifriend_067_s110_bootloader_merged.bin"
#define BASE_ADDR   0

/* UICR setting for bootloader */
#define UICR_BOOTLOADER         0x10001014
#define UICR_BOOTLOADER_VAL     0x0003C000

// SWD pin configure
#define SWDIO 12
#define SWCLK 11
#define SWRST 9

#define BUFSIZE   (1024)
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a DAP for programming nRF devices
Adafruit_DAP_nRF5x dap;

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

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1) yield();
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  dap.begin(SWCLK, SWDIO, SWRST, &error);

  // Init SPI flash
  flash.begin();

  // Set up USB MSC
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setCapacity(flash.size()/512, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  Serial.begin(115200);
  while(!Serial) {
    delay(1);         // will pause the chip until it opens serial console
  }

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Failed to mount filesystem!");
    Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
    while(1) delay(1);
  }
  Serial.println("Mounted filesystem!");

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
  File32 dataFile = fatfs.open(filename, FILE_READ);

  if(!dataFile){
    Serial.print("Couldn't open file ");
    Serial.println(filename);
    error("Try to copy file over USB MSC and reset the board");
  }

  Serial.print("Programming... ");
  Serial.println(filename);

  uint32_t charcount = 0;
  while (dataFile.available())
  {
    digitalWrite(LED_BUILTIN, 1-digitalRead(LED_BUILTIN));
    yield(); // needed if use TinyUSB

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

void loop()
{
  // nothing to do
}

//--------------------------------------------------------------------+
// USB MSC
//--------------------------------------------------------------------+

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  digitalWrite(LED_BUILTIN, HIGH);

  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  digitalWrite(LED_BUILTIN, LOW);
}

//--------------------------------------------------------------------+
// For debugging
//--------------------------------------------------------------------+
static void dump_str_line(uint8_t const* buf)
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
        dump_str_line(&buffer[i-16]);
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

  dump_str_line(&buffer[bufsize-16]);
}
