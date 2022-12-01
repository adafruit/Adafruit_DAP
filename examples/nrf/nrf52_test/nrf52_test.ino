#include "Adafruit_DAP.h"
#include <SPI.h>

//teensy only, otherwise change sd cs pin
#define SWDIO 12
#define SWCLK 11
#define SWRST 9

#define BUFSIZE   4096
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a programming DAP
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

  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");

  uint32_t start_ms = millis();
   
  dap.program_start();
  Serial.print("Programming 32K ... ");
  
  uint32_t addr = 0;
  for(size_t i=0; i<sizeof(buf); i++) buf[i] = i;

  for(int i=0; i<8; i++) 
  {
    dap.programFlash(addr, buf, sizeof(buf));
    addr += BUFSIZE;
  }

  Serial.print("\nDone in ");
  Serial.print(millis()-start_ms);
  Serial.println(" ms");


  for(int i=0; i<8; i++)
  {
    print_memory(i*BUFSIZE, buf, sizeof(buf));
  }

#if 0
  print_memory(0x10001000, buf, 256);
#endif

  dap.deselect();
  dap.dap_disconnect();
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

void loop() {
  //blink led on the host to show we're done
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}
