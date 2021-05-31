/*
  Update your DAP from SPIFFS. ESP8266 & ESP32
  Based on the flash from SD examples that come with the Adafruit_DAP library.
  
  This example uploads a firmware file stored in SPIFFS to your DAP device.
  You can add your own files:
  - by placing them in the /data folder of your sketch
  - if necessary change the FILENAME(defined below)
  - and uploading them to you ESP
    - In Arduino IDE > Tools > ESP Sketch Data Upload
    - In PlatformIO > Terminal > Run Task... > Upload File System Image
  
  For more info on using SPIFFS in the Arduino IDE:
  - https://www.instructables.com/Using-ESP8266-SPIFFS/
  - https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/
*/

#include <Adafruit_DAP.h>
#include <FS.h>
#ifdef ESP32
  #include <SPIFFS.h>
#endif


#define SWDIO 14
#define SWCLK 33
#define SWRST 26

#define FILENAME "/FIRMWARE.BIN"

// the more the better
#define BUFSIZE   (16*1024)

// buffer should be word algined for STM32
uint8_t buf[BUFSIZE]  __attribute__ ((aligned(4)));

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_STM32 dap;

// STM32 auto map 0x00 to 0x08000000, use 0 for simplicity
#define FLASH_START_ADDR    0

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

  //------------- SPIFFS begin -------------//
  // see if SPIFFS is present and can be initialized:
  if(!SPIFFS.begin()) {
    error("Could not start SPIFFS and load partition");
  }
  
  Serial.println("SPIFFS initialized");
  Serial.print("Opening file: ");
  Serial.println(FILENAME);
  
  File dataFile = SPIFFS.open(FILENAME, "r");

  if(!dataFile){
     error("Couldn't open file");
  }

  //------------- DAP connecting -------------//
  Serial.println("Connecting...");
  if ( !dap.targetConnect() ) {
    error(dap.error_message);
  }

  char debuggername[100];
  dap.dap_get_debugger_info(debuggername);
  Serial.print(debuggername); Serial.print("\n\r");

  uint32_t dsu_did;
  if (! dap.select(&dsu_did)) {
    error("No STM32 device found!");
  }

  Serial.print("Found Target\t");
  Serial.println(dap.target_device.name);
  Serial.print("Flash size\t");
  Serial.print(dap.target_device.flash_size / 1024);
  Serial.println(" KBs");

  //------------- Preparing sectors -------------//
  uint32_t start_ms;
  Serial.print("Preparing ... ");
  start_ms = millis();

  // preparing flash sector with address = 0, size = Binary size
  dap.programPrepare(FLASH_START_ADDR, dataFile.size());  
  Serial.print(" done in "); Serial.print(millis()-start_ms); Serial.println(" ms");

  //------------- Programming -------------//
  Serial.print("Programming ");
  Serial.print(dataFile.size());
  Serial.print(" bytes ...");

  uint32_t addr = FLASH_START_ADDR;
  start_ms = millis();
  
  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      uint32_t count = dataFile.read(buf, BUFSIZE);
      dap.programBlock(addr, buf, count);
      addr += count;
  }

  Serial.print(" done in "); Serial.print(millis()-start_ms); Serial.println(" ms");

  //------------- Verifying -------------//
  Serial.print("Verifying ...");
  dataFile.seek(0); // seek to file beginning

  addr = FLASH_START_ADDR;
  start_ms = millis();
  
  while (dataFile.available()) {
      memset(buf, BUFSIZE, 0xFF);  // empty it out
      uint32_t count = dataFile.read(buf, BUFSIZE);
      if ( !dap.verifyFlash(addr, buf, count) ) {
        error("Flash mismatched");
      }
      addr += count;
  }

  Serial.print(" done in "); Serial.print(millis()-start_ms); Serial.println(" ms");

  dataFile.close();
  SPIFFS.end();
  
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
