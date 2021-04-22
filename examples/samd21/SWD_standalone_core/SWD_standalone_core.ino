// This is a demo program for Adafruit_DAP where instructions are loaded from a text file on the SD
// card. it is totally unsupported! :)

#include "Adafruit_DAP.h"
#include <SPI.h>
#include <SD.h>


#if defined(__MK66FX1M0__)  // teensy 3.6
 #include "USBHost_t36.h"
  USBHost myusb;
  GenericUSB usbdev(myusb);

 #define SD_CS  BUILTIN_SDCARD
 #define SWRST 27
 #define SWCLK 29
 #define SWDIO 30

#else // Metro M0, Feather M0, Arduino Zero

 #include <usbhub.h>
 // USB host for enumaration 
 USBHost     usb;

 #define SD_CS  5
 #define SWRST 12
 #define SWCLK 11
 #define SWDIO 10
#endif

#define PAGESIZE  256 //don't change
uint8_t pagebuffer[PAGESIZE], verifybuffer[PAGESIZE];
#define SDBUFFER_SIZE (PAGESIZE)
uint8_t sdbuffer[SDBUFFER_SIZE];


void swd_error(const char *text) {
  Serial.println(text);
  while (1);
}


void info(int32_t i) {
  Serial.print(i);
}

void info(const char *str) {
  Serial.print(str);
}

void error(const char *str) {
  Serial.println(str);
  while (1);
}


File cmdFile, progFile, errorFile;
Adafruit_DAP_SAM dap;

uint16_t PID = 0;
char     PRODUCT_NAME[60];
char     CHIP[60];

uint32_t totaltime;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial);

  // enable teensy USB
#if defined(__MK66FX1M0__)  // teensy 3.6
  myusb.begin();
#endif

  totaltime = millis();
  
  Serial.println("initializing DAP");
  dap.begin(SWCLK, SWDIO, SWRST, &swd_error);

  
  info("Init SD Card...");

  // see if the card is present and can be initialized:
#if defined(__MK66FX1M0__)  // teensy 3.6
  if (!SD.begin(SD_CS)) {
#else
  if (!SD.begin(12000000, SD_CS)) {
#endif
    error("Card failed");
  }
  info("OK\n");
  info("Open test.txt\n");
  cmdFile = SD.open("test.txt");

  if (!cmdFile) {
    error("Could not find test.txt");
  }
}

void loop() {
  Serial.print('.');
  if (! cmdFile.available())   return;

  String cmd, arg;
  
  String line = cmdFile.readStringUntil('\n');
  int i = line.indexOf(' ');
  if (i == -1) {
    cmd = line;
    arg = String("");
  } else {
    cmd = line.substring(0, i);
    arg = line.substring(i+1);
  }
  Serial.print("\tCommand: \'"); Serial.print(cmd); Serial.println("\'");
  Serial.print("\tArgs: \'"); Serial.print(arg); Serial.println("\'");


  uint32_t progtime = millis();
  
  if (cmd == "PID") {
    PID = arg.toInt();
    info("PID = ");  info(PID);  info("\n");
  }
  if (cmd == "NAME") {
    arg.toCharArray(PRODUCT_NAME, 59);
    PRODUCT_NAME[59] = 0;
    info("Name = ");  info(PRODUCT_NAME);  info("\n");
  }
  if (cmd == "CHIP") {
    arg.toCharArray(CHIP, 59);
    CHIP[59] = 0;
    info("Chip = "); info(CHIP); info("\n");
  }
  
  if (cmd == "LOCK") {
    targetConnect();
    info("locking chip...");
    dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
    uint64_t fuses = dap._USER_ROW.fuses;
    Serial.print("Fuse high: 0x"); Serial.println((uint32_t)(fuses >> 32), HEX);
    Serial.print("Fuse low: 0x"); Serial.println((uint32_t)(fuses & 0xFFFFFFFF), HEX);
    Serial.print("Boot protect: 0x"); Serial.println(dap._USER_ROW.bit.BOOTPROT, HEX);

    uint16_t bootsize = arg.toInt();
    // lock it!
    if (bootsize == 8192) {
      dap._USER_ROW.bit.BOOTPROT = 0x02;
    } else if (bootsize == 16384) {
      dap._USER_ROW.bit.BOOTPROT = 0x01;
    } else {
      error("Unsupported bootprot size");
    }
    dap.fuseWrite();
    fuses = dap._USER_ROW.fuses;
    Serial.print("New Fuse high: 0x"); Serial.println((uint32_t)(fuses >> 32), HEX);
    Serial.print("New Fuse low: 0x"); Serial.println((uint32_t)(fuses & 0xFFFFFFFF), HEX);
    Serial.print("New Boot protect: 0x"); Serial.println(dap._USER_ROW.bit.BOOTPROT, HEX);
    info("done!\n");
    dap.deselect();
  }

  if (cmd == "UNLOCK") {
    targetConnect();
    info("Unlocking chip...");
    dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
    uint64_t fuses = dap._USER_ROW.fuses;
    Serial.print("Fuse high: 0x"); Serial.println((uint32_t)(fuses >> 32), HEX);
    Serial.print("Fuse low: 0x"); Serial.println((uint32_t)(fuses & 0xFFFFFFFF), HEX);
    Serial.print("Boot protect: 0x"); Serial.println(dap._USER_ROW.bit.BOOTPROT, HEX);
    if (dap._USER_ROW.bit.BOOTPROT != 0x07) {
      // unlock it!
      dap._USER_ROW.bit.BOOTPROT = 0x07;
      dap.fuseWrite();
      fuses = dap._USER_ROW.fuses;
      Serial.print("New Fuse high: 0x"); Serial.println((uint32_t)(fuses >> 32), HEX);
      Serial.print("New Fuse low: 0x"); Serial.println((uint32_t)(fuses & 0xFFFFFFFF), HEX);
      Serial.print("New Boot protect: 0x"); Serial.println(dap._USER_ROW.bit.BOOTPROT, HEX);
    }
    info("done!\n");
    dap.deselect();
  }

  
  if (cmd == "ERASE") {
    targetConnect();
    info("Erasing...");
    dap.erase();
    info("done!\n");
    dap.deselect();
  }

  if (cmd == "PROGRAM") {
    char filename[20];
    arg.toCharArray(filename, 19);
    filename[19] = 0;
    info("Programming "); 
    info(filename); info("\n");

    progFile = SD.open(filename);
    if (!progFile) {
      error("Could not open file");
    }
    
    info("Opened, Programming...");
    targetConnect();
    uint32_t addr = dap.program_start();
    while (progFile.available()) {
      //info(addr); info(", ");
      memset(pagebuffer, PAGESIZE, 0xFF);  // empty it out
      progFile.read(pagebuffer, PAGESIZE);
      dap.programBlock(addr, pagebuffer);
      addr += PAGESIZE;
    }
    info("Done!\n");
    dap.deselect();
    progFile.close();
  }
  
  if (cmd == "CRC") {
    int i = arg.indexOf(' ');
    if (i == -1) {
      return;
    }
    String arg1 = arg.substring(0, i);
    String arg2 = arg.substring(i+1);

    uint32_t flashsize = arg1.toInt();
    char hexbuff[16];
    arg2.toCharArray(hexbuff, 15);
    uint32_t check_crc = strtol(hexbuff, 0, 16);

    targetConnect();
    info("Opened, CRCing...");
    uint32_t crc;
    dap.readCRC(flashsize, &crc);
    Serial.println(crc, HEX);
    if (crc != check_crc) {
      error("CRC Failed");
    }
    info("Done!\n");
    dap.deselect();
  }
    
  if (cmd == "VERIFY") {
    char filename[20];
    arg.toCharArray(filename, 19);
    filename[19] = 0;
    info("Programming "); 
    info(filename); info("\n");

    progFile = SD.open(filename);
    if (!progFile) {
      error("Could not open file");
    }
    
    info("Verifying...");
    targetConnect();

    uint32_t addr = 0;
    dap.dap_set_clock(0);
    while (progFile.available()) {
      //info(addr); info(", ");
      memset(pagebuffer, PAGESIZE, 0xFF);  // empty it out
      progFile.read(pagebuffer, PAGESIZE);
      dap.readBlock(addr, verifybuffer);
      for (int i=0; i<PAGESIZE; i++) {
        if (verifybuffer[i] != pagebuffer[i]) {
          Serial.print("Verify error at $"); Serial.print(addr+i, HEX);
          Serial.print(" wrote: "); Serial.print(pagebuffer[i],HEX);
          Serial.print(" read: "); Serial.println(verifybuffer[i],HEX);
          error("Verify error");
        }
      }
      addr += PAGESIZE;
    }
    info("Done!\n");
    dap.deselect();
    progFile.close();
  }
  
  if (cmd == "RESET") {
    info("Resetting...");
    digitalWrite(SWRST, LOW);
    delay(10);
    digitalWrite(SWRST, HIGH);    
    info("done!\n");
  }

  if (cmd == "DONE") {
    Serial.println("Complete!");
    Serial.print("Took "); Serial.print(millis() - totaltime); Serial.println("ms");
    while (1);
  }
  
  if (cmd == "CHECK") {
    Serial1.begin(9600);
    while (1) {
      String line2 = Serial1.readStringUntil('\n');
      Serial.print("'"); Serial.print(line2); Serial.print("'\n"); 
      int i = line2.indexOf(arg);
      if (i != -1) {
        break;
      }
    }
  }
  if (cmd == "RXTXECHO") {
    Serial1.begin(9600);

    char c = 0;
    int retries = 5;
    while (retries--) {
      Serial1.write('a');
      while (!Serial1.available());
      c = Serial1.read();
      Serial.println(c);
      if (c == 'A') {
        break;
      }
    }
    if (!retries) {
      error("Failed RX TX Echo Test");
    }
  }

  if (cmd == "CHECKUSB") {
    char checkagainst[20];
    arg.toCharArray(checkagainst, 20);

#if defined(__MK66FX1M0__)  // teensy 3.6
    info("Waiting for USB...");

    while (1) {
      myusb.Task();
      if (usbdev.device) {
        Serial.println(usbdev.device->idVendor, HEX);
        Serial.println(usbdev.device->idProduct, HEX);
        char vidpid[20];
        
        snprintf(vidpid, 20, "0x%04X 0x%04X", usbdev.device->idVendor, usbdev.device->idProduct);
        Serial.print("'"); Serial.print(vidpid); Serial.println("'"); 

        if (0 == strncmp(checkagainst, vidpid, 20)) {
          break;
        } else {
          error("Found wrong PID/VID");
        }
      }
    }
#else
    info("Initializing USB\n");
    if (usb.Init() == -1) {
      error("USBhost did not start");
    }
    info("Waiting for USB...");
    delay(20);

    while (1) {
      usb.Task();
      if( usb.getUsbTaskState() == USB_STATE_RUNNING ) {
        USB_DEVICE_DESCRIPTOR buf;
        byte rcode;
        rcode = usb.getDevDescr(1, 0, 0x12, ( uint8_t *)&buf );
        if( rcode ) {
           Serial.print("Got return code: 0x"); Serial.println( rcode, HEX );
           return;
        }
        //Serial.print("VID: 0x"); Serial.print(buf.idVendor, HEX);
        //Serial.print("  PID: 0x"); Serial.println(buf.idProduct, HEX);
        char vidpid[20];
        
        snprintf(vidpid, 20, "0x%04X 0x%04X", buf.idVendor, buf.idProduct);
        Serial.print("'"); Serial.print(vidpid); Serial.println("'"); 

        if (0 == strncmp(checkagainst, vidpid, 20)) {
          break;
        } else {
          error("Found wrong PID/VID");
        }
      }
    } 
 #endif
  }
  
  if (cmd == "READ") {
    char filename[20];
    arg.toCharArray(filename, 19);
    filename[19] = 0;
    info("Reading to "); 
    info(filename); info("\n");

    progFile = SD.open(filename, FILE_WRITE);
    if (!progFile) {
      error("Could not open file");
    }

    targetConnect();
    info("Opened, Reading...");
    for (uint32_t addr=0; addr < 0x3FFFF; addr+=PAGESIZE) {
      info(addr); info(", ");
      memset(pagebuffer, PAGESIZE, 0xFF);  // empty it out
      dap.readBlock(addr, pagebuffer);
      progFile.write(pagebuffer, PAGESIZE);
    }
    info("Done!\n");
    dap.deselect();
    progFile.close();
  }


  Serial.print("\tTook "); Serial.print(millis()-progtime); Serial.println("ms");
}


void targetConnect(void) {
  char debuggername[250];
  
  info("Connecting...");  
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
}

