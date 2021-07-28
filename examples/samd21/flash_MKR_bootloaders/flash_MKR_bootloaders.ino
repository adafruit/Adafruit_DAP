#include "Adafruit_DAP.h"
#include "boards.h"

#define SWDIO 1
#define SWCLK 2
#define SWRST 0
//#define SWDIO 9
//#define SWCLK 8
//#define SWRST 7

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_SAM dap;

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

  delay(2000);
  Serial.println();
  Serial.println("This sketch is for updating Arduino MKR and some other ATSAMD21G18, 32-Bit ARM Cortex M0+ boards");
  Serial.println();
  Serial.println("The Arduino board should have these connections:");
  Serial.print("SWD pad SWDCLCK connected to programmer board pin ");
  Serial.println(SWCLK);
  Serial.print("SWD pad SWDIO connected to programmer board pin ");
  Serial.println(SWDIO);
  Serial.print("SWD pad SWDRESET or RESET header pin connected to programmer board pin ");
  Serial.println(SWRST);
  Serial.println("Vcc (3.3V) pins should be connected between programmer and target boards");
  Serial.print("GND pins should be connected between programmer and target boards");
  Serial.println();
  Serial.println("See https://gojimmypi.blogspot.com/2018/12/swd-debugging-arduino-mkr-wifi-1010.html");
  Serial.println();

  dap.begin(SWCLK, SWDIO, SWRST, &error);

  Serial.println("Connecting...");
  if ( !dap.targetConnect() ) {
    error(dap.error_message);
  }

  char debuggername[100];
  dap.dap_get_debugger_info(debuggername);
  Serial.print(debuggername); Serial.print("\n\r");

  uint32_t dsu_did;
  if (!dap.select(&dsu_did)) {
    Serial.print("Unknown device found 0x"); Serial.print(dsu_did, HEX);
    error("Unknown device found");
  }
  Serial.print("Found Target: ");
  Serial.print(dap.target_device.name);
  Serial.print("\tFlash size: ");
  Serial.print(dap.target_device.flash_size);
  Serial.print("\tFlash pages: ");
  Serial.println(dap.target_device.n_pages);

  bool isSamd21 = strstr(dap.target_device.name, "SAM D21") != NULL;

  /* Example of how to read and set fuses
  Serial.print("Fuses... ");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  dap._USER_ROW.bit.WDT_Period = 0x0A;
  dap.fuseWrite();
  Serial.println(" done.");
  */

#define DEVICE_ID_MASK         0xfffff0ff
#define DEVICE_REV_SHIFT       8
#define DEVICE_REV_MASK        0xf

  uint32_t mcu_id = dsu_did & DEVICE_ID_MASK; // 4 bits (bit 8-11) is just the revision 

  if (!isSamd21) {
    error("MCU is not a SAM D21. No bootloader available for this MCU!");
  }

  board_t *selectedBoard = NULL;
  while (selectedBoard == NULL) {
    // Show menu:
    Serial.println();
    Serial.println("Select Arduino MKR board to erase and flash with bootloader:");
    Serial.println();
    for (const board_t *board = bootloaders; board->bootloader != NULL; board++) {
      Serial.print(board->choice);
      Serial.print(" ");
      Serial.print(board->name);
      Serial.print(" (");
      Serial.print(board->size);
      Serial.println(" bytes)");
      //Serial.print(": 0x");
      //Serial.println((uint32_t)board->bootloader, HEX);
    }
    Serial.println();
    Serial.print("Arduino IDE: Please enable No Line Ending with 115200 Baud in the serial monitor.");
    Serial.println();
    Serial.print("Please select your board: ");
    while (!Serial.available());
    String choice = Serial.readStringUntil('\n');
    Serial.println();
    for (const board_t *board = bootloaders; board->bootloader != NULL; board++) {
      if (choice.equalsIgnoreCase(board->choice)) {
        selectedBoard = (board_t *)board;
        Serial.print("Selected: ");
        Serial.println(selectedBoard->name);
        break;
      }
    }
  }

  Serial.print("Erasing... ");
  dap.erase();

  dap.fuseRead();
  //if (dap._USER_ROW.fuses == 0ULL || dap._USER_ROW.fuses == ~0ULL) {
  //  // Entire User Row fuses shold never be 0 or all 1s
  //  Serial.println("User Row fuses not read OK!");
  //}

  //Serial.print("\nFuses: ");
  //uint64_t fuses = dap._USER_ROW.fuses;
  //Serial.print("Fuse high: 0x"); Serial.println((uint32_t)(fuses >> 32), HEX);
  //Serial.print("Fuse high: 0x"); Serial.println(dap._USER_ROW.fuseParts[1], HEX);
  //Serial.print("Fuse low: 0x"); Serial.println((uint32_t)(fuses & 0xFFFFFFFF), HEX);
  //Serial.print("Fuse low: 0x"); Serial.println(dap._USER_ROW.fuseParts[0], HEX);
  //Serial.print("Boot protect: 0x"); Serial.println(dap._USER_ROW.bit.BOOTPROT, HEX);
  //Serial.print("Region lock: 0x"); Serial.println(dap._USER_ROW.bit.LOCK, HEX);
  //Serial.print("BOD12_Config_Vreg: 0x"); Serial.println(dap._USER_ROW.bit._reserved_BOD12_Config_Vreg, HEX);
  //Serial.print("BOD12_Config: 0x"); Serial.println(dap._USER_ROW.bit._reserved_BOD12_Config, HEX);
  //if (dap._USER_ROW.fuses == 0ULL || dap._USER_ROW.fuses == ~0ULL) {
  //  // Entire User Row fuses shold never be 0 or all 1s
  //  error("Will never write User Row fuses as all0 or all 1s!");
  //}

  Serial.print("\nFuses: ");
  Serial.print("BOOTPROT: ");
  Serial.print(dap._USER_ROW.bit.BOOTPROT);
  Serial.print(" REGION LOCK: ");
  Serial.print(dap._USER_ROW.bit.LOCK);
  Serial.println();

  dap.programFlash(0, selectedBoard->bootloader, selectedBoard->size, true);

  Serial.println("\nDone!");

  //dap._USER_ROW.bit.BOOTPROT = 0x02;
  //dap.fuseWrite();
  //dap.fuseRead();
  //Serial.print("BOOTPROT: ");
  //Serial.print(dap._USER_ROW.bit.BOOTPROT);
  //Serial.print(" REGION LOCK: ");
  //Serial.print(dap._USER_ROW.bit.LOCK);
  //dap.lock();

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
