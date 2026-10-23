// Drag-and-drop programmer on a CH32V203 host
//
// The board appears as a USB drive: drop a firmware .bin onto it and the
// QT Py programs it into an STM32F4 target over SWD, then restarts the
// target. DETAILS.TXT on the drive shows the target and the last result;
// FAIL.TXT appears when a drop fails.
//
// Wiring (QT Py CH32V203 -> target):
//   A3 -> SWDIO, A2 -> SWCLK, A0 -> nRESET
//   TX -> target RX, RX -> target TX, GND -> GND
//
// Tools > USB support: "Adafruit TinyUSB with USBD".
// NeoPixel: blue = waiting for the PC, green = ready / last drop OK,
// yellow = programming, red = last drop failed, purple flicker = UART
// traffic, blinking red = the drive could not start.

#include <Adafruit_NeoPixel.h>

#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>

#include "src/Adafruit_CDC_UART_Bridge.h"
#include "src/Adafruit_USBD_DragDrop.h"

#if !defined(ARDUINO_ARCH_CH32)
#error "This example runs on a CH32V203 host board (e.g. QT Py CH32V203)"
#endif
#if !defined(CFG_TUD_WCH_USBIP_FSDEV) || !CFG_TUD_WCH_USBIP_FSDEV
#error "Select Tools > USB support > 'Adafruit TinyUSB with USBD'"
#endif

#define SWDIO A3
#define SWCLK A2
#define SWRST A0

Adafruit_DAP_STM32 dap;
Adafruit_USBD_DragDrop drive;
Adafruit_CDC_UART_Bridge bridge;
Adafruit_NeoPixel pixel(1, PA4, NEO_GRB + NEO_KHZ800);

static bool drive_ok = false;

void setup() {
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 0, 20));
  pixel.show();

  TinyUSBDevice.setManufacturerDescriptor("Adafruit");
  TinyUSBDevice.setProductDescriptor("QT Py CH32V203 Drag-and-Drop");

  Serial.begin(115200); // USB serial port: carries the target's UART
  bridge.begin(Serial); // UART starts at 115200 8N1, then follows the PC

  // Errors are recorded and shown on the drive (FAIL.TXT), not fatal
  dap.begin(SWCLK, SWDIO, SWRST, dragdrop_dap_error);
  drive_ok = drive.begin(&dap); // identifies the target, adds the drive
}

void loop() {
  bridge.task();
  drive.task();

  // Status LED, updated at most every 20 ms
  static uint32_t last_led = 0;
  uint32_t now = millis();
  if (now - last_led >= 20) {
    last_led = now;
    bool blink = (now / 40) & 1;
    uint32_t color = pixel.Color(0, 20, 0); // green: ready / last drop OK
    if (!drive_ok) {
      color = blink ? pixel.Color(20, 0, 0) : 0; // drive could not start
    } else if (drive.programming()) {
      color = pixel.Color(20, 15, 0); // yellow: programming
    } else if (!TinyUSBDevice.mounted()) {
      color = pixel.Color(0, 0, 20); // blue: waiting for the PC
    } else if (drive.lastResult() == DragDropEngine::FAILED) {
      color = pixel.Color(20, 0, 0); // red: last drop failed
    } else if ((bridge.bytesToTarget() || bridge.bytesFromTarget()) &&
               now - bridge.lastActivityMillis() < 50) {
      color = blink ? pixel.Color(15, 0, 20) : pixel.Color(0, 20, 0);
    }
    pixel.setPixelColor(0, color);
    pixel.show();
  }
}
