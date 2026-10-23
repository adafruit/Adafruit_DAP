// CMSIS-DAP debug probe on a CH32V203 host (e.g. Adafruit QT Py CH32V203).
//
// Turns the board into a USB debug probe for SWD targets such as an
// STM32F405: pyOCD, OpenOCD and other CMSIS-DAP v2 tools find it as
// "Adafruit CMSIS-DAP v2" (no driver needed). Its USB serial port is a
// bridge to the target's UART, following the baud rate and format the PC
// sets.
//
// Wiring (QT Py CH32V203 -> target):
//   A3 -> SWDIO, A2 -> SWCLK, A0 -> nRESET (optional)
//   TX -> target RX, RX -> target TX, GND -> GND
//
// Tools > USB support: "Adafruit TinyUSB with USBD".
// NeoPixel: blue = waiting for the PC, green = ready, white flicker = SWD
// commands, purple flicker = UART traffic.
//
// Example: pyocd commander -t stm32f405rg
//          openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg

// Include NeoPixel first: its timing-critical code must stay in the
// CH32V203's fast flash (the first 32 KB).
#include <Adafruit_NeoPixel.h>

#include <Adafruit_DAP.h>
#include <Adafruit_TinyUSB.h>
#include <dap.h>

#include "src/Adafruit_CDC_UART_Bridge.h"
#include "src/Adafruit_USBD_CMSISDAP.h"

#if !defined(ARDUINO_ARCH_CH32)
#error "This example runs on a CH32V203 host board (e.g. QT Py CH32V203)"
#endif
#if !defined(USE_TINYUSB)
#error "Select Tools > USB support > 'Adafruit TinyUSB with USBD'"
#endif

#define SWDIO A3
#define SWCLK A2
#define SWRST A0

Adafruit_USBD_CMSISDAP usb_dap;
Adafruit_CDC_UART_Bridge bridge;
Adafruit_NeoPixel pixel(1, PA4, NEO_GRB + NEO_KHZ800);

void setup() {
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 0, 20));
  pixel.show();

  // "CMSIS-DAP" in the product string helps tools that search for it there.
  TinyUSBDevice.setManufacturerDescriptor("Adafruit");
  TinyUSBDevice.setProductDescriptor("QT Py CH32V203 CMSIS-DAP");

  Serial.begin(115200); // USB serial port: carries the target's UART
  bridge.begin(Serial); // UART starts at 115200 8N1, then follows the PC

  dap_init(SWCLK, SWDIO, SWRST);
  usb_dap.begin(); // adds the CMSIS-DAP interface and re-enumerates
}

void loop() {
  usb_dap.task();
  bridge.task();

  // Status LED, updated at most every 20 ms
  static uint32_t last_led = 0;
  uint32_t now = millis();
  if (now - last_led >= 20) {
    last_led = now;
    bool blink = (now / 40) & 1;
    uint32_t color = pixel.Color(0, 20, 0); // green: ready
    if (!usb_dap.mounted()) {
      color = pixel.Color(0, 0, 20); // blue: waiting for the PC
    } else if (usb_dap.commandCount() &&
               now - usb_dap.lastCommandMillis() < 50) {
      color = blink ? pixel.Color(20, 20, 20) : pixel.Color(0, 20, 0);
    } else if ((bridge.bytesToTarget() || bridge.bytesFromTarget()) &&
               now - bridge.lastActivityMillis() < 50) {
      color = blink ? pixel.Color(15, 0, 20) : pixel.Color(0, 20, 0);
    }
    pixel.setPixelColor(0, color);
    pixel.show();
  }
}
