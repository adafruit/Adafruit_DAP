/*!
 * @file usb_functions.cpp
 *
 * Shared state of the library's USB functions.
 *
 */

#include "usb_functions.h"

#if defined(USE_TINYUSB)

#include <Adafruit_TinyUSB.h>

bool dap_ch32_usb_dap_added = false;
bool dap_ch32_usb_drive_added = false;

/*!
 * @brief Re-enumerate so the host sees interfaces added in setup().
 *
 * The core starts USB before setup(), so interfaces added later only
 * appear after a detach/attach.
 */
void dap_ch32_usb_reenumerate(void) {
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

#endif
