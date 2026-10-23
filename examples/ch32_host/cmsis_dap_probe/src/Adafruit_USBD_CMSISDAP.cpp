/*!
 * @file Adafruit_USBD_CMSISDAP.cpp
 *
 * CMSIS-DAP v2 USB interface.
 *
 * MIT license.
 */

#include "Adafruit_USBD_CMSISDAP.h"

#if defined(USE_TINYUSB)

#include "dap_usb_driver.h"
#include "host_requests.h"
#include "usb_functions.h"

/*!
 * @brief Add the CMSIS-DAP interface and re-enumerate.
 *
 * Call dap_init() first. Commands from the host run through
 * dap_host_request(), which applies the idle-cycle and clock policies.
 *
 * @return False if the drag-and-drop drive was already added (USBD) or the
 *         interface could not be added.
 */
bool Adafruit_USBD_CMSISDAP::begin(void) {
#if DAP_CH32_USB_SMALL_PMA
  if (dap_ch32_usb_drive_added) {
    return false;
  }
#endif
  // Hosts find the probe by "CMSIS-DAP" in the interface string.
  setStringDescriptor("Adafruit CMSIS-DAP v2");
  // USB 2.1 makes Windows read the BOS / MS OS 2.0 descriptors.
  TinyUSBDevice.setVersion(0x0210);
  if (!TinyUSBDevice.addInterface(*this)) {
    return false;
  }
  dap_ch32_usb_dap_added = true;
  dap_ch32_usb_reenumerate();
  return true;
}

/*!
 * @brief Interface descriptor (Adafruit_USBD_Interface).
 * @param itfnum_deprecated Unused.
 * @param buf Receives the descriptor, or nullptr to query the length.
 * @param bufsize Size of buf.
 * @return Descriptor length, or 0 if buf is too small.
 */
uint16_t
Adafruit_USBD_CMSISDAP::getInterfaceDescriptor(uint8_t itfnum_deprecated,
                                               uint8_t *buf, uint16_t bufsize) {
  (void)itfnum_deprecated;
  if (!buf) {
    return TUD_VENDOR_DESC_LEN;
  }
  uint8_t const itfnum = TinyUSBDevice.allocInterface(1);
  uint8_t const ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
  uint8_t const ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
  uint8_t desc[] = {TUD_VENDOR_DESCRIPTOR(itfnum, _strid, ep_out, ep_in, 64)};
  if (bufsize < sizeof(desc)) {
    return 0;
  }
  memcpy(buf, desc, sizeof(desc));
  dap_usb_set_interface(itfnum);
  return sizeof(desc);
}

/*! @return True once the host has configured the interface. */
bool Adafruit_USBD_CMSISDAP::mounted(void) { return dap_usb_mounted(); }

/*! @brief Run at most one waiting command. Call from loop(). */
void Adafruit_USBD_CMSISDAP::task(void) {
  uint8_t *req = nullptr;
  int len = dap_usb_get_request(&req);
  if (len < 0 || !dap_usb_tx_ready()) {
    return;
  }
  uint32_t rlen = dap_host_request(req, (uint16_t)len, _resp);
  dap_usb_release_request();
  if (rlen) { // DAP_TransferAbort has no response
    dap_usb_send(_resp, (uint16_t)(rlen > sizeof(_resp) ? sizeof(_resp) : rlen));
  }
  _commands++;
  _last_ms = millis();
}

#endif // USE_TINYUSB
