/*!
 * @file Adafruit_USBD_DragDrop.cpp
 *
 * Drag-and-drop programming drive with TinyUSB
 *
 */

#include "Adafruit_USBD_DragDrop.h"

#if defined(USE_TINYUSB)


// Re-enumerate so the host sees the drive added after the core's USB start.
static void reenumerate(void) {
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

Adafruit_USBD_DragDrop *Adafruit_USBD_DragDrop::_instance = nullptr;

/*!
 * @brief Identify the target (no halt or reset), add the drive and
 *        re-enumerate.
 * @param dap Probe; begin() must have been called on it, with
 *            dragdrop_dap_error as its error handler.
 * @param label Volume label, up to 11 characters.
 * @return False if the drive could not be added.
 */
bool Adafruit_USBD_DragDrop::begin(Adafruit_DAP_STM32 *dap, const char *label) {
  _instance = this;
  _engine.begin(dap, label);

  _msc.setID("Adafruit", "QT Py DAP Drive", "1.0");
  _msc.setCapacity(VirtualFat::TOTAL_SECTORS, VirtualFat::SECTOR_SIZE);
  _msc.setReadWriteCallback(readCb, writeCb, flushCb);
  _msc.setReadyCallback(readyCb);
  _msc.setUnitReady(true);
  if (!_msc.begin()) {
    return false;
  }
  reenumerate();
  return true;
}

/*!
 * @brief Program received sectors and end drops. Call from loop().
 *
 * Slow work runs here, outside the USB stack: write callbacks hand sectors
 * over with TUD_MSC_RET_ASYNC, completed by tud_msc_async_io_done().
 */
void Adafruit_USBD_DragDrop::task(void) {
  if (_engine.pending()) {
    _engine.process();
    tud_msc_async_io_done((int32_t)_pending_bytes, false);
  }
  _engine.poll(millis());
}

int32_t Adafruit_USBD_DragDrop::readCb(uint32_t lba, void *buffer,
                                       uint32_t bufsize) {
  uint8_t *p = (uint8_t *)buffer;
  for (uint32_t done = 0; done + VirtualFat::SECTOR_SIZE <= bufsize;
       done += VirtualFat::SECTOR_SIZE) {
    _instance->_engine.read(lba++, p + done);
  }
  return (int32_t)bufsize;
}

int32_t Adafruit_USBD_DragDrop::writeCb(uint32_t lba, uint8_t *buffer,
                                        uint32_t bufsize) {
  if (_instance->_engine.write(lba, buffer)) {
    _instance->_pending_bytes = bufsize;
    return TUD_MSC_RET_ASYNC; // buffer stays valid until io_done
  }
  return (int32_t)bufsize;
}

void Adafruit_USBD_DragDrop::flushCb(void) {}

bool Adafruit_USBD_DragDrop::readyCb(void) {
  return _instance->_engine.unitReady();
}

#endif // USE_TINYUSB
