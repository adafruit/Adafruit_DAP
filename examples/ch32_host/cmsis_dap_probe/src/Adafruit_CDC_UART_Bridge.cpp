/*!
 * @file Adafruit_CDC_UART_Bridge.cpp
 *
 * USB serial <-> target UART bridge.
 *
 * MIT license.
 */

#include "Adafruit_CDC_UART_Bridge.h"

#if defined(USE_TINYUSB) && defined(CH32V20x)

#include "dap_uart_ch32.h"

/*!
 * @brief Start the UART at initial_baud (8N1) until the terminal sets its
 *        own format.
 * @param cdc USB serial port (usually Serial).
 * @param initial_baud Baud rate until the terminal sets one.
 */
void Adafruit_CDC_UART_Bridge::begin(Adafruit_USBD_CDC &cdc,
                                     uint32_t initial_baud) {
  _cdc = &cdc;
  _baud = initial_baud;
  _bits = 8;
  _parity = DAP_CDC_PARITY_NONE;
  _stop = DAP_CDC_STOP_1;
  _fmt = dap_uart_format_from_cdc(_bits, _parity, _stop);
  dap_uart_begin(_baud, &_fmt);
}

void Adafruit_CDC_UART_Bridge::applyLineCoding(void) {
  uint32_t baud = _cdc->baud();
  uint8_t bits = _cdc->numbits();
  uint8_t parity = _cdc->paritytype();
  uint8_t stop = _cdc->stopbits();
  if (baud == 0 ||
      (baud == _baud && bits == _bits && parity == _parity && stop == _stop)) {
    return; // nothing set yet, or unchanged
  }
  _baud = baud;
  _bits = bits;
  _parity = parity;
  _stop = stop;
  _fmt = dap_uart_format_from_cdc(bits, parity, stop);
  dap_uart_begin(_baud, &_fmt);
}

/*! @brief Apply terminal settings and move data both ways. Call often. */
void Adafruit_CDC_UART_Bridge::task(void) {
  if (!_cdc) {
    return;
  }
  applyLineCoding();

  uint8_t buf[64];

  // Target -> PC. Without a terminal (DTR low) the bytes are dropped.
  uint32_t n = dap_uart_rx_available();
  if (n) {
    if (_cdc->dtr()) {
      uint32_t room = (uint32_t)_cdc->availableForWrite();
      if (n > room)
        n = room;
      if (n > sizeof(buf))
        n = sizeof(buf);
      if (n) {
        n = dap_uart_read(buf, n);
        _cdc->write(buf, n);
        _from_target += n;
        _last_ms = millis();
      }
    } else {
      while (dap_uart_read(buf, sizeof(buf))) {
      }
    }
  }

  // PC -> target, limited by free UART TX space so this never blocks.
  int avail = _cdc->available();
  if (avail > 0) {
    uint32_t m = (uint32_t)avail;
    uint32_t space = dap_uart_tx_free();
    if (m > space)
      m = space;
    if (m > sizeof(buf))
      m = sizeof(buf);
    if (m) {
      m = _cdc->read(buf, m);
      dap_uart_write(buf, m);
      _to_target += m;
      _last_ms = millis();
    }
  }

  dap_uart_task(); // keep the TX DMA moving
}

#endif // USE_TINYUSB && CH32V20x
