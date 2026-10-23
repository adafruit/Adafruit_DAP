/*!
 * @file Adafruit_CDC_UART_Bridge.h
 *
 * USB serial <-> target UART bridge. The UART (QT Py: TX = PA2, RX = PA3,
 * USART2, DMA driven) follows the terminal's baud rate, data bits, parity
 * and stop bits. Don't use Serial2 at the same time (same USART).
 *
 * MIT license.
 */

#ifndef ADAFRUIT_CDC_UART_BRIDGE_H
#define ADAFRUIT_CDC_UART_BRIDGE_H

#include <Arduino.h>

#if defined(USE_TINYUSB) && defined(CH32V20x)

#include <Adafruit_TinyUSB.h>

#include "uart_format.h"

/*!
 * @brief Bridges a TinyUSB CDC port to the target's UART.
 */
class Adafruit_CDC_UART_Bridge {
public:
  void begin(Adafruit_USBD_CDC &cdc, uint32_t initial_baud = 115200);

  void task(void);

  /*! @return Current baud rate. */
  uint32_t baud(void) const { return _baud; }
  /*! @return False if the terminal's format is unsupported (8N1 used). */
  bool formatExact(void) const { return _fmt.exact; }
  /*! @return Bytes sent to the target. */
  uint32_t bytesToTarget(void) const { return _to_target; }
  /*! @return Bytes received from the target. */
  uint32_t bytesFromTarget(void) const { return _from_target; }
  /*! @return millis() of the last traffic. */
  uint32_t lastActivityMillis(void) const { return _last_ms; }

private:
  void applyLineCoding(void);

  Adafruit_USBD_CDC *_cdc = nullptr;
  uint32_t _baud = 0;
  uint8_t _bits = 8, _parity = 0, _stop = 0;
  dap_uart_format_t _fmt = {0, 0, 0, 0xFF, 1};
  uint32_t _to_target = 0, _from_target = 0, _last_ms = 0;
};

#endif // USE_TINYUSB && CH32V20x
#endif
