/*!
 * @file Adafruit_CDC_UART_Bridge.h
 *
 * USB serial <-> target UART bridge
 *
 */

#ifndef ADAFRUIT_CDC_UART_BRIDGE_H
#define ADAFRUIT_CDC_UART_BRIDGE_H

#include <Arduino.h>

#if defined(USE_TINYUSB) && defined(CH32V20x)

#include <Adafruit_TinyUSB.h>

// ---- USART frame format for a CDC line coding ----

#include <stdint.h>

// CDC line coding values (USB CDC PSTN spec)
#define DAP_CDC_STOP_1 0       //!< 1 stop bit
#define DAP_CDC_STOP_1_5 1     //!< 1.5 stop bits
#define DAP_CDC_STOP_2 2       //!< 2 stop bits
#define DAP_CDC_PARITY_NONE 0  //!< no parity
#define DAP_CDC_PARITY_ODD 1   //!< odd parity
#define DAP_CDC_PARITY_EVEN 2  //!< even parity
#define DAP_CDC_PARITY_MARK 3  //!< mark parity (unsupported)
#define DAP_CDC_PARITY_SPACE 4 //!< space parity (unsupported)

/*! USART frame format for a CDC line coding. */
typedef struct {
  uint8_t word9;   //!< 1: 9-bit USART word, 0: 8-bit
  uint8_t parity;  //!< DAP_CDC_PARITY_NONE / _ODD / _EVEN
  uint8_t stop;    //!< DAP_CDC_STOP_1 / _1_5 / _2
  uint8_t rx_mask; //!< mask for received bytes (0x7F strips parity)
  uint8_t exact;   //!< 1 if the requested format is reproduced exactly
} dap_uart_format_t;

/*!
 * @brief Map a CDC line coding onto a USART frame format.
 * @param data_bits CDC data bits (5, 6, 7, 8 or 16).
 * @param parity CDC parity (DAP_CDC_PARITY_*).
 * @param stop_bits CDC stop bits (DAP_CDC_STOP_*).
 * @return Frame format; exact = 0 if it falls back to 8N1.
 */
static inline dap_uart_format_t
dap_uart_format_from_cdc(uint8_t data_bits, uint8_t parity, uint8_t stop_bits) {
  dap_uart_format_t f;
  f.exact = 1;

  // Parity: none, odd and even are supported; mark/space are not.
  if (parity == DAP_CDC_PARITY_ODD || parity == DAP_CDC_PARITY_EVEN) {
    f.parity = parity;
  } else {
    f.parity = DAP_CDC_PARITY_NONE;
    if (parity != DAP_CDC_PARITY_NONE) {
      f.exact = 0;
    }
  }

  // Data bits -> word length (which includes the parity bit).
  f.rx_mask = 0xFF;
  if (data_bits == 8) {
    f.word9 = (f.parity != DAP_CDC_PARITY_NONE);
  } else if (data_bits == 7 && f.parity != DAP_CDC_PARITY_NONE) {
    f.word9 = 0;      // 7 data + parity in an 8-bit word
    f.rx_mask = 0x7F; // received bit 7 is the parity bit
  } else {
    f.word9 = 0; // unsupported: fall back to 8N
    f.parity = DAP_CDC_PARITY_NONE;
    f.exact = 0;
  }

  // Stop bits: 1, 1.5 and 2 are all supported.
  if (stop_bits == DAP_CDC_STOP_1_5 || stop_bits == DAP_CDC_STOP_2) {
    f.stop = stop_bits;
  } else {
    f.stop = DAP_CDC_STOP_1;
    if (stop_bits != DAP_CDC_STOP_1) {
      f.exact = 0;
    }
  }
  return f;
}

// ---- bridge ----

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
