/*!
 * @file dap_uart_ch32.h
 *
 * Non-blocking DMA UART (USART2) for the USB serial bridge.
 *
 */

#ifndef ADAFRUIT_DAP_CH32_DAP_UART_H
#define ADAFRUIT_DAP_CH32_DAP_UART_H

#include <stdint.h>

#include "uart_format.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Start the UART and its DMA channels.
 * @param baud Baud rate.
 * @param fmt Frame format.
 */
void dap_uart_begin(uint32_t baud, const dap_uart_format_t *fmt);
/*! @brief Stop the UART and its DMA channels. */
void dap_uart_end(void);
/*! @return Received bytes waiting to be read. */
uint32_t dap_uart_rx_available(void);
/*!
 * @brief Read received bytes.
 * @param buf Receives the bytes.
 * @param max Buffer size.
 * @return Bytes read.
 */
uint32_t dap_uart_read(uint8_t *buf, uint32_t max);
/*! @return Free space in the transmit ring. */
uint32_t dap_uart_tx_free(void);
/*!
 * @brief Queue bytes for transmission (never blocks).
 * @param buf Bytes.
 * @param len Count.
 * @return Bytes queued.
 */
uint32_t dap_uart_write(const uint8_t *buf, uint32_t len);
/*! @brief Finish the running TX chunk and start the next. Call often. */
void dap_uart_task(void);

#ifdef __cplusplus
}
#endif

#endif
