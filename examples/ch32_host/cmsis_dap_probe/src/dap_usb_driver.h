/*!
 * @file dap_usb_driver.h
 *
 * TinyUSB class driver for the CMSIS-DAP v2 interface
 *
 */

#ifndef ADAFRUIT_DAP_CH32_DAP_USB_DRIVER_H
#define ADAFRUIT_DAP_CH32_DAP_USB_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Set the interface number assigned at enumeration.
 * @param itf Interface number.
 */
void dap_usb_set_interface(uint8_t itf);
/*! @return True once the host has configured the interface. */
bool dap_usb_mounted(void);
/*!
 * @brief Get the waiting command, if any.
 * @param buf Receives the command buffer.
 * @return Command length, or -1 if none is waiting.
 */
int dap_usb_get_request(uint8_t **buf);
/*! @brief Release the command buffer and accept the next command. */
void dap_usb_release_request(void);
/*! @return True if a response can be sent. */
bool dap_usb_tx_ready(void);
/*!
 * @brief Send one response packet (copied).
 * @param data Response.
 * @param len Length (at most 64).
 * @return True if queued.
 */
bool dap_usb_send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
