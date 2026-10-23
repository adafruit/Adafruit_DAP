/*!
 * @file usb_functions.h
 *
 * Shared state of the library's USB functions.
 *
 */

#ifndef ADAFRUIT_DAP_CH32_USB_FUNCTIONS_H
#define ADAFRUIT_DAP_CH32_USB_FUNCTIONS_H

#include <Arduino.h>

/*! Set by Adafruit_USBD_CMSISDAP::begin(). */
extern bool dap_ch32_usb_dap_added;
/*! Set by Adafruit_USBD_DragDrop::begin(). */
extern bool dap_ch32_usb_drive_added;

/*! True when built for the USBD peripheral (small packet memory). */
#if defined(CFG_TUD_WCH_USBIP_FSDEV) && CFG_TUD_WCH_USBIP_FSDEV
#define DAP_CH32_USB_SMALL_PMA 1
#else
#define DAP_CH32_USB_SMALL_PMA 0
#endif

void dap_ch32_usb_reenumerate(void);

#endif
