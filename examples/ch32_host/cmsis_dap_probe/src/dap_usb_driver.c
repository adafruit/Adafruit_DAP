/*!
 * @file dap_usb_driver.c
 *
 * TinyUSB class driver for the CMSIS-DAP v2 interface.
 *
 * TinyUSB's vendor class is not used: it buffers data in a byte FIFO (packet
 * boundaries lost) and sends a zero-length packet after a 64-byte response,
 * which hosts read as an empty response to the next command. Here each OUT
 * transfer is one command and each IN transfer one response. The OUT
 * endpoint is re-armed only after a command has run, so the host is NAKed
 * while the probe is busy.
 *
 * Also provides the BOS and Microsoft OS 2.0 descriptors that make Windows
 * bind WinUSB to the interface without a driver install.
 *
 * MIT license.
 */

#if defined(USE_TINYUSB)

#include "dap_usb_driver.h"

#include "device/usbd_pvt.h"
#include "tusb.h"

#define DAP_USB_PACKET_SIZE 64 //!< CMSIS-DAP packet size

static uint8_t _rhport = 0;
static uint8_t _itf_num = 0xFF; // interface number assigned at enumeration
static uint8_t _ep_out = 0, _ep_in = 0;
static volatile bool _mounted = false;
static volatile bool _rx_ready = false; // a command is waiting in _rx_buf
static volatile uint16_t _rx_len = 0;
static volatile bool _tx_busy = false; // a response is being sent

/// @cond
TU_ATTR_ALIGNED(4) static uint8_t _rx_buf[DAP_USB_PACKET_SIZE];
TU_ATTR_ALIGNED(4) static uint8_t _tx_buf[DAP_USB_PACKET_SIZE];
/// @endcond

bool dap_usb_mounted(void) { return _mounted; }

int dap_usb_get_request(uint8_t **buf) {
  if (!_mounted || !_rx_ready) {
    return -1;
  }
  *buf = _rx_buf;
  return _rx_len;
}

void dap_usb_release_request(void) {
  _rx_ready = false;
  if (_mounted) {
    usbd_edpt_xfer(_rhport, _ep_out, _rx_buf, DAP_USB_PACKET_SIZE, false);
  }
}

bool dap_usb_tx_ready(void) { return _mounted && !_tx_busy; }

bool dap_usb_send(const uint8_t *data, uint16_t len) {
  if (!dap_usb_tx_ready() || len > DAP_USB_PACKET_SIZE) {
    return false;
  }
  memcpy(_tx_buf, data, len);
  _tx_busy = true;
  if (!usbd_edpt_xfer(_rhport, _ep_in, _tx_buf, len, false)) {
    _tx_busy = false;
    return false;
  }
  return true;
}

static void dapd_reset_state(void) {
  _mounted = false;
  _rx_ready = false;
  _rx_len = 0;
  _tx_busy = false;
}

static void dapd_init(void) { dapd_reset_state(); }

static void dapd_reset(uint8_t rhport) {
  (void)rhport;
  dapd_reset_state();
}

static uint16_t dapd_open(uint8_t rhport, tusb_desc_interface_t const *itf,
                          uint16_t max_len) {
  // Claim only our own interface; other vendor interfaces go to other drivers.
  if (itf->bInterfaceNumber != _itf_num ||
      itf->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC ||
      itf->bNumEndpoints != 2) {
    return 0;
  }
  uint16_t const len =
      sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
  if (max_len < len) {
    return 0;
  }
  uint8_t const *p_ep = tu_desc_next(itf);
  if (!usbd_open_edpt_pair(rhport, p_ep, 2, TUSB_XFER_BULK, &_ep_out,
                           &_ep_in)) {
    return 0;
  }
  _rhport = rhport;
  dapd_reset_state();
  _mounted = true;
  usbd_edpt_xfer(rhport, _ep_out, _rx_buf, DAP_USB_PACKET_SIZE, false);
  return len;
}

static bool dapd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *request) {
  (void)rhport;
  (void)stage;
  (void)request;
  return false; // no class-specific requests
}

static bool dapd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                         uint32_t xferred_bytes) {
  if (ep_addr == _ep_out) {
    if (result != XFER_RESULT_SUCCESS || xferred_bytes == 0) {
      // Ignore errors and zero-length packets: re-arm and wait.
      usbd_edpt_xfer(rhport, _ep_out, _rx_buf, DAP_USB_PACKET_SIZE, false);
      return true;
    }
    _rx_len = (uint16_t)xferred_bytes;
    _rx_ready = true; // processed (and re-armed) by the main loop
    return true;
  }
  if (ep_addr == _ep_in) {
    _tx_busy = false;
    return true;
  }
  return false;
}

static usbd_class_driver_t const _dap_driver = {
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
    .name = "CMSIS-DAP",
#endif
    .init = dapd_init,
    .reset = dapd_reset,
    .open = dapd_open,
    .control_xfer_cb = dapd_control_xfer_cb,
    .xfer_cb = dapd_xfer_cb,
    .sof = NULL,
};

/*!
 * @brief Register the CMSIS-DAP class driver with TinyUSB (application
 *        drivers take precedence over the built-in ones).
 * @param driver_count Receives the number of drivers.
 * @return The driver table.
 */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &_dap_driver;
}

// BOS + Microsoft OS 2.0 descriptors

enum { DAP_VENDOR_REQUEST_MICROSOFT = 0x20 };

#define MS_OS_20_DESC_LEN 0xB2       //!< total MS OS 2.0 descriptor set length
#define MS_OS_20_SET_HEADER_LEN 0x0A //!< set header length
#define MS_OS_20_CONFIG_SUBSET_LEN 0x08 //!< configuration subset header length
#define MS_OS_20_FUNCTION_SUBSET_LEN 0x08 //!< function subset header length
#define MS_OS_20_COMPAT_ID_LEN 0x14       //!< compatible ID descriptor length
/*! Byte offset of bFirstInterface in the function subset header. */
#define MS_OS_20_FIRST_INTERFACE_OFFSET                                        \
  (MS_OS_20_SET_HEADER_LEN + MS_OS_20_CONFIG_SUBSET_LEN + 4)
/*! BOS descriptor length. */
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static uint8_t const _desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN,
                                DAP_VENDOR_REQUEST_MICROSOFT)};

// WinUSB compatible ID and the CMSIS-DAP v2 interface GUID (as DAPLink).
static uint8_t _desc_ms_os_20[] = {
    // Set header: length, type, Windows version, total length
    U16_TO_U8S_LE(MS_OS_20_SET_HEADER_LEN),
    U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000),
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, config index, reserved,
    // subset length
    U16_TO_U8S_LE(MS_OS_20_CONFIG_SUBSET_LEN),
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - MS_OS_20_SET_HEADER_LEN),

    // Function subset header: length, type, first interface (set at
    // enumeration), reserved, subset length
    U16_TO_U8S_LE(MS_OS_20_FUNCTION_SUBSET_LEN),
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 0, 0,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - MS_OS_20_SET_HEADER_LEN -
                  MS_OS_20_CONFIG_SUBSET_LEN),

    // Compatible ID: WINUSB
    U16_TO_U8S_LE(MS_OS_20_COMPAT_ID_LEN),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B',
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property: DeviceInterfaceGUIDs (REG_MULTI_SZ)
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - MS_OS_20_SET_HEADER_LEN -
                  MS_OS_20_CONFIG_SUBSET_LEN - MS_OS_20_FUNCTION_SUBSET_LEN -
                  MS_OS_20_COMPAT_ID_LEN),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(0x0007),
    U16_TO_U8S_LE(0x002A), // property name length: "DeviceInterfaceGUIDs\0"
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00,
    'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00,
    'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00,
    0x00,
    U16_TO_U8S_LE(0x0050), // property data length
    // "{CDB3B5AD-293B-4663-AA36-1AAE46463776}\0\0" in UTF-16
    '{', 0x00, 'C', 0x00, 'D', 0x00, 'B', 0x00, '3', 0x00, 'B', 0x00, '5', 0x00,
    'A', 0x00, 'D', 0x00, '-', 0x00, '2', 0x00, '9', 0x00, '3', 0x00, 'B', 0x00,
    '-', 0x00, '4', 0x00, '6', 0x00, '6', 0x00, '3', 0x00, '-', 0x00, 'A', 0x00,
    'A', 0x00, '3', 0x00, '6', 0x00, '-', 0x00, '1', 0x00, 'A', 0x00, 'A', 0x00,
    'E', 0x00, '4', 0x00, '6', 0x00, '4', 0x00, '6', 0x00, '3', 0x00, '7', 0x00,
    '7', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00};

/// @cond
TU_VERIFY_STATIC(sizeof(_desc_ms_os_20) == MS_OS_20_DESC_LEN,
                 "MS OS 2.0 descriptor size");
/// @endcond

void dap_usb_set_interface(uint8_t itf) {
  _itf_num = itf;
  _desc_ms_os_20[MS_OS_20_FIRST_INTERFACE_OFFSET] = itf;
}

/*! @return The BOS descriptor (TinyUSB callback). */
uint8_t const *tud_descriptor_bos_cb(void) { return _desc_bos; }

/*!
 * @brief Answer the Microsoft OS 2.0 descriptor request (TinyUSB callback).
 * @param rhport USB port.
 * @param stage Control transfer stage.
 * @param request Setup packet.
 * @return False to stall unknown requests.
 */
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  if (stage != CONTROL_STAGE_SETUP) {
    return true;
  }
  if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
      request->bRequest == DAP_VENDOR_REQUEST_MICROSOFT &&
      request->wIndex == 7) { // MS_OS_20_DESCRIPTOR_INDEX
    return tud_control_xfer(rhport, request, (void *)(uintptr_t)_desc_ms_os_20,
                            MS_OS_20_DESC_LEN);
  }
  return false; // stall anything else
}

#endif // USE_TINYUSB
