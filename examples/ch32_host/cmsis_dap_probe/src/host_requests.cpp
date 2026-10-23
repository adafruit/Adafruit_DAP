/*!
 * @file host_requests.cpp
 *
 * Runs CMSIS-DAP commands from a PC host on Adafruit_DAP's engine. Three
 * adjustments are made before a command reaches the engine:
 *
 * - DAP_TransferConfigure: idle cycles raised to DAP_HOST_MIN_IDLE_CYCLES
 * - DAP_SWJ_Clock: capped at DAP_HOST_MAX_CLOCK, then converted from Hz
 *   to the engine's delay count
 * - DAP_Info protocol version: reported as "1.10", the CMSIS-DAP version
 *   whose commands the engine implements
 *
 */

#include "host_requests.h"
#include <Arduino.h>
#include <Adafruit_DAP.h>
#include <dap.h>

// CH32V203 SWD timing, measured: per half SWCLK period, about 56 CPU cycles
// plus 5.06 per delay unit. Rounded so SWCLK stays at or below the request.
static const uint32_t HALF_PERIOD_OVERHEAD = 52;
static const uint32_t DELAY_LOOP_CYCLES = 5;

static uint32_t get32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put32(uint8_t *p, uint32_t v) {
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

// SWCLK in Hz -> engine delay count
static uint32_t clock_delay(uint32_t hz) {
  if (hz == 0) {
    hz = 1;
  }
  uint32_t half = SystemCoreClock / 2U / hz;
  if (half <= HALF_PERIOD_OVERHEAD) {
    return 0;
  }
  return (half - HALF_PERIOD_OVERHEAD + DELAY_LOOP_CYCLES - 1U) /
         DELAY_LOOP_CYCLES;
}

// Response to an unknown command (CMSIS-DAP)
static const uint8_t ID_DAP_INVALID = 0xFF;

// CMSIS-DAP request bits (DAP_Transfer / DAP_TransferBlock)
static const uint8_t TRANSFER_RnW = 1 << 1;
static const uint8_t TRANSFER_MATCH_VALUE = 1 << 4;

// Exact length of the engine's response to a request
static uint32_t response_length(const uint8_t *req, uint16_t len,
                                const uint8_t *resp) {
  if (resp[0] == ID_DAP_INVALID) {
    return 1; // unknown command
  }
  switch (req[0]) {
  case ID_DAP_TRANSFER_ABORT:
    return 0; // no response
  case ID_DAP_INFO:
    return 2 + resp[1];
  case ID_DAP_TRANSFER: {
    // [cmd, count, ack, data of each completed read]
    uint32_t words = 0;
    uint16_t i = 3; // first transfer request: [cmd, index, count, ...]
    for (uint8_t n = 0; n < resp[1] && i < len; n++) {
      uint8_t request = req[i++];
      if (!(request & TRANSFER_RnW)) {
        i += 4; // write data
      } else if (request & TRANSFER_MATCH_VALUE) {
        i += 4; // match value, no data returned
      } else {
        words++;
      }
    }
    return 3 + 4 * words;
  }
  case ID_DAP_TRANSFER_BLOCK: {
    // [cmd, count (2), ack, data if reading]; request [cmd, index, count
    // (2), request]
    uint32_t count = resp[1] | (resp[2] << 8);
    return 4 + ((len > 4 && (req[4] & TRANSFER_RnW)) ? 4 * count : 0);
  }
  case ID_DAP_RESET_TARGET:
    return 3;
  case ID_DAP_JTAG_IDCODE:
    return 6;
  default:
    return 2; // command + status
  }
}

/*!
 * @brief Run one command from the host.
 * @param request Command packet (modified in place by the adjustments).
 * @param len Command length.
 * @param response Receives the response packet (DAP_CONFIG_PACKET_SIZE).
 * @return Response length (0: send no response).
 */
uint32_t dap_host_request(uint8_t *request, uint16_t len, uint8_t *response) {
  if (len >= 2 && request[0] == ID_DAP_TRANSFER_CONFIGURE &&
      request[1] < DAP_HOST_MIN_IDLE_CYCLES) {
    request[1] = DAP_HOST_MIN_IDLE_CYCLES;
  }
  if (len >= 5 && request[0] == ID_DAP_SWJ_CLOCK) {
    uint32_t hz = get32(&request[1]);
    if (hz > DAP_HOST_MAX_CLOCK) {
      hz = DAP_HOST_MAX_CLOCK;
    }
    put32(&request[1], clock_delay(hz));
  }
  dap_process_request(request, response);

  if (len >= 2 && request[0] == ID_DAP_INFO) {
    uint8_t id = request[1];
    if (id == 0x04) {
      static const char version[] = "1.10"; // CMSIS-DAP protocol version
      response[1] = sizeof(version);        // length, including the NUL
      memcpy(&response[2], version, sizeof(version));
    } else if (!((id >= 0x01 && id <= 0x06) || id == 0xF0 || id == 0xFE ||
                 id == 0xFF)) {
      response[1] = 0; // not provided by the engine
    }
  }
  return response_length(request, len, response);
}
