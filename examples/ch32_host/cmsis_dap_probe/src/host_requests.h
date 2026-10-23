/*!
 * @file host_requests.h
 *
 * Runs CMSIS-DAP commands from a PC host with the adjustments
 * a CH32V203 host needs.
 *
 */

#ifndef HOST_REQUESTS_H
#define HOST_REQUESTS_H

#include <stdint.h>

/*! Highest SWD clock a host may set, in Hz. Faster clocks are unreliable
 *  on jumper wires with the CH32V203's 2 MHz output edges. */
#define DAP_HOST_MAX_CLOCK 1000000UL

/*! Minimum idle cycles after each transfer: an STM32F405 needs 2 with the
 *  CH32V203's SWD timing. */
#define DAP_HOST_MIN_IDLE_CYCLES 2

uint32_t dap_host_request(uint8_t *request, uint16_t len, uint8_t *response);

#endif
