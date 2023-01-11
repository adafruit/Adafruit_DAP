/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>, Adafruit
 * <info@adafruit.com>
 * All rights reserved.
 *
 * This is mostly a re-mix of Alex Taradovs excellent Free-DAP code - we just
 * put both halves into one library and wrapped it up in Arduino compatibility
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DAP_CONFIG_H_
#define _DAP_CONFIG_H_

/*- Includes ----------------------------------------------------------------*/
#include "Arduino.h"

static inline void DAP_CONFIG_SWCLK_TCK_clr(void);
static inline void DAP_CONFIG_SWCLK_TCK_set(void);
static inline void DAP_CONFIG_SWDIO_TMS_clr(void);
static inline void DAP_CONFIG_SWDIO_TMS_set(void);

#define DAP_CONFIG_ENABLE_SWD
//#define DAP_CONFIG_ENABLE_JTAG

#define DAP_CONFIG_DEFAULT_PORT DAP_PORT_SWD
#define DAP_CONFIG_DEFAULT_CLOCK 50

#define DAP_CONFIG_PACKET_SIZE 64
#define DAP_CONFIG_PACKET_COUNT 1

extern int DAP_CONFIG_SWCLK_PIN;
extern int DAP_CONFIG_SWDIO_PIN;
extern int DAP_CONFIG_nRESET_PIN;

#if defined(ARDUINO_ARCH_SAMD)
// ultra speedy
extern volatile uint32_t *SWCLK_OUTSETREG, *SWCLK_OUTCLRREG, *SWCLK_DIRREG,
    *SWCLK_INREG;
extern uint32_t SWCLK_PINMASK;
extern volatile uint32_t *SWDIO_OUTSETREG, *SWDIO_OUTCLRREG, *SWDIO_DIRREG,
    *SWDIO_INREG;
extern uint32_t SWDIO_PINMASK;

#elif defined(TEENSYDUINO)

extern volatile uint32_t SWCLK_BITMASK, *SWCLK_PORTCONFIG;
extern volatile uint8_t *SWCLK_PORTMODE, *SWCLK_PORTSET, *SWCLK_PORTCLEAR,
    *SWCLK_PORT_INPUT_REG;

extern volatile uint32_t SWDIO_BITMASK, *SWDIO_PORTCONFIG;
extern volatile uint8_t *SWDIO_PORTMODE, *SWDIO_PORTSET, *SWDIO_PORTCLEAR,
    *SWDIO_PORT_INPUT_REG;

#endif

// Set the value to NULL if you want to disable a string
// DAP_CONFIG_PRODUCT_STR must contain "CMSIS-DAP" to be compatible with the
// standard
#define DAP_CONFIG_VENDOR_STR "Adafruit"
#define DAP_CONFIG_PRODUCT_STR "Generic CMSIS-DAP Adapter"
#define DAP_CONFIG_SER_NUM_STR "123456"
#define DAP_CONFIG_FW_VER_STR "v0.1"
#define DAP_CONFIG_DEVICE_VENDOR_STR NULL
#define DAP_CONFIG_DEVICE_NAME_STR NULL

//#define DAP_CONFIG_RESET_TARGET_FN     target_specific_reset_function

// A value at which dap_clock_test() produces 1 kHz output on the SWCLK pin
#define DAP_CONFIG_DELAY_CONSTANT 4000

// A threshold for switching to fast clock (no added delays)
// This is the frequency produced by dap_clock_test(1) on the SWCLK pin
#define DAP_CONFIG_FAST_CLOCK 463000 // Hz

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_write(int value) {
  if (value == 0)
    DAP_CONFIG_SWCLK_TCK_clr();
  else
    DAP_CONFIG_SWCLK_TCK_set();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_write(int value) {
  if (value == 0)
    DAP_CONFIG_SWDIO_TMS_clr();
  else
    DAP_CONFIG_SWDIO_TMS_set();
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_TDO_write(int value) {
  (void) value;
  // DAP_gpio_write(DAP_CONFIG_TDO.ulPort, DAP_CONFIG_TDO.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_nTRST_write(int value) {
  (void) value;
  // DAP_gpio_write(DAP_CONFIG_nTRST.ulPort, DAP_CONFIG_nTRST.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_nRESET_write(int value) {
  digitalWrite(DAP_CONFIG_nRESET_PIN, value);
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWCLK_TCK_read(void) {
#if defined(ARDUINO_ARCH_SAMD)
  return (*SWCLK_INREG & SWCLK_PINMASK) != 0;
#elif defined(TEENSYDUINO)
  return (*SWCLK_PORT_INPUT_REG & SWCLK_BITMASK) ? 1 : 0;
#else
  return digitalRead(DAP_CONFIG_SWCLK_PIN);
#endif
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWDIO_TMS_read(void) {
#if defined(ARDUINO_ARCH_SAMD)
  return (*SWDIO_INREG & SWDIO_PINMASK) != 0;
#elif defined(TEENSYDUINO)
  return (*SWDIO_PORT_INPUT_REG & SWDIO_BITMASK) ? 1 : 0;
#else
  return digitalRead(DAP_CONFIG_SWDIO_PIN);
#endif
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_TDI_read(void) {
  // return (DAP_gpio_read_bulk(DAP_CONFIG_TDI.ulPort) & (1 <<
  // DAP_CONFIG_TDI.ulPin)) > 0;
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_TDO_read(void) {
  // return (DAP_gpio_read_bulk(DAP_CONFIG_TD0.ulPort) & (1 <<
  // DAP_CONFIG_TD0.ulPin)) > 0;
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_nTRST_read(void) {
  // return (DAP_gpio_read_bulk(DAP_CONFIG_nTRST.ulPort) & (1 <<
  // DAP_CONFIG_nTRST.ulPin)) > 0;
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_nRESET_read(void) {
  return digitalRead(DAP_CONFIG_nRESET_PIN);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_set(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWCLK_OUTSETREG = SWCLK_PINMASK;
#elif defined(TEENSYDUINO)
  *SWCLK_PORTSET = SWCLK_BITMASK;
#else
  digitalWrite(DAP_CONFIG_SWCLK_PIN, HIGH);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_clr(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWCLK_OUTCLRREG = SWCLK_PINMASK;
#elif defined(TEENSYDUINO)
  *SWCLK_PORTCLEAR = SWCLK_BITMASK;
#else
  digitalWrite(DAP_CONFIG_SWCLK_PIN, LOW);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_set(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWDIO_OUTSETREG = SWDIO_PINMASK;
#elif defined(TEENSYDUINO)
  *SWDIO_PORTSET = SWDIO_BITMASK;
#else
  digitalWrite(DAP_CONFIG_SWDIO_PIN, HIGH);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_clr(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWDIO_OUTCLRREG = SWDIO_PINMASK;
#elif defined(TEENSYDUINO)
  *SWDIO_PORTCLEAR = SWDIO_BITMASK;
#else
  digitalWrite(DAP_CONFIG_SWDIO_PIN, LOW);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_in(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWDIO_DIRREG &= ~SWDIO_PINMASK;
#elif defined(TEENSYDUINO)
  *SWDIO_PORTMODE &= ~SWDIO_BITMASK;
  *SWDIO_PORTCONFIG = PORT_PCR_MUX(1);
#else
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_out(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWDIO_DIRREG |= SWDIO_PINMASK;
#elif defined(TEENSYDUINO)
  *SWDIO_PORTMODE |= SWDIO_BITMASK;
  *SWDIO_PORTCONFIG = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
  *SWDIO_PORTCONFIG &= ~PORT_PCR_ODE;
#else
  pinMode(DAP_CONFIG_SWDIO_PIN, OUTPUT);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SETUP() {
  pinMode(DAP_CONFIG_SWCLK_PIN, INPUT);
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT_PULLUP);
  // pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  // pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
  pinMode(DAP_CONFIG_nRESET_PIN, INPUT);
  #ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  #endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_DISCONNECT(void) {
  pinMode(DAP_CONFIG_SWCLK_PIN, INPUT);
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  // pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
  pinMode(DAP_CONFIG_nRESET_PIN, INPUT);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_SWD(void) {
  pinMode(DAP_CONFIG_SWDIO_PIN, OUTPUT);
  digitalWrite(DAP_CONFIG_SWDIO_PIN, HIGH);

  pinMode(DAP_CONFIG_SWCLK_PIN, OUTPUT);
  digitalWrite(DAP_CONFIG_SWCLK_PIN, HIGH);

  pinMode(DAP_CONFIG_nRESET_PIN, OUTPUT);
  for (size_t i = 0; i < 1000000ul; i++) {
    asm("nop");
  }
  digitalWrite(DAP_CONFIG_nRESET_PIN, HIGH);

  // pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  // pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_JTAG(void) {
  /* TODO: this isn't supported anyway
DAP_gpio_init(PORTA, CONFIG_DAP_SWDIO, 1);
gpio_outset_bulk(PORTA, (1ul << CONFIG_DAP_SWDIO));

DAP_gpio_init(PORTA, CONFIG_DAP_SWCLK, 1);
gpio_outset_bulk(PORTA, (1ul << CONFIG_DAP_SWCLK));

DAP_gpio_init(PORTA, CONFIG_DAP_TDI, 1);
gpio_outset_bulk(PORTA, (1ul << CONFIG_DAP_TDI));

DAP_gpio_init(PORTA, CONFIG_DAP_TDO, 0);

DAP_gpio_init(PORTA, CONFIG_DAP_nRESET, 1);
gpio_outset_bulk(PORTA, (1ul << CONFIG_DAP_nRESET));

DAP_gpio_init(PORTA, CONFIG_DAP_nTRST, 1);
gpio_outset_bulk(PORTA, (1ul << CONFIG_DAP_nTRST));
*/
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_LED(int index, int state) {
  #ifdef LED_BUILTIN
    if (0 == index)
      digitalWrite(LED_BUILTIN, !state);
  #endif
}

#endif // _DAP_CONFIG_H_
