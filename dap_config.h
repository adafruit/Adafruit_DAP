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

#elif defined(ARDUINO_ARCH_CH32)
// Set in dap_init(): bit set/reset/input registers, and the SWDIO pin's
// 4-bit mode field in CFGLR/CFGHR
extern GPIO_TypeDef *SWCLK_PORT, *SWDIO_PORT;
extern uint32_t SWCLK_PINMASK, SWDIO_PINMASK;
extern volatile uint32_t *SWDIO_CFGREG;
extern uint32_t SWDIO_CFGSHIFT;
extern GPIO_TypeDef *NRESET_PORT; // NULL if no valid nRESET pin
extern uint32_t NRESET_PINMASK;

// CFGxR field values. Outputs use 2 MHz edges: faster edges ring on jumper
// wires and corrupt SWD transfers.
#define DAP_CH32_CFG_OUT_PP_2MHZ 0x2U // push-pull output, 2 MHz edges
#define DAP_CH32_CFG_IN_PULL 0x8U     // input, pull-up/down (OUTDR selects)
#define DAP_CH32_CFG_IN_FLOAT 0x4U    // input, floating
#define DAP_CH32_CFG_OUT_OD 0x7U      // open-drain output (nRESET)

static inline void DAP_CONFIG_CH32_mode(GPIO_TypeDef *port, uint32_t mask,
                                        uint32_t mode) {
  uint32_t pin = __builtin_ctz(mask);
  volatile uint32_t *cfg = (pin < 8) ? &port->CFGLR : &port->CFGHR;
  uint32_t shift = (pin & 7) * 4;
  *cfg = (*cfg & ~(0xFU << shift)) | (mode << shift);
}

#endif

// Status LED. The CH32 core defines LED_BUILTIN as PNUM_NOT_DEFINED on
// boards without one; writing to that pin dereferences a NULL GPIO port.
#if defined(LED_BUILTIN) &&                                                    \
    !(defined(ARDUINO_ARCH_CH32) && (LED_BUILTIN == PNUM_NOT_DEFINED))
#define DAP_CONFIG_HAS_LED
#endif

// SWD bit-bang functions are placed with this attribute. CH32V203: code past
// the first 32 KB of flash runs ~2.4x slower, plain .text is linked first.
#if defined(ARDUINO_ARCH_CH32)
#define DAP_CONFIG_PERFORMANCE_ATTR __attribute__((section(".text")))
#else
#define DAP_CONFIG_PERFORMANCE_ATTR
#endif

// Around each DAP command (Adafruit_DAP::dbg_dap_cmd()). Interrupts stay on
// on CH32: its USB serial is serviced from interrupts and yield(), and long
// interrupt-free stretches disrupt it. SWD is clocked by the host, so
// interrupts only stretch clock phases.
#if defined(ARDUINO_ARCH_CH32)
#define DAP_CONFIG_ENTER_CRITICAL()
#define DAP_CONFIG_EXIT_CRITICAL()
#else
#define DAP_CONFIG_ENTER_CRITICAL() noInterrupts()
#define DAP_CONFIG_EXIT_CRITICAL() interrupts()
#endif

// Idle cycles after each transfer, set by Adafruit_DAP::targetConnect().
// With the CH32V203's SWD timing an STM32F405 needs 2.
#if defined(ARDUINO_ARCH_CH32)
#define DAP_CONFIG_IDLE_CYCLES 2
#else
#define DAP_CONFIG_IDLE_CYCLES 0
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

#if defined(ARDUINO_ARCH_CH32)
// CH32V203 at 144 MHz, from the measured SWD timing: 56 + 5.06 cycles per
// delay unit per half SWCLK period (DAP_CONFIG_DEFAULT_CLOCK 50 -> ~240 kHz)
#define DAP_CONFIG_DELAY_CONSTANT 14218
#define DAP_CONFIG_FAST_CLOCK 1280000 // Hz
#else
// A value at which dap_clock_test() produces 1 kHz output on the SWCLK pin
#define DAP_CONFIG_DELAY_CONSTANT 4000

// A threshold for switching to fast clock (no added delays)
// This is the frequency produced by dap_clock_test(1) on the SWCLK pin
#define DAP_CONFIG_FAST_CLOCK 463000 // Hz
#endif

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
#if defined(ARDUINO_ARCH_CH32)
  if (NRESET_PORT) {
    if (value)
      NRESET_PORT->BSHR = NRESET_PINMASK; // open drain: released
    else
      NRESET_PORT->BCR = NRESET_PINMASK;
  }
#else
  digitalWrite(DAP_CONFIG_nRESET_PIN, value);
#endif
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWCLK_TCK_read(void) {
#if defined(ARDUINO_ARCH_SAMD)
  return (*SWCLK_INREG & SWCLK_PINMASK) != 0;
#elif defined(TEENSYDUINO)
  return (*SWCLK_PORT_INPUT_REG & SWCLK_BITMASK) ? 1 : 0;
#elif defined(ARDUINO_ARCH_CH32)
  return (SWCLK_PORT->INDR & SWCLK_PINMASK) ? 1 : 0;
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
#elif defined(ARDUINO_ARCH_CH32)
  return (SWDIO_PORT->INDR & SWDIO_PINMASK) ? 1 : 0;
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
#if defined(ARDUINO_ARCH_CH32)
  return NRESET_PORT ? ((NRESET_PORT->INDR & NRESET_PINMASK) ? 1 : 0) : 1;
#else
  return digitalRead(DAP_CONFIG_nRESET_PIN);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_set(void) {
#if defined(ARDUINO_ARCH_SAMD)
  *SWCLK_OUTSETREG = SWCLK_PINMASK;
#elif defined(TEENSYDUINO)
  *SWCLK_PORTSET = SWCLK_BITMASK;
#elif defined(ARDUINO_ARCH_CH32)
  SWCLK_PORT->BSHR = SWCLK_PINMASK;
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
#elif defined(ARDUINO_ARCH_CH32)
  SWCLK_PORT->BCR = SWCLK_PINMASK;
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
#elif defined(ARDUINO_ARCH_CH32)
  SWDIO_PORT->BSHR = SWDIO_PINMASK;
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
#elif defined(ARDUINO_ARCH_CH32)
  SWDIO_PORT->BCR = SWDIO_PINMASK;
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
#elif defined(ARDUINO_ARCH_CH32)
  SWDIO_PORT->BSHR = SWDIO_PINMASK; // pull-up
  *SWDIO_CFGREG = (*SWDIO_CFGREG & ~(0xFU << SWDIO_CFGSHIFT)) |
                  (DAP_CH32_CFG_IN_PULL << SWDIO_CFGSHIFT);
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
#elif defined(ARDUINO_ARCH_CH32)
  *SWDIO_CFGREG = (*SWDIO_CFGREG & ~(0xFU << SWDIO_CFGSHIFT)) |
                  (DAP_CH32_CFG_OUT_PP_2MHZ << SWDIO_CFGSHIFT);
#else
  pinMode(DAP_CONFIG_SWDIO_PIN, OUTPUT);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SETUP() {
#if defined(ARDUINO_ARCH_CH32)
  // Register level (the core's pinMode() selects 50 MHz output edges).
  // nRESET: open drain, released (the target's pull-up holds it high).
  DAP_CONFIG_CH32_mode(SWCLK_PORT, SWCLK_PINMASK, DAP_CH32_CFG_IN_FLOAT);
  DAP_CONFIG_CH32_mode(SWDIO_PORT, SWDIO_PINMASK, DAP_CH32_CFG_IN_FLOAT);
  if (NRESET_PORT) {
    NRESET_PORT->BSHR = NRESET_PINMASK;
    DAP_CONFIG_CH32_mode(NRESET_PORT, NRESET_PINMASK, DAP_CH32_CFG_OUT_OD);
  }
#else
  pinMode(DAP_CONFIG_SWCLK_PIN, INPUT);
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT_PULLUP);
  // pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  // pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
  pinMode(DAP_CONFIG_nRESET_PIN, INPUT);
  #ifdef DAP_CONFIG_HAS_LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  #endif
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_DISCONNECT(void) {
#if defined(ARDUINO_ARCH_CH32)
  DAP_CONFIG_CH32_mode(SWCLK_PORT, SWCLK_PINMASK, DAP_CH32_CFG_IN_FLOAT);
  DAP_CONFIG_CH32_mode(SWDIO_PORT, SWDIO_PINMASK, DAP_CH32_CFG_IN_FLOAT);
#else
  pinMode(DAP_CONFIG_SWCLK_PIN, INPUT);
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  // pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  // pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
  pinMode(DAP_CONFIG_nRESET_PIN, INPUT);
#endif
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_SWD(void) {
#if defined(ARDUINO_ARCH_CH32)
  // Drive SWCLK and SWDIO high with 2 MHz edges. nRESET stays released (no
  // reset pulse); Adafruit_DAP_STM32::select() resets the target over SWD.
  SWCLK_PORT->BSHR = SWCLK_PINMASK;
  SWDIO_PORT->BSHR = SWDIO_PINMASK;
  DAP_CONFIG_CH32_mode(SWCLK_PORT, SWCLK_PINMASK, DAP_CH32_CFG_OUT_PP_2MHZ);
  DAP_CONFIG_CH32_mode(SWDIO_PORT, SWDIO_PINMASK, DAP_CH32_CFG_OUT_PP_2MHZ);
#else
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
#endif
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
  #ifdef DAP_CONFIG_HAS_LED
    if (0 == index)
      digitalWrite(LED_BUILTIN, !state);
  #else
    (void)index;
    (void)state;
  #endif
}

#endif // _DAP_CONFIG_H_
