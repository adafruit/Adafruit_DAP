/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>, Adafruit <info@adafruit.com>
 * All rights reserved.
 *
 * This is mostly a re-mix of Alex Taradovs excellent Free-DAP code - we just put both halves into one library and wrapped it up in Arduino compatibility
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
#include "sam.h"
#include "Arduino.h"

#define DAP_CONFIG_ENABLE_SWD
//#define DAP_CONFIG_ENABLE_JTAG

#define DAP_CONFIG_DEFAULT_PORT        DAP_PORT_SWD
#define DAP_CONFIG_DEFAULT_CLOCK       1000000 // Hz

#define DAP_CONFIG_PACKET_SIZE         64
#define DAP_CONFIG_PACKET_COUNT        1

//#define DAP_CONFIG_SWCLK_PIN		9
//#define DAP_CONFIG_SWDIO_PIN		10
//#define DAP_CONFIG_nRESET_PIN		11

extern int DAP_CONFIG_SWCLK_PIN;
extern int DAP_CONFIG_SWDIO_PIN;
extern int DAP_CONFIG_nRESET_PIN;

#define DAP_CONFIG_SWCLK			   g_APinDescription[DAP_CONFIG_SWCLK_PIN]
#define DAP_CONFIG_SWDIO			   g_APinDescription[DAP_CONFIG_SWDIO_PIN]
#define DAP_CONFIG_nRESET			   g_APinDescription[DAP_CONFIG_nRESET_PIN]

// Set the value to NULL if you want to disable a string
// DAP_CONFIG_PRODUCT_STR must contain "CMSIS-DAP" to be compatible with the standard
#define DAP_CONFIG_VENDOR_STR          "Adafruit"
#define DAP_CONFIG_PRODUCT_STR         "Generic CMSIS-DAP Adapter"
#define DAP_CONFIG_SER_NUM_STR         "123456"
#define DAP_CONFIG_FW_VER_STR          "v0.1"
#define DAP_CONFIG_DEVICE_VENDOR_STR   NULL
#define DAP_CONFIG_DEVICE_NAME_STR     NULL

//#define DAP_CONFIG_RESET_TARGET_FN     target_specific_reset_function

// A value at which dap_clock_test() produces 1 kHz output on the SWCLK pin
#define DAP_CONFIG_DELAY_CONSTANT      4000

// A threshold for switching to fast clock (no added delays)
// This is the frequency produced by dap_clock_test(1) on the SWCLK pin 
#define DAP_CONFIG_FAST_CLOCK          463000 // Hz

static inline void DAP_gpio_write(int port, int pin, int val) {
	if(val) PORT->Group[port].OUTSET.reg = (1ul<<pin);
	else PORT->Group[port].OUTCLR.reg = (1ul<<pin);
}

static inline uint32_t DAP_gpio_read_bulk(int port){
	return PORT->Group[port].IN.reg;
}

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_write(int value)
{
	DAP_gpio_write(DAP_CONFIG_SWCLK.ulPort, DAP_CONFIG_SWCLK.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_write(int value)
{
	DAP_gpio_write(DAP_CONFIG_SWDIO.ulPort, DAP_CONFIG_SWDIO.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_TDO_write(int value)
{
	//DAP_gpio_write(DAP_CONFIG_TDO.ulPort, DAP_CONFIG_TDO.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_nTRST_write(int value)
{
  //DAP_gpio_write(DAP_CONFIG_nTRST.ulPort, DAP_CONFIG_nTRST.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_nRESET_write(int value)
{
  DAP_gpio_write(DAP_CONFIG_nRESET.ulPort, DAP_CONFIG_nRESET.ulPin, value);
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWCLK_TCK_read(void)
{
  return (DAP_gpio_read_bulk(DAP_CONFIG_SWCLK.ulPort) & (1 << DAP_CONFIG_SWCLK.ulPin)) > 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_SWDIO_TMS_read(void)
{
  return (DAP_gpio_read_bulk(DAP_CONFIG_SWDIO.ulPort) & (1 << DAP_CONFIG_SWDIO.ulPin)) > 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_TDI_read(void)
{
  //return (DAP_gpio_read_bulk(DAP_CONFIG_TDI.ulPort) & (1 << DAP_CONFIG_TDI.ulPin)) > 0;
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_TDO_read(void)
{
  //return (DAP_gpio_read_bulk(DAP_CONFIG_TD0.ulPort) & (1 << DAP_CONFIG_TD0.ulPin)) > 0;
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_nTRST_read(void)
{
  //return (DAP_gpio_read_bulk(DAP_CONFIG_nTRST.ulPort) & (1 << DAP_CONFIG_nTRST.ulPin)) > 0;
  return 0;
}

//-----------------------------------------------------------------------------
static inline int DAP_CONFIG_nRESET_read(void)
{
  return (DAP_gpio_read_bulk(DAP_CONFIG_nRESET.ulPort) & (1 << DAP_CONFIG_nRESET.ulPin)) > 0;
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_set(void)
{
  PORT->Group[DAP_CONFIG_SWCLK.ulPort].OUTSET.reg = (1ul<<DAP_CONFIG_SWCLK.ulPin);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWCLK_TCK_clr(void)
{
  PORT->Group[DAP_CONFIG_SWCLK.ulPort].OUTCLR.reg = (1ul<<DAP_CONFIG_SWCLK.ulPin);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_in(void)
{
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SWDIO_TMS_out(void)
{
  pinMode(DAP_CONFIG_SWDIO_PIN, OUTPUT);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_SETUP()
{
  pinMode(DAP_CONFIG_SWCLK_PIN, INPUT);
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT_PULLUP);
  //pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  //pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  //pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
  pinMode(DAP_CONFIG_nRESET_PIN, INPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_DISCONNECT(void)
{
  pinMode(DAP_CONFIG_SWCLK_PIN, INPUT);
  pinMode(DAP_CONFIG_SWDIO_PIN, INPUT);
  //pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  //pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  //pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
  pinMode(DAP_CONFIG_nRESET_PIN, INPUT);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_SWD(void)
{
  pinMode(DAP_CONFIG_SWDIO_PIN, OUTPUT);
  digitalWrite(DAP_CONFIG_SWDIO_PIN, HIGH);

  pinMode(DAP_CONFIG_SWCLK_PIN, OUTPUT);
  digitalWrite(DAP_CONFIG_SWCLK_PIN, HIGH);

  pinMode(DAP_CONFIG_nRESET_PIN, OUTPUT);
  digitalWrite(DAP_CONFIG_nRESET_PIN, HIGH);

  //pinMode(DAP_CONFIG_TDI_PIN, INPUT);
  //pinMode(DAP_CONFIG_TDO_PIN, INPUT);
  //pinMode(DAP_CONFIG_nTRST_PIN, INPUT);
}

//-----------------------------------------------------------------------------
static inline void DAP_CONFIG_CONNECT_JTAG(void)
{
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
static inline void DAP_CONFIG_LED(int index, int state)
{
  if (0 == index)
	digitalWrite(LED_BUILTIN, !state);
}

#endif // _DAP_CONFIG_H_

