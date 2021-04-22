#ifndef _MKR_BOARDS_H
#define _MKR_BOARDS_H

typedef struct {
	const char *choice;
	const char *name;
	const uint8_t* bootloader;
	const uint32_t size;
} board_t;


// Here are all Arduino MKR bootloaders from the Arduino SAMD boards version 1.8.9 package.

static const uint8_t circuitplay_m0_samd21g18_sam_ba[] = {
#include "circuitplay_m0_samd21g18_sam_ba.h";
};

static const uint8_t samd21_sam_ba_zero[] = {
#include "samd21_sam_ba.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrzero[] = {
#include "samd21_sam_ba_arduino_mkrzero.h";
};

static const uint8_t samd21_sam_ba_arduino_mkr1000[] = {
#include "samd21_sam_ba_arduino_mkr1000.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrfox1200[] = {
#include "samd21_sam_ba_arduino_mkrfox1200.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrgsm1400[] = {
#include "samd21_sam_ba_arduino_mkrgsm1400.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrnb1500[] = {
#include "samd21_sam_ba_arduino_mkrnb1500.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrvidor4000[] = {
#include "samd21_sam_ba_arduino_mkrvidor4000.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrwan1300[] = {
#include "samd21_sam_ba_arduino_mkrwan1300.h";
};

static const uint8_t samd21_sam_ba_arduino_mkrwan1310[] = {
#include "samd21_sam_ba_arduino_mkrwan1310.h";
};
static const uint8_t samd21_sam_ba_arduino_mkrwifi1010[] = {
#include "samd21_sam_ba_arduino_mkrwifi1010.h";
};
static const uint8_t samd21_sam_ba_arduino_nano_33_iot[] = {
#include "samd21_sam_ba_arduino_nano_33_iot.h";
};

static const board_t bootloaders[] = {

	{ "Z", "    -> Arduino Zero", samd21_sam_ba_zero, sizeof(samd21_sam_ba_zero)},
	{ "MZ", "   -> Arduino MKR Zero", samd21_sam_ba_arduino_mkrzero, sizeof(samd21_sam_ba_arduino_mkrzero)},
	{ "1000", " -> Arduino MKR 1000 WIFI", samd21_sam_ba_arduino_mkr1000, sizeof(samd21_sam_ba_arduino_mkr1000)},
	{ "1010", " -> Arduino MKR WIFI 1010", samd21_sam_ba_arduino_mkrwifi1010, sizeof(samd21_sam_ba_arduino_mkrwifi1010)},
	{ "1200", " -> Arduino MKR Fox 1200", samd21_sam_ba_arduino_mkrfox1200, sizeof(samd21_sam_ba_arduino_mkrfox1200)},
	{ "1400", " -> Arduino MKR GSM 1400", samd21_sam_ba_arduino_mkrgsm1400, sizeof(samd21_sam_ba_arduino_mkrgsm1400)},
	{ "1500", " -> Arduino MKR NB 1500", samd21_sam_ba_arduino_mkrnb1500, sizeof(samd21_sam_ba_arduino_mkrnb1500)},
	{ "1300", " -> Arduino MKR WAN 1300", samd21_sam_ba_arduino_mkrwan1300, sizeof(samd21_sam_ba_arduino_mkrwan1300)},
	{ "1310", " -> Arduino MKR WAN 1310", samd21_sam_ba_arduino_mkrwan1310, sizeof(samd21_sam_ba_arduino_mkrwan1310)},
	{ "4000", " -> Arduino MKR Vidor 4000", samd21_sam_ba_arduino_mkrvidor4000, sizeof(samd21_sam_ba_arduino_mkrvidor4000)},
	{ "33", "   -> Arduino Nano 33 IOT", samd21_sam_ba_arduino_nano_33_iot, sizeof(samd21_sam_ba_arduino_nano_33_iot)},
	{ "M0", "   -> Adafruit Circuit Playground M0", circuitplay_m0_samd21g18_sam_ba, sizeof(circuitplay_m0_samd21g18_sam_ba)},
	{ NULL, NULL, NULL, 0},
};

#endif
