// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Adafruit Industries
// Freestanding CH32V4x7 RAM loader. See ch32_loader.md for the build command.
#include <stdint.h>
#define FLASH_BASE 0x08000000u
#define FLASH_END 0x08080000u
#define BUFFER ((volatile uint32_t *)0x20001000u)
#define STATUS (*(volatile uint32_t *)0x4002200cu)
#define CONTROL (*(volatile uint32_t *)0x40022010u)
#define DONE 0x43524332u
#define POLLS 1000000u
static uint32_t crc32(volatile const uint8_t *data, uint32_t size) {
  uint32_t crc = ~0u;
  while (size--) {
    crc ^= *data++;
    for (unsigned bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1)));
  }
  return ~crc;
}
static int ready(uint32_t mask) {
  for (uint32_t polls = POLLS; polls; polls--)
    if (!(STATUS & mask))
      return !(STATUS & 0x10);
  return 0;
}
// a0=operation (0=CRC, 1=program), a1=flash address, a2=size, a3=source CRC.
// Returns a0=result (CRC or program status), a1=completion cookie.
// Host verifies this entire code before execution. No runtime or global data.
__attribute__((section(".text.entry")))
uint64_t loader(uint32_t operation, uint32_t address, uint32_t size,
                uint32_t expected) {
  uint32_t result = 1;
  if (address < FLASH_BASE || address >= FLASH_END || !size ||
      size > FLASH_END - address)
    return ((uint64_t)DONE << 32) | result;
  if (operation == 0)
    return ((uint64_t)DONE << 32) |
           crc32((volatile const uint8_t *)address, size);
  if (operation != 1 || size != 256 || address % 256)
    return ((uint64_t)DONE << 32) | result;
  // Validate received SRAM data before enabling flash writes.
  if (crc32((volatile const uint8_t *)BUFFER, size) != expected)
    return ((uint64_t)DONE << 32) | 2;
  uint32_t settings = CONTROL & 0x07000000;
  if (!ready(3))
    return ((uint64_t)DONE << 32) | 3;
  CONTROL = settings | 0x10000; // PAGE_PG
  if (!ready(3))
    goto failed;
  for (uint32_t word = 0; word < 64; word++) {
    ((volatile uint32_t *)address)[word] = BUFFER[word];
    if (!ready(2)) // WR_BSY only: BSY can remain set until PG_STRT.
      goto failed;
  }
  CONTROL = settings | 0x210000; // PAGE_PG | PG_STRT
  if (!ready(3))
    goto failed;
  CONTROL = settings;
  result = 4;
  if (crc32((volatile const uint8_t *)address, size) == expected)
    result = 0;
  return ((uint64_t)DONE << 32) | result;
failed:
  CONTROL = settings;
  return ((uint64_t)DONE << 32) | 3;
}
