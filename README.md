# Adafruit_DAP
Port of Free-DAP + edbg CMSIS-DAP controller software to Arduino

This is a library that allows you to program various Cortex M chips that support programming via DAP from an arduino-compatible board. Tested with a Metro M0 / Arduino Zero as the 'host' and with ATSAMDxx M0 (SAMD21, SAMD09, SAMD10 etc), M4 ATSAMD51, nRF51, nRF52840, STM32F405 as the client but in theory any 3.3V Arduino board will work as host (just slower)

Works completely stand-alone, firmware is read from an SD card and uploaded directly from chip-to-chip. There are no computers required!

This is mostly a re-mix of Alex Taradovs excellent Free-DAP code @ https://github.com/ataradov/free-dap - we just put both halves into one library and wrapped it up in Arduino compatibility

The WCH CH32V203 also works as host, with the WCH Arduino core: SWD on A3 (SWDIO) / A2 (SWCLK), nRESET on A0, see `examples/stm32/stm32f4_test`. It uses register-level pin access with slow (2 MHz) output edges and 2 idle cycles per SWD transfer.

Target drivers support the ATSAMD, nRF, STM and CH32V407/CH32V467 series. Contributions for other targets are welcome.

Other ARM chips should be (easily?) supportable. Copy Adafruit_DAP_SAM to a new file and make changes to support your favorite SWD-programmable chip.

Matching guide with some more information here https://learn.adafruit.com/programming-an-m0-using-an-arduino

## CH32V407/CH32V467 over single-wire debug

`Adafruit_DAP_CH32` programs the 512 KiB CH32V407RET and CH32V467RET from an
RP2040 running Arduino-Pico. It uses WCH SWIO, which is different from ARM SWD;
SWCLK is unused. The transport runs from RAM and requires Arduino-Pico's normal
CPU-clock SysTick timer (profiling/FreeRTOS timer configurations are rejected).

The common programming sequence is `begin(swio, reset)`, `targetConnect()`,
`select(&id)`, `program_start(address, length)`, `programFlash(...)`, and
`deselect()`. Check every returned status; `program_start()` returns `UINT32_MAX`
on failure. `error_message` describes a failure. Only the common programming API
is implemented for CH32; the base class's ARM CMSIS-DAP register commands do not
apply. Connection resets and halts the target; deselection resets and releases it.

Flash starts at `0x08000000`; zero-based addresses are also accepted. Images must
start on a 4 KiB sector boundary. `program_start()` erases the entire final sector,
including any bytes beyond the image. Write consecutive 256-byte pages, with a
partial final page padded to `0xff`. A failed write invalidates the prepared
range; start a new programming operation before retrying. Verification compares
all bytes, including padding. Option-byte and protection changes are unsupported.

The [embedded-image example](examples/ch32/flash_from_header) installs the saved
Metro CH32V407 Rev A TinyUF2 image from Pico flash. The
[microSD example](examples/ch32/flash_from_SD) reads a raw `METRO407.BIN` file and
checks its size and CRC before erasing. Both start without a Serial Monitor and
leave the onboard Pico LED on after success. They use PicoBrains GP2 for SWIO,
GP27 for RESET, and GP17 for SD CS. Connect a common ground and power the target
separately; SWIO is a 3.3 V signal.

Select **Raspberry Pi Pico**, with **Adafruit TinyUSB** as the USB stack. Each
example programs once at startup; reset the Pico to program another target.
For the SD example, create the matching raw binary from the included header
with Python, run from this repository's root, then copy it to a FAT-formatted
card:

```python
from pathlib import Path
import re

header = Path("examples/ch32/flash_from_header/tinyuf2_metro407.h").read_text()
array = header.split("firmware[] = {", 1)[1].split("};", 1)[0]
image = bytes(int(value, 16) for value in re.findall(r"0x([0-9a-fA-F]{2})", array))
with open("METRO407.BIN", "xb") as output:
    output.write(image)
```

The included image requires a Metro CH32V407RET6 Rev A with a 25 MHz crystal.
For another image or the CH32V467, replace the image, expected chip ID, length,
CRC, and filename together. Use the binary's intended load address: a TinyUF2
application linked at `0x8000` must not be programmed at zero. See the example's
image notice for provenance and licenses; the programmer's SWIO transport is
derived from MIT-licensed Open Embedded Probe v0.0.58 (`license-oep.txt`).
