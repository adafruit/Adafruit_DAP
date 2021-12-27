# Adafruit_DAP
Port of Free-DAP + edbg CMSIS-DAP controller software to Arduino

This is a library that allows you to program various Cortex M chips that support programming via DAP from an arduino-compatible board. Tested with a Metro M0 / Arduino Zero as the 'host' and with ATSAMDxx M0 (SAMD21, SAMD09, SAMD10 etc), M4 ATSAMD51, nRF51, nRF52840, STM32F405 as the client but in theory any 3.3V Arduino board will work as host (just slower)

Works completely stand-alone, firmware is read from an SD card and uploaded directly from chip-to-chip. There are no computers required!

This is mostly a re-mix of Alex Taradovs excellent Free-DAP code @ https://github.com/ataradov/free-dap - we just put both halves into one library and wrapped it up in Arduino compatibility

Right now we only have commands for the ATSAMD, nRF, STM series. We'd be really into having other people add the SWD command magic for other chips, so if you do adapt this code for your favorite Cortex, please submit a pull request!

Other ARM chips should be (easily?) supportable. Copy Adafruit_DAP_SAM to a new file and make changes to support your favorite SWD-programmable chip.

Matching guide with some more information here https://learn.adafruit.com/programming-an-m0-using-an-arduino
