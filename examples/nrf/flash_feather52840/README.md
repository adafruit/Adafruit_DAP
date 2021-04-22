## Preparing the .bin file

To prepare the .bin file, build the bootloader firmare (see
https://github.com/adafruit/Adafruit_nRF52_Bootloader) with the following
command:

```
$ make BOARD=feather_nrf52840_express clean all combinehex
```

Then go to `_build-feather_nrf52840_express` and edit the
`feather_nrf52840_express_bootloader_s140_6.1.1r0.hex` file to remove the
following two lines at the BOTTOM of the .hex file.

> These two lines must be removed since they contain two words that need to
  be written to the UICR region at address 0x1000000. Leaving these in will
  cause a 256MB .bin file to be generated, so we remove them and manually
  write these two values in the `flahse_feather25840.ino` sketch.

```
:020000041000EA
:0810140000400F0000E00F0096
```

You can now convert the trimmed .hex file to a .bin file as follows:

```
$ arm-none-eabi-objcopy -I ihex --output-target=binary feather_nrf52840_express_bootloader_s140_6.1.1r0.hex 50_BOOT.bin
```

Copy this file to the SD card and this folder for archiving purposes, and run
the sketch in this folder.
