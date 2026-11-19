# CH32V4x7 RAM helper

The freestanding RV32IMAC helper implements page programming and reflected IEEE
CRC32 (polynomial `0xedb88320`, initial/final XOR `0xffffffff`). It uses no
runtime, heap, interrupts, or vendor SDK. The host resets/halts the target first.

Memory contract:
- `0x20000000`: helper entry/code, below `0x20001000`.
- `0x20001000..0x200010ff`: one 256-byte staging page.
- `0x20002000`: initial downward-growing stack pointer.
- a0: operation (0 = CRC, 1 = program page); a1: physical flash address;
  a2: byte count; a3: expected page CRC; a4: helper entry for the program-buffer call.
- Return a0: CRC or program status; a1: completion cookie `0x43524332`.
- Program status: 0 success, 1 invalid range/operation, 2 staging CRC mismatch,
  3 flash error/timeout, 4 programmed flash CRC mismatch.

The host checks every code byte before execution, verifies call arguments and
program-buffer instructions, uses `fence.i`, and checks completion before accepting
the result. SRAM staging has an exclusive end guard and checked final pointer.
It deliberately omits per-word data readback: code gets full readback and page
data gets target-side CRC **before** any flash write. No autoexec is enabled.
Flash operations use PAGE_PG, WR_BSY polling, then PG_STRT, with bounded waits.
The helper neither unlocks flash nor touches option bytes; the host owns unlock,
lock, and the exact requested image range. The public programming API retains
byte-for-byte verification by default; CRC-only programming is explicit.

Rebuild from the repository root with WCH GCC 15.0.0:

```sh
riscv32-wch-elf-gcc -Os -march=rv32imac -mabi=ilp32 -mno-relax \
  -msmall-data-limit=0 -ffreestanding -fno-builtin -nostdlib \
  -Wl,-T,extras/ch32_loader.ld -Wl,--no-relax \
  -o ch32_loader.elf extras/ch32_loader.c
riscv32-wch-elf-objcopy -O binary ch32_loader.elf ch32_loader.bin
```

`Adafruit_DAP_CH32_loader.h` contains that binary as little-endian uint32_t words,
zero-padded to a multiple of four bytes. Keep generated ELF/bin files outside Git.
Assemble `extras/ch32_swio_programs.S` with `-march=rv32imac -mabi=ilp32
-mno-relax`; its reader and RAM-writer sections must match the header arrays.
