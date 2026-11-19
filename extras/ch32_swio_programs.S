# SPDX-License-Identifier: BSD-3-Clause AND MIT
# Copyright (c) 2026 Adafruit Industries
# Reader derived from Open Embedded Probe, Copyright (c) 2026 Open Embedded Probe.
# See license-oep.txt.
# Program-buffer routines used by Adafruit_DAP_CH32 (RV32IC).
# Assemble with: riscv32-wch-elf-as -march=rv32imac -mabi=ilp32 -mno-relax
# a0 = debug DATA0 mailbox, a1 = DATA1/address mailbox.
# Flash writer: a2 = FLASH_STATR, a3 = exclusive page end.
# Based on Open Embedded Probe's MIT-licensed debug-mailbox block reader.
.option rvc
.option norelax
.section .text.reader,"ax"
.global reader
reader:
  c.lw s0, 0(a1)
  c.lw s1, 0(s0)
  c.addi s0, 4
  c.sw s1, 0(a0)
  c.sw s0, 0(a1)
  c.ebreak

.section .text.flash_writer,"ax"
.global flash_writer
flash_writer:
  c.lw s0, 0(a1)
  bgeu s0, a3, done
  c.lw s1, 0(a0)
  c.sw s1, 0(s0)
  lui a4, 1
poll:
  c.lw s1, 0(a2)
  c.andi s1, 2
  c.beqz s1, finished
  c.addi a4, -1
  c.bnez a4, poll
  c.ebreak
finished:
  c.addi s0, 4
  c.sw s0, 0(a1)
done:
  c.ebreak
  c.nop
