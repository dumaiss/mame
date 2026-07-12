// license:BSD-3-Clause
// copyright-holders:Kitamura

#ifndef MAME_PBITZ_ZEPHYR80_MAP_H
#define MAME_PBITZ_ZEPHYR80_MAP_H

#pragma once


namespace zephyr80_map {

// I/O regions derived from IO_DECODER.pld.  The PLD decodes A7-A4 into
// 16-byte blocks; Phase 8 implements only MEMBANK, SIO0, and SIO1.
namespace io {
static constexpr u8 MEMBANK_START = 0x00;
static constexpr u8 MEMBANK_END = 0x0f;
static constexpr u8 RESERVED_10_START = 0x10;
static constexpr u8 RESERVED_10_END = 0x1f;
static constexpr u8 SIO0_START = 0x20;
static constexpr u8 SIO0_END = 0x2f;
static constexpr u8 SIO1_START = 0x30;
static constexpr u8 SIO1_END = 0x3f;
static constexpr u8 CTC_START = 0x40;
static constexpr u8 CTC_END = 0x4f;
static constexpr u8 RESERVED_50_START = 0x50;
static constexpr u8 RESERVED_50_END = 0x5f;
static constexpr u8 CART_IO_START = 0x60;
static constexpr u8 CART_IO_END = 0x6f;
static constexpr u8 RESERVED_70_START = 0x70;
static constexpr u8 RESERVED_70_END = 0x9f;
static constexpr u8 VDP_START = 0xa0;
static constexpr u8 VDP_END = 0xbf;
static constexpr u8 RESERVED_C0_START = 0xc0;
static constexpr u8 RESERVED_C0_END = 0xdf;
static constexpr u8 SOUND_CTRL_START = 0xe0;
static constexpr u8 SOUND_CTRL_END = 0xff;

// Z80SIO register order matches the firmware (platform_zephyr80.inc), which is
// MAME's ba_cd accessors: A1 = channel (B/A), A0 = control/data, giving
// A data, A control, B data, B control in each 4-byte window.
static constexpr u8 SIO_REGISTER_MASK = 0x03;
static constexpr u8 SIO_REGISTER_MIRROR = 0x0c;
static constexpr u8 SIO_CHAN_A_DATA = 0x00;
static constexpr u8 SIO_CHAN_A_CONTROL = 0x01;
static constexpr u8 SIO_CHAN_B_DATA = 0x02;
static constexpr u8 SIO_CHAN_B_CONTROL = 0x03;

// Z80CTC channels select on A1:A0 within the $40-$4f block.
static constexpr u8 CTC_REGISTER_MASK = 0x03;
static constexpr u8 CTC_REGISTER_MIRROR = 0x0c;

// V9958 ports select on A1:A0 within the $a0-$bf block (A2-A4 mirrored).
static constexpr u8 VDP_REGISTER_MASK = 0x03;
static constexpr u8 VDP_REGISTER_MIRROR = 0x1c;
} // namespace io

// Memory regions derived from MEM_DECODER.pld (Rev 09).
//   BIOS_RANGE = !A15 & (!A14 # !A13)   -> $0000-$5FFF
//   RAM_ONLY   = !A15 &  A14 &  A13     -> $6000-$7FFF
//   UPPER_32K  =  A15                   -> $8000-$FFFF
//   SAFE_RAM   =  A15 &  A14            -> $C000-$FFFF (high 16K common area)
//   BOOT_ROM   = BIOS_RANGE # SAFE_RAM  -> ROM-readable windows in normal mode
namespace mem {
static constexpr offs_t BIOS_RANGE_START = 0x0000;
static constexpr offs_t BIOS_RANGE_END = 0x5fff;
static constexpr offs_t RAM_ONLY_START = 0x6000;
static constexpr offs_t RAM_ONLY_END = 0x7fff;
static constexpr offs_t UPPER_32K_START = 0x8000;
static constexpr offs_t UPPER_32K_END = 0xffff;
static constexpr offs_t SAFE_RAM_START = 0xc000;
static constexpr offs_t SAFE_RAM_END = 0xffff;

static constexpr u32 RAM_BANK_SIZE = 0x10000;
static constexpr u32 RAM_BANKS = 8;
static constexpr u32 RAM_SIZE = RAM_BANK_SIZE * RAM_BANKS;

// ROM is paged by latch D5-D7 (A16-A18); A0-A15 come from the CPU.  Eight
// 64 KiB pages span the 512 KiB ROM address space; the current firmware
// populates pages 0-1 (128 KiB) and higher pages read as erased (0xff).
static constexpr u32 ROM_PAGE_SIZE = 0x10000;
static constexpr u32 ROM_PAGES = 8;
static constexpr u32 ROM_SIZE = ROM_PAGE_SIZE * ROM_PAGES;
} // namespace mem

// Memory banking latch at I/O port $00 (74HC273), bit assignment per the
// firmware's shadow/copy routine (HOST/HelloWorld shadow_copy_low.inc):
//   D0-D2 = SRAM bank number 0-7   (BANK_Q0..2 -> RAM A16-A18)
//   D3    = shadow/copy mode       (RAM_SHADOW)
//   D4    = ROM disable            (ROM_DIS)
//   D5-D7 = ROM page number 0-7    (ROM A16-A18)
namespace latch {
static constexpr u8 BANK_Q0 = 0x01;
static constexpr u8 BANK_Q1 = 0x02;
static constexpr u8 BANK_Q2 = 0x04;
static constexpr u8 BANK_MASK = BANK_Q0 | BANK_Q1 | BANK_Q2;
static constexpr u8 RAM_SHADOW = 0x08;
static constexpr u8 ROM_DIS = 0x10;
static constexpr u8 ROM_PAGE_MASK = 0xe0;
static constexpr u8 ROM_PAGE_SHIFT = 5;
static constexpr u8 WRITABLE_MASK = 0xff;
} // namespace latch

} // namespace zephyr80_map

#endif // MAME_PBITZ_ZEPHYR80_MAP_H
