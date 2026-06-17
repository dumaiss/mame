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

// Phase 9 Z80SIO register order uses MAME's cd_ba accessors:
// A0 = B/A and A1 = C/D, giving DA, DB, CA, CB in each 4-byte window.
static constexpr u8 SIO_REGISTER_MASK = 0x03;
static constexpr u8 SIO_REGISTER_MIRROR = 0x0c;
static constexpr u8 SIO_CHAN_A_DATA = 0x00;
static constexpr u8 SIO_CHAN_B_DATA = 0x01;
static constexpr u8 SIO_CHAN_A_CONTROL = 0x02;
static constexpr u8 SIO_CHAN_B_CONTROL = 0x03;
} // namespace io

// Memory regions derived from MEM_DECODER.pld.
namespace mem {
static constexpr offs_t COMMON_RAM_START = 0x0000;
static constexpr offs_t COMMON_RAM_END = 0x1fff;
static constexpr offs_t BIOS_RANGE_START = 0x0000;
static constexpr offs_t BIOS_RANGE_END = 0x5fff;
static constexpr offs_t RAM_ONLY_START = 0x6000;
static constexpr offs_t RAM_ONLY_END = 0x7fff;
static constexpr offs_t UPPER_32K_START = 0x8000;
static constexpr offs_t UPPER_32K_END = 0xffff;

static constexpr u32 RAM_BANK_SIZE = 0x10000;
static constexpr u32 RAM_BANKS = 8;
static constexpr u32 RAM_SIZE = RAM_BANK_SIZE * RAM_BANKS;
} // namespace mem

// Provisional software-visible memory latch layout for Phase 8.  The PLD only
// consumes BANK_Q0..2, RAM_SHADOW, and ROM_DIS; no authoritative latch bit
// assignment was present in the decoder equations.
namespace latch {
static constexpr u8 BANK_Q0 = 0x01;
static constexpr u8 BANK_Q1 = 0x02;
static constexpr u8 BANK_Q2 = 0x04;
static constexpr u8 RAM_SHADOW = 0x08;
static constexpr u8 ROM_DIS = 0x10;
static constexpr u8 RESERVED = 0xe0;
static constexpr u8 WRITABLE_MASK = BANK_Q0 | BANK_Q1 | BANK_Q2 | RAM_SHADOW | ROM_DIS;
} // namespace latch

} // namespace zephyr80_map

#endif // MAME_PBITZ_ZEPHYR80_MAP_H
