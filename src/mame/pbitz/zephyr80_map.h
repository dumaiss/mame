// license:BSD-3-Clause
// copyright-holders:Kitamura

#ifndef MAME_PBITZ_ZEPHYR80_MAP_H
#define MAME_PBITZ_ZEPHYR80_MAP_H

#pragma once

#include "osdcomm.h"


namespace zephyr80_map {

// I/O regions derived from IO_DECODER.pld.  The PLD decodes A7-A4 into
// 16-byte blocks.
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

// Z80CTC in the $40-$4f block (A3:A2 mirrored).  IO board nets
// `/IO Controller Card/A1` -> CTC CS0 (pin 18) and `/IO Controller Card/A0`
// -> CTC CS1 (pin 19), so the channel is A0*2 + A1: $40 ch0, $41 ch2, $42 ch1,
// $43 ch3.
static constexpr u8 CTC_REGISTER_MASK = 0x03;
static constexpr u8 CTC_REGISTER_MIRROR = 0x0c;

constexpr u8 ctc_channel(u8 offset)
{
	return ((offset & 0x01) << 1) | ((offset >> 1) & 0x01);
}

// V9958 ports select on A1:A0 within the $a0-$bf block (A2-A4 mirrored).
static constexpr u8 VDP_REGISTER_MASK = 0x03;
static constexpr u8 VDP_REGISTER_MIRROR = 0x1c;
} // namespace io

// MEM_DECODER.pld Rev 13: Q=D2:D0, mode=D4:D3, P=D7:D5.
namespace mem {
static constexpr u32 RAM_BANK_SIZE = 0x10000;
static constexpr u32 RAM_BANKS = 8;
static constexpr u32 RAM_SIZE = RAM_BANK_SIZE * RAM_BANKS;
static constexpr u32 ROM_SIZE = 0x80000;
static constexpr u16 COMMON_START = 0xe000;
static constexpr u16 WINDOW_START = 0x6000;
static constexpr u16 APERTURE_START = 0x8000;
static constexpr u16 APERTURE_END = 0xc000;
} // namespace mem

namespace latch {
static constexpr u8 BANK_MASK = 0x07;
static constexpr u8 MODE_MASK = 0x18;
static constexpr u8 MODE_ROM = 0x00;         // binary mode 00
static constexpr u8 MODE_FLAT = 0x08;        // binary mode 01
static constexpr u8 MODE_APPLICATION = 0x10; // binary mode 10
static constexpr u8 MODE_OS = 0x18;          // binary mode 11
static constexpr u8 PAGE_SHIFT = 5;
} // namespace latch

// Shared mapping facts for CPU handlers and debugger clients.  Physical A0-A15
// always equal the CPU address; the decoder only supplies chip/bank selects.
// Reads select flash only in mode 00; every memory write selects SRAM.
struct mapping
{
	bool read_flash;
	u32 read_address;
	u32 write_address;
};

constexpr mapping decode(u16 address, u8 value)
{
	u8 const mode = value & latch::MODE_MASK;
	u8 const page = value >> latch::PAGE_SHIFT;
	u8 bank = value & latch::BANK_MASK;
	if (mode == latch::MODE_APPLICATION || mode == latch::MODE_OS)
	{
		if (address >= mem::COMMON_START)
			bank = 0;
		else if (mode == latch::MODE_OS && address >= 0x2000)
			bank = address < mem::WINDOW_START ? 7 : 7 - page;
		else if (mode == latch::MODE_APPLICATION && page && address >= mem::APERTURE_START && address < mem::APERTURE_END)
			bank = 7 - page;
	}
	u32 const ram_address = (u32(bank) << 16) | address;
	bool const flash = mode == latch::MODE_ROM;
	return { flash, flash ? (u32(page) << 16) | address : ram_address, ram_address };
}

static_assert(mem::RAM_SIZE == 512 * 1024 && mem::ROM_SIZE == 512 * 1024);
static_assert(io::ctc_channel(0) == 0 && io::ctc_channel(1) == 2 && io::ctc_channel(2) == 1 && io::ctc_channel(3) == 3);

} // namespace zephyr80_map

#endif // MAME_PBITZ_ZEPHYR80_MAP_H
