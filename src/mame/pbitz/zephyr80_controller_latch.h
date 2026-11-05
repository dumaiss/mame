// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_CONTROLLER_LATCH_H
#define MAME_PBITZ_ZEPHYR80_CONTROLLER_LATCH_H

#pragma once

#include <array>

// The two controller latches the IO controller loads (Code/MCU/IOController
// src/controller_latch.c): the byte each port presents to the Z80 on a read of
// E0h-FFh, active low, as the cascaded 74AHC595 pair holds it.  Plain C++: it
// builds outside MAME.
//
//   D7 low, D6 fire, D5:D4 high, D3:D0 left/down/right/up or a ColecoVision
//   keypad nibble.  An idle pad and an absent pad both read 7Fh; there is no
//   presence bit.
//
// The firmware's three writers are modelled as it has them:
//   f310_report: a Logitech F310 DirectInput report (controller_latch_f310_report);
//   keypad: the keyboard's number keys on a port (controller_latch_keypad),
//           which outranks the stick while held;
//   release: a pad leaving (controller_latch_release), which parks the port
//           at 7Fh and leaves any held keypad code in place.
// The SPI shift into the 595s is not modelled: a value is on the port as soon
// as it is set.
class zephyr80_controller_latch
{
public:
	static constexpr unsigned PORTS = 2;                // CONTROLLER_LATCH_PORTS
	static constexpr u8 IDLE = 0x7f;                    // CONTROLLER_LATCH_IDLE
	static constexpr u8 KEYPAD_NONE = 0xff;             // CONTROLLER_KEYPAD_NONE

	// The bits (controller_latch.c CTRL_*), active low.
	static constexpr u8 UP = 0x01, RIGHT = 0x02, DOWN = 0x04, LEFT = 0x08, FIRE = 0x40;
	static constexpr u8 KEYPAD_1 = 0x0d, KEYPAD_2 = 0x07, KEYPAD_STAR = 0x09, KEYPAD_POUND = 0x06;

	// controller_latch_init: both ports idle, no keypad key held.
	void reset()
	{
		for (unsigned port = 0; port < PORTS; port++)
		{
			m_value[port] = IDLE;
			m_keypad[port] = KEYPAD_NONE;
		}
	}

	// controller_latch_value: an invalid port reads as idle.
	u8 value(unsigned port) const { return port < PORTS ? m_value[port] : IDLE; }
	u8 keypad_code(unsigned port) const { return port < PORTS ? m_keypad[port] : KEYPAD_NONE; }

	void set(unsigned port, u8 value)
	{
		if (port < PORTS)
			m_value[port] = value;
	}

	void release(unsigned port) { set(port, IDLE); }

	// controller_latch_keypad: `code` is the active-low matrix nibble, or
	// KEYPAD_NONE.  Applied to whatever the port presents now, so a keyboard
	// works with no pad; releasing parks the direction nibble idle until a pad
	// report refreshes it.
	void keypad(unsigned port, u8 code)
	{
		if (port >= PORTS || m_keypad[port] == code)
			return;
		m_keypad[port] = code;
		u8 value = m_value[port] | 0x0f;
		if (code != KEYPAD_NONE)
			value = u8((value & 0xf0) | (code & 0x0f));
		set(port, value);
	}

	// controller_latch_f310_report: bytes 0-1 the left stick (80h neutral),
	// byte 4 the hat in D3:D0 and X A B Y in D4-D7, byte 5 LB RB LT RT Back
	// Start L3 R3.  False for a bad port or a report shorter than six bytes.
	bool f310_report(unsigned port, u8 const *report, unsigned length)
	{
		if (port >= PORTS || report == nullptr || length < 6)
			return false;
		u8 value = IDLE;

		// Back and Start (star, pound), X and Y (keypad 1 and 2) outrank the
		// hat and the stick, in that order.
		if (report[5] & 0x10)
		{
			value = u8((value & 0xf0) | KEYPAD_STAR);
		}
		else if (report[5] & 0x20)
		{
			value = u8((value & 0xf0) | KEYPAD_POUND);
		}
		else if (report[4] & 0x10)
		{
			value = u8((value & 0xf0) | KEYPAD_1);
		}
		else if (report[4] & 0x80)
		{
			value = u8((value & 0xf0) | KEYPAD_2);
		}
		else
		{
			switch (report[4] & 0x0f)
			{
			case 0: value &= u8(~UP); break;
			case 1: value &= u8(~(UP | RIGHT)); break;
			case 2: value &= u8(~RIGHT); break;
			case 3: value &= u8(~(RIGHT | DOWN)); break;
			case 4: value &= u8(~DOWN); break;
			case 5: value &= u8(~(DOWN | LEFT)); break;
			case 6: value &= u8(~LEFT); break;
			case 7: value &= u8(~(LEFT | UP)); break;
			default: // no hat direction: the left stick (Mode LED on), a wide dead zone
				if (report[0] < 0x40) value &= u8(~LEFT);
				if (report[0] > 0xc0) value &= u8(~RIGHT);
				if (report[1] < 0x40) value &= u8(~UP);
				if (report[1] > 0xc0) value &= u8(~DOWN);
				break;
			}
		}

		// A or B is the fire line, also during a keypad override (the second fire).
		if (report[4] & 0x60)
			value &= u8(~FIRE);

		// A keypad key held on the keyboard outranks the stick.
		if (m_keypad[port] != KEYPAD_NONE)
			value = u8((value & 0xf0) | (m_keypad[port] & 0x0f));

		set(port, value);
		return true;
	}

	template <typename Save> void save_state(Save &&save)
	{
		save(m_value, "latch.m_value");
		save(m_keypad, "latch.m_keypad");
	}

private:
	std::array<u8, PORTS> m_value{ IDLE, IDLE };
	std::array<u8, PORTS> m_keypad{ KEYPAD_NONE, KEYPAD_NONE };
};

#endif // MAME_PBITZ_ZEPHYR80_CONTROLLER_LATCH_H
