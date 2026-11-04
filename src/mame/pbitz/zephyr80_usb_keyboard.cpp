// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_usb_keyboard.h"

#include <algorithm>

DEFINE_DEVICE_TYPE(ZEPHYR80_USB_KEYBOARD, zephyr80_usb_keyboard_device, "zephyr80_usb_keyboard", "Zephyr-80 USB keyboard")

// Port KEYSn bit b is usage n * 16 + b; MODIFIERS bit b is usage E0h + b,
// which is the boot report's modifier bit b.  Names follow the HID usage
// tables (US legends).
#define USB_KEY(bit, name, code) \
	PORT_BIT(1U << (bit), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) PORT_CODE(code)

static INPUT_PORTS_START(zephyr80_usb_keyboard)
	PORT_START("KEYS0") // 00h-0Fh
	USB_KEY(0x04 & 15, "A", KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A')
	USB_KEY(0x05 & 15, "B", KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B')
	USB_KEY(0x06 & 15, "C", KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
	USB_KEY(0x07 & 15, "D", KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D')
	USB_KEY(0x08 & 15, "E", KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E')
	USB_KEY(0x09 & 15, "F", KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F')
	USB_KEY(0x0a & 15, "G", KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G')
	USB_KEY(0x0b & 15, "H", KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H')
	USB_KEY(0x0c & 15, "I", KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I')
	USB_KEY(0x0d & 15, "J", KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')
	USB_KEY(0x0e & 15, "K", KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K')
	USB_KEY(0x0f & 15, "L", KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')

	PORT_START("KEYS1") // 10h-1Fh
	USB_KEY(0x10 & 15, "M", KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M')
	USB_KEY(0x11 & 15, "N", KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N')
	USB_KEY(0x12 & 15, "O", KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O')
	USB_KEY(0x13 & 15, "P", KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P')
	USB_KEY(0x14 & 15, "Q", KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q')
	USB_KEY(0x15 & 15, "R", KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')
	USB_KEY(0x16 & 15, "S", KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S')
	USB_KEY(0x17 & 15, "T", KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T')
	USB_KEY(0x18 & 15, "U", KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U')
	USB_KEY(0x19 & 15, "V", KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V')
	USB_KEY(0x1a & 15, "W", KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W')
	USB_KEY(0x1b & 15, "X", KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X')
	USB_KEY(0x1c & 15, "Y", KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y')
	USB_KEY(0x1d & 15, "Z", KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z')
	USB_KEY(0x1e & 15, "1 !", KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!')
	USB_KEY(0x1f & 15, "2 @", KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@')

	PORT_START("KEYS2") // 20h-2Fh
	USB_KEY(0x20 & 15, "3 #", KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#')
	USB_KEY(0x21 & 15, "4 $", KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')
	USB_KEY(0x22 & 15, "5 %", KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%')
	USB_KEY(0x23 & 15, "6 ^", KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^')
	USB_KEY(0x24 & 15, "7 &", KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&')
	USB_KEY(0x25 & 15, "8 *", KEYCODE_8) PORT_CHAR('8') PORT_CHAR('*')
	USB_KEY(0x26 & 15, "9 (", KEYCODE_9) PORT_CHAR('9') PORT_CHAR('(')
	USB_KEY(0x27 & 15, "0 )", KEYCODE_0) PORT_CHAR('0') PORT_CHAR(')')
	USB_KEY(0x28 & 15, "Enter", KEYCODE_ENTER) PORT_CHAR(13)
	USB_KEY(0x29 & 15, "Escape", KEYCODE_ESC) PORT_CHAR(UCHAR_MAMEKEY(ESC))
	USB_KEY(0x2a & 15, "Backspace", KEYCODE_BACKSPACE) PORT_CHAR(8)
	USB_KEY(0x2b & 15, "Tab", KEYCODE_TAB) PORT_CHAR(9)
	USB_KEY(0x2c & 15, "Space", KEYCODE_SPACE) PORT_CHAR(' ')
	USB_KEY(0x2d & 15, "- _", KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('_')
	USB_KEY(0x2e & 15, "= +", KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+')
	USB_KEY(0x2f & 15, "[ {", KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{')

	PORT_START("KEYS3") // 30h-3Fh
	USB_KEY(0x30 & 15, "] }", KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}')
	USB_KEY(0x31 & 15, "\\ |", KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(1U << (0x32 & 15), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Non-US # ~") // no MAME key code of its own
	USB_KEY(0x33 & 15, "; :", KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':')
	USB_KEY(0x34 & 15, "' \"", KEYCODE_QUOTE) PORT_CHAR('\'') PORT_CHAR('"')
	USB_KEY(0x35 & 15, "` ~", KEYCODE_TILDE) PORT_CHAR('`') PORT_CHAR('~')
	USB_KEY(0x36 & 15, ", <", KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')
	USB_KEY(0x37 & 15, ". >", KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>')
	USB_KEY(0x38 & 15, "/ ?", KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')
	USB_KEY(0x39 & 15, "Caps Lock", KEYCODE_CAPSLOCK)
	USB_KEY(0x3a & 15, "F1", KEYCODE_F1) PORT_CHAR(UCHAR_MAMEKEY(F1))
	USB_KEY(0x3b & 15, "F2", KEYCODE_F2) PORT_CHAR(UCHAR_MAMEKEY(F2))
	USB_KEY(0x3c & 15, "F3", KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3))
	USB_KEY(0x3d & 15, "F4", KEYCODE_F4) PORT_CHAR(UCHAR_MAMEKEY(F4))
	USB_KEY(0x3e & 15, "F5", KEYCODE_F5) PORT_CHAR(UCHAR_MAMEKEY(F5))
	USB_KEY(0x3f & 15, "F6", KEYCODE_F6) PORT_CHAR(UCHAR_MAMEKEY(F6))

	PORT_START("KEYS4") // 40h-4Fh
	USB_KEY(0x40 & 15, "F7", KEYCODE_F7) PORT_CHAR(UCHAR_MAMEKEY(F7))
	USB_KEY(0x41 & 15, "F8", KEYCODE_F8) PORT_CHAR(UCHAR_MAMEKEY(F8))
	USB_KEY(0x42 & 15, "F9", KEYCODE_F9) PORT_CHAR(UCHAR_MAMEKEY(F9))
	USB_KEY(0x43 & 15, "F10", KEYCODE_F10) PORT_CHAR(UCHAR_MAMEKEY(F10))
	USB_KEY(0x44 & 15, "F11", KEYCODE_F11) PORT_CHAR(UCHAR_MAMEKEY(F11))
	USB_KEY(0x45 & 15, "F12", KEYCODE_F12) PORT_CHAR(UCHAR_MAMEKEY(F12))
	USB_KEY(0x46 & 15, "Print Screen", KEYCODE_PRTSCR)
	PORT_BIT(1U << (0x47 & 15), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Scroll Lock") // the host's Scroll Lock toggles MAME's UI controls
	USB_KEY(0x48 & 15, "Pause", KEYCODE_PAUSE)
	USB_KEY(0x49 & 15, "Insert", KEYCODE_INSERT) PORT_CHAR(UCHAR_MAMEKEY(INSERT))
	USB_KEY(0x4a & 15, "Home", KEYCODE_HOME) PORT_CHAR(UCHAR_MAMEKEY(HOME))
	USB_KEY(0x4b & 15, "Page Up", KEYCODE_PGUP) PORT_CHAR(UCHAR_MAMEKEY(PGUP))
	USB_KEY(0x4c & 15, "Delete", KEYCODE_DEL) PORT_CHAR(UCHAR_MAMEKEY(DEL))
	USB_KEY(0x4d & 15, "End", KEYCODE_END) PORT_CHAR(UCHAR_MAMEKEY(END))
	USB_KEY(0x4e & 15, "Page Down", KEYCODE_PGDN) PORT_CHAR(UCHAR_MAMEKEY(PGDN))
	USB_KEY(0x4f & 15, "Right", KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT))

	PORT_START("KEYS5") // 50h-5Fh
	USB_KEY(0x50 & 15, "Left", KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT))
	USB_KEY(0x51 & 15, "Down", KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN))
	USB_KEY(0x52 & 15, "Up", KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP))
	USB_KEY(0x53 & 15, "Num Lock", KEYCODE_NUMLOCK)
	USB_KEY(0x54 & 15, "Keypad /", KEYCODE_SLASH_PAD)
	USB_KEY(0x55 & 15, "Keypad *", KEYCODE_ASTERISK)
	USB_KEY(0x56 & 15, "Keypad -", KEYCODE_MINUS_PAD)
	USB_KEY(0x57 & 15, "Keypad +", KEYCODE_PLUS_PAD)
	USB_KEY(0x58 & 15, "Keypad Enter", KEYCODE_ENTER_PAD)
	USB_KEY(0x59 & 15, "Keypad 1 End", KEYCODE_1_PAD)
	USB_KEY(0x5a & 15, "Keypad 2 Down", KEYCODE_2_PAD)
	USB_KEY(0x5b & 15, "Keypad 3 PgDn", KEYCODE_3_PAD)
	USB_KEY(0x5c & 15, "Keypad 4 Left", KEYCODE_4_PAD)
	USB_KEY(0x5d & 15, "Keypad 5", KEYCODE_5_PAD)
	USB_KEY(0x5e & 15, "Keypad 6 Right", KEYCODE_6_PAD)
	USB_KEY(0x5f & 15, "Keypad 7 Home", KEYCODE_7_PAD)

	PORT_START("KEYS6") // 60h-6Fh
	USB_KEY(0x60 & 15, "Keypad 8 Up", KEYCODE_8_PAD)
	USB_KEY(0x61 & 15, "Keypad 9 PgUp", KEYCODE_9_PAD)
	USB_KEY(0x62 & 15, "Keypad 0 Ins", KEYCODE_0_PAD)
	USB_KEY(0x63 & 15, "Keypad . Del", KEYCODE_DEL_PAD)
	USB_KEY(0x64 & 15, "Non-US \\ |", KEYCODE_BACKSLASH2)
	USB_KEY(0x65 & 15, "Application", KEYCODE_MENU)

	PORT_START("MODIFIERS") // E0h-E7h
	USB_KEY(0, "Left Ctrl", KEYCODE_LCONTROL) PORT_CHAR(UCHAR_MAMEKEY(LCONTROL))
	USB_KEY(1, "Left Shift", KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
	USB_KEY(2, "Left Alt", KEYCODE_LALT)
	USB_KEY(3, "Left GUI", KEYCODE_LWIN)
	USB_KEY(4, "Right Ctrl", KEYCODE_RCONTROL)
	USB_KEY(5, "Right Shift", KEYCODE_RSHIFT)
	USB_KEY(6, "Right Alt", KEYCODE_RALT)
	USB_KEY(7, "Right GUI", KEYCODE_RWIN)
INPUT_PORTS_END

zephyr80_usb_keyboard_device::zephyr80_usb_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_USB_KEYBOARD, tag, owner, clock)
	, m_keys(*this, "KEYS%u", 0U)
	, m_modifiers(*this, "MODIFIERS")
	, m_host(nullptr)
	, m_held_count(0)
{
}

ioport_constructor zephyr80_usb_keyboard_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(zephyr80_usb_keyboard);
}

void zephyr80_usb_keyboard_device::device_start()
{
	if (!m_host)
		throw emu_fatalerror("%s: no HID host attached", tag());
	// After the input manager's own frame update (registered before any
	// device starts): the ports hold this frame's values.
	machine().add_notifier(MACHINE_NOTIFY_FRAME, machine_notify_delegate(&zephyr80_usb_keyboard_device::scan, this));
	save_item(NAME(m_held));
	save_item(NAME(m_held_count));
	save_item(NAME(m_sent));
}

// The host restarts with the machine and enumerates the keyboard again: the
// next scan reports whatever is held then.  A restored state keeps the keys
// held when it was saved until the next scan reconciles them.
void zephyr80_usb_keyboard_device::device_reset()
{
	m_held.fill(0);
	m_held_count = 0;
	m_sent.fill(0);
}

// Once a frame: the held keys in press order, and the report when it changed.
void zephyr80_usb_keyboard_device::scan()
{
	std::array<bool, KEY_PORTS * 16> down{};
	for (unsigned port = 0; port < KEY_PORTS; port++)
	{
		ioport_value const value = m_keys[port]->read();
		for (unsigned bit = 0; bit < 16; bit++)
			down[port * 16 + bit] = BIT(value, bit);
	}

	// Keep the press order: drop released keys, append new ones.
	unsigned kept = 0;
	for (unsigned i = 0; i < m_held_count; i++)
		if (down[m_held[i]])
			m_held[kept++] = m_held[i];
	m_held_count = kept;
	for (unsigned usage = 0; usage < down.size(); usage++)
	{
		if (!down[usage])
			continue;
		auto const end = m_held.begin() + m_held_count;
		if (std::find(m_held.begin(), end, u8(usage)) == end && m_held_count < HELD_MAX)
			m_held[m_held_count++] = u8(usage);
	}

	std::array<u8, REPORT> report{};
	report[0] = u8(m_modifiers->read());
	for (unsigned slot = 0; slot < 6; slot++)
		report[2 + slot] = m_held_count > 6 ? 0x01 : (slot < m_held_count ? m_held[slot] : 0x00);

	if (report != m_sent)
	{
		m_sent = report;
		m_host->keyboard_report(report.data(), REPORT);
	}
}
