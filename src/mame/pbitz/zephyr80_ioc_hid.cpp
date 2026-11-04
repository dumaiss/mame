// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc_hid.h"

namespace {

// TinyUSB class/hid/hid.h usages and modifier bits, as the firmware uses them.
constexpr u8 KEY_A = 0x04;
constexpr u8 KEY_Z = 0x1d;
constexpr u8 KEY_2 = 0x1f;
constexpr u8 KEY_6 = 0x23;
constexpr u8 KEY_ESCAPE = 0x29;
constexpr u8 KEY_SPACE = 0x2c;
constexpr u8 KEY_MINUS = 0x2d;
constexpr u8 KEY_BRACKET_LEFT = 0x2f;
constexpr u8 KEY_BRACKET_RIGHT = 0x30;
constexpr u8 KEY_BACKSLASH = 0x31;
constexpr u8 KEY_CAPS_LOCK = 0x39;
constexpr u8 KEY_F1 = 0x3a, KEY_F2 = 0x3b, KEY_F3 = 0x3c, KEY_F4 = 0x3d, KEY_F5 = 0x3e, KEY_F6 = 0x3f;
constexpr u8 KEY_F7 = 0x40, KEY_F8 = 0x41, KEY_F9 = 0x42, KEY_F10 = 0x43, KEY_F11 = 0x44, KEY_F12 = 0x45;
constexpr u8 KEY_SCROLL_LOCK = 0x47;
constexpr u8 KEY_INSERT = 0x49;
constexpr u8 KEY_HOME = 0x4a;
constexpr u8 KEY_PAGE_UP = 0x4b;
constexpr u8 KEY_DELETE = 0x4c;
constexpr u8 KEY_END = 0x4d;
constexpr u8 KEY_PAGE_DOWN = 0x4e;
constexpr u8 KEY_ARROW_RIGHT = 0x4f;
constexpr u8 KEY_ARROW_LEFT = 0x50;
constexpr u8 KEY_ARROW_DOWN = 0x51;
constexpr u8 KEY_ARROW_UP = 0x52;
constexpr u8 KEY_NUM_LOCK = 0x53;
constexpr u8 KEY_KEYPAD_DIVIDE = 0x54;
constexpr u8 KEY_KEYPAD_MULTIPLY = 0x55;
constexpr u8 KEY_KEYPAD_SUBTRACT = 0x56;
constexpr u8 KEY_KEYPAD_ADD = 0x57;
constexpr u8 KEY_KEYPAD_ENTER = 0x58;
constexpr u8 KEY_KEYPAD_1 = 0x59, KEY_KEYPAD_2 = 0x5a, KEY_KEYPAD_3 = 0x5b, KEY_KEYPAD_4 = 0x5c, KEY_KEYPAD_5 = 0x5d;
constexpr u8 KEY_KEYPAD_6 = 0x5e, KEY_KEYPAD_7 = 0x5f, KEY_KEYPAD_8 = 0x60, KEY_KEYPAD_9 = 0x61, KEY_KEYPAD_0 = 0x62;
constexpr u8 KEY_KEYPAD_DECIMAL = 0x63;

constexpr u8 MOD_CTRL = 0x01 | 0x10;  // KEYBOARD_MODIFIER_LEFTCTRL | RIGHTCTRL
constexpr u8 MOD_SHIFT = 0x02 | 0x20; // LEFTSHIFT | RIGHTSHIFT
constexpr u8 MOD_ALT = 0x04 | 0x40;   // LEFTALT | RIGHTALT

constexpr u8 SPEED_FULL = 0x00;       // TUSB_SPEED_FULL: a device the hub path can reach

// HID_KEYCODE_TO_ASCII (TinyUSB class/hid/hid.h, the firmware's
// keycode_ascii[128][2]): unshifted and shifted byte per usage; usages
// 68h-7Fh are zero.
constexpr u8 KEYCODE_ASCII[0x68][2] = {
	{ 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
	{ 'a', 'A' }, { 'b', 'B' }, { 'c', 'C' }, { 'd', 'D' }, { 'e', 'E' }, { 'f', 'F' }, { 'g', 'G' }, { 'h', 'H' },
	{ 'i', 'I' }, { 'j', 'J' }, { 'k', 'K' }, { 'l', 'L' }, { 'm', 'M' }, { 'n', 'N' }, { 'o', 'O' }, { 'p', 'P' },
	{ 'q', 'Q' }, { 'r', 'R' }, { 's', 'S' }, { 't', 'T' }, { 'u', 'U' }, { 'v', 'V' }, { 'w', 'W' }, { 'x', 'X' },
	{ 'y', 'Y' }, { 'z', 'Z' },
	{ '1', '!' }, { '2', '@' }, { '3', '#' }, { '4', '$' }, { '5', '%' },
	{ '6', '^' }, { '7', '&' }, { '8', '*' }, { '9', '(' }, { '0', ')' },
	{ '\r', '\r' }, { 0x1b, 0x1b }, { '\b', '\b' }, { '\t', '\t' }, { ' ', ' ' },
	{ '-', '_' }, { '=', '+' }, { '[', '{' }, { ']', '}' }, { '\\', '|' }, { '#', '~' }, { ';', ':' }, { '\'', '"' },
	{ '`', '~' }, { ',', '<' }, { '.', '>' }, { '/', '?' },
	{ 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },      // 39h-40h
	{ 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },      // 41h-48h
	{ 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },      // 49h-50h
	{ 0, 0 }, { 0, 0 }, { 0, 0 },                                                          // 51h-53h
	{ '/', '/' }, { '*', '*' }, { '-', '-' }, { '+', '+' }, { '\r', '\r' },
	{ '1', 0 }, { '2', 0 }, { '3', 0 }, { '4', 0 }, { '5', '5' }, { '6', 0 }, { '7', 0 }, { '8', 0 }, { '9', 0 },
	{ '0', 0 }, { '.', 0 },
	{ 0, 0 }, { 0, 0 }, { 0, 0 }, { '=', '=' },                                            // 64h-67h
};

} // anonymous namespace

// The owner calls reset() before the first use; the doorbell's transport may
// not be attached yet.
zephyr80_ioc_hid::zephyr80_ioc_hid(doorbell &line)
	: m_doorbell(line)
	, m_queue{}
	, m_head(0), m_tail(0), m_count(0), m_dropped(0)
	, m_keyboard_addr(0), m_speed(0xff), m_reports(0), m_last{}
	, m_caps_lock(false), m_num_lock(true), m_leds(0)
	, m_repeat_key(0), m_repeat_mod(0), m_repeat_due(0)
	, m_reset_combo(false)
{
}

void zephyr80_ioc_hid::reset()
{
	m_queue.fill(0);
	m_head = m_tail = m_count = m_dropped = 0;
	m_keyboard_addr = 0;
	m_speed = 0xff;
	m_reports = 0;
	m_last.fill(0);
	m_caps_lock = false;
	m_num_lock = true;   // keypad numeric at power-on
	m_leds = 0;
	m_repeat_key = 0;
	m_repeat_mod = 0;
	m_repeat_due = 0;
	m_reset_combo = false;
	m_doorbell.hid_doorbell(false);
}

// tuh_hid_mount_cb's keyboard branch: a fresh keyboard has its LEDs dark,
// so the NumLock LED is pushed unconditionally.
void zephyr80_ioc_hid::mount()
{
	m_keyboard_addr = KEYBOARD_ADDRESS;
	m_reports = 0;
	m_caps_lock = false;
	m_num_lock = true;
	m_repeat_key = 0;
	m_leds = LED_NUM_LOCK;
	m_last.fill(0);
	m_speed = SPEED_FULL;
}

void zephyr80_ioc_hid::unmount()
{
	m_keyboard_addr = 0;
	m_reports = 0;
	m_speed = 0xff;
	m_repeat_key = 0;
}

bool zephyr80_ioc_hid::put(u8 const *bytes, unsigned length)
{
	if (QUEUE - m_count < length)
	{
		if (m_dropped != 0xff)
			m_dropped++;
		return false;
	}
	for (unsigned i = 0; i < length; i++)
	{
		m_queue[m_head] = bytes[i];
		m_head = (m_head + 1) & (QUEUE - 1);
		m_count++;
	}
	m_doorbell.hid_doorbell(m_count != 0);
	return true;
}

u8 zephyr80_ioc_hid::get()
{
	if (m_count == 0)
		return 0;
	u8 const value = m_queue[m_tail];
	m_tail = (m_tail + 1) & (QUEUE - 1);
	m_count--;
	m_doorbell.hid_doorbell(m_count != 0);
	return value;
}

void zephyr80_ioc_hid::put_byte(u8 value)
{
	put(&value, 1);
}

void zephyr80_ioc_hid::put_escape(u8 introducer, u8 final)
{
	u8 const sequence[] = { 0x1b, introducer, final };
	put(sequence, 3);
}

void zephyr80_ioc_hid::put_csi_number(u8 tens, u8 ones)
{
	u8 sequence[5] = { 0x1b, '[' };
	unsigned length = 2;
	if (tens != 0)
		sequence[length++] = u8('0' + tens);
	sequence[length++] = u8('0' + ones);
	sequence[length++] = '~';
	put(sequence, length);
}

bool zephyr80_ioc_hid::was_down(u8 keycode) const
{
	for (unsigned i = 2; i < REPORT; i++)
		if (m_last[i] == keycode)
			return true;
	return false;
}

void zephyr80_ioc_hid::led_refresh()
{
	m_leds = (m_num_lock ? LED_NUM_LOCK : 0) | (m_caps_lock ? LED_CAPS_LOCK : 0);
}

// translate_key
void zephyr80_ioc_hid::translate(u8 keycode, u8 modifier)
{
	bool const shift = modifier & MOD_SHIFT;
	bool const ctrl = modifier & MOD_CTRL;

	if (keycode == KEY_CAPS_LOCK)
	{
		m_caps_lock = !m_caps_lock;
		led_refresh();
		return;
	}
	if (keycode == KEY_NUM_LOCK)
	{
		m_num_lock = !m_num_lock;
		led_refresh();
		return;
	}

	if (ctrl)
	{
		if (keycode >= KEY_A && keycode <= KEY_Z) { put_byte(u8(keycode - KEY_A + 1)); return; }
		if (keycode == KEY_BRACKET_LEFT) { put_byte(0x1b); return; }
		if (keycode == KEY_BACKSLASH) { put_byte(0x1c); return; }
		if (keycode == KEY_BRACKET_RIGHT) { put_byte(0x1d); return; }
		if (keycode == KEY_6) { put_byte(0x1e); return; }
		if (keycode == KEY_MINUS) { put_byte(0x1f); return; }
		if (keycode == KEY_2 || keycode == KEY_SPACE) { put_byte(0x00); return; }
	}

	// NumLock off: the keypad's navigation layer, ahead of the table.
	if (!m_num_lock)
	{
		switch (keycode)
		{
		case KEY_KEYPAD_7: put_escape('[', 'H'); return;
		case KEY_KEYPAD_8: put_escape('[', 'A'); return;
		case KEY_KEYPAD_9: put_csi_number(0, 5); return;
		case KEY_KEYPAD_4: put_escape('[', 'D'); return;
		case KEY_KEYPAD_6: put_escape('[', 'C'); return;
		case KEY_KEYPAD_1: put_escape('[', 'F'); return;
		case KEY_KEYPAD_2: put_escape('[', 'B'); return;
		case KEY_KEYPAD_3: put_csi_number(0, 6); return;
		case KEY_KEYPAD_0: put_csi_number(0, 2); return;
		case KEY_KEYPAD_DECIMAL: put_csi_number(0, 3); return;
		default: break;
		}
	}

	if (keycode < 0x68)
	{
		bool upper = shift;
		if (keycode >= KEY_A && keycode <= KEY_Z)
			upper = shift != m_caps_lock;
		u8 const value = KEYCODE_ASCII[keycode][upper ? 1 : 0];
		if (value != 0)
		{
			put_byte(value);
			return;
		}
	}

	switch (keycode)
	{
	case KEY_ARROW_UP:    put_escape('[', 'A'); return;
	case KEY_ARROW_DOWN:  put_escape('[', 'B'); return;
	case KEY_ARROW_RIGHT: put_escape('[', 'C'); return;
	case KEY_ARROW_LEFT:  put_escape('[', 'D'); return;
	case KEY_HOME:        put_escape('[', 'H'); return;
	case KEY_END:         put_escape('[', 'F'); return;
	case KEY_INSERT:      put_csi_number(0, 2); return;
	case KEY_DELETE:      put_csi_number(0, 3); return;
	case KEY_PAGE_UP:     put_csi_number(0, 5); return;
	case KEY_PAGE_DOWN:   put_csi_number(0, 6); return;
	case KEY_F1:  put_escape('O', 'P'); return;
	case KEY_F2:  put_escape('O', 'Q'); return;
	case KEY_F3:  put_escape('O', 'R'); return;
	case KEY_F4:  put_escape('O', 'S'); return;
	case KEY_F5:  put_csi_number(1, 5); return;
	case KEY_F6:  put_csi_number(1, 7); return;
	case KEY_F7:  put_csi_number(1, 8); return;
	case KEY_F8:  put_csi_number(1, 9); return;
	case KEY_F9:  put_csi_number(2, 0); return;
	case KEY_F10: put_csi_number(2, 1); return;
	case KEY_F11: put_csi_number(2, 3); return;
	case KEY_F12: put_csi_number(2, 4); return;
	case KEY_KEYPAD_DIVIDE:   put_byte('/'); return;
	case KEY_KEYPAD_MULTIPLY: put_byte('*'); return;
	case KEY_KEYPAD_SUBTRACT: put_byte('-'); return;
	case KEY_KEYPAD_ADD:      put_byte('+'); return;
	case KEY_KEYPAD_ENTER:    put_byte('\r'); return;
	case KEY_KEYPAD_1: put_byte('1'); return;
	case KEY_KEYPAD_2: put_byte('2'); return;
	case KEY_KEYPAD_3: put_byte('3'); return;
	case KEY_KEYPAD_4: put_byte('4'); return;
	case KEY_KEYPAD_5: put_byte('5'); return;
	case KEY_KEYPAD_6: put_byte('6'); return;
	case KEY_KEYPAD_7: put_byte('7'); return;
	case KEY_KEYPAD_8: put_byte('8'); return;
	case KEY_KEYPAD_9: put_byte('9'); return;
	case KEY_KEYPAD_0: put_byte('0'); return;
	case KEY_KEYPAD_DECIMAL: put_byte('.'); return;
	default: return;
	}
}

// tuh_hid_report_received_cb, keyboard branch.  The controller-latch keypad
// mirror of the number keys belongs to the controller ports, not to the
// terminal input.
void zephyr80_ioc_hid::report(u8 const *report, unsigned length, u32 now_ms)
{
	if (!mounted())
		return;
	unsigned const copy = length < REPORT ? length : REPORT;

	// keyboard_reset_combo: either Ctrl, either Alt, Escape in any slot.
	if (copy > 0 && (report[0] & MOD_CTRL) && (report[0] & MOD_ALT))
	{
		for (unsigned i = 2; i < copy; i++)
		{
			if (report[i] == KEY_ESCAPE)
			{
				m_reset_combo = true;
				return;
			}
		}
	}

	u8 pressed = 0;
	for (unsigned i = 2; i < copy; i++)
	{
		u8 const keycode = report[i];
		if (keycode > 3 && !was_down(keycode))
		{
			translate(keycode, report[0]);
			if (keycode != KEY_CAPS_LOCK && keycode != KEY_NUM_LOCK && keycode != KEY_SCROLL_LOCK)
				pressed = keycode;
		}
	}

	for (unsigned i = 0; i < REPORT; i++)
		m_last[i] = i < copy ? report[i] : 0;

	if (pressed != 0)
	{
		m_repeat_key = pressed;
		m_repeat_mod = report[0];
		m_repeat_due = now_ms + REPEAT_DELAY_MS;
	}
	else if (m_repeat_key != 0 && !was_down(m_repeat_key))
	{
		m_repeat_key = 0;
	}
	else if (m_repeat_key != 0)
	{
		m_repeat_mod = report[0];
	}

	m_reports++;
}

// hid_host_task's repeat: never let it consume more than half the queue.
void zephyr80_ioc_hid::task(u32 now_ms)
{
	if (m_repeat_key == 0 || !mounted())
		return;
	if (s32(now_ms - m_repeat_due) >= 0)
	{
		if (m_count < QUEUE / 2)
			translate(m_repeat_key, m_repeat_mod);
		m_repeat_due = now_ms + REPEAT_PERIOD_MS;
	}
}

bool zephyr80_ioc_hid::repeat_due(u32 &due_ms) const
{
	if (m_repeat_key == 0 || !mounted())
		return false;
	due_ms = m_repeat_due;
	return true;
}
