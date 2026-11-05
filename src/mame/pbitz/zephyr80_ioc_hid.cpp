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
	, m_device_count(0), m_mount_count(0), m_unmount_count(0), m_last_addr(0xff)
	, m_last_vid(0xffff), m_last_pid(0xffff), m_hid_mounts(0), m_last_hid_addr(0xff), m_last_hid_proto(0xff)
	, m_pad_addr{}, m_pad_first_arm{ 0xff, 0xff }, m_pad_reports{}, m_pad_last_len{}, m_pad_next_poll{}
	, m_other_addr(0), m_other_reports(0), m_other_last_len(0), m_other_last{}
	, m_mouse_x(0), m_mouse_y(0), m_mouse_wheel(0), m_mouse_buttons(0), m_mouse_decoded(false)
	, m_pend_x(0), m_pend_y(0), m_pend_wheel(0), m_mouse_pressed(0)
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
	m_device_count = m_mount_count = m_unmount_count = 0;
	m_last_addr = 0xff;
	m_last_vid = m_last_pid = 0xffff;
	m_hid_mounts = 0;
	m_last_hid_addr = m_last_hid_proto = 0xff;
	m_pad_addr.fill(0);
	m_pad_first_arm.fill(0xff);
	m_pad_reports.fill(0);
	m_pad_last_len.fill(0);
	m_pad_next_poll.fill(0);
	m_latch.reset();
	m_other_addr = 0;
	m_other_reports = 0;
	m_other_last_len = 0;
	m_other_last.fill(0);
	m_mouse_x = m_mouse_y = m_mouse_wheel = 0;
	m_mouse_buttons = 0;
	m_mouse_decoded = false;
	m_pend_x = m_pend_y = m_pend_wheel = 0;
	m_mouse_pressed = 0;
	m_doorbell.hid_doorbell(false);
}

// tuh_mount_cb and tuh_umount_cb: the device counters of HID_STATUS page 6.
void zephyr80_ioc_hid::usb_mounted(u8 address, u16 vid, u16 pid)
{
	if (m_device_count != 0xff)
		m_device_count++;
	if (m_mount_count != 0xff)
		m_mount_count++;
	m_last_addr = address;
	m_last_vid = vid;
	m_last_pid = pid;
}

void zephyr80_ioc_hid::usb_unmounted()
{
	if (m_device_count != 0)
		m_device_count--;
	if (m_unmount_count != 0xff)
		m_unmount_count++;
}

// tuh_hid_mount_cb's record of what it was handed, before any filtering.
void zephyr80_ioc_hid::hid_mounted(u8 address, u8 protocol)
{
	if (m_hid_mounts != 0xff)
		m_hid_mounts++;
	m_last_hid_addr = address;
	m_last_hid_proto = protocol;
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
	usb_mounted(KEYBOARD_ADDRESS, 0xffff, 0xffff);
	hid_mounted(KEYBOARD_ADDRESS, 1);
}

void zephyr80_ioc_hid::unmount()
{
	if (m_keyboard_addr != 0)
		usb_unmounted();
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

	// The number keys onto port 0's keypad lines, on every report (an empty one
	// releases), without consuming the key: the digits still reach the queue.
	m_latch.keypad(0, keypad_for_report(report, copy));

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

// coleco_keypad_nibble: the ColecoVision BIOS decode table at 10F5h,
// complemented; order 0-9, then * and #.
namespace {
constexpr u8 KEYPAD_NIBBLE[12] = { 0x0a, 0x0d, 0x07, 0x0c, 0x02, 0x03, 0x0e, 0x05, 0x01, 0x0b, 0x09, 0x06 };
constexpr u8 KEY_1 = 0x1e, KEY_3 = 0x20, KEY_8 = 0x25, KEY_0 = 0x27;
}

// coleco_keypad_for_report: the keypad code a boot report holds.  The number
// row and the keypad give 0-9; keypad * and Shift+8 are *, Shift+3 is #;
// other shifted keys give nothing.  The first matching slot wins.
u8 zephyr80_ioc_hid::keypad_for_report(u8 const *report, unsigned length)
{
	if (length < 3)
		return zephyr80_controller_latch::KEYPAD_NONE;
	bool const shifted = (report[0] & MOD_SHIFT) != 0;
	for (unsigned index = 2; index < length; index++)
	{
		u8 const usage = report[index];
		if (usage == KEY_KEYPAD_MULTIPLY)
			return KEYPAD_NIBBLE[10];
		if (shifted && usage == KEY_8)
			return KEYPAD_NIBBLE[10];
		if (shifted && usage == KEY_3)
			return KEYPAD_NIBBLE[11];
		if (shifted)
			continue;
		if (usage >= KEY_1 && usage <= KEY_1 + 8)
			return KEYPAD_NIBBLE[usage - KEY_1 + 1];
		if (usage == KEY_0)
			return KEYPAD_NIBBLE[0];
		if (usage >= KEY_KEYPAD_1 && usage <= KEY_KEYPAD_1 + 8)
			return KEYPAD_NIBBLE[usage - KEY_KEYPAD_1 + 1];
		if (usage == KEY_KEYPAD_0)
			return KEYPAD_NIBBLE[0];
	}
	return zephyr80_controller_latch::KEYPAD_NONE;
}

// tuh_hid_mount_cb for an interface that is not the keyboard and not an
// F310.  A boot mouse (protocol 2) returns before anything is armed; with no
// keyboard yet the interface would become the keyboard, which is not modelled.
bool zephyr80_ioc_hid::mount_mouse(u8 protocol)
{
	hid_mounted(KEYBOARD_ADDRESS, protocol);
	if (protocol == 2 || m_keyboard_addr == 0)
		return false;
	m_other_addr = m_keyboard_addr;
	m_other_reports = 0;
	m_other_last_len = 0;
	return true;
}

// tuh_hid_umount_cb for that interface.
void zephyr80_ioc_hid::unmount_mouse()
{
	if (m_other_addr == 0)
		return;
	m_other_addr = 0;
	m_mouse_buttons = 0;
	m_mouse_decoded = false;
	m_pend_x = m_pend_y = m_pend_wheel = 0;
	m_mouse_pressed = 0;
}

namespace {

// The capped add of tuh_hid_report_received_cb: a slow reader sees a long
// throw clipped, never reversed.
s16 capped_add(s16 pending, s16 delta)
{
	s32 sum = s32(pending) + s32(delta);
	if (sum > 32767)
		sum = 32767;
	if (sum < -32767)
		sum = -32767;
	return s16(sum);
}

} // anonymous namespace

// tuh_hid_report_received_cb, the other-interface branch: the report is
// counted and kept raw; a 7-byte report with ID 02 is the dongle's mouse.
void zephyr80_ioc_hid::mouse_report(u8 const *report, unsigned length)
{
	if (m_other_addr == 0)
		return;
	unsigned const copy = length < REPORT ? length : REPORT;
	if (m_other_reports != 0xffff)
		m_other_reports++;
	m_other_last_len = length > 0xff ? 0xff : u8(length);
	for (unsigned i = 0; i < REPORT; i++)
		m_other_last[i] = i < copy ? report[i] : 0;

	if (length != MOUSE_REPORT_LEN || report[0] != MOUSE_REPORT_ID)
		return;
	u16 dx = u16(report[2]) | (u16(report[3] & 0x0f) << 8);
	if (dx & 0x0800)
		dx |= 0xf000;
	u16 dy = u16(report[3] >> 4) | (u16(report[4]) << 4);
	if (dy & 0x0800)
		dy |= 0xf000;
	m_mouse_pressed |= u8(report[1] & u8(~m_mouse_buttons));
	m_mouse_buttons = report[1];
	m_mouse_decoded = true;
	m_mouse_x += dx;
	m_mouse_y += dy;
	m_mouse_wheel += (report[5] & 0x80) ? u16(0xff00 | report[5]) : u16(report[5]);
	m_pend_x = capped_add(m_pend_x, s16(dx));
	m_pend_y = capped_add(m_pend_y, s16(dy));
	m_pend_wheel = capped_add(m_pend_wheel, s16(s8(report[5])));
}

// hid_mouse_take: flags, the buttons held, the buttons pressed since the last
// take, and the motion since it; the take clears the last two and the motion.
zephyr80_ioc_hid::mouse_read zephyr80_ioc_hid::mouse_take()
{
	mouse_read out;
	out.flags = u8((m_other_addr != 0 ? HID_MOUSE_MOUNTED : 0) | (m_mouse_decoded ? HID_MOUSE_DECODED : 0));
	out.buttons = m_mouse_buttons;
	out.pressed = m_mouse_pressed;
	out.dx = m_pend_x;
	out.dy = m_pend_y;
	out.wheel = m_pend_wheel;
	m_mouse_pressed = 0;
	m_pend_x = m_pend_y = m_pend_wheel = 0;
	return out;
}

// tuh_mount_cb and the F310 branch of tuh_hid_mount_cb: the first free slot,
// polled at once; a device address from 2 (the keyboard dongle is 1).
u8 zephyr80_ioc_hid::mount_pad(u8 &address)
{
	address = 0;
	u8 candidate = 2;
	while (candidate < 4 && (m_pad_addr[0] == candidate || m_pad_addr[1] == candidate))
		candidate++;
	usb_mounted(candidate, F310_VID, F310_PID);
	hid_mounted(candidate, 0);
	for (unsigned slot = 0; slot < GAMEPADS; slot++)
	{
		if (m_pad_addr[slot] == 0)
		{
			m_pad_addr[slot] = candidate;
			m_pad_next_poll[slot] = 0;
			m_pad_first_arm[slot] = 1;
			m_pad_reports[slot] = 0;
			m_pad_last_len[slot] = 0;
			address = candidate;
			return u8(slot);
		}
	}
	return GAMEPAD_NONE;
}

// gamepad_unmount: the slot is freed and its latch returns to idle (a held
// keypad key stays held).
void zephyr80_ioc_hid::pad_release(u8 slot)
{
	m_pad_addr[slot] = 0;
	m_pad_next_poll[slot] = 0;
	m_pad_first_arm[slot] = 0xff;
	m_latch.release(slot);
}

void zephyr80_ioc_hid::unmount_pad(u8 slot)
{
	if (slot >= GAMEPADS || m_pad_addr[slot] == 0)
		return;
	pad_release(slot);
	usb_unmounted();
}

bool zephyr80_ioc_hid::gamepad_due(unsigned slot, u32 now_ms) const
{
	return slot < GAMEPADS && m_pad_addr[slot] != 0 && s32(now_ms - m_pad_next_poll[slot]) >= 0;
}

// The F310 branch of tuh_hid_report_received_cb: counted, decoded into the
// latch, and the next poll GAMEPAD_POLL_MS after it.
void zephyr80_ioc_hid::gamepad_report(unsigned slot, u8 const *report, unsigned length, u32 now_ms)
{
	if (slot >= GAMEPADS || m_pad_addr[slot] == 0)
		return;
	if (m_pad_reports[slot] != 0xffff)
		m_pad_reports[slot]++;
	m_pad_last_len[slot] = length > 0xff ? 0xff : u8(length);
	m_latch.f310_report(slot, report, length);
	m_pad_next_poll[slot] = now_ms + GAMEPAD_POLL_MS;
}

bool zephyr80_ioc_hid::gamepad_next_due(u32 &due_ms) const
{
	bool found = false;
	for (unsigned slot = 0; slot < GAMEPADS; slot++)
	{
		if (m_pad_addr[slot] != 0 && (!found || s32(m_pad_next_poll[slot] - due_ms) < 0))
		{
			due_ms = m_pad_next_poll[slot];
			found = true;
		}
	}
	return found;
}

// hid_host_gamepad_state.  The live-device fields count the non-hub
// addresses the stack has (1 and 2); the keyboard dongle is address 1.
void zephyr80_ioc_hid::state(gamepad_state &out) const
{
	out = gamepad_state{};
	out.last_device_addr = 0xff;
	out.last_device_vid = out.last_device_pid = 0xffff;
	for (u8 address = 1; address <= 2; address++)
	{
		bool live = address == m_keyboard_addr;
		for (unsigned slot = 0; slot < GAMEPADS; slot++)
			live = live || m_pad_addr[slot] == address;
		if (!live)
			continue;
		out.device_count++;
		out.last_device_addr = address;
		bool pad = false;
		for (unsigned slot = 0; slot < GAMEPADS; slot++)
			pad = pad || m_pad_addr[slot] == address;
		out.last_device_vid = pad ? F310_VID : 0xffff;
		out.last_device_pid = pad ? F310_PID : 0xffff;
	}
	out.mount_count = m_mount_count;
	out.unmount_count = m_unmount_count;
	out.hid_mount_count = m_hid_mounts;
	out.last_hid_addr = m_last_hid_addr;
	out.last_hid_protocol = m_last_hid_proto;
	for (unsigned slot = 0; slot < GAMEPADS; slot++)
	{
		out.gamepad_addr[slot] = m_pad_addr[slot];
		out.gamepad_instance[slot] = 0;
		out.gamepad_first_arm[slot] = m_pad_first_arm[slot];
		out.gamepad_reports[slot] = m_pad_reports[slot];
		out.gamepad_last_len[slot] = m_pad_last_len[slot];
		out.gamepad_latch[slot] = m_latch.value(slot);
	}
	out.other_addr = m_other_addr;
	out.other_instance = m_other_addr != 0 ? MOUSE_INSTANCE : 0;
	out.other_reports = m_other_reports;
	out.other_last_len = m_other_last_len;
	out.other_last = m_other_last;
	out.mouse_x = m_mouse_x;
	out.mouse_y = m_mouse_y;
	out.mouse_wheel = m_mouse_wheel;
	out.mouse_buttons = m_mouse_buttons;
}
