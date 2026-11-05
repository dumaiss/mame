// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_usb_mouse.h"

#include <algorithm>

DEFINE_DEVICE_TYPE(ZEPHYR80_USB_MOUSE, zephyr80_usb_mouse_device, "zephyr80_usb_mouse", "Zephyr-80 USB mouse")

// A relative field reads as a 16-bit position that wraps (the host's pixels at
// sensitivity 100; +Y is down, as in the report), so the motion since the last
// scan is the signed 16-bit difference of two reads.  The wheel counts 1.2
// units per host notch (120 at 1/100); the difference between two reads is the
// notches turned.
static INPUT_PORTS_START(zephyr80_usb_mouse)
	PORT_START("MOUSE_X")
	PORT_BIT(0xffff, 0x0000, IPT_MOUSE_X) PORT_SENSITIVITY(100) PORT_MINMAX(0x0000, 0xffff) PORT_KEYDELTA(0) PORT_NAME("Mouse X")

	PORT_START("MOUSE_Y")
	PORT_BIT(0xffff, 0x0000, IPT_MOUSE_Y) PORT_SENSITIVITY(100) PORT_MINMAX(0x0000, 0xffff) PORT_KEYDELTA(0) PORT_NAME("Mouse Y")

	PORT_START("MOUSE_WHEEL")
	PORT_BIT(0xffff, 0x0000, IPT_DIAL_V) PORT_SENSITIVITY(1) PORT_MINMAX(0x0000, 0xffff) PORT_KEYDELTA(0) PORT_CODE(MOUSECODE_Z) PORT_NAME("Mouse Wheel")

	PORT_START("MOUSE_BUTTONS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Mouse Left Button") PORT_CODE(MOUSECODE_BUTTON1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Mouse Right Button") PORT_CODE(MOUSECODE_BUTTON3)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Mouse Middle Button") PORT_CODE(MOUSECODE_BUTTON2)

	PORT_START("MOUSE_CONFIG")
	PORT_CONFNAME(0x01, 0x01, "USB mouse")
	PORT_CONFSETTING(0x00, "Not connected")
	PORT_CONFSETTING(0x01, "Connected")
INPUT_PORTS_END

zephyr80_usb_mouse_device::zephyr80_usb_mouse_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_USB_MOUSE, tag, owner, clock)
	, m_x(*this, "MOUSE_X")
	, m_y(*this, "MOUSE_Y")
	, m_wheel_port(*this, "MOUSE_WHEEL")
	, m_buttons(*this, "MOUSE_BUTTONS")
	, m_config(*this, "MOUSE_CONFIG")
	, m_host(nullptr)
	, m_last_x(0), m_last_y(0), m_last_wheel(0), m_synced(false)
	, m_dx(0), m_dy(0), m_wheel(0)
	, m_ring{}, m_ring_count(0), m_scanned(0), m_sent(0), m_was_connected(false)
{
}

ioport_constructor zephyr80_usb_mouse_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(zephyr80_usb_mouse);
}

void zephyr80_usb_mouse_device::device_start()
{
	if (!m_host)
		throw emu_fatalerror("%s: no HID host attached", tag());
	// After the input manager's own frame update: the ports hold this frame's values.
	machine().add_notifier(MACHINE_NOTIFY_FRAME, machine_notify_delegate(&zephyr80_usb_mouse_device::scan, this));
	save_item(NAME(m_last_x));
	save_item(NAME(m_last_y));
	save_item(NAME(m_last_wheel));
	save_item(NAME(m_synced));
	save_item(NAME(m_dx));
	save_item(NAME(m_dy));
	save_item(NAME(m_wheel));
	save_item(NAME(m_ring));
	save_item(NAME(m_ring_count));
	save_item(NAME(m_scanned));
	save_item(NAME(m_sent));
	save_item(NAME(m_was_connected));
}

// The controller restarts with the board and enumerates the mouse again: no
// motion owed, no button state kept; the first scan after the reset takes the
// positions as they are and the buttons held then are reported after the
// mount.
void zephyr80_usb_mouse_device::device_reset()
{
	m_dx = m_dy = m_wheel = 0;
	m_synced = false;
	m_ring.fill(0);
	m_ring_count = 0;
	m_scanned = 0;
	m_sent = 0;
	m_was_connected = false;
}

bool zephyr80_usb_mouse_device::connected() const
{
	return (m_config->read() & 1) != 0;
}

namespace {

s32 saturate(s32 total, s32 delta)
{
	s64 const sum = s64(total) + delta;
	return s32(std::clamp<s64>(sum, -zephyr80_usb_mouse_device::TOTAL_MAX, zephyr80_usb_mouse_device::TOTAL_MAX));
}

} // anonymous namespace

void zephyr80_usb_mouse_device::push_buttons(u8 state)
{
	if (m_ring_count < RING)
		m_ring[m_ring_count++] = state;
	else
		m_ring[RING - 1] = state;
}

// Once a frame: the motion since the last scan and the buttons now.
void zephyr80_usb_mouse_device::scan()
{
	// The positions are followed whether or not the mouse is connected, so a
	// mouse connected later starts from where the host is, not from a jump.
	u16 const x = u16(m_x->read());
	u16 const y = u16(m_y->read());
	u16 const wheel = u16(m_wheel_port->read());
	s16 const move_x = m_synced ? s16(x - m_last_x) : 0;
	s16 const move_y = m_synced ? s16(y - m_last_y) : 0;
	s16 const move_wheel = m_synced ? s16(wheel - m_last_wheel) : 0;
	m_last_x = x;
	m_last_y = y;
	m_last_wheel = wheel;
	m_synced = true;

	bool const present = connected();
	if (!present)
	{
		// Removed: nothing a mouse held survives, and a later connection starts clean.
		m_dx = m_dy = m_wheel = 0;
		m_ring_count = 0;
		m_scanned = 0;
		m_sent = 0;
		if (m_was_connected)
		{
			m_was_connected = false;
			m_host->usb_input_ready();
		}
		return;
	}

	bool const arrived = !m_was_connected;
	m_was_connected = true;

	m_dx = saturate(m_dx, move_x);
	m_dy = saturate(m_dy, move_y);
	m_wheel = saturate(m_wheel, move_wheel);

	u8 const buttons = u8(m_buttons->read() & 7);
	if (buttons != m_scanned)
	{
		push_buttons(buttons);
		m_scanned = buttons;
	}

	if (arrived || m_dx || m_dy || m_wheel || m_ring_count)
		m_host->usb_input_ready();
}

// The controller's poll: the oldest button state with a bounded step of the
// motion; false when nothing is owed.
bool zephyr80_usb_mouse_device::poll(u8 *report)
{
	if (!m_was_connected || (!m_dx && !m_dy && !m_wheel && !m_ring_count))
		return false;
	if (m_ring_count)
	{
		m_sent = m_ring[0];
		std::copy(m_ring.begin() + 1, m_ring.end(), m_ring.begin());
		m_ring_count--;
	}
	s32 const dx = std::clamp<s32>(m_dx, -AXIS_MAX, AXIS_MAX);
	s32 const dy = std::clamp<s32>(m_dy, -AXIS_MAX, AXIS_MAX);
	s32 const wheel = std::clamp<s32>(m_wheel, -WHEEL_MAX, WHEEL_MAX);
	m_dx -= dx;
	m_dy -= dy;
	m_wheel -= wheel;
	u16 const x = u16(dx) & 0x0fff;
	u16 const y = u16(dy) & 0x0fff;
	report[0] = 0x02;
	report[1] = m_sent;
	report[2] = u8(x);
	report[3] = u8((x >> 8) | ((y & 0x0f) << 4));
	report[4] = u8(y >> 4);
	report[5] = u8(wheel);
	report[6] = 0;
	return true;
}
