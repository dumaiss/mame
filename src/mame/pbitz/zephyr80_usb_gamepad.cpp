// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_usb_gamepad.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_USB_GAMEPAD, zephyr80_usb_gamepad_device, "zephyr80_usb_gamepad", "Zephyr-80 USB gamepad (Logitech F310)")

// Defaults follow the host joystick of the pad's number (MAME's SDL game
// controller order: A, B, X, Y, LB, RB, Back, Start, L3, R3 are buttons 1-10;
// the D-pad is the hat, the left stick the X/Y axes as switches).  Triggers
// have no default.
#define USB_PAD_PORTS(n) \
	PORT_START("DPAD") \
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("D-Pad Up") PORT_CODE(JOYCODE_HAT1UP_INDEXED(n)) \
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("D-Pad Right") PORT_CODE(JOYCODE_HAT1RIGHT_INDEXED(n)) \
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("D-Pad Down") PORT_CODE(JOYCODE_HAT1DOWN_INDEXED(n)) \
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("D-Pad Left") PORT_CODE(JOYCODE_HAT1LEFT_INDEXED(n)) \
	PORT_START("STICK") \
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Left Stick Up") PORT_CODE(JOYCODE_Y_UP_SWITCH_INDEXED(n)) \
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Left Stick Right") PORT_CODE(JOYCODE_X_RIGHT_SWITCH_INDEXED(n)) \
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Left Stick Down") PORT_CODE(JOYCODE_Y_DOWN_SWITCH_INDEXED(n)) \
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Left Stick Left") PORT_CODE(JOYCODE_X_LEFT_SWITCH_INDEXED(n)) \
	PORT_START("FACE") \
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("X") PORT_CODE(JOYCODE_BUTTON3_INDEXED(n)) \
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("A") PORT_CODE(JOYCODE_BUTTON1_INDEXED(n)) \
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("B") PORT_CODE(JOYCODE_BUTTON2_INDEXED(n)) \
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Y") PORT_CODE(JOYCODE_BUTTON4_INDEXED(n)) \
	PORT_START("SHOULDER") \
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("LB") PORT_CODE(JOYCODE_BUTTON5_INDEXED(n)) \
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("RB") PORT_CODE(JOYCODE_BUTTON6_INDEXED(n)) \
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("LT") \
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("RT") \
	PORT_START("SYSTEM") \
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Back") PORT_CODE(JOYCODE_BUTTON7_INDEXED(n)) \
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Start") PORT_CODE(JOYCODE_BUTTON8_INDEXED(n)) \
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("L3") PORT_CODE(JOYCODE_BUTTON9_INDEXED(n)) \
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("R3") PORT_CODE(JOYCODE_BUTTON10_INDEXED(n)) \
	PORT_START("PAD_CONFIG")

static INPUT_PORTS_START(zephyr80_usb_gamepad_1)
	USB_PAD_PORTS(0)
	PORT_CONFNAME(0x01, 0x00, "USB gamepad 1")
	PORT_CONFSETTING(0x00, "Not connected")
	PORT_CONFSETTING(0x01, "Connected")
INPUT_PORTS_END

static INPUT_PORTS_START(zephyr80_usb_gamepad_2)
	USB_PAD_PORTS(1)
	PORT_CONFNAME(0x01, 0x00, "USB gamepad 2")
	PORT_CONFSETTING(0x00, "Not connected")
	PORT_CONFSETTING(0x01, "Connected")
INPUT_PORTS_END

zephyr80_usb_gamepad_device::zephyr80_usb_gamepad_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_USB_GAMEPAD, tag, owner, clock)
	, m_dpad(*this, "DPAD")
	, m_stick(*this, "STICK")
	, m_face(*this, "FACE")
	, m_shoulder(*this, "SHOULDER")
	, m_system(*this, "SYSTEM")
	, m_config(*this, "PAD_CONFIG")
	, m_host(nullptr)
	, m_number(0)
	, m_was_connected(false)
{
}

ioport_constructor zephyr80_usb_gamepad_device::device_input_ports() const
{
	return m_number ? INPUT_PORTS_NAME(zephyr80_usb_gamepad_2) : INPUT_PORTS_NAME(zephyr80_usb_gamepad_1);
}

void zephyr80_usb_gamepad_device::device_start()
{
	if (!m_host)
		throw emu_fatalerror("%s: no HID host attached", tag());
	machine().add_notifier(MACHINE_NOTIFY_FRAME, machine_notify_delegate(&zephyr80_usb_gamepad_device::scan, this));
	save_item(NAME(m_was_connected));
}

void zephyr80_usb_gamepad_device::device_reset()
{
	m_was_connected = false;
}

bool zephyr80_usb_gamepad_device::connected() const
{
	return (m_config->read() & 1) != 0;
}

// Once a frame: a connection change is the controller's to act on.
void zephyr80_usb_gamepad_device::scan()
{
	bool const present = connected();
	if (present != m_was_connected)
	{
		m_was_connected = present;
		m_host->usb_input_ready();
	}
}

// The F310's DirectInput report for the fields now.
void zephyr80_usb_gamepad_device::report(u8 *out) const
{
	u32 const dpad = m_dpad->read();
	u32 const stick = m_stick->read();
	u32 const face = m_face->read();
	u32 const shoulder = m_shoulder->read();
	u32 const system = m_system->read();

	// Opposite directions cancel, as a hat and a stick axis do.
	auto axis = [] (u32 value, u32 negative, u32 positive) { return int(BIT(value, positive)) - int(BIT(value, negative)); };

	// Hat: 0 up, clockwise to 7 up-left, 8 neutral.
	static constexpr u8 HAT[3][3] = { { 7, 0, 1 }, { 6, 8, 2 }, { 5, 4, 3 } }; // [vertical + 1][horizontal + 1], vertical +1 down
	u8 const hat = HAT[axis(dpad, 0, 2) + 1][axis(dpad, 3, 1) + 1];

	static constexpr u8 THROW[3] = { 0x00, 0x80, 0xff }; // negative, neutral, positive
	out[0] = THROW[axis(stick, 3, 1) + 1];
	out[1] = THROW[axis(stick, 0, 2) + 1];
	out[2] = 0x80;
	out[3] = 0x80;
	out[4] = u8(hat | (BIT(face, 0) << 4) | (BIT(face, 1) << 5) | (BIT(face, 2) << 6) | (BIT(face, 3) << 7));
	out[5] = u8((shoulder & 0x0f) | ((system & 0x0f) << 4));
	out[6] = 0;
	out[7] = 0;
}
