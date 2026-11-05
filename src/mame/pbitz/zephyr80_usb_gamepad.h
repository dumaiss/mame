// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_USB_GAMEPAD_H
#define MAME_PBITZ_ZEPHYR80_USB_GAMEPAD_H

#pragma once

#include <array>

// A Logitech F310 in DirectInput mode, as the IO controller's HID host polls
// it: an 8-byte report from MAME input fields,
//
//   byte 0, 1 = left stick X and Y (80h neutral; 00h left or up, FFh right or
//               down), 2, 3 = right stick (80h),
//   byte 4 = hat in D3:D0 (0 up, clockwise to 7 up-left, 8 none), X D4,
//            A D5, B D6, Y D7,
//   byte 5 = LB D0, RB D1, LT D2, RT D3, Back D4, Start D5, L3 D6, R3 D7,
//   bytes 6, 7 = 0.
//
// The firmware decodes it into a ColecoVision controller byte
// (zephyr80_controller_latch).  Every field is a digital MAME input: the
// D-pad and the left stick are four directions each (the stick at full
// deflection, which the firmware's wide dead zone treats the same as an
// analog throw), the buttons are the F310's twelve.  The host bindings are
// MAME's own: this device takes the host joystick of its number (1 or 2) for
// its defaults, and the Input (this Machine) menu rebinds any field. The F310
// answers each poll with its current state, so poll() always returns a
// report; the controller asks every 10 ms (GAMEPAD_POLL_MS).
//
// "USB gamepad 1" and "USB gamepad 2" are configuration settings: a pad that
// is not connected is not mounted, and the latch of an unmounted pad reads
// 7Fh.  USB enumeration, the hub, transfers and the poll interval are not
// modelled.
class zephyr80_usb_gamepad_device : public device_t
{
public:
	static constexpr unsigned REPORT = 8;

	// The controller the pad reports to.
	class host
	{
	public:
		// The pad was connected or disconnected.
		virtual void usb_input_ready() = 0;

	protected:
		~host() = default;
	};

	zephyr80_usb_gamepad_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// The host joystick number the defaults follow: 0 or 1.
	void set_number(unsigned number) { m_number = number; }
	void set_host(host &receiver) { m_host = &receiver; }

	bool connected() const;
	// Connected as of the last scan: what the controller has mounted.
	bool attached() const { return m_was_connected; }
	// The pad's current report (REPORT bytes).
	void report(u8 *out) const;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	void scan();

	required_ioport m_dpad;
	required_ioport m_stick;
	required_ioport m_face;
	required_ioport m_shoulder;
	required_ioport m_system;
	required_ioport m_config;
	host *m_host;
	unsigned m_number;
	bool m_was_connected;
};

DECLARE_DEVICE_TYPE(ZEPHYR80_USB_GAMEPAD, zephyr80_usb_gamepad_device)

#endif // MAME_PBITZ_ZEPHYR80_USB_GAMEPAD_H
