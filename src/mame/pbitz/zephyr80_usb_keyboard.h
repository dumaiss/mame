// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_USB_KEYBOARD_H
#define MAME_PBITZ_ZEPHYR80_USB_KEYBOARD_H

#pragma once

#include <array>

// A USB boot-protocol keyboard as the IO controller's HID host receives it:
// one input field per HID usage (keys 04h-65h, the eight modifiers E0h-E7h),
// mapped to MAME's positional key codes, so the host's layout, its text
// translation and its key repeat never reach the guest.  Whenever the held
// keys change the keyboard sends its 8-byte boot report: the modifier byte,
// a reserved zero, then up to six usages in the order they were pressed;
// with more than six keys held every slot reads 01h (ErrorRollOver).  A boot
// keyboard reports only on change, so a held key sends nothing more.
//
// The keyboard scans its fields once a frame, after MAME's input update, so
// the report is the live input state, not a stream of host events: a key
// MAME sees released (focus loss, the UI menu taking the input) is released
// in the next report, and after a reset or a state load the next scan
// reports the keys held now.  The held keys in press order and the last
// report are save-registered.  USB enumeration, transfers and the LED output
// report are not modelled.
class zephyr80_usb_keyboard_device : public device_t
{
public:
	static constexpr unsigned REPORT = 8;

	// The HID host the reports go to (the IO controller).
	class host
	{
	public:
		virtual void keyboard_report(u8 const *report, unsigned length) = 0;

	protected:
		~host() = default;
	};

	zephyr80_usb_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void set_host(host &receiver) { m_host = &receiver; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	static constexpr unsigned KEY_PORTS = 7;   // usages 00h-6Fh, 16 a port
	static constexpr unsigned HELD_MAX = 32;   // press order tracked for this many keys

	void scan();

	required_ioport_array<KEY_PORTS> m_keys;
	required_ioport m_modifiers;
	host *m_host;

	std::array<u8, HELD_MAX> m_held; // held usages, oldest press first
	u8 m_held_count;
	std::array<u8, REPORT> m_sent;   // the last report sent
};

DECLARE_DEVICE_TYPE(ZEPHYR80_USB_KEYBOARD, zephyr80_usb_keyboard_device)

#endif // MAME_PBITZ_ZEPHYR80_USB_KEYBOARD_H
