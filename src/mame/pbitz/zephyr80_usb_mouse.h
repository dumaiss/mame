// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_USB_MOUSE_H
#define MAME_PBITZ_ZEPHYR80_USB_MOUSE_H

#pragma once

#include <array>

// A USB mouse as the IO controller's HID host receives it: the dongle's second
// HID interface, whose reports the firmware decodes (ioc_hid.c, report ID 02):
//
//   byte 0 = 02h, 1 = buttons (D0 left, D1 right, D2 middle),
//   2..4 = X and Y as two 12-bit signed fields (X low byte; X high nibble
//          with Y low nibble above it; Y high byte), 5 = wheel (signed),
//   6 = horizontal scroll (always 0).
//
// MAME's host mouse reaches it through three analog fields (X, Y, wheel; each
// a position that wraps at 16 bits) and three buttons.  Once a frame, after MAME's input
// update, the mouse folds that frame into what a mouse holds between polls: a
// motion total per axis, bounded at +/-32767 like the firmware's own, and the
// button states in the order they were seen (a ring of eight; a state beyond
// that replaces the newest).  The controller polls it in its idle pass: poll()
// builds one report with the oldest button state and up to +/-2047 of X and Y
// and +/-127 of wheel, leaving the rest for the next, so a click never
// vanishes between polls and a long throw arrives whole.  A mouse reports
// only on change: with nothing moved or pressed, poll() answers nothing, and
// the firmware's decode flag stays clear until the first report.
//
// The USB mouse can be disconnected (the "USB mouse" configuration): its
// state is then dropped and the controller unmounts the interface.  The
// motion total, the ring and the button states are save-registered.  USB
// enumeration, transfers and the poll interval are not modelled.
class zephyr80_usb_mouse_device : public device_t
{
public:
	static constexpr unsigned REPORT = 7;           // MOUSE_REPORT_LEN
	static constexpr unsigned RING = 8;
	static constexpr s32 AXIS_MAX = 2047;           // a 12-bit signed field
	static constexpr s32 WHEEL_MAX = 127;
	static constexpr s32 TOTAL_MAX = 32767;

	// The controller the mouse reports to.
	class host
	{
	public:
		// The mouse has something to poll, or was connected or disconnected.
		virtual void usb_input_ready() = 0;

	protected:
		~host() = default;
	};

	zephyr80_usb_mouse_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void set_host(host &receiver) { m_host = &receiver; }

	bool connected() const;
	// Connected as of the last scan: what the controller has mounted.
	bool attached() const { return m_was_connected; }
	// The next report (REPORT bytes), or false when nothing changed.
	bool poll(u8 *report);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	void scan();
	void push_buttons(u8 state);

	required_ioport m_x;
	required_ioport m_y;
	required_ioport m_wheel_port;
	required_ioport m_buttons;
	required_ioport m_config;
	host *m_host;

	u16 m_last_x, m_last_y, m_last_wheel;  // the fields at the last scan
	bool m_synced;                         // m_last_* hold a scan since the reset
	s32 m_dx, m_dy, m_wheel;               // motion not yet polled
	std::array<u8, RING> m_ring;           // button states not yet polled, oldest first
	u8 m_ring_count;
	u8 m_scanned;                          // the buttons at the last scan
	u8 m_sent;                             // the buttons in the last report
	bool m_was_connected;
};

DECLARE_DEVICE_TYPE(ZEPHYR80_USB_MOUSE, zephyr80_usb_mouse_device)

#endif // MAME_PBITZ_ZEPHYR80_USB_MOUSE_H
