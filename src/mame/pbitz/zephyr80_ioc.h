// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_H
#define MAME_PBITZ_ZEPHYR80_IOC_H

#pragma once

#include "zephyr80_ioc_core.h"
#include "zephyr80_ioc_lane.h"
#include "zephyr80_ioc_speed.h"
#include "zephyr80_usb_gamepad.h"
#include "zephyr80_usb_keyboard.h"
#include "zephyr80_usb_mouse.h"

// The IO controller as the Z80 sees it through SIO1: a behavioural model of
// the firmware's transport (Code/MCU/IOController src/main.c,
// src/external_sync.c, src/bulk_channel.c) driving the lanes, with the
// protocol in zephyr80_ioc_core.  Not a PIC18: no firmware runs, and nothing
// below the lane pins (the PIC, SPI2, the clock gates, USB, the card) is
// modelled.
//
// Command lane (SIO1/B), one service per request:
//   COMMAND_READY (/DCDB) is asserted while idle.  A request is /RTSB held
//   low (a level; it is serviced whenever it is found asserted while idle):
//   READY drops, the controller clocks the 36-byte request window (FFh
//   towards the Z80, the SIO's transmitter captured), requires /RTSB
//   released, finds the request (zephyr80_ioc_core::decode) and dispatches
//   it.  After the reply guard it clocks the reply, preceded by the
//   disposable establishing byte and the falling /SYNCB edge while the lane
//   is not yet synchronised, then one trailing FFh that pushes the last
//   reply byte out of the SIO's receive pipeline.  /SYNCB then stays low
//   until LINK_SYNC releases it.  A window with no valid request, or a
//   request not released in time, gets no reply.
// Bulk lane (SIO1/A), after a reply that armed a transfer (READY):
//   the controller waits for the host's /RTSA, asserts /CTSA (and /DCDA for
//   a transfer to the Z80), waits the admission guard, then clocks the
//   transfer: to the Z80 the establishing byte and falling /SYNCA edge while
//   that lane is not synchronised, the packet and one flush byte; from the
//   Z80 the capture window, whose packet the core then checks and commits.
//   /CTSA and /DCDA close after the bytes; a FS2_READ_RUN's further packets
//   each wait for /RTSA to be released and asserted again.  A host that
//   never asserts /RTSA fails the transfer (BULK_NO_HOST).
// READY returns only when the whole service, bulk phase and backend effect
// included, has ended.  /CTSB, the HID doorbell, follows the core's input
// queue.  CMD_RESET answers nothing and resets the machine.
//
// USB keyboard (the `keyboard` subdevice, zephyr80_usb_keyboard): its boot
// reports reach the HID module (zephyr80_ioc_hid) only in the controller's
// idle pass, as hid_host_task runs only between services: a report that
// arrives during a service waits for its end.  The HID module's millisecond
// clock is the firmware's 10 ms timebase, counted from the controller's
// reset; a held key's repeat runs at the first idle pass on or after its due
// tick.  Ctrl+Alt+Esc drops READY and resets the machine (handler_reset).
//
// USB mouse and gamepads (the `mouse` and `pad1`, `pad2` subdevices,
// zephyr80_usb_mouse, zephyr80_usb_gamepad): the idle pass also mounts or
// unmounts each as its configuration says, polls the mouse until it has
// nothing to report and each mounted pad when its 10 ms poll is due (the
// pad's report is its state at that instant), and feeds the reports to the
// HID module.  A mouse or pad that changes while a service runs is taken at
// the service's end; the pads' poll timer is the repeat's.  The controller
// latches the pads and the keyboard's number keys drive are read through
// controller().
//
// Timing is a short table of named delays (zephyr80_ioc.cpp); command
// processing itself takes no emulated time.
//
// Paced (`-ioc_speed paced`, the default) clocks every byte on the table's
// period.  Fast clocks a byte as soon as the other end can take it: a
// character to the Z80 when its receive FIFO has room, a character from the
// Z80 when its transmitter holds one.  A byte whose other end is not ready is
// clocked at a deadline: a character from the Z80 one paced period after the
// last, a character to the Z80 five (what a paced lane gives a guest before
// its first overrun), so a guest that stops overruns or underruns in both
// modes.  The guards (reply, admission) and a request window's length stay: a
// guest arms its receiver in the time they leave.  A fast slot never takes
// less than one yield, so a polling guest always runs.
//
// All state is save-registered and the scheduler saves the timer, so a state
// saved mid-transfer resumes it.  A reset cancels the timer and every phase.
class zephyr80_ioc_device : public device_t, public zephyr80_ioc_lane_client, private zephyr80_ioc_core::link,
	private zephyr80_usb_keyboard_device::host, private zephyr80_usb_mouse_device::host,
	private zephyr80_usb_gamepad_device::host
{
public:
	zephyr80_ioc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void set_lane(zephyr80_ioc_lane_port &lane) { m_lane = &lane; }
	// The `ioc_speed` slot: paced or fast.
	void set_speed(zephyr80_ioc_speed_slot_device &speed) { m_speed = &speed; }
	// The card socket's media (zephyr80_card_socket).
	void set_storage(zephyr80_ioc_storage &media) { m_core.set_storage(media); }

	// zephyr80_ioc_lane_client
	virtual void lane_rts_changed(zephyr80_ioc_lane lane, bool asserted) override;
	virtual void lane_data_event(zephyr80_ioc_lane lane) override;

	// The byte controller latch `port` (0 or 1) presents on a read of E0h-FFh.
	u8 controller(unsigned port) const { return m_core.hid().latch().value(port); }

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	enum : u8
	{
		IDLE,         // READY asserted, waiting for a request
		WINDOW,       // clocking the request window
		RELEASE,      // window done, waiting for /RTSB to rise
		GUARD,        // reply turnaround guard
		REPLY,        // clocking the reply
		BULK_HOST,    // waiting for the host's /RTSA
		BULK_GUARD,   // admission asserted, guard before the clock
		BULK_SEND,    // clocking a packet to the Z80
		BULK_RECEIVE, // clocking the capture window from the Z80
		BULK_RELEASE, // between a run's packets: waiting for /RTSA to rise
		RESETTING     // CMD_RESET: the host reset follows
	};

	// zephyr80_ioc_core::link
	virtual void resync_lanes() override;
	virtual bool command_lane_synced() const override { return m_command_synced; }
	virtual void hid_doorbell(bool asserted) override;
	virtual void host_reset() override;

	// zephyr80_usb_keyboard_device::host
	virtual void keyboard_report(u8 const *report, unsigned length) override;

	// zephyr80_usb_mouse_device::host, zephyr80_usb_gamepad_device::host
	virtual void usb_input_ready() override;

	TIMER_CALLBACK_MEMBER(command_step);
	TIMER_CALLBACK_MEMBER(hid_tick);
	void hid_service();
	void usb_sync(u32 now);
	u32 hid_now_ms() const;
	void poll_request();
	void serve();
	void after_reply();
	void bulk_admit();
	void bulk_step();
	void end_service();
	void set_ready(bool ready);
	u32 bulk_send_period(unsigned index) const;
	void next_byte(u32 period_us, zephyr80_ioc_lane lane, bool to_z80);
	bool byte_ready() const;
	void wake();

	zephyr80_ioc_lane_port *m_lane;
	zephyr80_ioc_speed_slot_device *m_speed;
	bool m_fast;           // -ioc_speed fast
	zephyr80_ioc_core m_core;
	required_device<zephyr80_usb_keyboard_device> m_keyboard;
	required_device<zephyr80_usb_mouse_device> m_mouse;
	required_device_array<zephyr80_usb_gamepad_device, zephyr80_ioc_hid::GAMEPADS> m_pads;
	emu_timer *m_command_timer;
	emu_timer *m_hid_timer;  // the repeat's due tick

	u8 m_phase;
	bool m_command_synced; // link_synced: /SYNCB is held low
	bool m_bulk_synced;    // bulk_synced: /SYNCA is held low
	bool m_waiting;        // fast: a byte slot waits for its counterpart
	u8 m_wait_lane;        // ... on this lane (zephyr80_ioc_lane)
	bool m_wait_to_z80;    // ... for room (to the Z80) or a loaded transmitter
	bool m_starved;        // fast: the other end missed a deadline in this transfer
	u8 m_count;            // window bytes clocked / reply bytes sent
	u8 m_reply_length;
	u8 m_window[36];
	u8 m_reply[zephyr80_ioc_core::MAX_WIRE];
	u16 m_bulk_count;      // bulk bytes clocked
	u16 m_bulk_total;      // bulk bytes to clock
	u8 m_bulk[zephyr80_ioc_core::BULK_WINDOW_MAX]; // the packet sent, or the window captured
	attotime m_hid_epoch;  // the controller's reset: its millisecond clock's zero
	u8 m_kbd_report[zephyr80_usb_keyboard_device::REPORT]; // a report waiting for the idle pass
	bool m_kbd_pending;
	u8 m_pad_slot[zephyr80_ioc_hid::GAMEPADS]; // the HID module's slot each pad took, or GAMEPAD_NONE
};

DECLARE_DEVICE_TYPE(ZEPHYR80_IOC, zephyr80_ioc_device)

#endif // MAME_PBITZ_ZEPHYR80_IOC_H
