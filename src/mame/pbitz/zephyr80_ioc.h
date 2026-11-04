// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_H
#define MAME_PBITZ_ZEPHYR80_IOC_H

#pragma once

#include "zephyr80_ioc_core.h"
#include "zephyr80_ioc_lane.h"
#include "zephyr80_ioc_speed.h"

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
class zephyr80_ioc_device : public device_t, public zephyr80_ioc_lane_client, private zephyr80_ioc_core::link
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

protected:
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

	TIMER_CALLBACK_MEMBER(command_step);
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
	emu_timer *m_command_timer;

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
};

DECLARE_DEVICE_TYPE(ZEPHYR80_IOC, zephyr80_ioc_device)

#endif // MAME_PBITZ_ZEPHYR80_IOC_H
