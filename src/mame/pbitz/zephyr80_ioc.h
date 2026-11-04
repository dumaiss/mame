// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_H
#define MAME_PBITZ_ZEPHYR80_IOC_H

#pragma once

#include "zephyr80_ioc_core.h"
#include "zephyr80_ioc_lane.h"

// The IO controller as the Z80 sees it through SIO1: a behavioural model of
// the firmware's command transport (Code/MCU/IOController src/main.c and
// src/external_sync.c) driving the lanes, with the protocol in
// zephyr80_ioc_core.  Not a PIC18: no firmware runs, and nothing below the
// lane pins (the PIC, SPI2, the clock gates, USB, the card) is modelled.
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
//   until LINK_SYNC releases it.  READY returns when the service ends.  A
//   window with no valid request, or a request not released in time, gets
//   no reply.
// HID doorbell (/CTSB) and bulk admission (/DCDA, /CTSA) stay deasserted:
// no HID queue or bulk transfer exists in this model yet.
//
// Timing is a short table of named delays (zephyr80_ioc.cpp); command
// processing itself takes no emulated time.
class zephyr80_ioc_device : public device_t, public zephyr80_ioc_lane_client, private zephyr80_ioc_core::link
{
public:
	zephyr80_ioc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void set_lane(zephyr80_ioc_lane_port &lane) { m_lane = &lane; }

	// zephyr80_ioc_lane_client
	virtual void lane_rts_changed(zephyr80_ioc_lane lane, bool asserted) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	enum : u8
	{
		IDLE,     // READY asserted, waiting for a request
		WINDOW,   // clocking the request window
		RELEASE,  // window done, waiting for /RTSB to rise
		GUARD,    // reply turnaround guard
		REPLY     // clocking the reply
	};

	// zephyr80_ioc_core::link
	virtual void resync_lanes() override;
	virtual bool command_lane_synced() const override { return m_command_synced; }

	TIMER_CALLBACK_MEMBER(command_step);
	void poll_request();
	void serve();
	void end_service();
	void set_ready(bool ready);

	zephyr80_ioc_lane_port *m_lane;
	zephyr80_ioc_core m_core;
	emu_timer *m_command_timer;

	u8 m_phase;
	bool m_command_synced; // link_synced: /SYNCB is held low
	bool m_bulk_synced;    // the bulk lane's /SYNCA likewise
	u8 m_count;            // window bytes clocked / reply bytes sent
	u8 m_reply_length;
	u8 m_window[36];
	u8 m_reply[zephyr80_ioc_core::MAX_WIRE];
};

DECLARE_DEVICE_TYPE(ZEPHYR80_IOC, zephyr80_ioc_device)

#endif // MAME_PBITZ_ZEPHYR80_IOC_H
