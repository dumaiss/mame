// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_LANE_H
#define MAME_PBITZ_ZEPHYR80_IOC_LANE_H

#pragma once

// The serial side of SIO1 as the IO controller drives it.  Channel A is the
// bulk lane (ports 30h/31h), channel B the command lane (32h/33h).  The
// controller is the clock master on both: every character moves on its clock,
// one full-duplex byte per call, so nothing reaches or leaves the CPU side
// without it.
enum class zephyr80_ioc_lane : unsigned { BULK = 0, COMMAND = 1 };

// What the controller drives and samples on a lane.
class zephyr80_ioc_lane_port
{
public:
	// One character time on the lane's x1 clock: `to_z80` is shifted into the
	// receiver, and the result is what the transmitter shifted out (marking
	// FFh while it is disabled).
	virtual u8 lane_clock(zephyr80_ioc_lane lane, u8 to_z80) = 0;

	// The controller's outputs into the SIO, as asserted (low) levels.
	virtual void lane_sync(zephyr80_ioc_lane lane, bool asserted) = 0; // /SYNC
	virtual void lane_dcd(zephyr80_ioc_lane lane, bool asserted) = 0;  // /DCD
	virtual void lane_cts(zephyr80_ioc_lane lane, bool asserted) = 0;  // /CTS

	// The SIO's /RTS output, as an asserted (low) level.
	virtual bool lane_rts(zephyr80_ioc_lane lane) const = 0;

protected:
	~zephyr80_ioc_lane_port() = default;
};

// Status notifications from the SIO to the controller.
class zephyr80_ioc_lane_client
{
public:
	virtual void lane_rts_changed(zephyr80_ioc_lane lane, bool asserted) = 0;

protected:
	~zephyr80_ioc_lane_client() = default;
};

#endif // MAME_PBITZ_ZEPHYR80_IOC_LANE_H
