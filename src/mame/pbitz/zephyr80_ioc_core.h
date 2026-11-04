// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_CORE_H
#define MAME_PBITZ_ZEPHYR80_IOC_CORE_H

#pragma once

#include <array>

// The IO controller's command protocol, independent of any transport: the
// request frame found in a captured command window, its dispatch, and the
// reply packet.  The specification is the controller firmware
// (Code/MCU/IOController): include/ioc_frame.h for the packet, the classes
// and the layouts, src/external_sync.c for request validation
// (packet_at, is_command_class), src/main.c service_command_request for the
// service counters and src/handlers.c for the replies.
//
//   A5 5A LEN_LO LEN_HI TYPE SEQ STATUS DATA... CRC_HI CRC_LO
//
// LEN counts TYPE + SEQ + STATUS + DATA (3..29 on the command lane); the CRC
// is CRC-16-CCITT (1021h, preset 0, MSB-first, no final XOR) over LEN_LO
// through the last DATA byte.  Frames travel as the firmware's 32-byte
// compatibility mailbox: TYPE, SEQ, STATUS, DATA length, DATA.
//
// Served: PING (handler_ping) and LINK_SYNC (handler_link_sync).  Every other
// class the firmware's request gate admits is answered as the firmware
// answers a class it does not serve (handler_unknown): RSP_UNKNOWN_COMMAND
// (FEh) with IOC_STATUS_UNKNOWN_CMD.  A window holding no valid request gets
// no reply, as on the board.
class zephyr80_ioc_core
{
public:
	using frame = std::array<u8, 32>;

	static constexpr unsigned FRAME_SIZE = 32;
	static constexpr unsigned MAX_DATA = 26;           // IOC_COMMAND_MAX_DATA
	static constexpr unsigned MAX_WIRE = 2 + 2 + 3 + MAX_DATA + 2;

	// What the protocol asks of the transport it runs on.
	class link
	{
	public:
		// Both lanes drop their character boundary: the next transfer on
		// each re-establishes it (external_sync_request_resync and
		// bulk_channel_request_resync).
		virtual void resync_lanes() = 0;
		// The command lane's boundary is established (PING reports it).
		virtual bool command_lane_synced() const = 0;

	protected:
		~link() = default;
	};

	explicit zephyr80_ioc_core(link &transport);

	void reset();
	void register_save(device_t &owner);

	// One command service: begin() counts it, then either abort() (the
	// request was not released or did not decode) or decode() + dispatch()
	// + encode().
	void begin() { m_calls++; }
	void abort() { m_aborts++; }
	bool decode(const u8 *window, unsigned length, frame &request) const;
	void dispatch(const frame &request, frame &reply);
	static unsigned encode(const frame &reply, u8 *wire);

	static u16 crc16(u16 crc, u8 data);

private:
	bool packet_at(const u8 *packet, unsigned available, frame &request) const;
	void ping(const frame &request, frame &reply) const;

	link &m_link;
	u16 m_calls;  // svc_calls
	u16 m_aborts; // svc_aborts
};

#endif // MAME_PBITZ_ZEPHYR80_IOC_CORE_H
