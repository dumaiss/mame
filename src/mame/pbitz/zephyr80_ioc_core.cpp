// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc_core.h"

#include <algorithm>

namespace {

// include/ioc_frame.h
constexpr u8 SYNC0 = 0xa5;
constexpr u8 SYNC1 = 0x5a;
constexpr unsigned FIXED_LEN = 3;     // TYPE + SEQ + STATUS
constexpr unsigned OFF_CLASS = 0;
constexpr unsigned OFF_SEQ = 1;
constexpr unsigned OFF_STATUS = 2;
constexpr unsigned OFF_LEN = 3;
constexpr unsigned OFF_PAYLOAD = 4;

constexpr u8 CMD_PING = 0x01;
constexpr u8 CMD_RESET = 0x02;
constexpr u8 CMD_SD_READ = 0x03;
constexpr u8 CMD_BULK_TEST = 0x04;
constexpr u8 CMD_SD_READ_BULK = 0x05;
constexpr u8 CMD_XFER_STATUS = 0x06;
constexpr u8 CMD_SD_WRITE_BULK = 0x07;
constexpr u8 CMD_SD_READ_REC = 0x08;
constexpr u8 CMD_SD_WRITE_REC = 0x09;
constexpr u8 CMD_SD_FLUSH = 0x0a;
constexpr u8 CMD_PROFILE = 0x0b;
constexpr u8 CMD_LINK_SYNC = 0x0c;
constexpr u8 CMD_HID_STATUS = 0x0d;
constexpr u8 CMD_HID_INPUT = 0x0e;
constexpr u8 CMD_SD_READ_BLOCK = 0x0f;
constexpr u8 CMD_VOL_MOUNT = 0x10;
constexpr u8 CMD_VOL_INFO = 0x11;
constexpr u8 CMD_HID_MOUSE = 0x12;
constexpr u8 CMD_FS_OPENDIR = 0x20;
constexpr u8 CMD_FS_SELFTEST = 0x28;
constexpr u8 CMD_FS2_CAPS = 0x30;
constexpr u8 CMD_FS2_READ_RUN = 0x45;

constexpr u8 RSP_PING = 0x81;
constexpr u8 RSP_LINK_SYNC = 0x8c;
constexpr u8 RSP_UNKNOWN_COMMAND = 0xfe;

constexpr u8 IOC_STATUS_OK = 0x00;
constexpr u8 IOC_STATUS_UNKNOWN_CMD = 0x02;

constexpr u8 IOC_FW_LEVEL = 75;
constexpr unsigned IOC_OFF_PING_LEVEL = 20;
constexpr unsigned IOC_OFF_PING_POWER = 21;
constexpr unsigned IOC_OFF_PING_CALLS_LO = 22;
constexpr unsigned IOC_OFF_PING_ABORTS_LO = 24;
constexpr unsigned PING_ECHO_BYTES = 16;

// The PING power byte of a controller on a powered board with no shutdown
// request: /PWR_OFF high, latched high and driven; /SHUTDOWN_RQ high, no edge
// latched, its pull-up on (power_init); no stack-overflow reset.
constexpr u8 IOC_PING_PWR_OFF_PIN = 0x01;
constexpr u8 IOC_PING_PWR_OFF_LAT = 0x02;
constexpr u8 IOC_PING_PWR_OFF_DRIVEN = 0x04;
constexpr u8 IOC_PING_SHUTDOWN_PIN = 0x08;
constexpr u8 IOC_PING_SHUTDOWN_WPU = 0x20;
constexpr u8 IOC_PING_LINK_SYNCED = 0x40;
constexpr u8 PING_POWER_IDLE = IOC_PING_PWR_OFF_PIN | IOC_PING_PWR_OFF_LAT | IOC_PING_PWR_OFF_DRIVEN |
		IOC_PING_SHUTDOWN_PIN | IOC_PING_SHUTDOWN_WPU;

// external_sync.c is_command_class (normal profile, IOC_FS_COMMANDS = 1).
bool is_command_class(u8 value)
{
	switch (value)
	{
	case CMD_PING: case CMD_RESET: case CMD_SD_READ: case CMD_BULK_TEST:
	case CMD_SD_READ_BULK: case CMD_SD_WRITE_BULK: case CMD_SD_READ_REC:
	case CMD_SD_READ_BLOCK: case CMD_SD_WRITE_REC: case CMD_SD_FLUSH:
	case CMD_PROFILE: case CMD_LINK_SYNC: case CMD_HID_STATUS: case CMD_HID_INPUT:
	case CMD_HID_MOUSE: case CMD_XFER_STATUS: case CMD_VOL_MOUNT: case CMD_VOL_INFO:
		return true;
	default:
		return (value >= CMD_FS_OPENDIR && value <= CMD_FS_SELFTEST) ||
				(value >= CMD_FS2_CAPS && value <= CMD_FS2_READ_RUN);
	}
}

} // anonymous namespace

zephyr80_ioc_core::zephyr80_ioc_core(link &transport)
	: m_link(transport)
	, m_calls(0)
	, m_aborts(0)
{
}

void zephyr80_ioc_core::reset()
{
	m_calls = 0;
	m_aborts = 0;
}

void zephyr80_ioc_core::register_save(device_t &owner)
{
	owner.save_item(NAME(m_calls));
	owner.save_item(NAME(m_aborts));
}

u16 zephyr80_ioc_core::crc16(u16 crc, u8 data)
{
	crc ^= u16(data) << 8;
	for (int i = 0; i < 8; i++)
		crc = (crc & 0x8000) ? u16((crc << 1) ^ 0x1021) : u16(crc << 1);
	return crc;
}

// packet_at: the marker, a command-lane length, the whole packet inside the
// window, an admitted class, a zero request STATUS and the CRC.
bool zephyr80_ioc_core::packet_at(const u8 *packet, unsigned available, frame &request) const
{
	if (available < 2 + 2 + FIXED_LEN + 2 || packet[0] != SYNC0 || packet[1] != SYNC1)
		return false;
	unsigned const length = packet[2] | (packet[3] << 8);
	if (length < FIXED_LEN || length > FIXED_LEN + MAX_DATA || 2 + 2 + length + 2 > available)
		return false;
	u8 const type = packet[4];
	if (!is_command_class(type) || packet[6] != IOC_STATUS_OK)
		return false;

	u16 crc = 0;
	for (unsigned i = 2; i < 4 + length; i++)
		crc = crc16(crc, packet[i]);
	if (crc != ((packet[4 + length] << 8) | packet[5 + length]))
		return false;

	request.fill(0);
	request[OFF_CLASS] = type;
	request[OFF_SEQ] = packet[5];
	request[OFF_STATUS] = packet[6];
	request[OFF_LEN] = length - FIXED_LEN;
	std::copy_n(packet + 7, length - FIXED_LEN, request.begin() + OFF_PAYLOAD);
	return true;
}

// copy_received_frame: a window of marking idle holds no request; otherwise
// the first position where a whole valid packet starts.
bool zephyr80_ioc_core::decode(const u8 *window, unsigned length, frame &request) const
{
	if (std::all_of(window, window + length, [] (u8 b) { return b == 0xff; }))
		return false;
	for (unsigned start = 0; start < length; start++)
		if (packet_at(window + start, length - start, request))
			return true;
	return false;
}

void zephyr80_ioc_core::ping(const frame &request, frame &reply) const
{
	reply[OFF_LEN] = MAX_DATA;
	std::copy_n(request.begin() + OFF_PAYLOAD, PING_ECHO_BYTES, reply.begin() + OFF_PAYLOAD);
	reply[IOC_OFF_PING_LEVEL] = IOC_FW_LEVEL;
	reply[IOC_OFF_PING_POWER] = PING_POWER_IDLE | (m_link.command_lane_synced() ? IOC_PING_LINK_SYNCED : 0);
	reply[IOC_OFF_PING_CALLS_LO] = u8(m_calls);
	reply[IOC_OFF_PING_CALLS_LO + 1] = u8(m_calls >> 8);
	reply[IOC_OFF_PING_ABORTS_LO] = u8(m_aborts);
	reply[IOC_OFF_PING_ABORTS_LO + 1] = u8(m_aborts >> 8);
	// Bytes 26-29 are diagnostic-profile edge counts and read zero.
}

void zephyr80_ioc_core::dispatch(const frame &request, frame &reply)
{
	reply.fill(0);
	reply[OFF_SEQ] = request[OFF_SEQ];
	reply[OFF_STATUS] = IOC_STATUS_OK;
	switch (request[OFF_CLASS])
	{
	case CMD_PING:
		reply[OFF_CLASS] = RSP_PING;
		ping(request, reply);
		break;

	case CMD_LINK_SYNC:
		// Both lanes, before the reply: the reply re-establishes the
		// command lane's boundary.
		m_link.resync_lanes();
		reply[OFF_CLASS] = RSP_LINK_SYNC;
		break;

	default:
		reply[OFF_CLASS] = RSP_UNKNOWN_COMMAND;
		reply[OFF_STATUS] = IOC_STATUS_UNKNOWN_CMD;
		break;
	}
}

// external_sync_send: the reply packet, CRC high byte first.
unsigned zephyr80_ioc_core::encode(const frame &reply, u8 *wire)
{
	unsigned const data = std::min<unsigned>(reply[OFF_LEN], MAX_DATA);
	unsigned const length = FIXED_LEN + data;
	unsigned n = 0;
	wire[n++] = SYNC0;
	wire[n++] = SYNC1;
	wire[n++] = u8(length);
	wire[n++] = u8(length >> 8);
	wire[n++] = reply[OFF_CLASS];
	wire[n++] = reply[OFF_SEQ];
	wire[n++] = reply[OFF_STATUS];
	for (unsigned i = 0; i < data; i++)
		wire[n++] = reply[OFF_PAYLOAD + i];
	u16 crc = 0;
	for (unsigned i = 2; i < n; i++)
		crc = crc16(crc, wire[i]);
	wire[n++] = u8(crc >> 8);
	wire[n++] = u8(crc);
	return n;
}
