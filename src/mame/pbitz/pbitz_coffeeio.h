// license:BSD-3-Clause
// copyright-holders:Kitamura

#ifndef MAME_PBITZ_PBITZ_COFFEEIO_H
#define MAME_PBITZ_PBITZ_COFFEEIO_H

#pragma once

#include <array>


class pbitz_coffeeio_device : public device_t
{
public:
	pbitz_coffeeio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto response_a_callback() { return m_response_a_cb.bind(); }

	void rx_a_w(u8 data);
	void rx_b_w(u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr u8 REQUEST_SYNC = 0x55;
	static constexpr u8 RESPONSE_SYNC = 0xaa;

	static constexpr u8 CMD_PING = 0x00;
	static constexpr u8 CMD_ECHO = 0x01;
	static constexpr u8 CMD_GET_VERSION = 0x02;

	enum parse_state : u8
	{
		WAIT_SYNC = 0,
		CMD,
		LEN,
		PAYLOAD,
		CHK
	};

	void reset_parser();
	void handle_packet();
	void send_response(u8 command, u8 const *payload, u8 length);
	void send_response_text(u8 command, char const *payload);
	void send_error_response(u8 command);

	devcb_write8 m_response_a_cb;

	u8 m_state = WAIT_SYNC;
	u8 m_command = 0;
	u8 m_length = 0;
	u8 m_payload_index = 0;
	u8 m_checksum = 0;
	std::array<u8, 256> m_payload = { };
};

DECLARE_DEVICE_TYPE(PBITZ_COFFEEIO, pbitz_coffeeio_device)

#endif // MAME_PBITZ_PBITZ_COFFEEIO_H
