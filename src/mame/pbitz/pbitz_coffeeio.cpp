// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_coffeeio.h"


DEFINE_DEVICE_TYPE(PBITZ_COFFEEIO, pbitz_coffeeio_device, "pbitz_coffeeio", "pBITz COFFEE-IO protocol stub")

pbitz_coffeeio_device::pbitz_coffeeio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, PBITZ_COFFEEIO, tag, owner, clock)
	, m_response_a_cb(*this)
{
}

void pbitz_coffeeio_device::device_start()
{
	save_item(NAME(m_state));
	save_item(NAME(m_command));
	save_item(NAME(m_length));
	save_item(NAME(m_payload_index));
	save_item(NAME(m_checksum));
	save_item(NAME(m_payload));
}

void pbitz_coffeeio_device::device_reset()
{
	reset_parser();
}

void pbitz_coffeeio_device::rx_a_w(u8 data)
{
	logerror("%s: SIO1-A RX byte %02x\n", tag(), data);

	switch (m_state)
	{
	case WAIT_SYNC:
		if (data == REQUEST_SYNC)
			m_state = CMD;
		else
			logerror("%s: dropping byte before sync: %02x\n", tag(), data);
		break;

	case CMD:
		m_command = data;
		m_checksum = data;
		m_state = LEN;
		break;

	case LEN:
		m_length = data;
		m_payload_index = 0;
		m_checksum += data;
		m_state = m_length ? PAYLOAD : CHK;
		break;

	case PAYLOAD:
		m_payload[m_payload_index++] = data;
		m_checksum += data;
		if (m_payload_index == m_length)
			m_state = CHK;
		break;

	case CHK:
		if (data == m_checksum)
			handle_packet();
		else
		{
			logerror("%s: bad checksum for command %02x: got %02x expected %02x\n", tag(), m_command, data, m_checksum);
			send_error_response(m_command);
		}
		reset_parser();
		break;
	}
}

void pbitz_coffeeio_device::rx_b_w(u8 data)
{
	// Reserved for a later split command/data or second-service lane.  Phase 10
	// keeps it visible for diagnostics but does not implement a B-side protocol.
	logerror("%s: SIO1-B reserved byte %02x\n", tag(), data);
}

void pbitz_coffeeio_device::reset_parser()
{
	m_state = WAIT_SYNC;
	m_command = 0;
	m_length = 0;
	m_payload_index = 0;
	m_checksum = 0;
}

void pbitz_coffeeio_device::handle_packet()
{
	switch (m_command)
	{
	case CMD_PING:
		if (m_length == 0)
			send_response_text(CMD_PING, "OK");
		else
			send_error_response(m_command);
		break;

	case CMD_ECHO:
		send_response(CMD_ECHO, m_payload.data(), m_length);
		break;

	case CMD_GET_VERSION:
		if (m_length == 0)
			send_response_text(CMD_GET_VERSION, "COFFEEIO-0");
		else
			send_error_response(m_command);
		break;

	default:
		logerror("%s: unknown command %02x length %u\n", tag(), m_command, m_length);
		send_error_response(m_command);
		break;
	}
}

void pbitz_coffeeio_device::send_response(u8 command, u8 const *payload, u8 length)
{
	u8 checksum = command + length;

	logerror("%s: response command %02x length %u\n", tag(), command, length);
	m_response_a_cb(RESPONSE_SYNC);
	m_response_a_cb(command);
	m_response_a_cb(length);

	for (u8 index = 0; index < length; index++)
	{
		u8 const data = payload[index];
		checksum += data;
		m_response_a_cb(data);
	}

	m_response_a_cb(checksum);
}

void pbitz_coffeeio_device::send_response_text(u8 command, char const *payload)
{
	u8 length = 0;
	while (payload[length])
		length++;

	send_response(command, reinterpret_cast<u8 const *>(payload), length);
}

void pbitz_coffeeio_device::send_error_response(u8 command)
{
	static constexpr u8 ERROR_PAYLOAD[] = { 'E', 'R', 'R' };
	send_response(command, ERROR_PAYLOAD, std::size(ERROR_PAYLOAD));
}
