// license:BSD-3-Clause
// copyright-holders:Kitamura

#ifndef MAME_PBITZ_PBITZ_ZSIO_H
#define MAME_PBITZ_PBITZ_ZSIO_H

#pragma once

#include <array>
#include <string>


class pbitz_zsio_device : public device_t
{
public:
	enum channel_mode : u8
	{
		MODE_ASYNC_EXTERNAL = 0,
		MODE_SYNC_INTERNAL
	};

	pbitz_zsio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto irq_callback() { return m_irq_cb.bind(); }

	void set_channel_config(unsigned channel, channel_mode mode, const char *tx_prefix, const char *boot_rx_data);

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr unsigned CHANNELS = 2;
	static constexpr unsigned RX_FIFO_SIZE = 64;

	static constexpr u8 STATUS_TX_READY = 0x01;
	static constexpr u8 STATUS_RX_READY = 0x02;
	static constexpr u8 STATUS_SYNC_INTERNAL = 0x04;
	static constexpr u8 STATUS_ASYNC_EXTERNAL = 0x08;
	static constexpr u8 STATUS_IRQ_PENDING = 0x80;

	static constexpr u8 CONTROL_RX_IRQ_ENABLE = 0x01;
	static constexpr u8 CONTROL_RESET_RX_FIFO = 0x02;
	static constexpr u8 CONTROL_IRQ_ACK = 0x80;

	struct channel_config
	{
		channel_mode mode = MODE_ASYNC_EXTERNAL;
		std::string tx_prefix;
		std::string boot_rx_data;
	};

	u8 status_r(unsigned channel) const;
	u8 data_r(unsigned channel);
	void control_w(unsigned channel, u8 data);
	void data_w(unsigned channel, u8 data);

	void rx_fifo_clear(unsigned channel);
	void rx_fifo_push(unsigned channel, u8 data);
	bool channel_irq_pending(unsigned channel) const;
	void update_irq();

	devcb_write_line m_irq_cb;
	std::array<channel_config, CHANNELS> m_config;
	std::array<std::array<u8, RX_FIFO_SIZE>, CHANNELS> m_rx_fifo = { };
	std::array<u8, CHANNELS> m_rx_head = { };
	std::array<u8, CHANNELS> m_rx_tail = { };
	std::array<u8, CHANNELS> m_rx_count = { };
	std::array<u8, CHANNELS> m_control = { };
	std::array<bool, CHANNELS> m_tx_line_start = { };
	bool m_irq_state = false;
};

DECLARE_DEVICE_TYPE(PBITZ_ZSIO, pbitz_zsio_device)

#endif // MAME_PBITZ_PBITZ_ZSIO_H
