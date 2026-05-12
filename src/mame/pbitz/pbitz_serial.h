// license:BSD-3-Clause
// copyright-holders:Kitamura

#ifndef MAME_PBITZ_PBITZ_SERIAL_H
#define MAME_PBITZ_PBITZ_SERIAL_H

#pragma once

#include <array>


class pbitz_serial_device : public device_t
{
public:
	pbitz_serial_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto irq_callback() { return m_irq_cb.bind(); }

	u8 status_r();
	u8 data_r();
	void data_w(u8 data);
	u8 control_r();
	void control_w(u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr u8 STATUS_TX_READY = 0x01;
	static constexpr u8 STATUS_RX_READY = 0x02;
	static constexpr u8 CONTROL_RX_IRQ_ENABLE = 0x01;
	static constexpr u8 CONTROL_TX_IRQ_ENABLE = 0x02;
	static constexpr u8 CONTROL_IRQ_ACK = 0x80;
	static constexpr unsigned RX_FIFO_SIZE = 64;

	void rx_fifo_clear();
	void rx_fifo_push(u8 data);
	void update_irq();

	devcb_write_line m_irq_cb;
	std::array<u8, RX_FIFO_SIZE> m_rx_fifo = { };
	u8 m_rx_head = 0;
	u8 m_rx_tail = 0;
	u8 m_rx_count = 0;
	u8 m_control = 0;
	bool m_irq_state = false;
};

DECLARE_DEVICE_TYPE(PBITZ_SERIAL, pbitz_serial_device)

#endif // MAME_PBITZ_PBITZ_SERIAL_H
