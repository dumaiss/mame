// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_zsio.h"


DEFINE_DEVICE_TYPE(PBITZ_ZSIO, pbitz_zsio_device, "pbitz_zsio", "pBITz behavioral Zephyr SIO")

pbitz_zsio_device::pbitz_zsio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, PBITZ_ZSIO, tag, owner, clock)
	, m_irq_cb(*this)
{
}

void pbitz_zsio_device::set_channel_config(unsigned channel, channel_mode mode, const char *tx_prefix, const char *boot_rx_data)
{
	if (channel >= CHANNELS)
		return;

	m_config[channel].mode = mode;
	m_config[channel].tx_prefix = tx_prefix ? tx_prefix : "";
	m_config[channel].boot_rx_data = boot_rx_data ? boot_rx_data : "";
}

void pbitz_zsio_device::device_start()
{
	save_item(NAME(m_rx_fifo));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_tail));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_control));
	save_item(NAME(m_tx_line_start));
	save_item(NAME(m_irq_state));
}

void pbitz_zsio_device::device_reset()
{
	// Behavioral frontend for Zephyr's two Z80 SIO chips.  This deliberately
	// does not emulate Z80 SIO register sequencing, baud clocks, sync framing,
	// or the RX660/PIC-side protocol yet.
	m_irq_state = false;
	m_irq_cb(CLEAR_LINE);

	for (unsigned channel = 0; channel < CHANNELS; channel++)
	{
		m_control[channel] = 0;
		m_tx_line_start[channel] = true;
		rx_fifo_clear(channel);

		for (char const ch : m_config[channel].boot_rx_data)
			rx_fifo_push(channel, u8(ch));
	}

	update_irq();
}

u8 pbitz_zsio_device::read(offs_t offset)
{
	unsigned const channel = BIT(offset, 1);
	if (BIT(offset, 0))
		return data_r(channel);

	return status_r(channel);
}

void pbitz_zsio_device::write(offs_t offset, u8 data)
{
	unsigned const channel = BIT(offset, 1);
	if (BIT(offset, 0))
		data_w(channel, data);
	else
		control_w(channel, data);
}

u8 pbitz_zsio_device::status_r(unsigned channel) const
{
	u8 status = STATUS_TX_READY;

	if (m_rx_count[channel])
		status |= STATUS_RX_READY;

	if (m_config[channel].mode == MODE_SYNC_INTERNAL)
		status |= STATUS_SYNC_INTERNAL;
	else
		status |= STATUS_ASYNC_EXTERNAL;

	if (channel_irq_pending(channel))
		status |= STATUS_IRQ_PENDING;

	return status;
}

u8 pbitz_zsio_device::data_r(unsigned channel)
{
	if (!m_rx_count[channel])
		return 0x00;

	u8 const data = m_rx_fifo[channel][m_rx_head[channel]];
	m_rx_head[channel] = (m_rx_head[channel] + 1) % RX_FIFO_SIZE;
	m_rx_count[channel]--;
	update_irq();

	return data;
}

void pbitz_zsio_device::control_w(unsigned channel, u8 data)
{
	m_control[channel] = data & CONTROL_RX_IRQ_ENABLE;

	if (data & CONTROL_RESET_RX_FIFO)
		rx_fifo_clear(channel);

	if (data & CONTROL_IRQ_ACK)
	{
		m_irq_cb(CLEAR_LINE);
		m_irq_state = false;
	}

	update_irq();
}

void pbitz_zsio_device::data_w(unsigned channel, u8 data)
{
	if (m_tx_line_start[channel] && !m_config[channel].tx_prefix.empty())
		osd_printf_info("%s ", m_config[channel].tx_prefix);

	osd_printf_info("%c", data);
	m_tx_line_start[channel] = (data == '\r') || (data == '\n');
}

void pbitz_zsio_device::rx_fifo_clear(unsigned channel)
{
	m_rx_head[channel] = 0;
	m_rx_tail[channel] = 0;
	m_rx_count[channel] = 0;
	update_irq();
}

void pbitz_zsio_device::rx_fifo_push(unsigned channel, u8 data)
{
	if (m_rx_count[channel] == RX_FIFO_SIZE)
		return;

	m_rx_fifo[channel][m_rx_tail[channel]] = data;
	m_rx_tail[channel] = (m_rx_tail[channel] + 1) % RX_FIFO_SIZE;
	m_rx_count[channel]++;
	update_irq();
}

bool pbitz_zsio_device::channel_irq_pending(unsigned channel) const
{
	return bool(m_control[channel] & CONTROL_RX_IRQ_ENABLE) && m_rx_count[channel];
}

void pbitz_zsio_device::update_irq()
{
	bool const irq_state = channel_irq_pending(0) || channel_irq_pending(1);
	if (irq_state != m_irq_state)
	{
		m_irq_state = irq_state;
		m_irq_cb(m_irq_state ? ASSERT_LINE : CLEAR_LINE);
	}
}
