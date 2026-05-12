// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_serial.h"


DEFINE_DEVICE_TYPE(PBITZ_SERIAL, pbitz_serial_device, "pbitz_serial", "pBITz polling serial proof")

namespace {

static constexpr char BOOT_RX_DATA[] = "help\r";

} // anonymous namespace

pbitz_serial_device::pbitz_serial_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, PBITZ_SERIAL, tag, owner, clock)
	, m_irq_cb(*this)
{
}

void pbitz_serial_device::device_start()
{
	save_item(NAME(m_rx_fifo));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_tail));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_control));
	save_item(NAME(m_irq_state));
}

void pbitz_serial_device::device_reset()
{
	// This is a behavioral polling/IRQ serial device for early Zephyr software
	// validation.  It is not a hardware-accurate UART or Z80 SIO model, and
	// the interrupt behavior is deliberately a simple RX-ready level.
	m_control = 0;
	m_irq_state = false;
	m_irq_cb(CLEAR_LINE);

	rx_fifo_clear();

	for (char const ch : BOOT_RX_DATA)
	{
		if (ch)
			rx_fifo_push(u8(ch));
	}

	update_irq();
}

u8 pbitz_serial_device::status_r()
{
	// TX is always ready for now.  RX ready tracks the internal FIFO state.
	return STATUS_TX_READY | (m_rx_count ? STATUS_RX_READY : 0);
}

u8 pbitz_serial_device::data_r()
{
	if (!m_rx_count)
		return 0x00;

	u8 const data = m_rx_fifo[m_rx_head];
	m_rx_head = (m_rx_head + 1) % RX_FIFO_SIZE;
	m_rx_count--;
	update_irq();

	return data;
}

void pbitz_serial_device::data_w(u8 data)
{
	osd_printf_info("%c", data);
}

u8 pbitz_serial_device::control_r()
{
	return m_control;
}

void pbitz_serial_device::control_w(u8 data)
{
	// Bit 1 is reserved for a future TX IRQ path and is only mirrored here.
	// Bit 7 is accepted as a software ACK, but RX IRQ is level-like: if RX IRQ
	// remains enabled and FIFO data is still waiting, the line stays asserted.
	m_control = data & (CONTROL_RX_IRQ_ENABLE | CONTROL_TX_IRQ_ENABLE);
	if (data & CONTROL_IRQ_ACK)
	{
		m_irq_cb(CLEAR_LINE);
		m_irq_state = false;
	}

	update_irq();
}

void pbitz_serial_device::rx_fifo_clear()
{
	m_rx_head = 0;
	m_rx_tail = 0;
	m_rx_count = 0;
	update_irq();
}

void pbitz_serial_device::rx_fifo_push(u8 data)
{
	if (m_rx_count == RX_FIFO_SIZE)
		return;

	m_rx_fifo[m_rx_tail] = data;
	m_rx_tail = (m_rx_tail + 1) % RX_FIFO_SIZE;
	m_rx_count++;
	update_irq();
}

void pbitz_serial_device::update_irq()
{
	bool const irq_state = bool(m_control & CONTROL_RX_IRQ_ENABLE) && m_rx_count;
	if (irq_state != m_irq_state)
	{
		m_irq_state = irq_state;
		m_irq_cb(m_irq_state ? ASSERT_LINE : CLEAR_LINE);
	}
}
