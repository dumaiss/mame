// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_sio1.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_SIO1, zephyr80_sio1_device, "zephyr80_sio1", "Zephyr-80 SIO1 IO-controller lanes")

namespace {

// RR0
constexpr u8 RR0_RX_AVAILABLE = 0x01;
constexpr u8 RR0_TX_EMPTY     = 0x04;
constexpr u8 RR0_DCD          = 0x08;
constexpr u8 RR0_SYNC_HUNT    = 0x10;
constexpr u8 RR0_CTS          = 0x20;
constexpr u8 RR0_TX_EOM       = 0x40;
// RR1
constexpr u8 RR1_ALL_SENT     = 0x01; // always set in a synchronous mode
constexpr u8 RR1_RX_OVERRUN   = 0x20;
// WR3
constexpr u8 WR3_RX_ENABLE    = 0x01;
constexpr u8 WR3_ENTER_HUNT   = 0x10;
constexpr u8 WR3_AUTO_ENABLES = 0x20;
// WR4: External Sync, x1 clock, no parity: the one mode both lanes use.
constexpr u8 WR4_EXTSYNC_X1   = 0x30;
// WR5
constexpr u8 WR5_TX_CRC       = 0x01;
constexpr u8 WR5_RTS          = 0x02;
constexpr u8 WR5_CRC16        = 0x04;
constexpr u8 WR5_TX_ENABLE    = 0x08;

// Unmodelled features, each logged once.
constexpr u8 WARN_INTERRUPTS  = 0x01;
constexpr u8 WARN_MODE        = 0x02;
constexpr u8 WARN_AUTO        = 0x04;
constexpr u8 WARN_REGISTER    = 0x08;

constexpr unsigned channel_of(zephyr80_ioc_lane lane) { return unsigned(lane); }
constexpr zephyr80_ioc_lane lane_of(unsigned ch) { return zephyr80_ioc_lane(ch); }

} // anonymous namespace

zephyr80_sio1_device::zephyr80_sio1_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_SIO1, tag, owner, clock)
	, m_client(nullptr)
{
}

void zephyr80_sio1_device::device_start()
{
	for (unsigned ch = 0; ch < 2; ch++)
	{
		std::fill(std::begin(m_wr[ch]), std::end(m_wr[ch]), 0);
		m_dcd[ch] = m_cts[ch] = m_sync[ch] = m_rts[ch] = false;
		m_data[ch] = 0xff;
	}
	m_warned = 0;

	save_item(NAME(m_wr));
	save_item(NAME(m_pointer));
	save_item(NAME(m_hunt));
	save_item(NAME(m_pipe_full));
	save_item(NAME(m_pipe));
	save_item(NAME(m_fifo));
	save_item(NAME(m_fifo_depth));
	save_item(NAME(m_data));
	save_item(NAME(m_overrun));
	save_item(NAME(m_tx_full));
	save_item(NAME(m_tx_buffer));
	save_item(NAME(m_eom));
	save_item(NAME(m_crc));
	save_item(NAME(m_crc_left));
	save_item(NAME(m_dcd));
	save_item(NAME(m_cts));
	save_item(NAME(m_sync));
	save_item(NAME(m_rts));
	save_item(NAME(m_latched));
	save_item(NAME(m_ext));
	save_item(NAME(m_warned));
}

// /RESET: both channels as after a channel reset, every register cleared.  The
// controller's lines are its own outputs and keep their levels.
void zephyr80_sio1_device::device_reset()
{
	for (unsigned ch = 0; ch < 2; ch++)
	{
		std::fill(std::begin(m_wr[ch]), std::end(m_wr[ch]), 0);
		channel_reset(ch);
	}
}

// WR0 18h: receiver and transmitter disabled, Hunt, FIFO and pipeline empty,
// errors cleared, the EOM latch set, /RTS released, the pointer at 0.
void zephyr80_sio1_device::channel_reset(unsigned ch)
{
	m_wr[ch][1] = 0;
	m_wr[ch][3] &= ~WR3_RX_ENABLE;
	m_wr[ch][5] = 0;
	m_pointer[ch] = 0;
	m_hunt[ch] = true;
	m_pipe_full[ch] = false;
	m_pipe[ch] = 0xff;
	std::fill(std::begin(m_fifo[ch]), std::end(m_fifo[ch]), 0xff);
	m_fifo_depth[ch] = 0;
	m_overrun[ch] = false;
	m_tx_full[ch] = false;
	m_tx_buffer[ch] = 0xff;
	m_eom[ch] = true;
	m_crc[ch] = 0;
	m_crc_left[ch] = 0;
	m_latched[ch] = false;
	m_ext[ch] = 0;
	update_rts(ch);
}

// Character assembly ends: only a channel reset, disabling the receiver or
// Enter Hunt does this.  The receiver waits for the next /SYNC edge.
void zephyr80_sio1_device::receiver_restart(unsigned ch)
{
	m_hunt[ch] = true;
	m_pipe_full[ch] = false;
}

u8 zephyr80_sio1_device::rr0(unsigned ch) const
{
	u8 value = m_latched[ch] ? m_ext[ch] : u8((m_dcd[ch] ? RR0_DCD : 0) | (m_cts[ch] ? RR0_CTS : 0));
	if (m_fifo_depth[ch])
		value |= RR0_RX_AVAILABLE;
	if (!m_tx_full[ch] && !m_crc_left[ch])
		value |= RR0_TX_EMPTY;
	if (m_hunt[ch])
		value |= RR0_SYNC_HUNT;
	if (m_eom[ch])
		value |= RR0_TX_EOM;
	return value;
}

u8 zephyr80_sio1_device::ba_cd_r(offs_t offset)
{
	unsigned const ch = BIT(offset, 1);
	return BIT(offset, 0) ? control_read(ch) : data_read(ch);
}

void zephyr80_sio1_device::ba_cd_w(offs_t offset, u8 data)
{
	unsigned const ch = BIT(offset, 1);
	if (BIT(offset, 0))
	{
		control_write(ch, data);
	}
	else
	{
		// A write over a full buffer replaces the waiting character.
		m_tx_buffer[ch] = data;
		m_tx_full[ch] = true;
		if (m_client)
			m_client->lane_data_event(lane_of(ch));
	}
}

u8 zephyr80_sio1_device::control_read(unsigned ch)
{
	unsigned const reg = m_pointer[ch];
	if (!machine().side_effects_disabled())
		m_pointer[ch] = 0;
	switch (reg)
	{
	case 0: return rr0(ch);
	case 1: return RR1_ALL_SENT | (m_overrun[ch] ? RR1_RX_OVERRUN : 0);
	case 2: return m_wr[1][2]; // the vector register lives in channel B
	default:
		if (!machine().side_effects_disabled() && !(m_warned & WARN_REGISTER))
		{
			m_warned |= WARN_REGISTER;
			logerror("channel %c: read of RR%u, which this model does not provide\n", 'A' + ch, reg);
		}
		return 0xff;
	}
}

u8 zephyr80_sio1_device::data_read(unsigned ch)
{
	// An empty FIFO reads its last character again.
	if (m_fifo_depth[ch] && !machine().side_effects_disabled())
	{
		m_data[ch] = m_fifo[ch][0];
		m_fifo[ch][0] = m_fifo[ch][1];
		m_fifo[ch][1] = m_fifo[ch][2];
		m_fifo_depth[ch]--;
	}
	else if (m_fifo_depth[ch])
	{
		return m_fifo[ch][0];
	}
	if (m_client && !machine().side_effects_disabled())
		m_client->lane_data_event(lane_of(ch));
	return m_data[ch];
}

void zephyr80_sio1_device::control_write(unsigned ch, u8 data)
{
	if (m_pointer[ch])
	{
		unsigned const reg = m_pointer[ch];
		m_pointer[ch] = 0;
		write_register(ch, reg, data);
		return;
	}

	m_pointer[ch] = data & 0x07;
	switch ((data >> 3) & 0x07)
	{
	case 2: // Reset External/Status Interrupts: RR0 follows the lines again
		m_latched[ch] = false;
		break;
	case 3: // Channel Reset
		channel_reset(ch);
		break;
	case 6: // Error Reset
		m_overrun[ch] = false;
		break;
	default: // null, Send Abort (SDLC), interrupt commands: no state here
		break;
	}
	switch (data >> 6)
	{
	case 2: // Reset Tx CRC Generator: zeros in a synchronous mode
		m_crc[ch] = 0;
		break;
	case 3: // Reset Tx Underrun/EOM Latch
		m_eom[ch] = false;
		break;
	default: // Reset Rx CRC Checker: receive CRC is not modelled
		break;
	}
}

void zephyr80_sio1_device::write_register(unsigned ch, unsigned reg, u8 data)
{
	u8 const old = m_wr[ch][reg];
	m_wr[ch][reg] = data;
	switch (reg)
	{
	case 1:
		if (data && !(m_warned & WARN_INTERRUPTS))
		{
			m_warned |= WARN_INTERRUPTS;
			logerror("channel %c: WR1 %02X enables interrupts or WAIT/READY, which this model does not provide\n", 'A' + ch, data);
		}
		break;
	case 3:
		if (!(data & WR3_RX_ENABLE) || !(old & WR3_RX_ENABLE) || (data & WR3_ENTER_HUNT))
			receiver_restart(ch);
		if ((data & WR3_AUTO_ENABLES) && !(m_warned & WARN_AUTO))
		{
			m_warned |= WARN_AUTO;
			logerror("channel %c: WR3 %02X sets Auto Enables, which this model does not provide\n", 'A' + ch, data);
		}
		break;
	case 4:
		if (data != WR4_EXTSYNC_X1 && !(m_warned & WARN_MODE))
		{
			m_warned |= WARN_MODE;
			logerror("channel %c: WR4 %02X; only External Sync x1 (30h) is modelled\n", 'A' + ch, data);
		}
		break;
	case 5:
		if (!(data & WR5_TX_ENABLE))
		{
			// A disabled transmitter marks; what it held never goes out.
			m_tx_full[ch] = false;
			m_crc_left[ch] = 0;
			m_eom[ch] = true;
		}
		update_rts(ch);
		break;
	default: // WR2 vector, WR6/WR7 sync and fill characters
		break;
	}
}

void zephyr80_sio1_device::update_rts(unsigned ch)
{
	bool rts = m_rts[ch];
	if (m_wr[ch][5] & WR5_RTS)
		rts = true;
	else if (!(m_wr[ch][5] & WR5_TX_ENABLE))
		rts = false;
	if (rts != m_rts[ch])
	{
		m_rts[ch] = rts;
		if (m_client)
			m_client->lane_rts_changed(lane_of(ch), rts);
	}
}

// The SIO's transmit CRC in wire (LSB-first) order: CCITT (WR5 D2 = 0) or
// CRC-16, preset to zeros by the Tx CRC reset.
void zephyr80_sio1_device::crc_update(unsigned ch, u8 data)
{
	u16 const poly = (m_wr[ch][5] & WR5_CRC16) ? 0xa001 : 0x8408;
	u16 crc = m_crc[ch] ^ data;
	for (int i = 0; i < 8; i++)
		crc = (crc & 1) ? ((crc >> 1) ^ poly) : (crc >> 1);
	m_crc[ch] = crc;
}

u8 zephyr80_sio1_device::lane_clock(zephyr80_ioc_lane lane, u8 to_z80)
{
	unsigned const ch = channel_of(lane);

	// Receiver: the character in the pipeline enters the FIFO as the next
	// one is assembled.
	if ((m_wr[ch][3] & WR3_RX_ENABLE) && !m_hunt[ch])
	{
		if (m_pipe_full[ch])
		{
			if (m_fifo_depth[ch] < 3)
			{
				m_fifo[ch][m_fifo_depth[ch]++] = m_pipe[ch];
			}
			else
			{
				m_fifo[ch][2] = m_pipe[ch];
				m_overrun[ch] = true;
			}
		}
		m_pipe[ch] = to_z80;
		m_pipe_full[ch] = true;
	}

	// Transmitter.
	u8 out = 0xff;
	if (m_wr[ch][5] & WR5_TX_ENABLE)
	{
		if (m_crc_left[ch])
		{
			out = m_crc[ch] >> 8;
			m_crc_left[ch] = 0;
		}
		else if (m_tx_full[ch])
		{
			out = m_tx_buffer[ch];
			m_tx_full[ch] = false;
			if (m_wr[ch][5] & WR5_TX_CRC)
				crc_update(ch, out);
		}
		else if (!m_eom[ch] && (m_wr[ch][5] & WR5_TX_CRC))
		{
			// Underrun with the latch reset: the CRC, then fill.
			m_eom[ch] = true;
			out = m_crc[ch] & 0xff;
			m_crc_left[ch] = 1;
		}
		else
		{
			m_eom[ch] = true;
			out = m_wr[ch][7];
		}
	}
	return out;
}

void zephyr80_sio1_device::lane_sync(zephyr80_ioc_lane lane, bool asserted)
{
	unsigned const ch = channel_of(lane);
	// External Sync: the falling edge fixes the character boundary; the next
	// clocked character is the first one assembled.
	if (asserted && !m_sync[ch] && (m_wr[ch][3] & WR3_RX_ENABLE))
		m_hunt[ch] = false;
	m_sync[ch] = asserted;
}

void zephyr80_sio1_device::lines_changed(unsigned ch)
{
	if (!m_latched[ch])
	{
		m_ext[ch] = (m_dcd[ch] ? RR0_DCD : 0) | (m_cts[ch] ? RR0_CTS : 0);
		m_latched[ch] = true;
	}
}

void zephyr80_sio1_device::lane_dcd(zephyr80_ioc_lane lane, bool asserted)
{
	unsigned const ch = channel_of(lane);
	if (m_dcd[ch] != asserted)
	{
		m_dcd[ch] = asserted;
		lines_changed(ch);
	}
}

void zephyr80_sio1_device::lane_cts(zephyr80_ioc_lane lane, bool asserted)
{
	unsigned const ch = channel_of(lane);
	if (m_cts[ch] != asserted)
	{
		m_cts[ch] = asserted;
		lines_changed(ch);
	}
}

bool zephyr80_sio1_device::lane_rts(zephyr80_ioc_lane lane) const
{
	return m_rts[channel_of(lane)];
}

// A character clocked now enters the FIFO or waits in the pipeline unless the
// FIFO is full and the pipeline holds one already; a receiver that is off or
// hunting drops what it is given.
bool zephyr80_sio1_device::lane_rx_room(zephyr80_ioc_lane lane) const
{
	unsigned const ch = channel_of(lane);
	if (!(m_wr[ch][3] & WR3_RX_ENABLE) || m_hunt[ch])
		return true;
	return m_fifo_depth[ch] < 3 || !m_pipe_full[ch];
}

bool zephyr80_sio1_device::lane_tx_loaded(zephyr80_ioc_lane lane) const
{
	unsigned const ch = channel_of(lane);
	return (m_wr[ch][5] & WR5_TX_ENABLE) && (m_tx_full[ch] || m_crc_left[ch]);
}
