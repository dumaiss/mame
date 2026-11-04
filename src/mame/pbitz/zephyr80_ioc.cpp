// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_IOC, zephyr80_ioc_device, "zephyr80_ioc", "Zephyr-80 IO controller")

namespace {

// The transport's delays.  The window, the guards, the bounded waits and the
// reset hold are the firmware's constants (include/external_sync.h,
// src/bulk_channel.c, src/handlers.c).  The byte periods are not all: the
// command lane's EXTSYNC_TARGET_BYTE_US (16 us) is the firmware's target,
// and the board's period is longer by the controller's per-byte overhead.
// The KonaOS request loop (ioc_io_tx, 175 T-states a byte, one transmit
// buffer of slack) works on the board, so its period is at least 17.5 us;
// 20 us is this model's value until a board measurement replaces it.  The
// bulk lane's send target is 6 us (BULK_TARGET_BYTE_US) and the KonaOS
// read-path measurement records about 25 us a byte; this model sends at the
// command lane's 20 us until a board measurement replaces it, and receives
// at the firmware's BULK_RX_TARGET_BYTE_US.  The controller's own
// processing takes no emulated time.
constexpr unsigned WINDOW_BYTES = 36;       // EXTSYNC_RX_WINDOW_BYTES
constexpr u32 COMMAND_BYTE_US = 20;         // one command-lane byte
constexpr u32 REPLY_GUARD_US = 200;         // EXTSYNC_REPLY_GUARD_US
// main.c bounds the wait for /RTSB to rise after the window by a loop count
// (REQUEST_RELEASE_TIMEOUT_LOOPS); this model allows one millisecond.
constexpr u32 RELEASE_WAIT_US = 1000;
constexpr u32 BULK_SEND_BYTE_US = 20;       // one bulk byte to the Z80
constexpr u32 BULK_RECEIVE_BYTE_US = 24;    // BULK_RX_TARGET_BYTE_US
constexpr u32 BULK_ADMISSION_US = 100;      // BULK_ADMISSION_GUARD_US
constexpr u32 BULK_HOST_WAIT_MS = 500;      // BULK_HOST_READY_TIMEOUT_MS
constexpr u32 HOST_RESET_MS = 10;           // handler_reset's hold before the restart

constexpr zephyr80_ioc_lane COMMAND = zephyr80_ioc_lane::COMMAND;
constexpr zephyr80_ioc_lane BULK = zephyr80_ioc_lane::BULK;

using bulk_dir = zephyr80_ioc_core::bulk_dir;

} // anonymous namespace

zephyr80_ioc_device::zephyr80_ioc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_IOC, tag, owner, clock)
	, m_lane(nullptr)
	, m_core(*this)
	, m_command_timer(nullptr)
{
}

void zephyr80_ioc_device::device_start()
{
	if (!m_lane)
		throw emu_fatalerror("%s: no SIO1 lane attached", tag());
	m_command_timer = timer_alloc(FUNC(zephyr80_ioc_device::command_step), this);

	m_core.save_state([this] (auto &&... item) { save_item(std::forward<decltype(item)>(item)...); });
	save_item(NAME(m_phase));
	save_item(NAME(m_command_synced));
	save_item(NAME(m_bulk_synced));
	save_item(NAME(m_count));
	save_item(NAME(m_reply_length));
	save_item(NAME(m_window));
	save_item(NAME(m_reply));
	save_item(NAME(m_bulk_count));
	save_item(NAME(m_bulk_total));
	save_item(NAME(m_bulk));
}

// The controller restarts with the board: no service in flight, both /SYNC
// lines released, READY asserted, the doorbell and bulk admission idle.
void zephyr80_ioc_device::device_reset()
{
	m_command_timer->adjust(attotime::never);
	m_phase = IDLE;
	m_count = 0;
	m_reply_length = 0;
	m_bulk_count = 0;
	m_bulk_total = 0;
	std::fill(std::begin(m_window), std::end(m_window), 0xff);
	std::fill(std::begin(m_reply), std::end(m_reply), 0);
	std::fill(std::begin(m_bulk), std::end(m_bulk), 0);
	m_core.reset();
	resync_lanes();
	m_lane->lane_cts(COMMAND, false);
	m_lane->lane_dcd(BULK, false);
	m_lane->lane_cts(BULK, false);
	set_ready(true);
	poll_request();
}

void zephyr80_ioc_device::resync_lanes()
{
	m_command_synced = false;
	m_bulk_synced = false;
	m_lane->lane_sync(COMMAND, false);
	m_lane->lane_sync(BULK, false);
}

void zephyr80_ioc_device::hid_doorbell(bool asserted)
{
	m_lane->lane_cts(COMMAND, asserted);
}

// handler_reset holds the host reset pair, then the controller restarts;
// the board's reset restarts both, as a machine reset does here.
void zephyr80_ioc_device::host_reset()
{
	m_phase = RESETTING;
	m_command_timer->adjust(attotime::from_msec(HOST_RESET_MS));
}

void zephyr80_ioc_device::set_ready(bool ready)
{
	m_lane->lane_dcd(COMMAND, ready);
}

void zephyr80_ioc_device::lane_rts_changed(zephyr80_ioc_lane lane, bool asserted)
{
	if (lane == BULK)
	{
		if ((asserted && m_phase == BULK_HOST) || (!asserted && m_phase == BULK_RELEASE))
		{
			m_command_timer->adjust(attotime::never);
			bulk_step();
		}
		return;
	}
	if (asserted)
	{
		poll_request();
	}
	else if (m_phase == RELEASE)
	{
		m_command_timer->adjust(attotime::never);
		serve();
	}
}

// A request is the /RTSB level, taken only while idle.
void zephyr80_ioc_device::poll_request()
{
	if (m_phase != IDLE || !m_lane->lane_rts(COMMAND))
		return;
	set_ready(false);
	m_core.begin();
	m_phase = WINDOW;
	m_count = 0;
	m_command_timer->adjust(attotime::from_usec(COMMAND_BYTE_US));
}

void zephyr80_ioc_device::serve()
{
	zephyr80_ioc_core::frame request, reply;
	if (!m_core.decode(m_window, WINDOW_BYTES, request))
	{
		m_core.abort();
		end_service();
		return;
	}
	if (!m_core.dispatch(request, reply))
		return; // CMD_RESET: host_reset() took over
	m_reply_length = zephyr80_ioc_core::encode(reply, m_reply);
	m_count = 0;
	m_phase = GUARD;
	m_command_timer->adjust(attotime::from_usec(REPLY_GUARD_US));
}

// main.c after external_sync_send: the transfer the reply armed, if any.
void zephyr80_ioc_device::after_reply()
{
	if (m_core.bulk_armed() == bulk_dir::NONE)
	{
		m_core.run_end();
		end_service();
		return;
	}
	m_phase = BULK_HOST;
	bulk_step();
}

// The service's last step: READY returns after the controller's idle pass.
void zephyr80_ioc_device::end_service()
{
	m_phase = IDLE;
	m_core.idle();
	set_ready(true);
	poll_request();
}

// Admission: /CTSA marks the phase live, /DCDA also admits a read; the clock
// starts after the guard.
void zephyr80_ioc_device::bulk_admit()
{
	if (m_core.bulk_armed() == bulk_dir::TO_Z80)
	{
		m_lane->lane_dcd(BULK, true);
		m_bulk_total = m_core.bulk_packet(m_bulk);
	}
	else
	{
		m_bulk_total = m_core.bulk_window();
	}
	m_lane->lane_cts(BULK, true);
	m_bulk_count = 0;
	m_phase = BULK_GUARD;
	m_command_timer->adjust(attotime::from_usec(BULK_ADMISSION_US));
}

// The bulk phase's waits: the host's /RTSA (wait_for_host_ready), and
// between a run's packets its release (bulk_channel_wait_host_release).
void zephyr80_ioc_device::bulk_step()
{
	switch (m_phase)
	{
	case BULK_HOST:
		if (m_lane->lane_rts(BULK))
			bulk_admit();
		else
			m_command_timer->adjust(attotime::from_msec(BULK_HOST_WAIT_MS));
		break;

	case BULK_RELEASE:
		if (!m_lane->lane_rts(BULK))
		{
			m_core.run_next();
			m_phase = BULK_HOST;
			bulk_step();
		}
		else
		{
			m_command_timer->adjust(attotime::from_msec(BULK_HOST_WAIT_MS));
		}
		break;

	default:
		break;
	}
}

TIMER_CALLBACK_MEMBER(zephyr80_ioc_device::command_step)
{
	switch (m_phase)
	{
	case WINDOW:
		m_window[m_count++] = m_lane->lane_clock(COMMAND, 0xff);
		if (m_count < WINDOW_BYTES)
		{
			m_command_timer->adjust(attotime::from_usec(COMMAND_BYTE_US));
		}
		else if (!m_lane->lane_rts(COMMAND))
		{
			serve();
		}
		else
		{
			m_phase = RELEASE;
			m_command_timer->adjust(attotime::from_usec(RELEASE_WAIT_US));
		}
		break;

	case RELEASE: // /RTSB never rose: no reply
		m_core.abort();
		end_service();
		break;

	case GUARD:
		m_phase = REPLY;
		if (!m_command_synced)
		{
			// The disposable establishing byte, then the /SYNCB edge that
			// fixes the receiver's boundary; held low from here on.
			m_lane->lane_clock(COMMAND, 0xff);
			m_lane->lane_sync(COMMAND, true);
			m_command_synced = true;
			m_command_timer->adjust(attotime::from_usec(COMMAND_BYTE_US));
			break;
		}
		[[fallthrough]];

	case REPLY:
		if (m_count < m_reply_length)
		{
			m_lane->lane_clock(COMMAND, m_reply[m_count++]);
			m_command_timer->adjust(attotime::from_usec(COMMAND_BYTE_US));
		}
		else
		{
			// The trailing FFh: its clocks move the CRC's low byte into the
			// FIFO, and it stays in the SIO's receive pipeline.
			m_lane->lane_clock(COMMAND, 0xff);
			after_reply();
		}
		break;

	case BULK_HOST:    // the host never asserted /RTSA
	case BULK_RELEASE: // nor released it between a run's packets
		if (m_phase == BULK_HOST)
			m_core.bulk_no_host();
		m_core.run_end();
		end_service();
		break;

	case BULK_GUARD:
		if (m_core.bulk_armed() == bulk_dir::TO_Z80)
		{
			m_phase = BULK_SEND;
			if (!m_bulk_synced)
			{
				m_lane->lane_clock(BULK, 0xff);
				m_lane->lane_sync(BULK, true);
				m_bulk_synced = true;
				m_command_timer->adjust(attotime::from_usec(BULK_SEND_BYTE_US));
				break;
			}
		}
		else
		{
			m_phase = BULK_RECEIVE;
		}
		[[fallthrough]];

	case BULK_SEND:
	case BULK_RECEIVE:
		if (m_bulk_count < m_bulk_total)
		{
			if (m_phase == BULK_SEND)
			{
				m_lane->lane_clock(BULK, m_bulk[m_bulk_count++]);
				m_command_timer->adjust(attotime::from_usec(BULK_SEND_BYTE_US));
			}
			else
			{
				m_bulk[m_bulk_count++] = m_lane->lane_clock(BULK, 0xff);
				m_command_timer->adjust(attotime::from_usec(BULK_RECEIVE_BYTE_US));
			}
			break;
		}
		if (m_phase == BULK_SEND)
		{
			// One flush byte, then admission closes; the bytes are out.
			m_lane->lane_clock(BULK, 0xff);
			m_lane->lane_cts(BULK, false);
			m_lane->lane_dcd(BULK, false);
			m_core.bulk_sent();
			if (m_core.run_more())
			{
				m_phase = BULK_RELEASE;
				bulk_step();
				break;
			}
		}
		else
		{
			// /CTSA closes once the packet is accepted, before the commit.
			m_lane->lane_cts(BULK, false);
			m_core.bulk_received(m_bulk, m_bulk_total);
		}
		m_core.run_end();
		end_service();
		break;

	case RESETTING:
		machine().schedule_soft_reset();
		break;

	default:
		break;
	}
}
