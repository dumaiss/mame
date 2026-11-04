// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_IOC, zephyr80_ioc_device, "zephyr80_ioc", "Zephyr-80 IO controller")

namespace {

// The transport's delays.  The window and the guard are the firmware's
// constants (include/external_sync.h).  The byte period is not:
// EXTSYNC_TARGET_BYTE_US (16 us) is the firmware's target, and the board's
// period is longer by the controller's per-byte overhead.  The KonaOS request
// loop (ioc_io_tx, 175 T-states a byte, one transmit buffer of slack) works on
// the board, so its period is at least 17.5 us; 20 us is this model's value
// until a board measurement replaces it.  The controller's own processing
// takes no emulated time.
constexpr unsigned WINDOW_BYTES = 36;       // EXTSYNC_RX_WINDOW_BYTES
constexpr u32 COMMAND_BYTE_US = 20;         // one command-lane byte
constexpr u32 REPLY_GUARD_US = 200;         // EXTSYNC_REPLY_GUARD_US
// main.c bounds the wait for /RTSB to rise after the window by a loop count
// (REQUEST_RELEASE_TIMEOUT_LOOPS); this model allows one millisecond.
constexpr u32 RELEASE_WAIT_US = 1000;

constexpr zephyr80_ioc_lane COMMAND = zephyr80_ioc_lane::COMMAND;
constexpr zephyr80_ioc_lane BULK = zephyr80_ioc_lane::BULK;

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

	m_core.register_save(*this);
	save_item(NAME(m_phase));
	save_item(NAME(m_command_synced));
	save_item(NAME(m_bulk_synced));
	save_item(NAME(m_count));
	save_item(NAME(m_reply_length));
	save_item(NAME(m_window));
	save_item(NAME(m_reply));
}

// The controller restarts with the board: no service in flight, both /SYNC
// lines released, READY asserted, the doorbell and bulk admission idle.
void zephyr80_ioc_device::device_reset()
{
	m_command_timer->adjust(attotime::never);
	m_core.reset();
	m_phase = IDLE;
	m_count = 0;
	m_reply_length = 0;
	std::fill(std::begin(m_window), std::end(m_window), 0xff);
	std::fill(std::begin(m_reply), std::end(m_reply), 0);
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

void zephyr80_ioc_device::set_ready(bool ready)
{
	m_lane->lane_dcd(COMMAND, ready);
}

void zephyr80_ioc_device::lane_rts_changed(zephyr80_ioc_lane lane, bool asserted)
{
	if (lane != COMMAND)
		return;
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
	m_core.dispatch(request, reply);
	m_reply_length = zephyr80_ioc_core::encode(reply, m_reply);
	m_count = 0;
	m_phase = GUARD;
	m_command_timer->adjust(attotime::from_usec(REPLY_GUARD_US));
}

void zephyr80_ioc_device::end_service()
{
	m_phase = IDLE;
	set_ready(true);
	poll_request();
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
			end_service();
		}
		break;

	default:
		break;
	}
}
