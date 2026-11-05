// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_IOC, zephyr80_ioc_device, "zephyr80_ioc", "Zephyr-80 IO controller")

namespace {

// The transport's delays: a short table of named values from the controller
// firmware's constants and the retained records, never from counting
// instructions.  The guards, the bounded waits and the reset hold are the
// firmware's constants (include/external_sync.h, src/bulk_channel.c,
// src/handlers.c).  The command byte and the two bulk byte rows are not.
//
//  command byte       the firmware's target is EXTSYNC_TARGET_BYTE_US (16 us)
//                     plus the controller's own per-byte overhead.  The
//                     KonaOS request loop (ioc_tx_body, 175 T-states a byte,
//                     one transmit buffer of slack) works on the board, so the
//                     board period is at least 17.5 us; 16 us underran it
//                     here.  No record gives the board value: 20 us stays
//                     provisional until it is measured.
//  bulk fixed byte    the marker, header and CRC trailer go through
//                     bulk_send_packet_byte (firmware target about 7 us: 4 us
//                     of SPI clocking, a 2 us gap and the call; the KonaOS
//                     simulator's BULK_FIXED_BYTE_TSTATES is 70 at 10 MHz).
//                     The establishing byte of an unsynchronised lane is
//                     given the same time.  Lower bound inferred from ZSDOS
//                     working on the board: its marker scan calls a one-byte
//                     helper (111 T-states = 11.1 us a byte), so a wire byte
//                     faster than that builds a backlog the 3-deep FIFO plus
//                     pipeline cannot hold behind the 17 us of setup before
//                     its payload loop.  11.1 us plus 6 us (the firmware's payload
//                     period) of margin, rounded up: 18 us.  Calibrate with the board
//                     measurement of the bulk packet's byte spacing.
//  bulk payload byte  the firmware target BULK_TARGET_BYTE_US is 6 us (4 us of
//                     SPI clocking plus BULK_PAYLOAD_GAP_US and the loop).
//                     The same ZSDOS reader reads its first payload byte
//                     21.2-26.3 us after it arrives (INI poll phase 4.0-9.1 us,
//                     then 17.2 us of setup) against a cover of four payload
//                     periods, so the board's period is at least 6.6 us.
//                     With one period of cover as margin (4P - P >= 26.3):
//                     9 us.  Inferred lower bound; calibrate likewise.
//  bulk byte from Z80 BULK_RX_TARGET_BYTE_US, every byte of the window.
//  backend latency    none: no record separates the card's access time from
//                     the lane and the controller's request handling, so the
//                     controller's processing and the card take no emulated
//                     time.
constexpr unsigned WINDOW_BYTES = 36;       // EXTSYNC_RX_WINDOW_BYTES
constexpr u32 COMMAND_BYTE_US = 20;         // one command-lane byte (provisional)
constexpr u32 REPLY_GUARD_US = 200;         // EXTSYNC_REPLY_GUARD_US
// main.c bounds the wait for /RTSB to rise after the window by a loop count
// (REQUEST_RELEASE_TIMEOUT_LOOPS); this model allows one millisecond.
constexpr u32 RELEASE_WAIT_US = 1000;
constexpr u32 BULK_FIXED_BYTE_US = 18;      // marker, header, CRC trailer, establishing byte (inferred lower bound)
constexpr u32 BULK_PAYLOAD_BYTE_US = 9;     // payload byte (inferred lower bound)
constexpr u32 BULK_RECEIVE_BYTE_US = 24;    // BULK_RX_TARGET_BYTE_US
constexpr u32 BULK_ADMISSION_US = 100;      // BULK_ADMISSION_GUARD_US
constexpr u32 BULK_HOST_WAIT_MS = 500;      // BULK_HOST_READY_TIMEOUT_MS
constexpr u32 HOST_RESET_MS = 10;           // handler_reset's hold before the restart
// A packet to the Z80: A5 5A LEN_LO LEN_HI TYPE SEQ STATUS, the payload, two
// CRC bytes (zero under the deployed bypass), then the flush byte.
constexpr unsigned BULK_HEADER_BYTES = 7;
constexpr unsigned BULK_TRAILER_BYTES = 2;
// Fast mode: the least a byte slot takes, so a polling guest always runs.
constexpr u32 FAST_YIELD_US = 1;
// Fast mode keeps the receiver's buffering full, so a character to the Z80
// waits for room as long as a paced lane runs before its first overrun: the
// FIFO (3), the pipeline (1) and the character being clocked.
constexpr u32 FAST_ROOM_PERIODS = 5;

constexpr zephyr80_ioc_lane COMMAND = zephyr80_ioc_lane::COMMAND;
constexpr zephyr80_ioc_lane BULK = zephyr80_ioc_lane::BULK;

using bulk_dir = zephyr80_ioc_core::bulk_dir;

} // anonymous namespace

zephyr80_ioc_device::zephyr80_ioc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_IOC, tag, owner, clock)
	, m_lane(nullptr)
	, m_speed(nullptr)
	, m_fast(false)
	, m_core(*this)
	, m_keyboard(*this, "keyboard")
	, m_mouse(*this, "mouse")
	, m_pads(*this, "pad%u", 1U)
	, m_command_timer(nullptr)
	, m_hid_timer(nullptr)
	, m_kbd_pending(false)
	, m_pad_slot{ zephyr80_ioc_hid::GAMEPAD_NONE, zephyr80_ioc_hid::GAMEPAD_NONE }
{
}

void zephyr80_ioc_device::device_add_mconfig(machine_config &config)
{
	ZEPHYR80_USB_KEYBOARD(config, m_keyboard);
	m_keyboard->set_host(*this);
	ZEPHYR80_USB_MOUSE(config, m_mouse);
	m_mouse->set_host(*this);
	for (unsigned pad = 0; pad < zephyr80_ioc_hid::GAMEPADS; pad++)
	{
		ZEPHYR80_USB_GAMEPAD(config, m_pads[pad]);
		m_pads[pad]->set_number(pad);
		m_pads[pad]->set_host(*this);
	}
}

void zephyr80_ioc_device::device_start()
{
	if (!m_lane)
		throw emu_fatalerror("%s: no SIO1 lane attached", tag());
	if (!m_core.has_storage())
		throw emu_fatalerror("%s: no card socket attached", tag());
	if (!m_speed)
		throw emu_fatalerror("%s: no ioc_speed slot attached", tag());
	m_fast = m_speed->fast();
	m_command_timer = timer_alloc(FUNC(zephyr80_ioc_device::command_step), this);
	m_hid_timer = timer_alloc(FUNC(zephyr80_ioc_device::hid_tick), this);

	m_core.save_state([this] (auto &&... item) { save_item(std::forward<decltype(item)>(item)...); });
	save_item(NAME(m_phase));
	save_item(NAME(m_command_synced));
	save_item(NAME(m_bulk_synced));
	save_item(NAME(m_waiting));
	save_item(NAME(m_wait_lane));
	save_item(NAME(m_wait_to_z80));
	save_item(NAME(m_starved));
	save_item(NAME(m_count));
	save_item(NAME(m_reply_length));
	save_item(NAME(m_window));
	save_item(NAME(m_reply));
	save_item(NAME(m_bulk_count));
	save_item(NAME(m_bulk_total));
	save_item(NAME(m_bulk));
	save_item(NAME(m_hid_epoch));
	save_item(NAME(m_kbd_report));
	save_item(NAME(m_kbd_pending));
	save_item(NAME(m_pad_slot));
}

// The controller restarts with the board: no service in flight, both /SYNC
// lines released, READY asserted, the doorbell and bulk admission idle.
void zephyr80_ioc_device::device_reset()
{
	m_command_timer->adjust(attotime::never);
	m_phase = IDLE;
	m_waiting = false;
	m_wait_lane = unsigned(COMMAND);
	m_wait_to_z80 = false;
	m_starved = false;
	m_count = 0;
	m_reply_length = 0;
	m_bulk_count = 0;
	m_bulk_total = 0;
	std::fill(std::begin(m_window), std::end(m_window), 0xff);
	std::fill(std::begin(m_reply), std::end(m_reply), 0);
	std::fill(std::begin(m_bulk), std::end(m_bulk), 0);
	m_core.reset();
	// Enumeration is not modelled: the keyboard is mounted with the reset.
	m_core.hid().mount();
	m_hid_epoch = machine().time();
	m_hid_timer->adjust(attotime::never);
	std::fill(std::begin(m_kbd_report), std::end(m_kbd_report), 0);
	m_kbd_pending = false;
	std::fill(std::begin(m_pad_slot), std::end(m_pad_slot), zephyr80_ioc_hid::GAMEPAD_NONE);
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

// The latest report waits for the idle pass; a boot keyboard's report is a
// state, so a newer one replaces an older one still waiting.
void zephyr80_ioc_device::keyboard_report(u8 const *report, unsigned length)
{
	std::fill(std::begin(m_kbd_report), std::end(m_kbd_report), 0);
	std::copy_n(report, std::min<unsigned>(length, std::size(m_kbd_report)), m_kbd_report);
	m_kbd_pending = true;
	if (m_phase == IDLE)
		hid_service();
}

// tusb_time_millis_api: whole 10 ms timebase ticks since the controller's reset.
u32 zephyr80_ioc_device::hid_now_ms() const
{
	return u32((machine().time() - m_hid_epoch).as_ticks(1000 / zephyr80_ioc_hid::TICK_MS)) * zephyr80_ioc_hid::TICK_MS;
}

// A mouse or pad changed (a motion, a button, a connection).
void zephyr80_ioc_device::usb_input_ready()
{
	if (m_phase == IDLE)
		hid_service();
}

// The devices the host sees: each mouse or pad whose connection changed at its
// last scan is mounted or unmounted (tuh_hid_mount_cb, tuh_umount_cb), the mouse is polled
// until it has nothing to report, and each mounted pad whose 10 ms poll has
// come reports its state.
void zephyr80_ioc_device::usb_sync(u32 now)
{
	zephyr80_ioc_hid &hid = m_core.hid();
	if (m_mouse->attached() != hid.mouse_mounted())
	{
		if (m_mouse->attached())
			hid.mount_mouse();
		else
			hid.unmount_mouse();
	}
	u8 report[zephyr80_usb_gamepad_device::REPORT];
	for (unsigned pad = 0; pad < zephyr80_ioc_hid::GAMEPADS; pad++)
	{
		u8 &slot = m_pad_slot[pad];
		if (m_pads[pad]->attached() && slot == zephyr80_ioc_hid::GAMEPAD_NONE)
		{
			u8 address;
			slot = hid.mount_pad(address);
		}
		else if (!m_pads[pad]->attached() && slot != zephyr80_ioc_hid::GAMEPAD_NONE)
		{
			hid.unmount_pad(slot);
			slot = zephyr80_ioc_hid::GAMEPAD_NONE;
		}
	}
	u8 mouse[zephyr80_usb_mouse_device::REPORT];
	while (hid.mouse_mounted() && m_mouse->poll(mouse))
		hid.mouse_report(mouse, std::size(mouse));
	for (unsigned pad = 0; pad < zephyr80_ioc_hid::GAMEPADS; pad++)
	{
		u8 const slot = m_pad_slot[pad];
		if (slot != zephyr80_ioc_hid::GAMEPAD_NONE && hid.gamepad_due(slot, now))
		{
			m_pads[pad]->report(report);
			hid.gamepad_report(slot, report, std::size(report), now);
		}
	}
}

// hid_host_task in the idle pass: the waiting report (tuh_task), the devices,
// the repeat, then hid_reset_requested.  The timer is set for the next due
// tick, the repeat's or a pad's poll.
void zephyr80_ioc_device::hid_service()
{
	zephyr80_ioc_hid &hid = m_core.hid();
	u32 const now = hid_now_ms();
	if (m_kbd_pending)
	{
		m_kbd_pending = false;
		hid.report(m_kbd_report, std::size(m_kbd_report), now);
	}
	usb_sync(now);
	hid.task(now);
	if (hid.reset_requested())
	{
		m_hid_timer->adjust(attotime::never);
		set_ready(false);
		host_reset();
		return;
	}
	u32 due = 0;
	bool have = hid.repeat_due(due);
	u32 pad_due;
	if (hid.gamepad_next_due(pad_due) && (!have || s32(pad_due - due) < 0))
	{
		due = pad_due;
		have = true;
	}
	if (have)
	{
		attotime const at = m_hid_epoch + attotime::from_msec(due);
		m_hid_timer->adjust(at > machine().time() ? at - machine().time() : attotime::zero);
	}
	else
	{
		m_hid_timer->adjust(attotime::never);
	}
}

// The repeat's tick: served now when idle, else by the service's end.
TIMER_CALLBACK_MEMBER(zephyr80_ioc_device::hid_tick)
{
	if (m_phase == IDLE)
		hid_service();
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

void zephyr80_ioc_device::lane_data_event(zephyr80_ioc_lane lane)
{
	if (m_waiting && unsigned(lane) == m_wait_lane)
		wake();
}

// The period that follows byte `index` of a packet to the Z80.
u32 zephyr80_ioc_device::bulk_send_period(unsigned index) const
{
	bool const payload = index >= BULK_HEADER_BYTES && index + BULK_TRAILER_BYTES < m_bulk_total;
	return payload ? BULK_PAYLOAD_BYTE_US : BULK_FIXED_BYTE_US;
}

// The next byte slot of a transfer.  Paced: one period.  Fast: a yield when
// the other end is ready now, else the period, shortened by wake() the moment
// it becomes ready.
void zephyr80_ioc_device::next_byte(u32 period_us, zephyr80_ioc_lane lane, bool to_z80)
{
	if (!m_fast)
	{
		m_command_timer->adjust(attotime::from_usec(period_us));
		return;
	}
	m_waiting = true;
	m_wait_lane = unsigned(lane);
	m_wait_to_z80 = to_z80;
	u32 const deadline = (to_z80 && !m_starved) ? FAST_ROOM_PERIODS * period_us : period_us;
	m_command_timer->adjust(attotime::from_usec(byte_ready() ? FAST_YIELD_US : deadline));
}

bool zephyr80_ioc_device::byte_ready() const
{
	zephyr80_ioc_lane const lane = zephyr80_ioc_lane(m_wait_lane);
	if (m_wait_to_z80)
		return m_lane->lane_rx_room(lane);
	if (m_lane->lane_tx_loaded(lane))
		return true;
	// A bulk write has no release, so a missed deadline ends the Z80's
	// transmission and the rest of the window is fill.  A request window
	// keeps its length: the guest arms its receiver in the time it leaves,
	// and a late byte lands in its own slot as it does paced.
	return lane == BULK && m_starved;
}

void zephyr80_ioc_device::wake()
{
	if (!m_fast || !m_waiting || !byte_ready())
		return;
	attotime const yield = attotime::from_usec(FAST_YIELD_US);
	if (m_command_timer->remaining() > yield)
		m_command_timer->adjust(yield);
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
	m_starved = false;
	next_byte(COMMAND_BYTE_US, COMMAND, false);
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
	m_starved = false;
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

// The service's last step: READY returns after the controller's idle pass
// (hid_host_task, then the volume service).
void zephyr80_ioc_device::end_service()
{
	m_phase = IDLE;
	hid_service();
	if (m_phase != IDLE)
		return; // Ctrl+Alt+Esc: the reset follows
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
	m_starved = false;
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
	// Fast: a slot that reaches its deadline with the other end not ready
	// means that end has stopped; the rest of the transfer runs at the paced
	// periods (to the Z80) or is fill (from the Z80).
	if (m_waiting && !byte_ready())
		m_starved = true;
	m_waiting = false;
	switch (m_phase)
	{
	case WINDOW:
		m_window[m_count++] = m_lane->lane_clock(COMMAND, 0xff);
		if (m_count < WINDOW_BYTES)
		{
			next_byte(COMMAND_BYTE_US, COMMAND, false);
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
			next_byte(COMMAND_BYTE_US, COMMAND, true);
			break;
		}
		[[fallthrough]];

	case REPLY:
		if (m_count < m_reply_length)
		{
			m_lane->lane_clock(COMMAND, m_reply[m_count++]);
			next_byte(COMMAND_BYTE_US, COMMAND, true);
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
				next_byte(BULK_FIXED_BYTE_US, BULK, true);
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
				u32 const period = bulk_send_period(m_bulk_count);
				m_lane->lane_clock(BULK, m_bulk[m_bulk_count++]);
				next_byte(period, BULK, true);
			}
			else
			{
				m_bulk[m_bulk_count++] = m_lane->lane_clock(BULK, 0xff);
				next_byte(BULK_RECEIVE_BYTE_US, BULK, false);
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
