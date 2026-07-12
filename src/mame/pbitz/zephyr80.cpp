// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_coffeeio.h"
#include "zephyr80_map.h"

#include "bus/rs232/loopback.h"
#include "bus/rs232/pty.h"
#include "bus/rs232/rs232.h"
#include "cpu/z80/z80.h"
#include "machine/clock.h"
#include "machine/z80ctc.h"
#include "machine/z80daisy.h"
#include "machine/z80sio.h"
#include "video/v9938.h"

#include "screen.h"

#include <array>


namespace {

void pbitz_rs232_devices(device_slot_interface &device)
{
	device.option_add("pty", PSEUDO_TERMINAL);
	device.option_add("loopback", RS232_LOOPBACK);
}

// The SIO0/B console link runs at a fixed 115200 8N1 in hardware, so default any
// device attached to that port to 115200 (matching the SIO channel B clock).
static DEVICE_INPUT_DEFAULTS_START(console_baud_115200)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_115200)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_115200)
DEVICE_INPUT_DEFAULTS_END

class zephyr80_state : public driver_device
{
public:
	zephyr80_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_sio0(*this, "sio0")
		, m_sio1(*this, "sio1")
		, m_ctc(*this, "ctc")
		, m_vdp(*this, "vdp")
		, m_coffeeio(*this, "coffeeio")
		, m_rom_region(*this, "maincpu")
	{
	}

	void zephyr80(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<z80_device> m_maincpu;
	required_device<z80sio_device> m_sio0;
	required_device<z80sio_device> m_sio1;
	required_device<z80ctc_device> m_ctc;
	required_device<v9958_device> m_vdp;
	required_device<pbitz_coffeeio_device> m_coffeeio;
	required_memory_region m_rom_region;

	static constexpr unsigned COFFEEIO_RX_QUEUE_SIZE = 256;
	static constexpr unsigned COFFEEIO_RX_BYTE_RATE = 960; // 9600 8N1 characters/second

	std::unique_ptr<u8[]> m_ram;
	u8 m_latch = 0;
	bool m_sio_tx_line_start[4] = { };
	std::array<u8, COFFEEIO_RX_QUEUE_SIZE> m_coffeeio_rx_queue = { };
	u8 m_coffeeio_rx_head = 0;
	u8 m_coffeeio_rx_tail = 0;
	u8 m_coffeeio_rx_count = 0;
	bool m_coffeeio_rx_timer_active = false;
	emu_timer *m_coffeeio_rx_timer = nullptr;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
	u8 mem_latch_r();
	void mem_latch_w(u8 data);
	u8 sio0_r(offs_t offset);
	void sio0_w(offs_t offset, u8 data);
	u8 sio1_r(offs_t offset);
	void sio1_w(offs_t offset, u8 data);
	void log_sio_tx(unsigned channel, const char *prefix, u8 data);
	void coffeeio_a_response_w(u8 data);
	void coffeeio_start_rx();
	TIMER_CALLBACK_MEMBER(coffeeio_rx_tick);

	bool bios_range(offs_t offset) const;
	bool safe_ram(offs_t offset) const;
	bool ram_shadow() const;
	bool rom_disabled() const;
	u8 rom_page() const;
	u8 selected_ram_bank(offs_t offset) const;
	u8 rom_r(offs_t offset) const;
	u8 ram_r(offs_t offset) const;
	void ram_w(offs_t offset, u8 data);
};

void zephyr80_state::machine_start()
{
	m_ram = std::make_unique<u8[]>(zephyr80_map::mem::RAM_SIZE);
	std::fill_n(m_ram.get(), zephyr80_map::mem::RAM_SIZE, 0x00);
	m_coffeeio_rx_timer = timer_alloc(FUNC(zephyr80_state::coffeeio_rx_tick), this);

	save_pointer(NAME(m_ram), zephyr80_map::mem::RAM_SIZE);
	save_item(NAME(m_latch));
	save_item(NAME(m_sio_tx_line_start));
	save_item(NAME(m_coffeeio_rx_queue));
	save_item(NAME(m_coffeeio_rx_head));
	save_item(NAME(m_coffeeio_rx_tail));
	save_item(NAME(m_coffeeio_rx_count));
	save_item(NAME(m_coffeeio_rx_timer_active));
}

void zephyr80_state::machine_reset()
{
	m_latch = 0; // Reset: normal ROM mode, ROM page 0, SRAM bank 0, shadow off.
	for (bool &line_start : m_sio_tx_line_start)
		line_start = true;
	m_coffeeio_rx_head = 0;
	m_coffeeio_rx_tail = 0;
	m_coffeeio_rx_count = 0;
	m_coffeeio_rx_timer_active = false;
	m_coffeeio_rx_timer->adjust(attotime::never);
	m_sio1->rxa_w(1);
	m_sio1->rxb_w(1);
}

void zephyr80_state::mem_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(zephyr80_state::mem_r), FUNC(zephyr80_state::mem_w));
}

void zephyr80_state::io_map(address_map &map)
{
	using namespace zephyr80_map;

	map.global_mask(0xff);

	// IO_DECODER.pld: $00-$0f is the memory banking latch read/write block.
	map(io::MEMBANK_START, io::MEMBANK_END).rw(FUNC(zephyr80_state::mem_latch_r), FUNC(zephyr80_state::mem_latch_w));

	// IO_DECODER.pld: SIO0 at $20-$2f and SIO1 at $30-$3f.  The firmware
	// (platform_zephyr80.inc) addresses each SIO as ba_cd: A1 selects the
	// channel and A0 selects data/control, giving +0 A data, +1 A control,
	// +2 B data, +3 B control within each 4-byte window.
	map(io::SIO0_START, io::SIO0_START + io::SIO_REGISTER_MASK).mirror(io::SIO_REGISTER_MIRROR).rw(FUNC(zephyr80_state::sio0_r), FUNC(zephyr80_state::sio0_w));
	map(io::SIO1_START, io::SIO1_START + io::SIO_REGISTER_MASK).mirror(io::SIO_REGISTER_MIRROR).rw(FUNC(zephyr80_state::sio1_r), FUNC(zephyr80_state::sio1_w));

	// IO_DECODER.pld: CTC at $40-$4f.  Channels 0-3 select on A1:A0
	// (CTC0_CTRL..CTC3_CTRL = $40..$43).
	map(io::CTC_START, io::CTC_START + io::CTC_REGISTER_MASK).mirror(io::CTC_REGISTER_MIRROR).rw(m_ctc, FUNC(z80ctc_device::read), FUNC(z80ctc_device::write));

	// IO_DECODER.pld: V9958 VDP at $a0-$bf.  A1:A0 select the four V9958 ports:
	// +0 VRAM data, +1 command/status, +2 palette, +3 register (read: +0 VRAM,
	// +1 status).
	map(io::VDP_START, io::VDP_START + io::VDP_REGISTER_MASK).mirror(io::VDP_REGISTER_MIRROR).rw(m_vdp, FUNC(v9958_device::read), FUNC(v9958_device::write));

	// Reserved by IO_DECODER.pld for future phases:
	// $60-$6f cartridge I/O,
	// $e0-$ff SOUND on write and CTRL on read.
}

u8 zephyr80_state::mem_r(offs_t offset)
{
	// MEM_DECODER.pld (Rev 09) read-side decode:
	//   ROM_CS = !MREQ & RD & !ROM_DIS &
	//            ((RAM_SHADOW & !SAFE_RAM) # (!RAM_SHADOW & (BIOS_RANGE # SAFE_RAM)))
	// In normal mode ROM is visible in the BIOS window ($0000-$5FFF) and the
	// high common window ($C000-$FFFF); in shadow/copy mode ROM covers
	// $0000-$BFFF while the high 16K reads SRAM.  Everything not selecting ROM
	// selects SRAM (the SRAM_CS read terms are the exact complement here), and
	// ROM-disabled mode reads SRAM everywhere.
	bool const rom_cs = !rom_disabled()
		&& ((ram_shadow() && !safe_ram(offset))
			|| (!ram_shadow() && (bios_range(offset) || safe_ram(offset))));

	return rom_cs ? rom_r(offset) : ram_r(offset);
}

void zephyr80_state::mem_w(offs_t offset, u8 data)
{
	// MEM_DECODER.pld (Rev 09): SRAM_CS carries an unconditional WR term, so
	// every CPU write lands in SRAM (ROM is never written).  selected_ram_bank
	// applies the BANK_Q selection and the SAFE_RAM force-bank-0 common rule.
	ram_w(offset, data);
}

u8 zephyr80_state::mem_latch_r()
{
	return m_latch;
}

void zephyr80_state::mem_latch_w(u8 data)
{
	// 74HC273 banking latch: D0-D2 SRAM bank, D3 RAM_SHADOW, D4 ROM_DIS,
	// D5-D7 ROM page.  All eight bits are latched.
	m_latch = data & zephyr80_map::latch::WRITABLE_MASK;
}

u8 zephyr80_state::sio0_r(offs_t offset)
{
	return m_sio0->ba_cd_r(offset & zephyr80_map::io::SIO_REGISTER_MASK);
}

void zephyr80_state::sio0_w(offs_t offset, u8 data)
{
	offs_t const sio_offset = offset & zephyr80_map::io::SIO_REGISTER_MASK;

	switch (sio_offset)
	{
	case zephyr80_map::io::SIO_CHAN_A_DATA:
		log_sio_tx(0, "[SIO0-A]", data);
		break;
	case zephyr80_map::io::SIO_CHAN_B_DATA:
		log_sio_tx(1, "[SIO0-B]", data);
		break;
	}

	m_sio0->ba_cd_w(sio_offset, data);
}

u8 zephyr80_state::sio1_r(offs_t offset)
{
	return m_sio1->ba_cd_r(offset & zephyr80_map::io::SIO_REGISTER_MASK);
}

void zephyr80_state::sio1_w(offs_t offset, u8 data)
{
	offs_t const sio_offset = offset & zephyr80_map::io::SIO_REGISTER_MASK;

	switch (sio_offset)
	{
	case zephyr80_map::io::SIO_CHAN_A_DATA:
		m_coffeeio->rx_a_w(data);
		break;
	case zephyr80_map::io::SIO_CHAN_B_DATA:
		logerror("%s: SIO1-B reserved TX byte %02x\n", machine().describe_context(), data);
		m_coffeeio->rx_b_w(data);
		break;
	}

	m_sio1->ba_cd_w(sio_offset, data);
}

void zephyr80_state::log_sio_tx(unsigned channel, const char *prefix, u8 data)
{
	if (channel >= 4)
		return;

	if (m_sio_tx_line_start[channel])
		osd_printf_info("%s ", prefix);

	osd_printf_info("%c", data);
	m_sio_tx_line_start[channel] = (data == '\r') || (data == '\n');
}

void zephyr80_state::coffeeio_a_response_w(u8 data)
{
	if (m_coffeeio_rx_count == COFFEEIO_RX_QUEUE_SIZE)
	{
		logerror("%s: COFFEE-IO RX queue overflow, dropping %02x\n", machine().describe_context(), data);
		return;
	}

	m_coffeeio_rx_queue[m_coffeeio_rx_tail] = data;
	m_coffeeio_rx_tail = (m_coffeeio_rx_tail + 1) % COFFEEIO_RX_QUEUE_SIZE;
	m_coffeeio_rx_count++;
	coffeeio_start_rx();
}

void zephyr80_state::coffeeio_start_rx()
{
	if (m_coffeeio_rx_timer_active || !m_coffeeio_rx_count)
		return;

	m_coffeeio_rx_timer_active = true;
	m_coffeeio_rx_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(zephyr80_state::coffeeio_rx_tick)
{
	if (!m_coffeeio_rx_count)
	{
		m_coffeeio_rx_timer_active = false;
		return;
	}

	u8 const data = m_coffeeio_rx_queue[m_coffeeio_rx_head];
	m_coffeeio_rx_head = (m_coffeeio_rx_head + 1) % COFFEEIO_RX_QUEUE_SIZE;
	m_coffeeio_rx_count--;

	// The internal Phase 10 backend operates at byte-packet level.  Bytes are
	// paced at the equivalent of 9600 8N1 so the real Z80SIO receive FIFO is
	// not overrun while software observes RR0 RX-ready and data-port reads.
	m_sio1->rxa_byte_w(data);
	if (m_coffeeio_rx_count)
		m_coffeeio_rx_timer->adjust(attotime::from_hz(COFFEEIO_RX_BYTE_RATE));
	else
		m_coffeeio_rx_timer_active = false;
}

bool zephyr80_state::bios_range(offs_t offset) const
{
	return offset <= zephyr80_map::mem::BIOS_RANGE_END;
}

bool zephyr80_state::safe_ram(offs_t offset) const
{
	// SAFE_RAM = A15 & A14 -> $C000-$FFFF (high 16K common area).
	return offset >= zephyr80_map::mem::SAFE_RAM_START;
}

bool zephyr80_state::ram_shadow() const
{
	return bool(m_latch & zephyr80_map::latch::RAM_SHADOW);
}

bool zephyr80_state::rom_disabled() const
{
	return bool(m_latch & zephyr80_map::latch::ROM_DIS);
}

u8 zephyr80_state::rom_page() const
{
	return (m_latch & zephyr80_map::latch::ROM_PAGE_MASK) >> zephyr80_map::latch::ROM_PAGE_SHIFT;
}

u8 zephyr80_state::selected_ram_bank(offs_t offset) const
{
	// MEM_DECODER.pld:
	//   FORCE_BANK0 = SAFE_RAM & (ROM_DIS # RAM_SHADOW)
	//   RAM_A16..A18 = BANK_Q0..2 & !FORCE_BANK0
	// The high 16K common area resolves to SRAM bank 0 whenever ROM is disabled
	// or shadow/copy mode is active, so it stays shared across all banks.
	if (safe_ram(offset) && (rom_disabled() || ram_shadow()))
		return 0;

	return m_latch & zephyr80_map::latch::BANK_MASK;
}

u8 zephyr80_state::rom_r(offs_t offset) const
{
	// ROM page (latch D5-D7) drives ROM A16-A18; the CPU supplies A0-A15.
	u32 const rom_addr = (u32(rom_page()) << 16) | (offset & 0xffff);
	u8 const *rom = m_rom_region->base();
	return (rom_addr < m_rom_region->bytes()) ? rom[rom_addr] : 0xff;
}

u8 zephyr80_state::ram_r(offs_t offset) const
{
	return m_ram[(selected_ram_bank(offset) * zephyr80_map::mem::RAM_BANK_SIZE) + (offset & 0xffff)];
}

void zephyr80_state::ram_w(offs_t offset, u8 data)
{
	m_ram[(selected_ram_bank(offset) * zephyr80_map::mem::RAM_BANK_SIZE) + (offset & 0xffff)] = data;
}

// IM2 interrupt daisy chain.  Priority order (highest first) is a hardware
// property still to be confirmed against the CPU/IO board; CTC ahead of the
// SIOs is the conventional Z80 arrangement and is functionally moot today since
// the firmware disables the CTC and polls SIO1 (only SIO0/B raises interrupts).
static const z80_daisy_config zephyr80_daisy[] =
{
	{ "ctc" },
	{ "sio0" },
	{ "sio1" },
	{ nullptr }
};

void zephyr80_state::zephyr80(machine_config &config)
{
	Z80(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &zephyr80_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &zephyr80_state::io_map);
	m_maincpu->set_daisy_config(zephyr80_daisy);

	Z80SIO(config, m_sio0, 10_MHz_XTAL);
	m_sio0->set_cputag(m_maincpu);
	m_sio0->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_sio0->out_txda_callback().set("sio0a_rs232", FUNC(rs232_port_device::write_txd));
	m_sio0->out_dtra_callback().set("sio0a_rs232", FUNC(rs232_port_device::write_dtr));
	m_sio0->out_rtsa_callback().set("sio0a_rs232", FUNC(rs232_port_device::write_rts));
	m_sio0->out_txdb_callback().set("sio0b_rs232", FUNC(rs232_port_device::write_txd));
	m_sio0->out_dtrb_callback().set("sio0b_rs232", FUNC(rs232_port_device::write_dtr));
	m_sio0->out_rtsb_callback().set("sio0b_rs232", FUNC(rs232_port_device::write_rts));

	rs232_port_device &sio0a_rs232(RS232_PORT(config, "sio0a_rs232", pbitz_rs232_devices, nullptr));
	sio0a_rs232.rxd_handler().set(m_sio0, FUNC(z80sio_device::rxa_w));
	sio0a_rs232.cts_handler().set(m_sio0, FUNC(z80sio_device::ctsa_w));
	sio0a_rs232.dcd_handler().set(m_sio0, FUNC(z80sio_device::dcda_w));

	rs232_port_device &sio0b_rs232(RS232_PORT(config, "sio0b_rs232", pbitz_rs232_devices, nullptr));
	sio0b_rs232.rxd_handler().set(m_sio0, FUNC(z80sio_device::rxb_w));
	sio0b_rs232.cts_handler().set(m_sio0, FUNC(z80sio_device::ctsb_w));
	sio0b_rs232.dcd_handler().set(m_sio0, FUNC(z80sio_device::dcdb_w));
	sio0b_rs232.set_option_device_input_defaults("pty", DEVICE_INPUT_DEFAULTS_NAME(console_baud_115200));
	sio0b_rs232.set_option_device_input_defaults("loopback", DEVICE_INPUT_DEFAULTS_NAME(console_baud_115200));

	Z80SIO(config, m_sio1, 10_MHz_XTAL);
	m_sio1->set_cputag(m_maincpu);
	m_sio1->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	// CTC: four channels at $40-$43.  On real hardware the console baud is fixed
	// (not CTC-derived); only the user channel (SIO0/A) takes its clock from the
	// CTC.  Route CTC channel 0 to SIO0/A rx/tx.  The CTC clock crystal and the
	// exact channel->baud/tick assignment are hardware details still to be
	// confirmed; the firmware only disables the CTC at boot.
	Z80CTC(config, m_ctc, 10_MHz_XTAL);
	m_ctc->intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_ctc->zc_callback<0>().set(m_sio0, FUNC(z80sio_device::rxca_w));
	m_ctc->zc_callback<0>().append(m_sio0, FUNC(z80sio_device::txca_w));

	// V9958 video card at $a0-$bf, driven directly by CP/M/BIOS I/O.  The card's
	// /INT is wired to the CPU maskable interrupt: it is not part of the Z80 IM2
	// daisy chain, so on interrupt-acknowledge the (pulled-high) data bus returns
	// $ff -- the Z80 core reproduces this by falling back to the default vector
	// when no daisy device is the source, giving IM2 vector $ff (I:$ff).
	V9958(config, m_vdp, 21.477272_MHz_XTAL);
	m_vdp->set_screen_ntsc("screen");
	m_vdp->set_vram_size(0x20000); // 128 KiB
	m_vdp->int_cb().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	SCREEN(config, "screen", SCREEN_TYPE_RASTER);

	PBITZ_COFFEEIO(config, m_coffeeio);
	m_coffeeio->response_a_callback().set(FUNC(zephyr80_state::coffeeio_a_response_w));

	// Console (SIO0/B) baud is fixed in hardware at 115200 8N1.  The firmware
	// programs WR4 for a x16 clock, so feed channel B a 115200 x 16 clock.
	clock_device &sio0b_clock(CLOCK(config, "sio0b_clock", 115'200 * 16));
	sio0b_clock.signal_handler().set(m_sio0, FUNC(z80sio_device::rxtxcb_w));

	// SIO1 is the synchronous IO-controller link, externally clocked by the MCU
	// glue during a transaction.  These placeholder clocks let the MAME SIO shift
	// and report TX-empty/RX-ready until the COFFEE-IO transport drives them.
	clock_device &sio1a_clock(CLOCK(config, "sio1a_clock", 9'600 * 16));
	sio1a_clock.signal_handler().set(m_sio1, FUNC(z80sio_device::rxca_w));
	sio1a_clock.signal_handler().append(m_sio1, FUNC(z80sio_device::txca_w));

	clock_device &sio1b_clock(CLOCK(config, "sio1b_clock", 9'600 * 16));
	sio1b_clock.signal_handler().set(m_sio1, FUNC(z80sio_device::rxtxcb_w));
}

ROM_START(zephyr80)
	// 512 KiB ROM address space (8 pages x 64 KiB, latch D5-D7 = page).  The
	// current CP/M firmware populates pages 0-1 (128 KiB); higher pages read 0xff.
	ROM_REGION(zephyr80_map::mem::ROM_SIZE, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("zephyr80.bin", 0x00000, 0x20000, CRC(458d5603) SHA1(3aed6264ab20cd30140e49c7e9c8faa705c00998))
ROM_END

} // anonymous namespace


COMP(2026, zephyr80, 0, 0, zephyr80, 0, zephyr80_state, empty_init,
	"Kitamura", "Zephyr-80 prototype decode shell", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
