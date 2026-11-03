// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "zephyr80_map.h"
#include "zephyr80_flash.h"

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


namespace {

void pbitz_rs232_devices(device_slot_interface &device)
{
	device.option_add("pty", PSEUDO_TERMINAL);
	device.option_add("loopback", RS232_LOOPBACK);
}

// The SIO0/B console link runs at a fixed 115200 8N1 (1.8432 MHz x16), so default
// any device attached to that port to 115200.
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
		, m_flash(*this, "flash")
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
	required_device<zephyr80_flash_device> m_flash;

	std::unique_ptr<u8[]> m_ram;
	u8 m_latch = 0;

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
	u8 ctc_r(offs_t offset);
	void ctc_w(offs_t offset, u8 data);

	void vdp_int_w(int state);
	void daisy_int_w(int state);
	void update_irq();
	IRQ_CALLBACK_MEMBER(unclaimed_ack);

	u8 m_vdp_int = 0;
};

void zephyr80_state::machine_start()
{
	m_ram = std::make_unique<u8[]>(zephyr80_map::mem::RAM_SIZE);
	std::fill_n(m_ram.get(), zephyr80_map::mem::RAM_SIZE, 0x00);

	save_pointer(NAME(m_ram), zephyr80_map::mem::RAM_SIZE);
	save_item(NAME(m_latch));
	save_item(NAME(m_vdp_int));
}

void zephyr80_state::machine_reset()
{
	m_latch = 0; // Mode 00, flash page 0, SRAM bank 0.
	m_sio1->rxa_w(1);
	m_sio1->rxb_w(1);
	m_vdp_int = 0; // V9958 reset clears its /INT output.
	update_irq();
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

	// IO_DECODER.pld: CTC at $40-$4f, A3:A2 mirrored.  CS0/CS1 are wired to
	// A1/A0 (see zephyr80_map::io::ctc_channel): $40 ch0, $41 ch2, $42 ch1, $43 ch3.
	map(io::CTC_START, io::CTC_START + io::CTC_REGISTER_MASK).mirror(io::CTC_REGISTER_MIRROR).rw(FUNC(zephyr80_state::ctc_r), FUNC(zephyr80_state::ctc_w));

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
	auto const mapping = zephyr80_map::decode(u16(offset), m_latch);
	return mapping.read_flash ? m_flash->read(mapping.read_address) : m_ram[mapping.read_address];
}

void zephyr80_state::mem_w(offs_t offset, u8 data)
{
	m_ram[zephyr80_map::decode(u16(offset), m_latch).write_address] = data;
}

u8 zephyr80_state::mem_latch_r()
{
	return m_latch;
}

void zephyr80_state::mem_latch_w(u8 data)
{
	// 74HC273: Q=D2:D0, binary mode=D4:D3, P=D7:D5.
	m_latch = data;
}

u8 zephyr80_state::sio0_r(offs_t offset)
{
	return m_sio0->ba_cd_r(offset & zephyr80_map::io::SIO_REGISTER_MASK);
}

void zephyr80_state::sio0_w(offs_t offset, u8 data)
{
	m_sio0->ba_cd_w(offset & zephyr80_map::io::SIO_REGISTER_MASK, data);
}

u8 zephyr80_state::sio1_r(offs_t offset)
{
	return m_sio1->ba_cd_r(offset & zephyr80_map::io::SIO_REGISTER_MASK);
}

void zephyr80_state::sio1_w(offs_t offset, u8 data)
{
	m_sio1->ba_cd_w(offset & zephyr80_map::io::SIO_REGISTER_MASK, data);
}

u8 zephyr80_state::ctc_r(offs_t offset)
{
	return m_ctc->read(zephyr80_map::io::ctc_channel(offset & zephyr80_map::io::CTC_REGISTER_MASK));
}

void zephyr80_state::ctc_w(offs_t offset, u8 data)
{
	m_ctc->write(zephyr80_map::io::ctc_channel(offset & zephyr80_map::io::CTC_REGISTER_MASK), data);
}

// /INT (IO board net `~{INT}`, bus CTRL_14, CPU pin 16) is a wired-OR of
// open-drain requests with a backplane pull-up (RN7).  The daisy devices
// (CTC, SIO0, SIO1) request only while their IEI is high, which the Z80 core
// evaluates from the chain on every IRQ0 update.  So IRQ0 carries the level of
// the sources outside the chain (the V9958) and every daisy change re-presents
// it: the core then asserts the CPU line while any source requests, and no
// source releasing its request clears another's.
void zephyr80_state::update_irq()
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, m_vdp_int ? ASSERT_LINE : CLEAR_LINE);
}

void zephyr80_state::vdp_int_w(int state)
{
	// The V9958 reports its level every scanline; forward only changes.
	if (u8(state != CLEAR_LINE) != m_vdp_int)
	{
		m_vdp_int = state != CLEAR_LINE;
		update_irq();
	}
}

void zephyr80_state::daisy_int_w(int)
{
	update_irq();
}

// An acknowledge that no daisy device claims (the V9958's request) reads the
// idle data bus: the backplane pulls D0-D7 high (RN5/RN6/RN7, 4.7 kOhm), so the
// IM2 vector is $ff.
IRQ_CALLBACK_MEMBER(zephyr80_state::unclaimed_ack)
{
	return 0xff;
}

// IM2 daisy chain, highest priority first, from the IO board nets: bus CTRL_18
// (backplane RN5 pull-up) -> `/IO Controller Card/IEI` -> CTC IEI;
// `Net-(IC1-IEO)` CTC IEO -> SIO0 IEI; `Net-(IC2-IEO)` SIO0 IEO -> SIO1 IEI;
// SIO1 IEO -> `/IO Controller Card/IEO` (bus CTRL_17, unused).  The V9958 is
// outside the chain.
static const z80_daisy_config zephyr80_daisy[] =
{
	{ "ctc" },
	{ "sio0" },
	{ "sio1" },
	{ nullptr }
};

void zephyr80_state::zephyr80(machine_config &config)
{
	ZEPHYR80_FLASH(config, m_flash, 0);
	m_flash->set_must_be_loaded(true);

	Z80(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &zephyr80_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &zephyr80_state::io_map);
	m_maincpu->set_daisy_config(zephyr80_daisy);
	m_maincpu->set_irq_acknowledge_callback(FUNC(zephyr80_state::unclaimed_ack));

	Z80SIO(config, m_sio0, 10_MHz_XTAL);
	m_sio0->set_cputag(m_maincpu);
	m_sio0->out_int_callback().set(FUNC(zephyr80_state::daisy_int_w));
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
	m_sio1->out_int_callback().set(FUNC(zephyr80_state::daisy_int_w));

	// CTC: CLK is the 10 MHz bus clock (`CLK_10M`), so timer mode counts
	// 10 MHz through the 16/256 prescaler.  CLK/TRG0-3 come from the 74HC4040
	// (U8) dividing the 14.7456 MHz oscillator Y1: `/Baud Clock/CLK_1M8432`
	// (Q2) -> TRG0, `CLK_3M6864` (Q1) -> TRG1, `CLK_7M3728` (Q0) -> TRG2 and
	// TRG3.  The channel clocks give counter mode its rate and start an
	// externally triggered timer one trigger period after its constant; edge
	// polarity and the phase between the divider outputs are not modelled.
	// ZC/TO0 (`USR_UART_BAUD_CLK`) clocks SIO0/A RxCA and TxCA.
	Z80CTC(config, m_ctc, 10_MHz_XTAL);
	m_ctc->set_clk<0>(14.7456_MHz_XTAL / 8);
	m_ctc->set_clk<1>(14.7456_MHz_XTAL / 4);
	m_ctc->set_clk<2>(14.7456_MHz_XTAL / 2);
	m_ctc->set_clk<3>(14.7456_MHz_XTAL / 2);
	m_ctc->intr_callback().set(FUNC(zephyr80_state::daisy_int_w));
	m_ctc->zc_callback<0>().set(m_sio0, FUNC(z80sio_device::rxca_w));
	m_ctc->zc_callback<0>().append(m_sio0, FUNC(z80sio_device::txca_w));

	// V9958 video card at $a0-$bf.  Its /INT shares the CPU maskable interrupt
	// outside the daisy chain; its acknowledge reads $ff (unclaimed_ack).
	V9958(config, m_vdp, 21.477272_MHz_XTAL);
	m_vdp->set_screen_ntsc("screen");
	m_vdp->set_vram_size(0x20000); // 128 KiB
	m_vdp->int_cb().set(FUNC(zephyr80_state::vdp_int_w));

	SCREEN(config, "screen", SCREEN_TYPE_RASTER);

	// Console SIO0/B RxTxCB is `/Baud Clock/CLK_1M8432` (U8 Q2), the same net as
	// CTC TRG0; guests program x16, giving 115200 baud.
	clock_device &sio0b_clock(CLOCK(config, "sio0b_clock", 14.7456_MHz_XTAL / 8));
	sio0b_clock.signal_handler().set(m_sio0, FUNC(z80sio_device::rxtxcb_w));

	// SIO1 is the IO-controller link.  These free-running clocks are
	// placeholders that keep the unconnected channels shifting; they do not
	// model the link's clocking.
	clock_device &sio1a_clock(CLOCK(config, "sio1a_clock", 9'600 * 16));
	sio1a_clock.signal_handler().set(m_sio1, FUNC(z80sio_device::rxca_w));
	sio1a_clock.signal_handler().append(m_sio1, FUNC(z80sio_device::txca_w));

	clock_device &sio1b_clock(CLOCK(config, "sio1b_clock", 9'600 * 16));
	sio1b_clock.signal_handler().set(m_sio1, FUNC(z80sio_device::rxtxcb_w));
}

ROM_START(zephyr80)
ROM_END

} // anonymous namespace


COMP(2026, zephyr80, 0, 0, zephyr80, 0, zephyr80_state, empty_init,
	"Kitamura", "Zephyr-80 prototype decode shell", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
