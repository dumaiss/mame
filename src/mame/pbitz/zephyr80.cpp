// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "zephyr80_map.h"
#include "zephyr80_card.h"
#include "zephyr80_card_slot.h"
#include "zephyr80_flash.h"
#include "zephyr80_ioc.h"
#include "zephyr80_ioc_speed.h"
#include "zephyr80_sio1.h"

#include "bus/rs232/loopback.h"
#include "bus/rs232/pty.h"
#include "bus/rs232/rs232.h"
#include "cpu/z80/z80.h"
#include "machine/clock.h"
#include "machine/z80ctc.h"
#include "machine/z80daisy.h"
#include "machine/z80sio.h"
#include "sound/dac.h"
#include "sound/sn76496.h"
#include "video/v9938.h"

#include "screen.h"
#include "speaker.h"


namespace {

void pbitz_rs232_devices(device_slot_interface &device)
{
	device.option_add("pty", PSEUDO_TERMINAL);
	device.option_add("loopback", RS232_LOOPBACK);
}

// -ioc_speed: how the IO controller's lanes keep time.
void pbitz_ioc_speed_devices(device_slot_interface &device)
{
	device.option_add("paced", ZEPHYR80_IOC_PACED);
	device.option_add("fast", ZEPHYR80_IOC_FAST);
}

// The SIO0/B console link runs at a fixed 115200 8N1 (1.8432 MHz x16), so default
// any device attached to that port to 115200.
static DEVICE_INPUT_DEFAULTS_START(console_baud_115200)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_115200)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_115200)
DEVICE_INPUT_DEFAULTS_END

// A card input's weight in the mono mix: the inverting stage's 10 kOhm
// feedback over each side's input resistor (kOhm), the two sides averaged,
// times the scale.
constexpr double mix_gain(double left_kohm, double right_kohm)
{
	constexpr double MIX_SCALE = 0.66;
	return MIX_SCALE * 0.5 * (10.0 / left_kohm + 10.0 / right_kohm);
}

class zephyr80_state : public driver_device
{
public:
	zephyr80_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_sio0(*this, "sio0")
		, m_sio1(*this, "sio1")
		, m_ioc(*this, "ioc")
		, m_ctc(*this, "ctc")
		, m_vdp(*this, "vdp")
		, m_psg(*this, "psg%u", 0U)
		, m_dac(*this, "dac")
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
	required_device<zephyr80_sio1_device> m_sio1;
	required_device<zephyr80_ioc_device> m_ioc;
	required_device<z80ctc_device> m_ctc;
	required_device<v9958_device> m_vdp;
	required_device_array<sn76489a_device, 4> m_psg;
	required_device<dac_8bit_r2r_device> m_dac;
	required_device<zephyr80_flash_device> m_flash;

	// The IO controller's SD socket; -card and -carddir insert its media.
	zephyr80_card_socket m_card;

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
	u8 vdp_r(offs_t offset);
	void vdp_w(offs_t offset, u8 data);
	u8 ctrl_r();
	void sound_w(offs_t offset, u8 data);
	void psg_ready_w(int state);

	void vdp_int_w(int state);
	void daisy_int_w(int state);
	void update_irq();
	IRQ_CALLBACK_MEMBER(unclaimed_ack);

	u8 m_vdp_int = 0;    // V9958 /INT level
	u8 m_vdp_config = 0; // LunchCrema 74HC273: bit 0 route, bit 1 ~WS_EN
};

void zephyr80_state::machine_start()
{
	m_ram = std::make_unique<u8[]>(zephyr80_map::mem::RAM_SIZE);
	std::fill_n(m_ram.get(), zephyr80_map::mem::RAM_SIZE, 0x00);

	save_pointer(NAME(m_ram), zephyr80_map::mem::RAM_SIZE);
	save_item(NAME(m_latch));
	save_item(NAME(m_vdp_int));
	save_item(NAME(m_vdp_config));
}

void zephyr80_state::machine_reset()
{
	m_latch = 0; // Mode 00, flash page 0, SRAM bank 0.
	m_vdp_int = 0; // V9958 reset clears its /INT output.
	m_vdp_config = 0; // ~RESET clears the 74HC273.
	update_irq();
	// ~RESET reaches the AD7801 ~CLR, which zeroes its DAC register.  The PSGs
	// have no reset input and keep their registers.
	m_dac->data_w(0x00);
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

	// IO_DECODER.pld: LunchCrema V9958 card at $a0-$bf.  A1:A0 select the four
	// V9958 ports: +0 VRAM data, +1 command/status, +2 palette, +3 register
	// (read: +0 VRAM, +1 status).  A write with A2 set is the interrupt
	// configuration latch instead (see vdp_w).
	map(io::VDP_START, io::VDP_END).rw(FUNC(zephyr80_state::vdp_r), FUNC(zephyr80_state::vdp_w));

	// IO_DECODER.pld: $e0-$ff is SOUND on write and CTRL on read.  The Afternoon
	// Blend card decodes only A2:A0 (74AHCT138 U8), so its eight ports repeat
	// four times across the block (see sound_w).  Reads are the controller
	// latches: all ones until they exist, and no sound device sees a read.
	map(io::SOUND_CTRL_START, io::SOUND_CTRL_END).rw(FUNC(zephyr80_state::ctrl_r), FUNC(zephyr80_state::sound_w));

	// Reserved by IO_DECODER.pld for future phases:
	// $60-$6f cartridge I/O.
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

u8 zephyr80_state::vdp_r(offs_t offset)
{
	// 74AHCT139 half 1 (A0 = ~RD, A1 = GND): every read in the window is ~CSR.
	return m_vdp->read(offset & zephyr80_map::io::VDP_REGISTER_MASK);
}

void zephyr80_state::vdp_w(offs_t offset, u8 data)
{
	if (zephyr80_map::io::vdp_config_select(offset))
	{
		// ~INT_MODE clocks the 74HC273.  D1 (~WS_EN) only enables the card's
		// WAIT generator, which this machine does not model: it is stored and
		// has no timing effect.  A latch write re-routes an asserted request.
		m_vdp_config = data & zephyr80_map::io::VDP_CONFIG_MASK;
		update_irq();
	}
	else
	{
		m_vdp->write(offset & zephyr80_map::io::VDP_REGISTER_MASK, data);
	}
}

u8 zephyr80_state::ctrl_r()
{
	return 0xff;
}

// Afternoon Blend (Sound.kicad_sch).  U8 (74AHCT138) decodes A2:A0 with /WR on
// its enable: Y0-Y3 select PSG0-PSG3 (SN76489AN, pin 6 /CE), Y4 the AD7801
// (/CS) and Y5-Y7 are unconnected, so writes to ports 5-7 do nothing.  The bus
// D7 (MSB) reaches the PSG pin the datasheet calls D0 (its MSB) through U4, so
// the PSGs take conventional bytes (bit 7 = latch) and the AD7801 an offset
// binary code, DB7 = MSB.
//
// A PSG write also sets U5A, which holds the bus /WAIT low through Q1 until
// READY_ALL (the open-collector READY pins of the four PSGs) rises: the chip
// holds READY low for 32 clocks of its input after a write.  The Z80 core has
// no WAIT input model, so the cycle is stretched by suspending the CPU until
// the stock device's READY rises (psg_ready_w).
void zephyr80_state::sound_w(offs_t offset, u8 data)
{
	unsigned const port = offset & zephyr80_map::io::SOUND_PORT_MASK;
	if (port < zephyr80_map::io::SOUND_PSG_COUNT)
	{
		m_psg[port]->write(data);
		if (!m_psg[port]->ready_r())
			m_maincpu->suspend_until_trigger(zephyr80_map::io::SOUND_READY_TRIGGER, true);
	}
	else if (port == zephyr80_map::io::SOUND_DAC_PORT)
	{
		m_dac->data_w(data);
	}
}

void zephyr80_state::psg_ready_w(int)
{
	// READY_ALL is the wired-AND of the four READY outputs.
	for (auto const &psg : m_psg)
		if (!psg->ready_r())
			return;
	machine().scheduler().trigger(zephyr80_map::io::SOUND_READY_TRIGGER);
}

// /INT (IO board net `~{INT}`, bus CTRL_14, CPU pin 16) is a wired-OR of
// open-drain requests with a backplane pull-up (RN7).  The daisy devices
// (CTC, SIO0) request only while their IEI is high, which the Z80 core
// evaluates from the chain on every IRQ0 update.  So IRQ0 carries the level of
// the sources outside the chain (the V9958) and every daisy change re-presents
// it: the core then asserts the CPU line while any source requests, and no
// source releasing its request clears another's.
//
// The LunchCrema card steers the V9958 ~INT with the 74HC273 bit Q0 through
// the 74LVC1G19 (U8): Q0 = 0 drives /Video/~INT (CTRL_14, the wired-OR above),
// Q0 = 1 drives /Video/~NMI (CTRL_15), which the CPU board wires to the Z80
// ~NMI (J2.A12 -> U3.17).  An NMI is not acknowledged and no vector is read.
void zephyr80_state::update_irq()
{
	bool const nmi = m_vdp_config & zephyr80_map::io::VDP_CONFIG_NMI;
	m_maincpu->set_input_line(INPUT_LINE_NMI, m_vdp_int && nmi ? ASSERT_LINE : CLEAR_LINE);
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, m_vdp_int && !nmi ? ASSERT_LINE : CLEAR_LINE);
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
// SIO1 IEO -> `/IO Controller Card/IEO` (bus CTRL_17, unused).  SIO1 is the
// IO-controller link, polled with its interrupts disabled by both guests; its
// model (zephyr80_sio1) has no interrupt logic and passes IEI to IEO.  The
// V9958 is outside the chain.
static const z80_daisy_config zephyr80_daisy[] =
{
	{ "ctc" },
	{ "sio0" },
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

	// SIO1: the IO-controller link, both channels External Sync x1 lanes
	// clocked by the controller (A bulk, B command).
	ZEPHYR80_SIO1(config, m_sio1);
	ZEPHYR80_IOC(config, m_ioc);
	m_sio1->set_client(*m_ioc);
	m_ioc->set_lane(*m_sio1);
	m_ioc->set_speed(ZEPHYR80_IOC_SPEED(config, "ioc_speed", pbitz_ioc_speed_devices, "paced"));
	m_ioc->set_storage(m_card);
	ZEPHYR80_CARD_IMAGE(config, "card").set_socket(m_card);
	ZEPHYR80_CARD_FOLDER(config, "carddir").set_socket(m_card);

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

	// V9958 video card at $a0-$bf.  Its /INT joins the CPU maskable interrupt
	// outside the daisy chain (acknowledge reads $ff, unclaimed_ack) or drives
	// the CPU NMI, as the card's configuration latch selects (update_irq).
	V9958(config, m_vdp, 21.477272_MHz_XTAL);
	m_vdp->set_screen_ntsc("screen");
	m_vdp->set_vram_size(0x20000); // 128 KiB
	m_vdp->int_cb().set(FUNC(zephyr80_state::vdp_int_w));

	SCREEN(config, "screen", SCREEN_TYPE_RASTER);

	// Afternoon Blend sound card at $e0-$ff (writes): four SN76489AN PSGs on
	// the card's 3.579545 MHz oscillator (Y1) and an AD7801 DAC, mixed to one
	// speaker.  The card's analogue stage sums them with inverting op-amp
	// stages and mixes left and right for this mono output; the gains below
	// are its resistor ratios (feedback 10 kOhm over the input resistor,
	// left and right averaged) times one scale that keeps four PSGs at full
	// level plus the DAC at its positive extreme just under full scale.
	// Device output levels (the PSG's output swing against the DAC's 0-5 V)
	// are not in the schematic: PSG full scale and DAC full scale are taken
	// as equal.
	SPEAKER(config, "mono").front_center();
	for (unsigned i = 0; i < m_psg.size(); i++)
		SN76489A(config, m_psg[i], 3.579545_MHz_XTAL).ready_cb().set(FUNC(zephyr80_state::psg_ready_w));
	// PSG0, PSG1: 33 kOhm to both sides.  PSG2: 22 kOhm left, 47 kOhm right.
	// PSG3: 47 kOhm left, 22 kOhm right.  DAC: 47 kOhm to both sides.
	m_psg[0]->add_route(ALL_OUTPUTS, "mono", mix_gain(33, 33));
	m_psg[1]->add_route(ALL_OUTPUTS, "mono", mix_gain(33, 33));
	m_psg[2]->add_route(ALL_OUTPUTS, "mono", mix_gain(22, 47));
	m_psg[3]->add_route(ALL_OUTPUTS, "mono", mix_gain(47, 22));
	// The DAC is stock 8-bit offset binary: 80h is zero, 00h is -1.0.
	DAC_8BIT_R2R(config, m_dac).add_route(ALL_OUTPUTS, "mono", mix_gain(47, 47));

	// Console SIO0/B RxTxCB is `/Baud Clock/CLK_1M8432` (U8 Q2), the same net as
	// CTC TRG0; guests program x16, giving 115200 baud.
	clock_device &sio0b_clock(CLOCK(config, "sio0b_clock", 14.7456_MHz_XTAL / 8));
	sio0b_clock.signal_handler().set(m_sio0, FUNC(z80sio_device::rxtxcb_w));
}

ROM_START(zephyr80)
ROM_END

} // anonymous namespace


COMP(2026, zephyr80, 0, 0, zephyr80, 0, zephyr80_state, empty_init,
	"Kitamura", "Zephyr-80 prototype decode shell", MACHINE_NOT_WORKING)
