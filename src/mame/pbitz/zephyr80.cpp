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
#include "machine/z80sio.h"

#include <array>


namespace {

void pbitz_rs232_devices(device_slot_interface &device)
{
	device.option_add("pty", PSEUDO_TERMINAL);
	device.option_add("loopback", RS232_LOOPBACK);
}

class zephyr80_state : public driver_device
{
public:
	zephyr80_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_sio0(*this, "sio0")
		, m_sio1(*this, "sio1")
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
	required_device<pbitz_coffeeio_device> m_coffeeio;
	required_memory_region m_rom_region;

	static constexpr unsigned COFFEEIO_RX_QUEUE_SIZE = 256;
	static constexpr unsigned COFFEEIO_RX_BYTE_RATE = 960; // 9600 8N1 characters/second

	std::unique_ptr<u8[]> m_ram;
	u8 m_latch = 0;
	bool m_cart_detect = false;
	bool m_prog = false;
	bool m_sio0_irq = false;
	bool m_sio1_irq = false;
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
	void sio0_irq_w(int state);
	void sio1_irq_w(int state);
	void update_irq();
	void log_sio_tx(unsigned channel, const char *prefix, u8 data);
	void coffeeio_a_response_w(u8 data);
	void coffeeio_start_rx();
	TIMER_CALLBACK_MEMBER(coffeeio_rx_tick);

	bool bios_range(offs_t offset) const;
	bool common_ram(offs_t offset) const;
	bool ram_only(offs_t offset) const;
	bool upper_32k(offs_t offset) const;
	bool ram_shadow() const;
	bool rom_disabled() const;
	bool all_ram_mode() const;
	u8 selected_ram_bank(offs_t offset) const;
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
	save_item(NAME(m_cart_detect));
	save_item(NAME(m_prog));
	save_item(NAME(m_sio0_irq));
	save_item(NAME(m_sio1_irq));
	save_item(NAME(m_sio_tx_line_start));
	save_item(NAME(m_coffeeio_rx_queue));
	save_item(NAME(m_coffeeio_rx_head));
	save_item(NAME(m_coffeeio_rx_tail));
	save_item(NAME(m_coffeeio_rx_count));
	save_item(NAME(m_coffeeio_rx_timer_active));
}

void zephyr80_state::machine_reset()
{
	m_latch = 0;
	m_cart_detect = false; // No cartridge emulation in Phase 8.
	m_prog = false;        // Programming mode is represented but not functional yet.
	m_sio0_irq = false;
	m_sio1_irq = false;
	for (bool &line_start : m_sio_tx_line_start)
		line_start = true;
	m_coffeeio_rx_head = 0;
	m_coffeeio_rx_tail = 0;
	m_coffeeio_rx_count = 0;
	m_coffeeio_rx_timer_active = false;
	m_coffeeio_rx_timer->adjust(attotime::never);
	m_sio1->rxa_w(1);
	m_sio1->rxb_w(1);
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

	// IO_DECODER.pld: SIO0 at $20-$2f and SIO1 at $30-$3f.  Phase 9 uses real
	// MAME Z80SIO devices with cd_ba register order mirrored through each
	// 16-byte PLD block: +0 A data, +1 B data, +2 A control, +3 B control.
	map(io::SIO0_START, io::SIO0_START + io::SIO_REGISTER_MASK).mirror(io::SIO_REGISTER_MIRROR).rw(FUNC(zephyr80_state::sio0_r), FUNC(zephyr80_state::sio0_w));
	map(io::SIO1_START, io::SIO1_START + io::SIO_REGISTER_MASK).mirror(io::SIO_REGISTER_MIRROR).rw(FUNC(zephyr80_state::sio1_r), FUNC(zephyr80_state::sio1_w));

	// Reserved by IO_DECODER.pld for future phases:
	// $40-$4f CTC, $60-$6f cartridge I/O, $a0-$bf VDP,
	// $e0-$ff SOUND on write and CTRL on read.
}

u8 zephyr80_state::mem_r(offs_t offset)
{
	// MEM_DECODER.pld read-side behavior:
	// ROM_CS = !MREQ & (PROG # (RD & !ROM_DIS & (RAM_SHADOW # BIOS_RANGE)))
	// CART_CS is represented but no cartridge backing exists yet.
	// SRAM_CS for reads is active in RAM_ONLY, UPPER_32K without cartridge,
	// and BIOS_RANGE when ROM_DIS exposes all-RAM mode.
	bool const rom_cs = m_prog || (!rom_disabled() && (ram_shadow() || bios_range(offset)));
	bool const cart_cs = !m_prog && upper_32k(offset) && m_cart_detect && !ram_shadow();

	if (rom_cs)
	{
		u8 const *rom = m_rom_region->base();
		u32 const rom_bytes = m_rom_region->bytes();
		return (offset < rom_bytes) ? rom[offset] : 0xff;
	}

	if (cart_cs)
		return 0xff;

	if (!m_prog && !ram_shadow() && ((bios_range(offset) && rom_disabled()) || ram_only(offset) || (upper_32k(offset) && !m_cart_detect)))
		return ram_r(offset);

	return 0xff;
}

void zephyr80_state::mem_w(offs_t offset, u8 data)
{
	// MEM_DECODER.pld write-side SRAM_CS behavior:
	// SRAM_CS = !MREQ & !PROG & (RAM_SHADOW & WR # !RAM_SHADOW &
	//   (BIOS_RANGE & (WR # ROM_DIS) # RAM_ONLY # UPPER_32K & !CART_DETECT)).
	bool const sram_cs = !m_prog && (ram_shadow() || (!ram_shadow() && (bios_range(offset) || ram_only(offset) || (upper_32k(offset) && !m_cart_detect))));

	if (sram_cs)
		ram_w(offset, data);
}

u8 zephyr80_state::mem_latch_r()
{
	return m_latch;
}

void zephyr80_state::mem_latch_w(u8 data)
{
	// Provisional Phase 8 latch layout:
	// bit 0-2 BANK_Q0..2, bit 3 RAM_SHADOW, bit 4 ROM_DIS, bit 5-7 reserved.
	m_latch = data & zephyr80_map::latch::WRITABLE_MASK;
}

u8 zephyr80_state::sio0_r(offs_t offset)
{
	return m_sio0->cd_ba_r(offset & zephyr80_map::io::SIO_REGISTER_MASK);
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

	m_sio0->cd_ba_w(sio_offset, data);
}

u8 zephyr80_state::sio1_r(offs_t offset)
{
	return m_sio1->cd_ba_r(offset & zephyr80_map::io::SIO_REGISTER_MASK);
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

	m_sio1->cd_ba_w(sio_offset, data);
}

void zephyr80_state::sio0_irq_w(int state)
{
	m_sio0_irq = bool(state);
	update_irq();
}

void zephyr80_state::sio1_irq_w(int state)
{
	m_sio1_irq = bool(state);
	update_irq();
}

void zephyr80_state::update_irq()
{
	// Phase 9 uses a simple wired-OR IRQ approximation.  The real SIO/CTC/Pio
	// interrupt daisy chain can be added once the remaining Z80 peripherals are
	// present in the Zephyr-80 machine.
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, (m_sio0_irq || m_sio1_irq) ? ASSERT_LINE : CLEAR_LINE);
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

bool zephyr80_state::common_ram(offs_t offset) const
{
	return offset <= zephyr80_map::mem::COMMON_RAM_END;
}

bool zephyr80_state::ram_only(offs_t offset) const
{
	return (offset >= zephyr80_map::mem::RAM_ONLY_START) && (offset <= zephyr80_map::mem::RAM_ONLY_END);
}

bool zephyr80_state::upper_32k(offs_t offset) const
{
	return offset >= zephyr80_map::mem::UPPER_32K_START;
}

bool zephyr80_state::ram_shadow() const
{
	return bool(m_latch & zephyr80_map::latch::RAM_SHADOW);
}

bool zephyr80_state::rom_disabled() const
{
	return bool(m_latch & zephyr80_map::latch::ROM_DIS);
}

bool zephyr80_state::all_ram_mode() const
{
	return rom_disabled() && !m_cart_detect;
}

u8 zephyr80_state::selected_ram_bank(offs_t offset) const
{
	// MEM_DECODER.pld:
	// ALL_RAM_MODE = ROM_DIS && !CART_DETECT
	// FORCE_BANK0 = COMMON_RAM && ALL_RAM_MODE
	// RAM_A16..A18 = BANK_Q0..2 && !FORCE_BANK0
	if (common_ram(offset) && all_ram_mode())
		return 0;

	return m_latch & (zephyr80_map::latch::BANK_Q0 | zephyr80_map::latch::BANK_Q1 | zephyr80_map::latch::BANK_Q2);
}

u8 zephyr80_state::ram_r(offs_t offset) const
{
	return m_ram[(selected_ram_bank(offset) * zephyr80_map::mem::RAM_BANK_SIZE) + (offset & 0xffff)];
}

void zephyr80_state::ram_w(offs_t offset, u8 data)
{
	m_ram[(selected_ram_bank(offset) * zephyr80_map::mem::RAM_BANK_SIZE) + (offset & 0xffff)] = data;
}

void zephyr80_state::zephyr80(machine_config &config)
{
	Z80(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &zephyr80_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &zephyr80_state::io_map);

	Z80SIO(config, m_sio0, 4_MHz_XTAL);
	m_sio0->set_cputag(m_maincpu);
	m_sio0->out_int_callback().set(FUNC(zephyr80_state::sio0_irq_w));
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

	Z80SIO(config, m_sio1, 4_MHz_XTAL);
	m_sio1->set_cputag(m_maincpu);
	m_sio1->out_int_callback().set(FUNC(zephyr80_state::sio1_irq_w));

	PBITZ_COFFEEIO(config, m_coffeeio);
	m_coffeeio->response_a_callback().set(FUNC(zephyr80_state::coffeeio_a_response_w));

	// Temporary Phase 9 clocks: enough to let the real MAME SIO shift TX data
	// and return TX-empty/RX-ready status.  Final baud-rate generation and
	// CTC/SIO clock topology still need to be derived from the hardware.
	clock_device &sio0a_clock(CLOCK(config, "sio0a_clock", 9'600 * 16));
	sio0a_clock.signal_handler().set(m_sio0, FUNC(z80sio_device::rxca_w));
	sio0a_clock.signal_handler().append(m_sio0, FUNC(z80sio_device::txca_w));

	clock_device &sio0b_clock(CLOCK(config, "sio0b_clock", 9'600 * 16));
	sio0b_clock.signal_handler().set(m_sio0, FUNC(z80sio_device::rxtxcb_w));

	clock_device &sio1a_clock(CLOCK(config, "sio1a_clock", 9'600 * 16));
	sio1a_clock.signal_handler().set(m_sio1, FUNC(z80sio_device::rxca_w));
	sio1a_clock.signal_handler().append(m_sio1, FUNC(z80sio_device::txca_w));

	clock_device &sio1b_clock(CLOCK(config, "sio1b_clock", 9'600 * 16));
	sio1b_clock.signal_handler().set(m_sio1, FUNC(z80sio_device::rxtxcb_w));
}

ROM_START(zephyr80)
	ROM_REGION(0x80000, "maincpu", ROMREGION_ERASEFF)
	ROM_LOAD("zephyr80.bin", 0x0000, 0x2000, BAD_DUMP CRC(00000000) SHA1(0000000000000000000000000000000000000000))
ROM_END

} // anonymous namespace


COMP(2026, zephyr80, 0, 0, zephyr80, 0, zephyr80_state, empty_init,
	"Kitamura", "Zephyr-80 prototype decode shell", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
