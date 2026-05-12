// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_memctl.h"
#include "pbitz_serial.h"

#include "cpu/z80/z80.h"


namespace {

class zephyr_banktst_state : public driver_device
{
public:
	zephyr_banktst_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_memctl(*this, "memctl")
		, m_serial(*this, "serial")
	{
	}

	void zephyr_banktst(machine_config &config);

private:
	required_device<z80_device> m_maincpu;
	required_device<pbitz_memctl_device> m_memctl;
	required_device<pbitz_serial_device> m_serial;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
};

void zephyr_banktst_state::mem_map(address_map &map)
{
	map(0x0000, 0x3fff).bankr("bank0000");   // 16 KiB banked ROM window
	map(0x4000, 0x7fff).bankr("bank4000");   // 16 KiB banked ROM/RAM proof window
	map(0x8000, 0xbfff).bankrw("bank8000");  // 16 KiB banked RAM window
	map(0xc000, 0xffff).ram();               // 16 KiB fixed RAM
}

void zephyr_banktst_state::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0x00).w(m_serial, FUNC(pbitz_serial_device::data_w)); // Legacy console output compatibility
	map(0x10, 0x10).w(m_memctl, FUNC(pbitz_memctl_device::low_rom_bank_w));
	map(0x11, 0x11).w(m_memctl, FUNC(pbitz_memctl_device::mid_bank_w));
	map(0x12, 0x12).w(m_memctl, FUNC(pbitz_memctl_device::high_ram_bank_w));
	map(0x20, 0x20).r(m_serial, FUNC(pbitz_serial_device::status_r));
	map(0x21, 0x21).rw(m_serial, FUNC(pbitz_serial_device::data_r), FUNC(pbitz_serial_device::data_w));
	map(0x22, 0x22).rw(m_serial, FUNC(pbitz_serial_device::control_r), FUNC(pbitz_serial_device::control_w));
}

void zephyr_banktst_state::zephyr_banktst(machine_config &config)
{
	Z80(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &zephyr_banktst_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &zephyr_banktst_state::io_map);

	PBITZ_MEMCTL(config, m_memctl);
	m_memctl->set_program_space(m_maincpu, AS_PROGRAM);
	m_memctl->set_rom_region(m_maincpu);
	m_memctl->set_low_rom_bank("bank0000");
	m_memctl->set_mid_bank("bank4000");
	m_memctl->set_high_ram_bank("bank8000");

	PBITZ_SERIAL(config, m_serial);
	m_serial->irq_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
}

ROM_START(zephyr_banktst)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("banktest.bin", 0x00000, 0x10000, BAD_DUMP CRC(00000000) SHA1(0000000000000000000000000000000000000000))
ROM_END

} // anonymous namespace


COMP(2026, zephyr_banktst, 0, 0, zephyr_banktst, 0, zephyr_banktst_state, empty_init,
	"Kitamura", "Zephyr-80 banking proof test", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
