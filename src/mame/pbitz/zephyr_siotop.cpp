// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_zsio.h"

#include "cpu/z80/z80.h"


namespace {

class zephyr_siotop_state : public driver_device
{
public:
	zephyr_siotop_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_sio_ext(*this, "sio_ext")
		, m_sio_int(*this, "sio_int")
	{
	}

	void zephyr_siotop(machine_config &config);

private:
	required_device<z80_device> m_maincpu;
	required_device<pbitz_zsio_device> m_sio_ext;
	required_device<pbitz_zsio_device> m_sio_int;
	bool m_sio_ext_irq = false;
	bool m_sio_int_irq = false;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
	void sio_ext_irq_w(int state);
	void sio_int_irq_w(int state);
	void update_irq();
};

void zephyr_siotop_state::mem_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("maincpu", 0);
	map(0x8000, 0xffff).ram();
}

void zephyr_siotop_state::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x30, 0x33).rw(m_sio_ext, FUNC(pbitz_zsio_device::read), FUNC(pbitz_zsio_device::write));
	map(0x34, 0x37).rw(m_sio_int, FUNC(pbitz_zsio_device::read), FUNC(pbitz_zsio_device::write));
}

void zephyr_siotop_state::sio_ext_irq_w(int state)
{
	m_sio_ext_irq = bool(state);
	update_irq();
}

void zephyr_siotop_state::sio_int_irq_w(int state)
{
	m_sio_int_irq = bool(state);
	update_irq();
}

void zephyr_siotop_state::update_irq()
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, (m_sio_ext_irq || m_sio_int_irq) ? ASSERT_LINE : CLEAR_LINE);
}

void zephyr_siotop_state::zephyr_siotop(machine_config &config)
{
	Z80(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &zephyr_siotop_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &zephyr_siotop_state::io_map);

	PBITZ_ZSIO(config, m_sio_ext);
	m_sio_ext->set_channel_config(0, pbitz_zsio_device::MODE_ASYNC_EXTERNAL, "[EXT-A]", "extA\r");
	m_sio_ext->set_channel_config(1, pbitz_zsio_device::MODE_ASYNC_EXTERNAL, "[EXT-B]", "extB\r");
	m_sio_ext->irq_callback().set(FUNC(zephyr_siotop_state::sio_ext_irq_w));

	PBITZ_ZSIO(config, m_sio_int);
	m_sio_int->set_channel_config(0, pbitz_zsio_device::MODE_SYNC_INTERNAL, "[INT-A]", "intA\r");
	m_sio_int->set_channel_config(1, pbitz_zsio_device::MODE_SYNC_INTERNAL, "[INT-B]", "intB\r");
	m_sio_int->irq_callback().set(FUNC(zephyr_siotop_state::sio_int_irq_w));
}

ROM_START(zephyr_siotop)
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD("siotop.bin", 0x0000, 0x2000, BAD_DUMP CRC(00000000) SHA1(0000000000000000000000000000000000000000))
ROM_END

} // anonymous namespace


COMP(2026, zephyr_siotop, 0, 0, zephyr_siotop, 0, zephyr_siotop_state, empty_init,
	"Kitamura", "Zephyr-80 SIO topology proof test", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
