// license:BSD-3-Clause
#include "emu.h"
#include "cpu/z80/z80.h"

namespace {

class coffeez80_state : public driver_device
{
public:
    coffeez80_state(const machine_config &mconfig, device_type type, const char *tag)
        : driver_device(mconfig, type, tag)
        , m_maincpu(*this, "maincpu")
    {
    }

    void coffeez80(machine_config &config);

private:
    required_device<z80_device> m_maincpu;

    void mem_map(address_map &map)
    {
        map(0x0000, 0x1fff).rom();
        map(0x8000, 0xffff).ram();
    }

    void io_map(address_map &map)
    {
        map.global_mask(0xff);
        map(0x00, 0x00).w(FUNC(coffeez80_state::console_w));
    }

    void console_w(uint8_t data)
    {
        osd_printf_info("%c", data);
    }
};

void coffeez80_state::coffeez80(machine_config &config)
{
    Z80(config, m_maincpu, 10_MHz_XTAL);
    m_maincpu->set_addrmap(AS_PROGRAM, &coffeez80_state::mem_map);
    m_maincpu->set_addrmap(AS_IO, &coffeez80_state::io_map);
}

ROM_START(coffeez80)
    ROM_REGION(0x2000, "maincpu", 0)
    ROM_LOAD("boot.bin", 0x0000, 0x2000, CRC(00000000) SHA1(0000000000000000000000000000000000000000))
ROM_END

} // anonymous namespace

COMP(2026, coffeez80, 0, 0, coffeez80, 0, coffeez80_state, empty_init,
     "Kitamura", "Coffee Z80 Test Machine", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
