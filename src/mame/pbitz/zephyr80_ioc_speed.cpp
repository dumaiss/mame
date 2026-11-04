// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc_speed.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_IOC_SPEED, zephyr80_ioc_speed_slot_device, "zephyr80_ioc_speed", "Zephyr-80 IO controller speed")
DEFINE_DEVICE_TYPE(ZEPHYR80_IOC_PACED, zephyr80_ioc_paced_device, "zephyr80_ioc_paced", "IO controller lanes paced on the controller's byte periods")
DEFINE_DEVICE_TYPE(ZEPHYR80_IOC_FAST, zephyr80_ioc_fast_device, "zephyr80_ioc_fast", "IO controller lanes clocked as fast as the Z80 takes them")

zephyr80_ioc_speed_slot_device::zephyr80_ioc_speed_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_IOC_SPEED, tag, owner, clock)
	, device_single_card_slot_interface<zephyr80_ioc_speed_interface>(mconfig, *this)
	, m_mode(nullptr)
{
}

void zephyr80_ioc_speed_slot_device::device_config_complete()
{
	m_mode = get_card_device();
}

void zephyr80_ioc_speed_slot_device::device_start()
{
}

zephyr80_ioc_paced_device::zephyr80_ioc_paced_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_IOC_PACED, tag, owner, clock)
	, zephyr80_ioc_speed_interface(mconfig, *this)
{
}

void zephyr80_ioc_paced_device::device_start()
{
}

zephyr80_ioc_fast_device::zephyr80_ioc_fast_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_IOC_FAST, tag, owner, clock)
	, zephyr80_ioc_speed_interface(mconfig, *this)
{
}

void zephyr80_ioc_fast_device::device_start()
{
}
