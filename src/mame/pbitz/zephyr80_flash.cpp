// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_flash.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_FLASH, zephyr80_flash_device, "zephyr80_flash", "Zephyr-80 flash image")

zephyr80_flash_device::zephyr80_flash_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ZEPHYR80_FLASH, tag, owner, clock)
	, device_image_interface(mconfig, *this)
{
	m_data.fill(0xff);
}

void zephyr80_flash_device::device_start()
{
	save_item(NAME(m_data));
}

std::pair<std::error_condition, std::string> zephyr80_flash_device::call_load()
{
	m_data.fill(0xff);
	u64 size;
	if (auto const err = image_core_file().length(size))
		return { err, "Cannot determine flash image size" };
	if (!size || size > m_data.size())
		return { image_error::INVALIDLENGTH, "Flash image must contain 1 to 524288 bytes (512 KiB); use -flash image.bin" };

	auto const [err, actual] = read_at(image_core_file(), 0, m_data.data(), size);
	if (err || actual != size)
	{
		m_data.fill(0xff);
		return { err ? err : std::make_error_condition(std::errc::io_error), "Cannot read complete flash image" };
	}
	return { std::error_condition(), std::string() };
}

void zephyr80_flash_device::call_unload()
{
	m_data.fill(0xff);
}
