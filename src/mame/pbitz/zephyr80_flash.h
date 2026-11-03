// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_FLASH_H
#define MAME_PBITZ_ZEPHYR80_FLASH_H

#pragma once

#include "zephyr80_map.h"

// Read-only board flash.  Programming commands and electrical timing are not
// modelled; media changes take effect immediately, with an explicit CPU reset
// needed to boot the replacement image.
class zephyr80_flash_device : public device_t, public device_image_interface
{
public:
	zephyr80_flash_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	u8 read(u32 address) const { return m_data[address]; }

	virtual bool is_readable() const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return false; }
	virtual bool is_creatable() const noexcept override { return false; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual const char *file_extensions() const noexcept override { return "bin,rom"; }
	virtual const char *image_type_name() const noexcept override { return "flash"; }
	virtual const char *image_brief_type_name() const noexcept override { return "flash"; }
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual void call_unload() override;

protected:
	virtual void device_start() override ATTR_COLD;

private:
	std::array<u8, zephyr80_map::mem::ROM_SIZE> m_data;
};

DECLARE_DEVICE_TYPE(ZEPHYR80_FLASH, zephyr80_flash_device)

#endif // MAME_PBITZ_ZEPHYR80_FLASH_H
