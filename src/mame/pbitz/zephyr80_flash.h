// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_FLASH_H
#define MAME_PBITZ_ZEPHYR80_FLASH_H

#pragma once

#include "zephyr80_map.h"

// Read-only board flash.  Programming commands and electrical timing are not
// modelled; media changes take effect immediately, with an explicit CPU reset
// needed to boot the replacement image.
//
// The contents are the host image, so a save state records their identity
// (the loaded length and a SHA-1 of the 512 KiB) instead of a copy, and a
// state restored with other flash contents is refused with a fatal error.
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
	virtual void device_pre_save() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

private:
	void identify(std::array<u8, 20> &digest) const;

	std::array<u8, zephyr80_map::mem::ROM_SIZE> m_data;
	u32 m_length;                     // bytes the image supplied (0: erased)
	u32 m_saved_length;               // the identity a state carries
	std::array<u8, 20> m_saved_digest;
};

DECLARE_DEVICE_TYPE(ZEPHYR80_FLASH, zephyr80_flash_device)

#endif // MAME_PBITZ_ZEPHYR80_FLASH_H
