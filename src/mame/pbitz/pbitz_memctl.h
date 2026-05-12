// license:BSD-3-Clause
// copyright-holders:Kitamura

#ifndef MAME_PBITZ_PBITZ_MEMCTL_H
#define MAME_PBITZ_PBITZ_MEMCTL_H

#pragma once


class pbitz_memctl_device : public device_t
{
public:
	pbitz_memctl_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T> void set_program_space(T &&tag, int spacenum) { m_program_space.set_tag(std::forward<T>(tag), spacenum); }
	template <typename T> void set_rom_region(T &&tag) { m_rom_region.set_tag(std::forward<T>(tag)); }
	template <typename T> void set_low_rom_bank(T &&tag) { m_low_rom_bank.set_tag(std::forward<T>(tag)); }
	template <typename T> void set_mid_bank(T &&tag) { m_mid_bank.set_tag(std::forward<T>(tag)); }
	template <typename T> void set_high_ram_bank(T &&tag) { m_high_ram_bank.set_tag(std::forward<T>(tag)); }

	void reset_banks();

	void low_rom_bank_w(u8 data);
	void mid_bank_w(u8 data);
	void high_ram_bank_w(u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr u32 BANK_SIZE = 0x4000;
	static constexpr u32 ROM_BANKS = 4;
	static constexpr u32 RAM_BANKS = 4;

	u8 safe_bank(u8 data, u32 bank_count) const;

	required_address_space m_program_space;
	required_memory_region m_rom_region;
	required_memory_bank m_low_rom_bank;
	required_memory_bank m_mid_bank;
	required_memory_bank m_high_ram_bank;

	std::unique_ptr<u8[]> m_bank_ram;
};

DECLARE_DEVICE_TYPE(PBITZ_MEMCTL, pbitz_memctl_device)

#endif // MAME_PBITZ_PBITZ_MEMCTL_H
