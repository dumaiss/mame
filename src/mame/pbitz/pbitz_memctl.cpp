// license:BSD-3-Clause
// copyright-holders:Kitamura

#include "emu.h"

#include "pbitz_memctl.h"


DEFINE_DEVICE_TYPE(PBITZ_MEMCTL, pbitz_memctl_device, "pbitz_memctl", "pBITz Zephyr memory controller proof")

pbitz_memctl_device::pbitz_memctl_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, PBITZ_MEMCTL, tag, owner, clock)
	, m_program_space(*this, finder_base::DUMMY_TAG, -1)
	, m_rom_region(*this, finder_base::DUMMY_TAG)
	, m_low_rom_bank(*this, finder_base::DUMMY_TAG)
	, m_mid_bank(*this, finder_base::DUMMY_TAG)
	, m_high_ram_bank(*this, finder_base::DUMMY_TAG)
{
}

void pbitz_memctl_device::device_start()
{
	u8 *rom = m_rom_region->base();

	m_bank_ram = std::make_unique<u8[]>(RAM_BANKS * BANK_SIZE);
	std::fill_n(m_bank_ram.get(), RAM_BANKS * BANK_SIZE, 0x00);

	m_low_rom_bank->configure_entries(0, ROM_BANKS, rom, BANK_SIZE);

	// Phase 3 still models only the banking behavior needed by the Zephyr
	// bank-test ROM.  This is a proof-of-concept memory controller for MAME,
	// not final hardware-accurate Zephyr decode logic.
	m_mid_bank->configure_entries(0, ROM_BANKS, rom, BANK_SIZE);
	m_mid_bank->configure_entries(ROM_BANKS, RAM_BANKS, m_bank_ram.get(), BANK_SIZE);

	m_high_ram_bank->configure_entries(0, RAM_BANKS, m_bank_ram.get(), BANK_SIZE);

	save_pointer(NAME(m_bank_ram), RAM_BANKS * BANK_SIZE);
}

void pbitz_memctl_device::device_reset()
{
	reset_banks();
}

void pbitz_memctl_device::reset_banks()
{
	low_rom_bank_w(0x00);  // 0000-3FFF -> ROM bank 0
	mid_bank_w(0x01);      // 4000-7FFF -> ROM bank 1
	high_ram_bank_w(0x00); // 8000-BFFF -> RAM bank 0
}

void pbitz_memctl_device::low_rom_bank_w(u8 data)
{
	m_low_rom_bank->set_entry(safe_bank(data, ROM_BANKS));
}

void pbitz_memctl_device::mid_bank_w(u8 data)
{
	const u8 bank = safe_bank(data, BIT(data, 7) ? RAM_BANKS : ROM_BANKS);

	if (BIT(data, 7))
	{
		m_mid_bank->set_entry(ROM_BANKS + bank);
		m_program_space->install_readwrite_bank(0x4000, 0x7fff, m_mid_bank);
	}
	else
	{
		m_mid_bank->set_entry(bank);
		m_program_space->install_read_bank(0x4000, 0x7fff, m_mid_bank);
		m_program_space->unmap_write(0x4000, 0x7fff);
	}
}

void pbitz_memctl_device::high_ram_bank_w(u8 data)
{
	m_high_ram_bank->set_entry(safe_bank(data, RAM_BANKS));
}

u8 pbitz_memctl_device::safe_bank(u8 data, u32 bank_count) const
{
	return bank_count ? (data % bank_count) : 0;
}
