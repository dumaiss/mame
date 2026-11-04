// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_card_slot.h"

DEFINE_DEVICE_TYPE(ZEPHYR80_CARD_IMAGE, zephyr80_card_image_device, "zephyr80_card", "Zephyr-80 SD card image")
DEFINE_DEVICE_TYPE(ZEPHYR80_CARD_FOLDER, zephyr80_card_folder_device, "zephyr80_carddir", "Zephyr-80 SD card folder")

zephyr80_card_slot_device::zephyr80_card_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_image_interface(mconfig, *this)
	, m_socket(nullptr)
{
}

zephyr80_card_image_device::zephyr80_card_image_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: zephyr80_card_slot_device(mconfig, ZEPHYR80_CARD_IMAGE, tag, owner, clock)
{
}

zephyr80_card_folder_device::zephyr80_card_folder_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: zephyr80_card_slot_device(mconfig, ZEPHYR80_CARD_FOLDER, tag, owner, clock)
{
}

void zephyr80_card_slot_device::device_start()
{
	if (!m_socket)
		throw emu_fatalerror("%s: no card socket attached", tag());
}

std::pair<std::error_condition, std::string> zephyr80_card_slot_device::call_load()
{
	if (m_socket->inserted())
		return { image_error::ALREADYOPEN, "the card socket already holds media; -card and -carddir are exclusive" };
	std::string const error = insert(filename());
	if (!error.empty())
		return { image_error::INVALIDIMAGE, error };
	return { std::error_condition(), std::string() };
}

void zephyr80_card_slot_device::call_unload()
{
	m_socket->eject();
}
