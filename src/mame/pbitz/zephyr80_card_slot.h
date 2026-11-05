// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_CARD_SLOT_H
#define MAME_PBITZ_ZEPHYR80_CARD_SLOT_H

#pragma once

#include "zephyr80_card.h"

// The two ways media reach the IO controller's one card socket
// (zephyr80_card_socket): `-card FILE` inserts a card image, `-carddir DIR`
// a host folder.  They are exclusive: loading one while the socket holds
// media from the other is refused.  Unloading ejects the card; the
// controller is not told (it notices through a failing access).  The socket
// itself opens the path, so neither device creates media.
class zephyr80_card_slot_device : public device_t, public device_image_interface
{
public:
	void set_socket(zephyr80_card_socket &socket) { m_socket = &socket; }

	virtual bool is_readable() const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return true; }
	virtual bool is_creatable() const noexcept override { return false; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool core_opens_image_file() const noexcept override { return false; }
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual void call_unload() override;

protected:
	zephyr80_card_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	virtual void device_start() override ATTR_COLD;
	virtual std::string insert(std::string const &path) = 0;

	zephyr80_card_socket *m_socket;
};

class zephyr80_card_image_device : public zephyr80_card_slot_device
{
public:
	zephyr80_card_image_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	virtual const char *file_extensions() const noexcept override { return "img,bin"; }
	virtual const char *image_type_name() const noexcept override { return "card"; }
	virtual const char *image_brief_type_name() const noexcept override { return "card"; }

protected:
	virtual std::string insert(std::string const &path) override { return m_socket->insert_image(path); }
};

class zephyr80_card_folder_device : public zephyr80_card_slot_device
{
public:
	zephyr80_card_folder_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	virtual const char *file_extensions() const noexcept override { return ""; }
	virtual bool is_directory_media() const noexcept override { return true; }
	virtual const char *image_type_name() const noexcept override { return "carddir"; }
	virtual const char *image_brief_type_name() const noexcept override { return "carddir"; }

protected:
	virtual std::string insert(std::string const &path) override { return m_socket->insert_folder(path); }
};

DECLARE_DEVICE_TYPE(ZEPHYR80_CARD_IMAGE, zephyr80_card_image_device)
DECLARE_DEVICE_TYPE(ZEPHYR80_CARD_FOLDER, zephyr80_card_folder_device)

#endif // MAME_PBITZ_ZEPHYR80_CARD_SLOT_H
