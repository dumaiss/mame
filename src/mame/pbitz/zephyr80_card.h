// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_CARD_H
#define MAME_PBITZ_ZEPHYR80_CARD_H

#pragma once

#include "zephyr80_ioc_storage.h"

#include <array>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// The IO controller's SD card socket: the zephyr80_ioc_storage backend.
//
// Two kinds of media go in it.  A card image (a file of 512-byte blocks,
// raw or FAT) is served as the firmware serves a card: FatFs (the
// firmware's own copy and configuration, 3rdparty/fatfs) over its blocks for
// the filesystem, the firmware's resolver (src/fatmap.c fatmap_resolve) for
// CP/M volume images, and the blocks themselves for the raw unit.  A host
// folder is served as a FAT volume with no card blocks: 8.3 names, no LFN,
// FatFs results, no raw unit; its volume images are host files.
//
// What survives a media change is what lives in the controller's RAM: the
// card session (card_ready), the FatFs mount and the open objects.  The
// firmware never notices a swap by itself (src/sd_card.c): an access with
// the socket empty fails (NO_CARD) and ends the session; the first read of a
// replaced card re-initialises it and succeeds (sd_card_read_block's retry);
// the first write to one fails (no retry) and ends the session.  Any other
// failed access ends the session too, except a read-only medium refusing a
// write.  The core learns of a change only through such a failure.
//
// Bounds: at most 16 objects (files, directories, volume images) at once;
// one 512-byte FatFs window (FF_FS_TINY); transfers read and write only the
// bytes asked for.
class zephyr80_card_socket : public zephyr80_ioc_storage
{
public:
	zephyr80_card_socket();
	~zephyr80_card_socket();

	zephyr80_card_socket(zephyr80_card_socket const &) = delete;
	zephyr80_card_socket &operator=(zephyr80_card_socket const &) = delete;

	// Insert a card image or a host folder; an empty string, or why not.
	std::string insert_image(std::string const &path);
	std::string insert_folder(std::string const &path);
	void eject();
	bool inserted() const { return bool(m_media); }

	// zephyr80_ioc_storage
	virtual card card_init() override;
	virtual bool card_ready() const override { return m_ready; }
	virtual void controller_reset() override;
	virtual bool raw_blocks() const override;
	virtual card block_read(u32 lba, u8 *data) override;
	virtual card block_write(u32 lba, u8 const *data) override;
	virtual card flush() override;
	virtual card fs_mount(bool &mounted) override;
	virtual void fs_unmount() override;
	virtual fr fs_geometry(geometry &info) override;
	virtual fr stat(char const *path, entry &info) override;
	virtual fr open(char const *path, u8 mode, object &file) override;
	virtual fr close(object file) override;
	virtual u32 size(object file) const override;
	virtual fr seek(object file, u32 offset) override;
	virtual fr read(object file, u8 *data, u16 length, u16 &done) override;
	virtual fr write(object file, u8 const *data, u16 length, u16 &done) override;
	virtual fr truncate(object file) override;
	virtual fr sync(object file) override;
	virtual fr opendir(char const *path, object &dir) override;
	virtual fr readdir(object dir, entry &info) override;
	virtual fr closedir(object dir) override;
	virtual fr unlink(char const *path) override;
	virtual fr mkdir(char const *path) override;
	virtual fr rename(char const *from, char const *to) override;
	virtual image_status image_open(u8 const *dir11, u8 const *name11, u32 bytes, image &info, object &img) override;
	virtual void image_close(object img) override;
	virtual card image_read(object img, u32 block, u8 *data) override;
	virtual card image_write(object img, u32 block, u8 const *data) override;

	// FatFs's media layer (diskio), for zephyr80_card.cpp's glue only.
	u8 disk_status() const;
	u8 disk_initialize();
	bool disk_read(u8 *data, u32 lba, unsigned count);
	bool disk_write(u8 const *data, u32 lba, unsigned count);
	bool disk_sync();

	static constexpr unsigned OBJECTS = 16;
	static constexpr unsigned IMAGE_EXTENTS = 16;   // VOL_MAX_EXTENTS

private:
	enum class kind : u8 { IMAGE, FOLDER };

	struct media
	{
		kind type;
		std::filesystem::path path;   // the image file, or the folder's root
		bool read_only;
		u32 serial;
		std::FILE *file;              // image
		u64 blocks;                   // image
	};

	// One name in a host directory, as the folder presents it.
	struct host_entry
	{
		std::array<u8, 11> sfn;
		std::filesystem::path host;
		bool dir;
		bool ambiguous;
	};

	struct walk_result;

	enum class slot : u8 { FREE, FAT_FILE, FAT_DIR, HOST_FILE, HOST_DIR, IMAGE_BLOCKS, IMAGE_HOST };

	struct extent
	{
		u32 start;
		u32 blocks;
	};

	struct objslot
	{
		slot type = slot::FREE;
		bool valid = false;           // host objects: false after an unmount
		u32 serial = 0;               // host objects and images: the media they belong to
		std::FILE *file = nullptr;    // host file or host volume image
		std::filesystem::path host;
		bool writable = false;
		u64 position = 0;
		u64 length = 0;
		std::vector<host_entry> listing;
		size_t next = 0;
		std::array<extent, IMAGE_EXTENTS> extents{};
		u8 extent_count = 0;
	};

	// Card blocks, with the session rules above.  `coherent` keeps the
	// FatFs window and the raw access in step (the raw and volume paths).
	card card_read(u32 lba, u8 *data, bool coherent);
	card card_write(u32 lba, u8 const *data, bool coherent);

	std::string insert(std::unique_ptr<media> &&m);
	fr allocate(slot type, object &id);
	void release(object id);
	objslot *fat_slot(object id, slot type);
	fr host_slot(object id, slot type, objslot *&o);

	// Folder filesystem
	fr folder_access(bool writing);
	fr host_list(std::filesystem::path const &dir, std::vector<host_entry> &out) const;
	fr walk(char const *path, walk_result &out);
	void host_info(host_entry const &e, entry &info) const;
	fr folder_stat(char const *path, entry &info);
	fr folder_open(char const *path, u8 mode, object &file);
	fr folder_opendir(char const *path, object &dir);
	fr folder_unlink(char const *path);
	fr folder_mkdir(char const *path);
	fr folder_rename(char const *from, char const *to);
	fr folder_geometry(geometry &info);
	image_status folder_image(u8 const *dir11, u8 const *name11, u32 bytes, image &info, object &img);

	// Card images: fatmap_resolve
	image_status fatmap_resolve(u8 const *dir11, u8 const *name11, u32 bytes, objslot &o);

	std::unique_ptr<media> m_media;
	u32 m_next_serial;
	bool m_ready;          // card_ready
	bool m_replaced;       // the media changed while the session was up
	bool m_fat_mounted;    // sdfs fs_mounted, over the card's blocks
	bool m_folder_mounted; // the folder's equivalent
	struct fatfs_state;                   // FatFs's FATFS, FIL and DIR objects
	std::unique_ptr<fatfs_state> m_fat;
	std::array<objslot, OBJECTS> m_objects;
};

#endif // MAME_PBITZ_ZEPHYR80_CARD_H
