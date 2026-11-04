// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_STORAGE_H
#define MAME_PBITZ_ZEPHYR80_IOC_STORAGE_H

#pragma once

// The media behind the IO controller's storage service: what
// zephyr80_ioc_core asks of a card backend.  The shape is the firmware's own
// layering (Code/MCU/IOController): the SD card session and its blocks
// (src/sd_card.c, src/sd_cache.c), the one FatFs volume (src/sdfs.c and the
// FatFs calls in src/fs2.c and src/fs_share.c) and the CP/M volume images
// (src/fatmap.c fatmap_resolve).  Results use the firmware's own codes
// (SdStatus, FRESULT, FatMapStatus); the core converts them to wire statuses
// exactly as the firmware does (ioc_status_from_sd, fs2.c map_result,
// fs_share.c fres_to_ioc, volume.c vol_mount), so no backend converts.
//
// Ownership.  The core owns every protocol object: FS2 tokens, cookies,
// cached positions, the resolver path (cwd) and the media generation; the
// legacy /SHARED handle; the volume unit table; transfer state and buffers.
// The backend owns the media and the objects open on it (files, directories,
// images), named by an `object` number of its choosing.  Paths are the
// firmware's FatFs paths, built by the core: absolute, '/'-separated 8.3
// components ("/SHARED/NAME.EXT"), at most 239 characters.  The backend
// resolves them on its media; nothing here is a host path.
//
// The core keeps at most eight objects open at once: two FS2 files, one FS2
// directory, the /SHARED file and directory, the self-test's directory or
// file, and two volume images.
class zephyr80_ioc_storage
{
public:
	// SdStatus (include/sd_card.h), in its order.
	enum class card : u8
	{
		OK, NO_CARD, NO_RESPONSE, UNUSABLE, NOT_READY, READ, NO_TOKEN, CRC, BUS,
		WRITE, WRITE_REJECTED, WRITE_BUSY
	};

	// FRESULT (FatFs ff.h), in its order: the self-test reports the raw value.
	enum class fr : u8
	{
		OK, DISK_ERR, INT_ERR, NOT_READY, NO_FILE, NO_PATH, INVALID_NAME, DENIED,
		EXIST, INVALID_OBJECT, WRITE_PROTECTED, INVALID_DRIVE, NOT_ENABLED,
		NO_FILESYSTEM, MKFS_ABORTED, TIMEOUT, LOCKED, NOT_ENOUGH_CORE,
		TOO_MANY_OPEN_FILES, INVALID_PARAMETER
	};

	// FatMapStatus (include/fatmap.h), in its order.
	enum class image_status : u8 { OK, IO, NO_FS, NO_FILE, BAD_SIZE, FRAGMENTED };

	using object = u8;

	static constexpr unsigned BLOCK_BYTES = 512;
	static constexpr u8 ATTR_DIR = 0x10;          // AM_DIR

	// f_open mode bits (FA_*).
	static constexpr u8 OPEN_READ = 0x01;
	static constexpr u8 OPEN_WRITE = 0x02;
	static constexpr u8 OPEN_CREATE_NEW = 0x04;
	static constexpr u8 OPEN_CREATE_ALWAYS = 0x08;

	// FILINFO: the 8.3 name as FatFs gives it ("NAME.EXT", NUL-terminated,
	// no LFN), the FAT attributes and the size.  An empty name ends a
	// listing.
	struct entry
	{
		char name[13];
		u8 attr;
		u32 size;
	};

	// The mounted FAT volume: FATFS fs_type (2 FAT16, 3 FAT32), csize,
	// database, n_fatent - 2, and the free count f_getfree reports.
	struct geometry
	{
		u8 fs_type;
		u16 cluster_blocks;
		u32 data_base;
		u32 clusters;
		u32 free_clusters;
	};

	// A resolved volume image, as VOL_INFO reports it: the extent count and
	// the first extent's card block.
	struct image
	{
		u8 extents;
		u32 base_lba;
	};

	// --- Card session (sd_card_init, sd_card_is_initialized) ---------------
	// Bring the card up, or report why not: NO_CARD for an empty socket.
	// Cheap once up.
	virtual card card_init() = 0;
	// The controller's belief that the card is up (card_ready).  It ends only
	// when an access fails: a media change is never announced, so the core
	// learns of one only through a failing operation, as the firmware does
	// (vol_ensure_mounted, fs2.c map_result).
	virtual bool card_ready() const = 0;
	// The controller restarted (its RAM is gone): forget every open object,
	// the filesystem mount and the card session.  Media contents persist.
	virtual void controller_reset() = 0;

	// --- Raw card blocks (sd_card_read_block, sd_cache) --------------------
	// Absolute 512-byte card blocks.  Media without card blocks (a host
	// folder) answer false: the volume layer then mounts no raw unit.
	virtual bool raw_blocks() const = 0;
	virtual card block_read(u32 lba, u8 *data) = 0;
	virtual card block_write(u32 lba, u8 const *data) = 0;
	// sd_cache_flush: every accepted write is on the media.
	virtual card flush() = 0;

	// --- The FAT volume (sdfs, FatFs) ---------------------------------------
	// sdfs_mount: the card status, and whether a FAT volume is mounted (OK
	// with mounted false is a card without a filesystem).
	virtual card fs_mount(bool &mounted) = 0;
	// sdfs_invalidate: drop the mount.  Every open file and directory object
	// stays allocated until closed but answers INVALID_OBJECT.
	virtual void fs_unmount() = 0;
	virtual fr fs_geometry(geometry &info) = 0;              // f_getfree
	virtual fr stat(char const *path, entry &info) = 0;      // f_stat
	// f_open with OPEN_* bits.  A directory is never opened as a file.
	virtual fr open(char const *path, u8 mode, object &file) = 0;
	virtual fr close(object file) = 0;                       // f_close (syncs)
	virtual u32 size(object file) const = 0;                 // f_size
	// f_lseek: a file open for reading clips the position to its size; one
	// open for writing grows to a position past its end.
	virtual fr seek(object file, u32 offset) = 0;
	virtual fr read(object file, u8 *data, u16 length, u16 &done) = 0;
	virtual fr write(object file, u8 const *data, u16 length, u16 &done) = 0;
	virtual fr truncate(object file) = 0;                    // at the position
	virtual fr sync(object file) = 0;
	virtual fr opendir(char const *path, object &dir) = 0;
	virtual fr readdir(object dir, entry &info) = 0;         // no "." / ".."
	virtual fr closedir(object dir) = 0;
	virtual fr unlink(char const *path) = 0;                 // a file or an empty directory
	virtual fr mkdir(char const *path) = 0;
	virtual fr rename(char const *from, char const *to) = 0;

	// --- CP/M volume images (fatmap_resolve) --------------------------------
	// Resolve /<dir11>/<name11> (packed 8.3, compared as the on-card short
	// name) to an image of exactly `bytes`; all or nothing.
	virtual image_status image_open(u8 const *dir11, u8 const *name11, u32 bytes, image &info, object &img) = 0;
	virtual void image_close(object img) = 0;
	// Image-relative 512-byte blocks.
	virtual card image_read(object img, u32 block, u8 *data) = 0;
	virtual card image_write(object img, u32 block, u8 const *data) = 0;

protected:
	~zephyr80_ioc_storage() = default;
};

#endif // MAME_PBITZ_ZEPHYR80_IOC_STORAGE_H
