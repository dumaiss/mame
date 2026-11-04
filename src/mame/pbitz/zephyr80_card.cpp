// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_card.h"

#include "ff.h"
#include "diskio.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <system_error>

namespace fs = std::filesystem;

namespace {

using card = zephyr80_ioc_storage::card;
using fr = zephyr80_ioc_storage::fr;
using image_status = zephyr80_ioc_storage::image_status;

constexpr unsigned BLOCK = zephyr80_ioc_storage::BLOCK_BYTES;

// AM_* attributes as FatFs reports them.
constexpr u8 ATTR_RDO = 0x01;
constexpr u8 ATTR_ARC = 0x20;

// The socket whose card FatFs is mounted on (FatFs has one volume table).
zephyr80_card_socket *s_fatfs_owner = nullptr;

bool seek_to(std::FILE *f, u64 offset)
{
#if defined(_WIN32)
	return _fseeki64(f, __int64(offset), SEEK_SET) == 0;
#else
	return fseeko(f, off_t(offset), SEEK_SET) == 0;
#endif
}

// Whole 512-byte blocks of a file; false on anything short.
bool file_read(std::FILE *f, u64 offset, u8 *data, size_t length)
{
	if (!seek_to(f, offset))
		return false;
	size_t const n = std::fread(data, 1, length, f);
	std::clearerr(f);
	return n == length;
}

bool file_write(std::FILE *f, u64 offset, u8 const *data, size_t length)
{
	if (!seek_to(f, offset))
		return false;
	size_t const n = std::fwrite(data, 1, length, f);
	std::clearerr(f);
	return n == length;
}

std::FILE *open_file(fs::path const &path, bool writable)
{
	std::FILE *f = std::fopen(path.string().c_str(), writable ? "r+b" : "rb");
	if (f)
		std::setvbuf(f, nullptr, _IONBF, 0); // every accepted write reaches the host file
	return f;
}

bool owner_writable(fs::path const &path)
{
	std::error_code ec;
	fs::file_status const st = fs::status(path, ec);
	return !ec && (st.permissions() & fs::perms::owner_write) != fs::perms::none;
}

// FatFs create_name for one path segment (FF_USE_LFN 0, FF_FS_RPATH 0):
// the packed 8.3 name, upper-cased; `p` moves past the segment and its
// separators.  Folder names are ASCII; a byte from 80h up is refused here
// (FatFs would fold it through code page 437), as is an empty base name.
fr sfn_segment(char const *&p, std::array<u8, 11> &sfn, bool &last)
{
	sfn.fill(' ');
	unsigned si = 0, i = 0, ni = 8;
	u8 c;
	for (;;)
	{
		c = u8(p[si++]);
		if (c <= ' ')
			break;
		if (c == '/' || c == '\\')
		{
			while (p[si] == '/' || p[si] == '\\')
				si++;
			break;
		}
		if (c == '.' || i >= ni)
		{
			if (ni == 11 || c != '.')
				return fr::INVALID_NAME;
			i = 8;
			ni = 11;
			continue;
		}
		if (c >= 0x80 || std::strchr("*+,:;<=>[]|\"?\x7f", c))
			return fr::INVALID_NAME;
		if (c >= 'a' && c <= 'z')
			c -= 0x20;
		sfn[i++] = c;
	}
	if (i == 0 || sfn[0] == ' ')
		return fr::INVALID_NAME;
	last = c <= ' ' || u8(p[si]) <= ' ';
	p += last ? si - 1 : si;
	return fr::OK;
}

// get_fileinfo's name: "NAME.EXT", or "NAME" without an extension.
std::string sfn_name(std::array<u8, 11> const &sfn)
{
	std::string s;
	for (unsigned i = 0; i < 8 && sfn[i] != ' '; i++)
		s += char(sfn[i]);
	if (sfn[8] != ' ')
	{
		s += '.';
		for (unsigned i = 8; i < 11 && sfn[i] != ' '; i++)
			s += char(sfn[i]);
	}
	return s;
}

// A host name the folder presents: exactly an 8.3 name in printable ASCII,
// which the card would show the same way once upper-cased.  Anything else
// (long names, spaces, dot files, non-ASCII) is not on the card.
bool host_sfn(std::string const &name, std::array<u8, 11> &sfn)
{
	if (name.empty() || name.size() > 12)
		return false;
	for (char ch : name)
		if (u8(ch) <= ' ' || u8(ch) >= 0x7f || ch == '/' || ch == '\\')
			return false;
	char const *p = name.c_str();
	bool last = false;
	if (sfn_segment(p, sfn, last) != fr::OK || !last || *p)
		return false;
	std::string upper = name;
	for (char &ch : upper)
		if (ch >= 'a' && ch <= 'z')
			ch -= 0x20;
	return sfn_name(sfn) == upper;
}

// A host error as FatFs would report the card refusing it.
fr host_result(std::error_code const &ec)
{
	if (ec == std::errc::permission_denied || ec == std::errc::operation_not_permitted || ec == std::errc::directory_not_empty ||
			ec == std::errc::no_space_on_device || ec == std::errc::file_exists || ec == std::errc::invalid_argument)
		return fr::DENIED;
	if (ec == std::errc::read_only_file_system)
		return fr::WRITE_PROTECTED;
	if (ec == std::errc::no_such_file_or_directory)
		return fr::NO_FILE;
	return fr::DISK_ERR;
}

// An 8.3 name into an entry's 13 bytes.
void copy_name(zephyr80_ioc_storage::entry &info, char const *name)
{
	size_t const n = strnlen(name, sizeof(info.name) - 1);
	std::memcpy(info.name, name, n);
	info.name[n] = 0;
}

bool host_full(int error)
{
	return error == ENOSPC || error == EFBIG
#if defined(EDQUOT)
			|| error == EDQUOT
#endif
			;
}

} // anonymous namespace


// FatFs's media layer (diskio.h), renamed by zephyr80_fatfs.c.
extern "C" {

FRESULT zephyr80_fatfs_sync_window(FATFS *fs);

DSTATUS zephyr80_card_disk_status(BYTE pdrv)
{
	return (pdrv || !s_fatfs_owner) ? STA_NOINIT : s_fatfs_owner->disk_status();
}

DSTATUS zephyr80_card_disk_initialize(BYTE pdrv)
{
	return (pdrv || !s_fatfs_owner) ? STA_NOINIT : s_fatfs_owner->disk_initialize();
}

DRESULT zephyr80_card_disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
	if (pdrv || !s_fatfs_owner)
		return RES_PARERR;
	return s_fatfs_owner->disk_read(buff, sector, count) ? RES_OK : RES_ERROR;
}

DRESULT zephyr80_card_disk_write(BYTE pdrv, BYTE const *buff, LBA_t sector, UINT count)
{
	if (pdrv || !s_fatfs_owner)
		return RES_PARERR;
	return s_fatfs_owner->disk_write(buff, sector, count) ? RES_OK : RES_ERROR;
}

// CTRL_SYNC is the only control FatFs issues in this configuration.
DRESULT zephyr80_card_disk_ioctl(BYTE pdrv, BYTE cmd, void *)
{
	if (pdrv || !s_fatfs_owner)
		return RES_PARERR;
	if (cmd == CTRL_SYNC)
		return s_fatfs_owner->disk_sync() ? RES_OK : RES_ERROR;
	return RES_PARERR;
}

} // extern "C"


struct zephyr80_card_socket::fatfs_state
{
	FATFS fs;
	std::array<FIL, OBJECTS> fil;
	std::array<DIR, OBJECTS> dir;
};

struct zephyr80_card_socket::walk_result
{
	bool root = false;        // the path names the root itself
	fs::path parent;          // the host directory holding the last segment
	std::array<u8, 11> sfn{}; // the last segment
	bool found = false;
	host_entry entry;
};

zephyr80_card_socket::zephyr80_card_socket()
	: m_next_serial(0)
	, m_ready(false)
	, m_replaced(false)
	, m_fat_mounted(false)
	, m_folder_mounted(false)
	, m_fat(std::make_unique<fatfs_state>())
{
	std::memset(&m_fat->fs, 0, sizeof(m_fat->fs));
}

zephyr80_card_socket::~zephyr80_card_socket()
{
	for (object id = 0; id < OBJECTS; id++)
		release(id);
	if (s_fatfs_owner == this)
	{
		f_mount(nullptr, "", 0);
		s_fatfs_owner = nullptr;
	}
	eject();
}


//**************************************************************************
//  Media
//**************************************************************************

std::string zephyr80_card_socket::insert(std::unique_ptr<media> &&m)
{
	m->serial = ++m_next_serial;
	eject();
	m_media = std::move(m);
	// The controller still believes in its session with the old card.
	m_replaced = m_ready;
	return std::string();
}

std::string zephyr80_card_socket::insert_image(std::string const &path)
{
	std::error_code ec;
	fs::path const p(path);
	fs::file_status const st = fs::status(p, ec);
	if (ec || !fs::exists(st))
		return "card image " + path + " does not exist";
	if (!fs::is_regular_file(st))
		return "card image " + path + " is not a file (a host folder is -carddir)";
	u64 const length = fs::file_size(p, ec);
	if (ec)
		return "cannot size card image " + path + ": " + ec.message();
	bool read_only = false;
	std::FILE *f = open_file(p, true);
	if (!f)
	{
		read_only = true;
		f = open_file(p, false);
	}
	if (!f)
		return "cannot open card image " + path + ": " + std::strerror(errno);
	return insert(std::make_unique<media>(media{ kind::IMAGE, p, read_only, 0, f, length / BLOCK }));
}

std::string zephyr80_card_socket::insert_folder(std::string const &path)
{
	std::error_code ec;
	fs::path const p = fs::canonical(fs::path(path), ec);
	if (ec)
		return "card folder " + path + " does not exist";
	if (!fs::is_directory(p, ec))
		return "card folder " + path + " is not a directory (a card image is -card)";
	return insert(std::make_unique<media>(media{ kind::FOLDER, p, !owner_writable(p), 0, nullptr, 0 }));
}

// The card leaves the socket; what the controller holds stays.
void zephyr80_card_socket::eject()
{
	if (m_media && m_media->file)
		std::fclose(m_media->file);
	m_media.reset();
}


//**************************************************************************
//  Card session and blocks (sd_card.c, sd_cache.c)
//**************************************************************************

// sd_card_init: the presence pin first, then the cached session.
zephyr80_card_socket::card zephyr80_card_socket::card_init()
{
	if (!m_media)
	{
		m_ready = false;
		return card::NO_CARD;
	}
	if (!m_ready)
	{
		m_ready = true;
		m_replaced = false;
	}
	return card::OK;
}

// The controller restarts: its RAM, and with it every object, the mount and
// the session, is gone.
void zephyr80_card_socket::controller_reset()
{
	for (object id = 0; id < OBJECTS; id++)
		release(id);
	fs_unmount();
	m_ready = false;
	m_replaced = false;
}

bool zephyr80_card_socket::raw_blocks() const
{
	return !m_media || m_media->type == kind::IMAGE;
}

// A raw block access next to the FatFs window: a read sees the window's
// pending bytes, a write is not later overwritten by them.
zephyr80_card_socket::card zephyr80_card_socket::card_read(u32 lba, u8 *data, bool coherent)
{
	FATFS &fs = m_fat->fs;
	if (coherent && m_fat_mounted && fs.fs_type && fs.wflag && fs.winsect == lba)
		zephyr80_fatfs_sync_window(&fs);
	card const st = card_init();
	if (st != card::OK)
		return st;
	// A replaced card fails the first command; the retry re-initialises it.
	m_replaced = false;
	if (m_media->type == kind::FOLDER)
	{
		// No card blocks: the card answers, block 0 reads blank.
		if (lba == 0)
		{
			std::fill_n(data, BLOCK, 0);
			return card::OK;
		}
		m_ready = false;
		return card::READ;
	}
	if (lba >= m_media->blocks || !file_read(m_media->file, u64(lba) * BLOCK, data, BLOCK))
	{
		m_ready = false;
		return card::READ;
	}
	return card::OK;
}

zephyr80_card_socket::card zephyr80_card_socket::card_write(u32 lba, u8 const *data, bool coherent)
{
	FATFS &fs = m_fat->fs;
	if (coherent && m_fat_mounted && fs.fs_type)
	{
		bool const window = fs.winsect == lba;
		bool const mirror = fs.n_fats == 2 && fs.winsect - fs.fatbase < fs.fsize && lba == fs.winsect + fs.fsize;
		if ((window || mirror) && fs.wflag)
			zephyr80_fatfs_sync_window(&fs);
		if (window)
			fs.winsect = LBA_t(0) - 1;
	}
	card const st = card_init();
	if (st != card::OK)
		return st;
	if (m_replaced)
	{
		// No retry on the write path: the session ends.
		m_replaced = false;
		m_ready = false;
		return card::WRITE;
	}
	if (m_media->read_only)
		return card::WRITE_REJECTED;
	if (m_media->type == kind::FOLDER || lba >= m_media->blocks ||
			!file_write(m_media->file, u64(lba) * BLOCK, data, BLOCK))
	{
		m_ready = false;
		return card::WRITE;
	}
	return card::OK;
}

zephyr80_card_socket::card zephyr80_card_socket::block_read(u32 lba, u8 *data)
{
	return card_read(lba, data, true);
}

zephyr80_card_socket::card zephyr80_card_socket::block_write(u32 lba, u8 const *data)
{
	return card_write(lba, data, true);
}

// Writes go straight to the host file, so nothing is ever left dirty.
zephyr80_card_socket::card zephyr80_card_socket::flush()
{
	if (m_media && m_media->file && std::fflush(m_media->file) != 0)
		return card::WRITE;
	return card::OK;
}

u8 zephyr80_card_socket::disk_status() const
{
	if (!m_ready)
		return STA_NOINIT;
	return (m_media && m_media->read_only) ? STA_PROTECT : 0;
}

u8 zephyr80_card_socket::disk_initialize()
{
	if (card_init() != card::OK)
		return STA_NOINIT;
	return m_media->read_only ? STA_PROTECT : 0;
}

bool zephyr80_card_socket::disk_read(u8 *data, u32 lba, unsigned count)
{
	for (unsigned i = 0; i < count; i++)
		if (card_read(lba + i, data + i * BLOCK, false) != card::OK)
			return false;
	return true;
}

bool zephyr80_card_socket::disk_write(u8 const *data, u32 lba, unsigned count)
{
	for (unsigned i = 0; i < count; i++)
		if (card_write(lba + i, data + i * BLOCK, false) != card::OK)
			return false;
	return true;
}

bool zephyr80_card_socket::disk_sync()
{
	return flush() == card::OK;
}


//**************************************************************************
//  Objects
//**************************************************************************

zephyr80_card_socket::fr zephyr80_card_socket::allocate(slot type, object &id)
{
	for (object i = 0; i < OBJECTS; i++)
		if (m_objects[i].type == slot::FREE)
		{
			objslot &o = m_objects[i];
			o.type = type;
			o.valid = true;
			o.serial = m_media ? m_media->serial : 0;
			o.file = nullptr;
			o.host.clear();
			o.writable = false;
			o.position = o.length = 0;
			o.listing.clear();
			o.next = 0;
			o.extent_count = 0;
			id = i;
			return fr::OK;
		}
	return fr::TOO_MANY_OPEN_FILES;
}

void zephyr80_card_socket::release(object id)
{
	if (id >= OBJECTS)
		return;
	objslot &o = m_objects[id];
	if (o.file)
		std::fclose(o.file);
	o.file = nullptr;
	o.listing.clear();
	o.type = slot::FREE;
}

zephyr80_card_socket::objslot *zephyr80_card_socket::fat_slot(object id, slot type)
{
	return (id < OBJECTS && m_objects[id].type == type) ? &m_objects[id] : nullptr;
}

// A host object answers as FatFs's validate() does: INVALID_OBJECT after an
// unmount or a lost session; one from media that left the socket fails
// the access.
zephyr80_card_socket::fr zephyr80_card_socket::host_slot(object id, slot type, objslot *&o)
{
	o = nullptr;
	if (id >= OBJECTS || m_objects[id].type != type || !m_objects[id].valid || !m_ready)
		return fr::INVALID_OBJECT;
	if (!m_media || m_media->serial != m_objects[id].serial)
	{
		m_ready = false;
		return fr::DISK_ERR;
	}
	o = &m_objects[id];
	return fr::OK;
}


//**************************************************************************
//  The FAT volume (sdfs.c, FatFs)
//**************************************************************************

// sdfs_mount: cheap once mounted; otherwise the card, then the boot
// signature at block 0 before FatFs sees the card, then f_mount.
zephyr80_card_socket::card zephyr80_card_socket::fs_mount(bool &mounted)
{
	mounted = false;
	if (m_fat_mounted || m_folder_mounted)
	{
		mounted = true;
		return card::OK;
	}
	card st = card_init();
	if (st != card::OK)
		return st;
	if (m_media->type == kind::FOLDER)
	{
		m_replaced = false;
		m_folder_mounted = true;
		mounted = true;
		return card::OK;
	}
	u8 block[BLOCK];
	if (card_read(0, block, false) != card::OK)
		return card::READ;
	if (block[BLOCK - 2] != 0x55 || block[BLOCK - 1] != 0xaa)
		return card::OK; // no signature: not a FAT card
	s_fatfs_owner = this;
	if (f_mount(&m_fat->fs, "", 1) != FR_OK)
	{
		std::memset(&m_fat->fs, 0, sizeof(m_fat->fs));
		return card::OK; // no filesystem; not a card failure
	}
	m_fat_mounted = true;
	mounted = true;
	return card::OK;
}

// sdfs_invalidate.  FatFs objects fail validate() once the mount is gone;
// host objects are marked so.
void zephyr80_card_socket::fs_unmount()
{
	if (m_fat_mounted && s_fatfs_owner == this)
		f_mount(nullptr, "", 0);
	m_fat_mounted = false;
	std::memset(&m_fat->fs, 0, sizeof(m_fat->fs));
	m_folder_mounted = false;
	for (objslot &o : m_objects)
		if (o.type == slot::HOST_FILE || o.type == slot::HOST_DIR)
			o.valid = false;
}

namespace {

void fill_entry(FILINFO const &fno, zephyr80_ioc_storage::entry &info)
{
	info = zephyr80_ioc_storage::entry{};
	copy_name(info, fno.fname);
	info.attr = fno.fattrib;
	info.size = u32(fno.fsize);
}

} // anonymous namespace

zephyr80_card_socket::fr zephyr80_card_socket::fs_geometry(geometry &info)
{
	if (m_folder_mounted)
		return folder_geometry(info);
	if (!m_fat_mounted)
		return fr::NOT_ENABLED;
	DWORD free_clusters = 0;
	FATFS *fs = nullptr;
	fr const result = fr(f_getfree("", &free_clusters, &fs));
	if (result != fr::OK)
		return result;
	info.fs_type = fs->fs_type;
	info.cluster_blocks = fs->csize;
	info.data_base = fs->database;
	info.clusters = fs->n_fatent - 2;
	info.free_clusters = free_clusters;
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::stat(char const *path, entry &info)
{
	if (m_folder_mounted)
		return folder_stat(path, info);
	if (!m_fat_mounted)
		return fr::NOT_ENABLED;
	FILINFO fno;
	fr const result = fr(f_stat(path, &fno));
	if (result == fr::OK)
		fill_entry(fno, info);
	return result;
}

zephyr80_card_socket::fr zephyr80_card_socket::open(char const *path, u8 mode, object &file)
{
	if (m_folder_mounted)
		return folder_open(path, mode, file);
	if (!m_fat_mounted)
		return fr::NOT_ENABLED;
	fr result = allocate(slot::FAT_FILE, file);
	if (result != fr::OK)
		return result;
	result = fr(f_open(&m_fat->fil[file], path, mode));
	if (result != fr::OK)
		release(file);
	return result;
}

zephyr80_card_socket::fr zephyr80_card_socket::close(object file)
{
	if (fat_slot(file, slot::FAT_FILE))
	{
		fr const result = fr(f_close(&m_fat->fil[file]));
		release(file);
		return result;
	}
	if (file < OBJECTS && m_objects[file].type == slot::HOST_FILE)
	{
		objslot *o;
		fr result = host_slot(file, slot::HOST_FILE, o);
		if (result == fr::OK && std::fflush(o->file) != 0)
			result = fr::DISK_ERR;
		release(file);
		return result;
	}
	return fr::INVALID_OBJECT;
}

u32 zephyr80_card_socket::size(object file) const
{
	if (file >= OBJECTS)
		return 0;
	objslot const &o = m_objects[file];
	if (o.type == slot::FAT_FILE)
		return u32(f_size(&m_fat->fil[file]));
	if (o.type == slot::HOST_FILE)
		return u32(o.length);
	return 0;
}

zephyr80_card_socket::fr zephyr80_card_socket::seek(object file, u32 offset)
{
	if (fat_slot(file, slot::FAT_FILE))
		return fr(f_lseek(&m_fat->fil[file], offset));
	objslot *o;
	fr const result = host_slot(file, slot::HOST_FILE, o);
	if (result != fr::OK)
		return result;
	u64 target = offset;
	if (target > o->length)
	{
		// f_lseek: a reader clips at the end; a writer grows the file, and
		// stops at its old end when the volume is full.
		std::error_code ec;
		if (!o->writable)
			target = o->length;
		else if (fs::resize_file(o->host, target, ec); ec)
			target = o->length;
		else
			o->length = target;
	}
	o->position = target;
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::read(object file, u8 *data, u16 length, u16 &done)
{
	done = 0;
	if (fat_slot(file, slot::FAT_FILE))
	{
		UINT got = 0;
		fr const result = fr(f_read(&m_fat->fil[file], data, length, &got));
		done = u16(got);
		return result;
	}
	objslot *o;
	fr const result = host_slot(file, slot::HOST_FILE, o);
	if (result != fr::OK)
		return result;
	u64 const n = o->position < o->length ? std::min<u64>(length, o->length - o->position) : 0;
	if (n && !file_read(o->file, o->position, data, size_t(n)))
	{
		m_ready = false;
		return fr::DISK_ERR;
	}
	o->position += n;
	done = u16(n);
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::write(object file, u8 const *data, u16 length, u16 &done)
{
	done = 0;
	if (fat_slot(file, slot::FAT_FILE))
	{
		UINT put = 0;
		fr const result = fr(f_write(&m_fat->fil[file], data, length, &put));
		done = u16(put);
		return result;
	}
	objslot *o;
	fr const result = host_slot(file, slot::HOST_FILE, o);
	if (result != fr::OK)
		return result;
	if (!o->writable)
		return fr::DENIED;
	u64 want = length;
	if (o->position + want > 0xffffffffU)
		want = 0xffffffffU - std::min<u64>(o->position, 0xffffffffU); // FAT's 4 GiB limit
	if (!want)
		return fr::OK;
	if (!seek_to(o->file, o->position))
	{
		m_ready = false;
		return fr::DISK_ERR;
	}
	errno = 0;
	size_t const n = std::fwrite(data, 1, size_t(want), o->file);
	int const error = errno;
	std::clearerr(o->file);
	if (n < want && !host_full(error))
	{
		m_ready = false;
		return fr::DISK_ERR;
	}
	// A full volume is a short write, as f_write reports one.
	o->position += n;
	o->length = std::max(o->length, o->position);
	done = u16(n);
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::truncate(object file)
{
	if (fat_slot(file, slot::FAT_FILE))
		return fr(f_truncate(&m_fat->fil[file]));
	objslot *o;
	fr const result = host_slot(file, slot::HOST_FILE, o);
	if (result != fr::OK)
		return result;
	if (!o->writable)
		return fr::DENIED;
	if (o->position < o->length)
	{
		std::error_code ec;
		fs::resize_file(o->host, o->position, ec);
		if (ec)
		{
			m_ready = false;
			return fr::DISK_ERR;
		}
		o->length = o->position;
	}
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::sync(object file)
{
	if (fat_slot(file, slot::FAT_FILE))
		return fr(f_sync(&m_fat->fil[file]));
	objslot *o;
	fr const result = host_slot(file, slot::HOST_FILE, o);
	if (result != fr::OK)
		return result;
	return std::fflush(o->file) == 0 ? fr::OK : fr::DISK_ERR;
}

zephyr80_card_socket::fr zephyr80_card_socket::opendir(char const *path, object &dir)
{
	if (m_folder_mounted)
		return folder_opendir(path, dir);
	if (!m_fat_mounted)
		return fr::NOT_ENABLED;
	fr result = allocate(slot::FAT_DIR, dir);
	if (result != fr::OK)
		return result;
	result = fr(f_opendir(&m_fat->dir[dir], path));
	if (result != fr::OK)
		release(dir);
	return result;
}

zephyr80_card_socket::fr zephyr80_card_socket::readdir(object dir, entry &info)
{
	if (fat_slot(dir, slot::FAT_DIR))
	{
		FILINFO fno;
		fr const result = fr(f_readdir(&m_fat->dir[dir], &fno));
		if (result == fr::OK)
			fill_entry(fno, info);
		return result;
	}
	objslot *o;
	fr const result = host_slot(dir, slot::HOST_DIR, o);
	if (result != fr::OK)
		return result;
	// The names are fixed at OPENDIR; each is checked again as it is read, so
	// one removed since is skipped and sizes are current.
	while (o->next < o->listing.size())
	{
		host_entry const &e = o->listing[o->next++];
		std::error_code ec;
		fs::file_status const st = fs::symlink_status(e.host, ec);
		if (ec || fs::is_symlink(st) || (e.dir ? !fs::is_directory(st) : !fs::is_regular_file(st)))
			continue;
		host_info(e, info);
		return fr::OK;
	}
	info = entry{};
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::closedir(object dir)
{
	if (fat_slot(dir, slot::FAT_DIR))
	{
		fr const result = fr(f_closedir(&m_fat->dir[dir]));
		release(dir);
		return result;
	}
	if (dir < OBJECTS && m_objects[dir].type == slot::HOST_DIR)
	{
		objslot *o;
		fr const result = host_slot(dir, slot::HOST_DIR, o);
		release(dir);
		return result;
	}
	return fr::INVALID_OBJECT;
}

zephyr80_card_socket::fr zephyr80_card_socket::unlink(char const *path)
{
	if (m_folder_mounted)
		return folder_unlink(path);
	return m_fat_mounted ? fr(f_unlink(path)) : fr::NOT_ENABLED;
}

zephyr80_card_socket::fr zephyr80_card_socket::mkdir(char const *path)
{
	if (m_folder_mounted)
		return folder_mkdir(path);
	return m_fat_mounted ? fr(f_mkdir(path)) : fr::NOT_ENABLED;
}

zephyr80_card_socket::fr zephyr80_card_socket::rename(char const *from, char const *to)
{
	if (m_folder_mounted)
		return folder_rename(from, to);
	return m_fat_mounted ? fr(f_rename(from, to)) : fr::NOT_ENABLED;
}


//**************************************************************************
//  The folder as a FAT volume
//**************************************************************************

// A folder access as FatFs makes a card access: with no session the card is
// brought up (no card: NOT_READY); with one, a card that is gone or is not a
// folder fails the access (DISK_ERR) and ends it.  A read-only folder is a
// write-protected card.
zephyr80_card_socket::fr zephyr80_card_socket::folder_access(bool writing)
{
	if (!m_ready && card_init() != card::OK)
		return fr::NOT_READY;
	if (!m_media || m_media->type != kind::FOLDER)
	{
		m_ready = false;
		return fr::DISK_ERR;
	}
	m_replaced = false;
	if (writing && m_media->read_only)
		return fr::WRITE_PROTECTED;
	return fr::OK;
}

// The names a host directory presents, in 8.3 order.  Symbolic links and
// anything neither a file nor a directory are not on the card; two host
// names that fold to one 8.3 name are both ambiguous.
zephyr80_card_socket::fr zephyr80_card_socket::host_list(fs::path const &dir, std::vector<host_entry> &out) const
{
	out.clear();
	std::error_code ec;
	fs::directory_iterator it(dir, ec);
	if (ec)
		return fr::DISK_ERR;
	for (fs::directory_iterator const end; it != end; it.increment(ec))
	{
		if (ec)
			return fr::DISK_ERR;
		host_entry e;
		if (!host_sfn(it->path().filename().string(), e.sfn))
			continue;
		std::error_code sec;
		fs::file_status const st = fs::symlink_status(it->path(), sec);
		if (sec || fs::is_symlink(st))
			continue;
		if (fs::is_directory(st))
			e.dir = true;
		else if (fs::is_regular_file(st))
			e.dir = false;
		else
			continue;
		e.host = it->path();
		e.ambiguous = false;
		out.push_back(std::move(e));
	}
	if (ec)
		return fr::DISK_ERR;
	std::sort(out.begin(), out.end(), [] (host_entry const &a, host_entry const &b) { return a.sfn < b.sfn; });
	for (size_t i = 1; i < out.size(); i++)
		if (out[i].sfn == out[i - 1].sfn)
			out[i].ambiguous = out[i - 1].ambiguous = true;
	return fr::OK;
}

// follow_path: segment by segment from the root, matching 8.3 names only, so
// nothing but host names the folder presents is ever reached.  A missing or
// non-directory intermediate segment is NO_PATH; an ambiguous one DENIED.
zephyr80_card_socket::fr zephyr80_card_socket::walk(char const *path, walk_result &out)
{
	out = walk_result{};
	fs::path dir = m_media->path;
	while (*path == '/' || *path == '\\')
		path++;
	if (u8(*path) < ' ')
	{
		out.root = true;
		out.parent = dir;
		return fr::OK;
	}
	std::vector<host_entry> listing;
	for (;;)
	{
		bool last = false;
		fr result = sfn_segment(path, out.sfn, last);
		if (result != fr::OK)
			return result;
		result = host_list(dir, listing);
		if (result != fr::OK)
		{
			m_ready = false;
			return result;
		}
		auto const it = std::find_if(listing.begin(), listing.end(), [&out] (host_entry const &e) { return e.sfn == out.sfn; });
		if (it != listing.end() && it->ambiguous)
			return fr::DENIED;
		if (last)
		{
			out.parent = dir;
			if (it != listing.end())
			{
				out.found = true;
				out.entry = *it;
			}
			return fr::OK;
		}
		if (it == listing.end() || !it->dir)
			return fr::NO_PATH;
		dir = it->host;
	}
}

void zephyr80_card_socket::host_info(host_entry const &e, entry &info) const
{
	info = entry{};
	copy_name(info, sfn_name(e.sfn).c_str());
	if (e.dir)
	{
		info.attr = ATTR_DIR;
		return;
	}
	std::error_code ec;
	u64 const length = fs::file_size(e.host, ec);
	info.size = ec ? 0 : u32(std::min<u64>(length, 0xffffffffU));
	info.attr = ATTR_ARC | (owner_writable(e.host) ? 0 : ATTR_RDO);
}

zephyr80_card_socket::fr zephyr80_card_socket::folder_stat(char const *path, entry &info)
{
	fr result = folder_access(false);
	walk_result w;
	if (result == fr::OK)
		result = walk(path, w);
	if (result != fr::OK)
		return result;
	if (w.root)
		return fr::INVALID_NAME;
	if (!w.found)
		return fr::NO_FILE;
	host_info(w.entry, info);
	return fr::OK;
}

// f_open: the create modes make a missing file; an existing one may not be
// a directory or read-only for them, nor exist at all for CREATE_NEW;
// CREATE_ALWAYS empties it.  Without them a directory is NO_FILE and a
// read-only file refuses writing.
zephyr80_card_socket::fr zephyr80_card_socket::folder_open(char const *path, u8 mode, object &file)
{
	u8 const create = mode & (OPEN_CREATE_NEW | OPEN_CREATE_ALWAYS);
	fr result = folder_access(mode & (OPEN_WRITE | OPEN_CREATE_NEW | OPEN_CREATE_ALWAYS));
	walk_result w;
	if (result == fr::OK)
		result = walk(path, w);
	if (result == fr::OK && w.root)
		result = fr::INVALID_NAME;
	if (result != fr::OK)
		return result;
	fs::path host;
	bool empty = false;
	if (create)
	{
		if (!w.found)
		{
			host = w.parent / sfn_name(w.sfn);
			empty = true;
		}
		else if (w.entry.dir || !owner_writable(w.entry.host))
			return fr::DENIED;
		else if (mode & OPEN_CREATE_NEW)
			return fr::EXIST;
		else
		{
			host = w.entry.host;
			empty = (mode & OPEN_CREATE_ALWAYS) != 0;
		}
	}
	else
	{
		if (!w.found || w.entry.dir)
			return fr::NO_FILE;
		if ((mode & OPEN_WRITE) && !owner_writable(w.entry.host))
			return fr::DENIED;
		host = w.entry.host;
	}
	if (empty)
	{
		std::FILE *f = std::fopen(host.string().c_str(), "wb");
		if (!f)
			return host_result(std::error_code(errno, std::generic_category()));
		std::fclose(f);
	}
	result = allocate(slot::HOST_FILE, file);
	if (result != fr::OK)
		return result;
	objslot &o = m_objects[file];
	o.writable = (mode & OPEN_WRITE) != 0;
	o.host = host;
	o.file = open_file(host, o.writable);
	std::error_code ec;
	o.length = fs::file_size(host, ec);
	if (!o.file || ec)
	{
		int const error = errno;
		release(file);
		return host_result(ec ? ec : std::error_code(error, std::generic_category()));
	}
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::folder_opendir(char const *path, object &dir)
{
	fr result = folder_access(false);
	walk_result w;
	if (result == fr::OK)
		result = walk(path, w);
	if (result != fr::OK)
		return result;
	fs::path host = w.parent;
	if (!w.root)
	{
		if (!w.found || !w.entry.dir)
			return fr::NO_PATH;
		host = w.entry.host;
	}
	std::vector<host_entry> listing;
	result = host_list(host, listing);
	if (result != fr::OK)
	{
		m_ready = false;
		return result;
	}
	listing.erase(std::remove_if(listing.begin(), listing.end(), [] (host_entry const &e) { return e.ambiguous; }), listing.end());
	result = allocate(slot::HOST_DIR, dir);
	if (result != fr::OK)
		return result;
	m_objects[dir].host = host;
	m_objects[dir].listing = std::move(listing);
	return fr::OK;
}

// f_unlink: a read-only file or a directory holding anything (on the host,
// shown or not) is DENIED.
zephyr80_card_socket::fr zephyr80_card_socket::folder_unlink(char const *path)
{
	fr result = folder_access(true);
	walk_result w;
	if (result == fr::OK)
		result = walk(path, w);
	if (result != fr::OK)
		return result;
	if (w.root)
		return fr::INVALID_NAME;
	if (!w.found)
		return fr::NO_FILE;
	std::error_code ec;
	if (w.entry.dir ? !fs::is_empty(w.entry.host, ec) : !owner_writable(w.entry.host))
		return fr::DENIED;
	if (ec || !fs::remove(w.entry.host, ec))
		return host_result(ec);
	return fr::OK;
}

zephyr80_card_socket::fr zephyr80_card_socket::folder_mkdir(char const *path)
{
	fr result = folder_access(true);
	walk_result w;
	if (result == fr::OK)
		result = walk(path, w);
	if (result != fr::OK)
		return result;
	if (w.root || w.found)
		return fr::EXIST;
	std::error_code ec;
	if (!fs::create_directory(w.parent / sfn_name(w.sfn), ec))
		return ec ? host_result(ec) : fr::EXIST;
	return fr::OK;
}

// f_rename: the old name must exist and the new one must not; the host entry
// takes the new 8.3 name.
zephyr80_card_socket::fr zephyr80_card_socket::folder_rename(char const *from, char const *to)
{
	fr result = folder_access(true);
	walk_result old_name, new_name;
	if (result == fr::OK)
		result = walk(from, old_name);
	if (result != fr::OK)
		return result;
	if (old_name.root)
		return fr::INVALID_NAME;
	if (!old_name.found)
		return fr::NO_FILE;
	result = walk(to, new_name);
	if (result != fr::OK)
		return result;
	if (new_name.root)
		return fr::INVALID_NAME;
	if (new_name.found)
		return (new_name.entry.host == old_name.entry.host) ? fr::NO_FILE : fr::EXIST;
	std::error_code ec;
	fs::rename(old_name.entry.host, new_name.parent / sfn_name(new_name.sfn), ec);
	return ec ? host_result(ec) : fr::OK;
}

// A folder has no clusters: its geometry is the host filesystem's space in
// 4 KiB units, reported as FAT32.
zephyr80_card_socket::fr zephyr80_card_socket::folder_geometry(geometry &info)
{
	fr const result = folder_access(false);
	if (result != fr::OK)
		return result;
	std::error_code ec;
	fs::space_info const space = fs::space(m_media->path, ec);
	if (ec)
	{
		m_ready = false;
		return fr::DISK_ERR;
	}
	u64 const cluster = 8 * BLOCK;
	u64 const clusters = std::min<u64>(space.capacity / cluster, 0x0ffffff5);
	info.fs_type = 3;
	info.cluster_blocks = 8;
	info.data_base = 0;
	info.clusters = u32(clusters);
	info.free_clusters = u32(std::min<u64>(space.available / cluster, clusters));
	return fr::OK;
}


//**************************************************************************
//  CP/M volume images (fatmap.c)
//**************************************************************************

zephyr80_card_socket::image_status zephyr80_card_socket::image_open(u8 const *dir11, u8 const *name11, u32 bytes, image &info, object &img)
{
	if (m_media && m_media->type == kind::FOLDER)
		return folder_image(dir11, name11, bytes, info, img);
	if (allocate(slot::IMAGE_BLOCKS, img) != fr::OK)
		return image_status::IO;
	image_status const st = fatmap_resolve(dir11, name11, bytes, m_objects[img]);
	if (st != image_status::OK)
	{
		release(img);
		return st;
	}
	info.extents = m_objects[img].extent_count;
	info.base_lba = m_objects[img].extents[0].start;
	return image_status::OK;
}

// A folder's volume image is the host file /<dir>/<name> (8.3 names, the
// folder's rules), exactly `bytes` long; one extent with no card address.
zephyr80_card_socket::image_status zephyr80_card_socket::folder_image(u8 const *dir11, u8 const *name11, u32 bytes, image &info, object &img)
{
	if (folder_access(false) != fr::OK)
		return image_status::IO;
	std::array<u8, 11> want;
	std::vector<host_entry> listing;
	fs::path dir;
	std::copy_n(dir11, 11, want.begin());
	if (host_list(m_media->path, listing) != fr::OK)
		return image_status::IO;
	auto it = std::find_if(listing.begin(), listing.end(), [&want] (host_entry const &e) { return e.sfn == want && e.dir; });
	if (it == listing.end() || it->ambiguous)
		return image_status::NO_FILE;
	dir = it->host;
	std::copy_n(name11, 11, want.begin());
	if (host_list(dir, listing) != fr::OK)
		return image_status::IO;
	it = std::find_if(listing.begin(), listing.end(), [&want] (host_entry const &e) { return e.sfn == want && !e.dir; });
	if (it == listing.end() || it->ambiguous)
		return image_status::NO_FILE;
	std::error_code ec;
	if (fs::file_size(it->host, ec) != bytes || ec)
		return image_status::BAD_SIZE;
	if (allocate(slot::IMAGE_HOST, img) != fr::OK)
		return image_status::IO;
	objslot &o = m_objects[img];
	o.writable = !m_media->read_only && owner_writable(it->host);
	o.host = it->host;
	o.length = bytes;
	o.file = open_file(it->host, o.writable);
	if (!o.file)
	{
		release(img);
		return image_status::IO;
	}
	info.extents = 1;
	info.base_lba = 0;
	return image_status::OK;
}

void zephyr80_card_socket::image_close(object img)
{
	if (img < OBJECTS && (m_objects[img].type == slot::IMAGE_BLOCKS || m_objects[img].type == slot::IMAGE_HOST))
		release(img);
}

zephyr80_card_socket::card zephyr80_card_socket::image_read(object img, u32 block, u8 *data)
{
	if (img >= OBJECTS)
		return card::READ;
	objslot &o = m_objects[img];
	if (o.type == slot::IMAGE_BLOCKS)
	{
		// vol_map: the block through the extent table.
		for (unsigned i = 0; i < o.extent_count; i++)
		{
			if (block < o.extents[i].blocks)
				return card_read(o.extents[i].start + block, data, true);
			block -= o.extents[i].blocks;
		}
		return card::READ;
	}
	if (o.type != slot::IMAGE_HOST)
		return card::READ;
	card const st = card_init();
	if (st != card::OK)
		return st;
	if (m_media->serial != o.serial || u64(block + 1) * BLOCK > o.length || !file_read(o.file, u64(block) * BLOCK, data, BLOCK))
	{
		m_ready = false;
		return card::READ;
	}
	return card::OK;
}

zephyr80_card_socket::card zephyr80_card_socket::image_write(object img, u32 block, u8 const *data)
{
	if (img >= OBJECTS)
		return card::WRITE;
	objslot &o = m_objects[img];
	if (o.type == slot::IMAGE_BLOCKS)
	{
		for (unsigned i = 0; i < o.extent_count; i++)
		{
			if (block < o.extents[i].blocks)
				return card_write(o.extents[i].start + block, data, true);
			block -= o.extents[i].blocks;
		}
		return card::WRITE;
	}
	if (o.type != slot::IMAGE_HOST)
		return card::WRITE;
	card const st = card_init();
	if (st != card::OK)
		return st;
	if (m_media->serial == o.serial && !o.writable)
		return card::WRITE_REJECTED;
	if (m_media->serial != o.serial || u64(block + 1) * BLOCK > o.length || !file_write(o.file, u64(block) * BLOCK, data, BLOCK))
	{
		m_ready = false;
		return card::WRITE;
	}
	return card::OK;
}

// fatmap_resolve over the card's blocks: the volume (a BPB at block 0, or
// the first MBR partition holding one; FAT16 or FAT32), /<dir11> in the
// root, <name11> in it, the exact size, then the cluster chain coalesced
// into at most IMAGE_EXTENTS runs that must cover the file.  Names compare
// as the on-card short names; LFN fragments and volume labels are skipped.
zephyr80_card_socket::image_status zephyr80_card_socket::fatmap_resolve(u8 const *dir11, u8 const *name11, u32 want, objslot &o)
{
	constexpr u32 MAX_CHAIN = 0x20000;  // FATMAP_MAX_CHAIN
	std::array<u8, BLOCK> buffer;
	u32 cached = ~u32(0);
	auto rd = [&] (u32 lba, unsigned offset, unsigned length, u8 *dst) -> bool
	{
		if (lba != cached)
		{
			cached = ~u32(0);
			if (card_read(lba, buffer.data(), true) != card::OK)
				return false;
			cached = lba;
		}
		std::copy_n(&buffer[offset], length, dst);
		return true;
	};
	auto rd16 = [&] (u32 lba, unsigned offset, u16 &v) -> bool
	{
		u8 b[2];
		if (!rd(lba, offset, 2, b))
			return false;
		v = u16(b[0] | (b[1] << 8));
		return true;
	};
	auto rd32 = [&] (u32 lba, unsigned offset, u32 &v) -> bool
	{
		u8 b[4];
		if (!rd(lba, offset, 4, b))
			return false;
		v = b[0] | (b[1] << 8) | (b[2] << 16) | (u32(b[3]) << 24);
		return true;
	};

	struct
	{
		bool fat32;
		u32 fat_lba, root_lba, root_clus, data_lba, root_sectors;
		u16 root_entries;
		u8 spc;
	} vol{};

	auto looks_like_bpb = [&] (u32 lba) -> bool
	{
		u16 bps, rsvd;
		u8 spc, nfats;
		if (!rd16(lba, 0x0b, bps) || !rd16(lba, 0x0e, rsvd) || !rd(lba, 0x0d, 1, &spc) || !rd(lba, 0x10, 1, &nfats))
			return false;
		return bps == BLOCK && spc && !(spc & (spc - 1)) && rsvd && (nfats == 1 || nfats == 2);
	};

	auto read_bpb = [&] (u32 base) -> image_status
	{
		u16 rsvd, root_ent, fatsz16, totsec16;
		u8 nfats;
		if (!rd(base, 0x0d, 1, &vol.spc) || !rd(base, 0x10, 1, &nfats) || !rd16(base, 0x0e, rsvd) ||
				!rd16(base, 0x11, root_ent) || !rd16(base, 0x16, fatsz16) || !rd16(base, 0x13, totsec16))
			return image_status::IO;
		u32 fatsz = fatsz16, totsec = totsec16;
		if (!fatsz && !rd32(base, 0x24, fatsz))
			return image_status::IO;
		if (!totsec && !rd32(base, 0x20, totsec))
			return image_status::IO;
		if (!fatsz || !totsec)
			return image_status::NO_FS;
		vol.root_sectors = (u32(root_ent) * 32 + BLOCK - 1) / BLOCK;
		vol.root_entries = root_ent;
		vol.fat_lba = base + rsvd;
		vol.root_lba = vol.fat_lba + u32(nfats) * fatsz;
		vol.data_lba = vol.root_lba + vol.root_sectors;
		if (vol.data_lba <= base)
			return image_status::NO_FS;
		u32 const clusters = (totsec - (vol.data_lba - base)) / vol.spc;
		if (clusters < 4085)
			return image_status::NO_FS; // FAT12
		vol.fat32 = clusters >= 65525;
		if (vol.fat32)
		{
			if (!rd32(base, 0x2c, vol.root_clus))
				return image_status::IO;
			if (vol.root_clus < 2)
				return image_status::NO_FS;
		}
		else
		{
			vol.root_clus = 0;
			if (!vol.root_entries)
				return image_status::NO_FS;
		}
		return image_status::OK;
	};

	// next_cluster: false only on a card failure.
	auto next_cluster = [&] (u32 clus, u32 &next, bool &end) -> bool
	{
		end = false;
		if (vol.fat32)
		{
			u32 v;
			u32 const off = clus * 4;
			if (!rd32(vol.fat_lba + off / BLOCK, off % BLOCK, v))
				return false;
			v &= 0x0fffffff;
			if (v >= 0x0ffffff8 || v < 2)
				end = true;
			else
				next = v;
		}
		else
		{
			u16 v;
			u32 const off = clus * 2;
			if (!rd16(vol.fat_lba + off / BLOCK, off % BLOCK, v))
				return false;
			if (v >= 0xfff8 || v < 2)
				end = true;
			else
				next = v;
		}
		return true;
	};
	auto clus_to_lba = [&] (u32 clus) { return vol.data_lba + (clus - 2) * vol.spc; };

	auto find_entry = [&] (u32 start_clus, bool linear_root, u8 const *name, bool want_dir, u32 &first_clus, u32 &size) -> image_status
	{
		u32 clus = start_clus, lba, sec_left, walked = 0;
		u32 ent_left = linear_root ? vol.root_entries : 0xffffffff;
		if (linear_root)
		{
			lba = vol.root_lba;
			sec_left = vol.root_sectors;
		}
		else
		{
			if (clus < 2)
				return image_status::NO_FILE;
			lba = clus_to_lba(clus);
			sec_left = vol.spc;
		}
		for (;;)
		{
			for (unsigned e = 0; e < BLOCK / 32; e++)
			{
				unsigned const off = e * 32;
				u8 nm[11], attr;
				if (!ent_left)
					return image_status::NO_FILE;
				ent_left--;
				if (!rd(lba, off, 11, nm))
					return image_status::IO;
				if (nm[0] == 0x00)
					return image_status::NO_FILE;
				if (nm[0] == 0xe5)
					continue;
				if (!rd(lba, off + 11, 1, &attr))
					return image_status::IO;
				if ((attr & 0x0f) == 0x0f || (attr & 0x08))
					continue;
				if (std::memcmp(nm, name, 11) || want_dir != ((attr & ATTR_DIR) != 0))
					continue;
				u16 hi = 0, lo = 0;
				if (vol.fat32 && !rd16(lba, off + 20, hi))
					return image_status::IO;
				if (!rd16(lba, off + 26, lo) || !rd32(lba, off + 28, size))
					return image_status::IO;
				first_clus = (u32(hi) << 16) | lo;
				return image_status::OK;
			}
			lba++;
			if (--sec_left)
				continue;
			if (linear_root)
				return image_status::NO_FILE;
			u32 next = 0;
			bool end = false;
			if (!next_cluster(clus, next, end))
				return image_status::IO;
			if (end)
				return image_status::NO_FILE;
			if (++walked > MAX_CHAIN)
				return image_status::NO_FS;
			clus = next;
			lba = clus_to_lba(clus);
			sec_left = vol.spc;
		}
	};

	// mount: a superfloppy, or the first MBR partition with a BPB.
	image_status st;
	if (looks_like_bpb(0))
		st = read_bpb(0);
	else
	{
		u8 sig[2];
		if (!rd(0, BLOCK - 2, 2, sig))
			return image_status::IO;
		if (sig[0] != 0x55 || sig[1] != 0xaa)
			return image_status::NO_FS;
		st = image_status::NO_FS;
		for (unsigned i = 0; i < 4; i++)
		{
			unsigned const pte = 446 + i * 16;
			u8 type;
			u32 start;
			if (!rd(0, pte + 4, 1, &type))
				return image_status::IO;
			if (!type)
				continue;
			if (!rd32(0, pte + 8, start))
				return image_status::IO;
			if (start && looks_like_bpb(start))
			{
				st = read_bpb(start);
				break;
			}
		}
	}
	if (st != image_status::OK)
		return st;

	u32 dir_clus = 0, dir_size = 0, clus = 0, size = 0;
	if ((st = find_entry(vol.root_clus, !vol.fat32, dir11, true, dir_clus, dir_size)) != image_status::OK)
		return st;
	if ((st = find_entry(dir_clus, false, name11, false, clus, size)) != image_status::OK)
		return st;
	if (size != want)
		return image_status::BAD_SIZE;
	if (clus < 2)
		return image_status::NO_FILE;

	u32 run_start = clus, run_len = 1, walked = 0;
	u64 mapped = 0;
	u8 n = 0;
	for (;;)
	{
		u32 next = 0;
		bool end = false;
		if (!next_cluster(clus, next, end))
			return image_status::IO;
		if (++walked > MAX_CHAIN)
			return image_status::NO_FS;
		if (!end && next == clus + 1)
		{
			clus = next;
			run_len++;
			continue;
		}
		if (n >= IMAGE_EXTENTS)
			return image_status::FRAGMENTED;
		o.extents[n] = extent{ clus_to_lba(run_start), run_len * vol.spc };
		mapped += o.extents[n].blocks;
		n++;
		if (end)
			break;
		clus = run_start = next;
		run_len = 1;
	}
	if (mapped * BLOCK < want)
		return image_status::NO_FS;
	o.extent_count = n;
	return image_status::OK;
}
