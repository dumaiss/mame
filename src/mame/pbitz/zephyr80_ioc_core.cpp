// license:BSD-3-Clause
// copyright-holders:Kitamura
#include "emu.h"
#include "zephyr80_ioc_core.h"

#include <algorithm>
#include <cstring>

namespace {

// include/ioc_frame.h
constexpr u8 SYNC0 = 0xa5;
constexpr u8 SYNC1 = 0x5a;
constexpr unsigned FIXED_LEN = 3;     // TYPE + SEQ + STATUS
constexpr unsigned OFF_CLASS = 0;
constexpr unsigned OFF_SEQ = 1;
constexpr unsigned OFF_STATUS = 2;
constexpr unsigned OFF_LEN = 3;
constexpr unsigned OFF_PAYLOAD = 4;

constexpr u8 CMD_PING = 0x01;
constexpr u8 CMD_RESET = 0x02;
constexpr u8 CMD_SD_READ = 0x03;
constexpr u8 CMD_BULK_TEST = 0x04;
constexpr u8 CMD_SD_READ_BULK = 0x05;
constexpr u8 CMD_XFER_STATUS = 0x06;
constexpr u8 CMD_SD_WRITE_BULK = 0x07;
constexpr u8 CMD_SD_READ_REC = 0x08;
constexpr u8 CMD_SD_WRITE_REC = 0x09;
constexpr u8 CMD_SD_FLUSH = 0x0a;
constexpr u8 CMD_PROFILE = 0x0b;
constexpr u8 CMD_LINK_SYNC = 0x0c;
constexpr u8 CMD_HID_STATUS = 0x0d;
constexpr u8 CMD_HID_INPUT = 0x0e;
constexpr u8 CMD_SD_READ_BLOCK = 0x0f;
constexpr u8 CMD_VOL_MOUNT = 0x10;
constexpr u8 CMD_VOL_INFO = 0x11;
constexpr u8 CMD_HID_MOUSE = 0x12;
constexpr u8 CMD_FS_OPENDIR = 0x20;
constexpr u8 CMD_FS_READDIR = 0x21;
constexpr u8 CMD_FS_OPEN = 0x22;
constexpr u8 CMD_FS_READ = 0x23;
constexpr u8 CMD_FS_WRITE = 0x24;
constexpr u8 CMD_FS_CLOSE = 0x25;
constexpr u8 CMD_FS_STAT = 0x26;
constexpr u8 CMD_FS_DELETE = 0x27;
constexpr u8 CMD_FS_SELFTEST = 0x28;
constexpr u8 CMD_FS2_CAPS = 0x30;
constexpr u8 CMD_FS2_GENERATION = 0x31;
constexpr u8 CMD_FS2_RESET = 0x32;
constexpr u8 CMD_FS2_ROOT = 0x33;
constexpr u8 CMD_FS2_PUSH = 0x34;
constexpr u8 CMD_FS2_OPEN_RO = 0x35;
constexpr u8 CMD_FS2_READ = 0x36;
constexpr u8 CMD_FS2_CLOSE = 0x37;
constexpr u8 CMD_FS2_OPENDIR = 0x38;
constexpr u8 CMD_FS2_READDIR = 0x39;
constexpr u8 CMD_FS2_CLOSEDIR = 0x3a;
constexpr u8 CMD_FS2_STAT = 0x3b;
constexpr u8 CMD_FS2_SPACE = 0x3c;
constexpr u8 CMD_FS2_OPEN_RW = 0x3d;
constexpr u8 CMD_FS2_WRITE = 0x3e;
constexpr u8 CMD_FS2_SYNC = 0x3f;
constexpr u8 CMD_FS2_TRUNCATE = 0x40;
constexpr u8 CMD_FS2_UNLINK = 0x41;
constexpr u8 CMD_FS2_RENAME = 0x42;
constexpr u8 CMD_FS2_MKDIR = 0x43;
constexpr u8 CMD_FS2_RMDIR = 0x44;
constexpr u8 CMD_FS2_READ_RUN = 0x45;

constexpr u8 RSP_PING = 0x81;
constexpr u8 RSP_SD_READ = 0x83;
constexpr u8 RSP_XFER_STATUS = 0x86;
constexpr u8 RSP_SD_READ_REC = 0x88;
constexpr u8 RSP_SD_WRITE_REC = 0x89;
constexpr u8 RSP_SD_FLUSH = 0x8a;
constexpr u8 RSP_LINK_SYNC = 0x8c;
constexpr u8 RSP_HID_STATUS = 0x8d;
constexpr u8 RSP_HID_INPUT = 0x8e;
constexpr u8 RSP_SD_READ_BLOCK = 0x8f;
constexpr u8 RSP_VOL_MOUNT = 0x90;
constexpr u8 RSP_VOL_INFO = 0x91;
constexpr u8 RSP_HID_MOUSE = 0x92;
constexpr u8 RSP_FS_OPENDIR = 0xa0;
constexpr u8 RSP_FS_READDIR = 0xa1;
constexpr u8 RSP_FS_OPEN = 0xa2;
constexpr u8 RSP_FS_READ = 0xa3;
constexpr u8 RSP_FS_WRITE = 0xa4;
constexpr u8 RSP_FS_CLOSE = 0xa5;
constexpr u8 RSP_FS_STAT = 0xa6;
constexpr u8 RSP_FS_DELETE = 0xa7;
constexpr u8 RSP_FS_SELFTEST = 0xa8;
constexpr u8 RSP_FS2_CAPS = 0xb0;
constexpr u8 RSP_FS2_GENERATION = 0xb1;
constexpr u8 RSP_FS2_RESET = 0xb2;
constexpr u8 RSP_FS2_ROOT = 0xb3;
constexpr u8 RSP_FS2_PUSH = 0xb4;
constexpr u8 RSP_FS2_OPEN_RO = 0xb5;
constexpr u8 RSP_FS2_READ = 0xb6;
constexpr u8 RSP_FS2_CLOSE = 0xb7;
constexpr u8 RSP_FS2_OPENDIR = 0xb8;
constexpr u8 RSP_FS2_READDIR = 0xb9;
constexpr u8 RSP_FS2_CLOSEDIR = 0xba;
constexpr u8 RSP_FS2_STAT = 0xbb;
constexpr u8 RSP_FS2_SPACE = 0xbc;
constexpr u8 RSP_FS2_OPEN_RW = 0xbd;
constexpr u8 RSP_FS2_WRITE = 0xbe;
constexpr u8 RSP_FS2_SYNC = 0xbf;
constexpr u8 RSP_FS2_TRUNCATE = 0xc0;
constexpr u8 RSP_FS2_UNLINK = 0xc1;
constexpr u8 RSP_FS2_RENAME = 0xc2;
constexpr u8 RSP_FS2_MKDIR = 0xc3;
constexpr u8 RSP_FS2_RMDIR = 0xc4;
constexpr u8 RSP_FS2_READ_RUN = 0xc5;
constexpr u8 RSP_UNKNOWN_COMMAND = 0xfe;

constexpr u8 IOC_STATUS_OK = 0x00;
constexpr u8 IOC_STATUS_ERROR = 0x01;
constexpr u8 IOC_STATUS_UNKNOWN_CMD = 0x02;
constexpr u8 IOC_STATUS_SD_NO_RESPONSE = 0x10;
constexpr u8 IOC_STATUS_SD_UNUSABLE = 0x11;
constexpr u8 IOC_STATUS_SD_NOT_READY = 0x12;
constexpr u8 IOC_STATUS_SD_READ_FAIL = 0x13;
constexpr u8 IOC_STATUS_SD_BUS = 0x14;
constexpr u8 IOC_STATUS_SD_NO_CARD = 0x15;
constexpr u8 IOC_STATUS_SD_NO_TOKEN = 0x16;
constexpr u8 IOC_STATUS_SD_CRC = 0x17;
constexpr u8 IOC_STATUS_SD_WRITE_FAIL = 0x18;
constexpr u8 IOC_STATUS_SD_WRITE_REJ = 0x19;
constexpr u8 IOC_STATUS_SD_WRITE_BUSY = 0x1a;
constexpr u8 IOC_STATUS_BULK_NO_HOST = 0x21;
constexpr u8 IOC_STATUS_BULK_NO_SYNC = 0x22;
constexpr u8 IOC_STATUS_VOL_UNMOUNTED = 0x30;
constexpr u8 IOC_STATUS_VOL_RANGE = 0x31;
constexpr u8 IOC_STATUS_VOL_BAD_IMAGE = 0x32;
constexpr u8 IOC_STATUS_FS_NOT_MOUNTED = 0x33;
constexpr u8 IOC_STATUS_FS_NO_FILE = 0x34;
constexpr u8 IOC_STATUS_FS_BAD_NAME = 0x35;
constexpr u8 IOC_STATUS_FS_NO_HANDLE = 0x36;
constexpr u8 IOC_STATUS_FS_RANGE = 0x37;
constexpr u8 IOC_STATUS_FS_ERROR = 0x38;
constexpr u8 IOC_STATUS_FS2_NOT_FOUND = 0x40;
constexpr u8 IOC_STATUS_FS2_END = 0x41;
constexpr u8 IOC_STATUS_FS2_EXISTS = 0x42;
constexpr u8 IOC_STATUS_FS2_BAD_NAME = 0x43;
constexpr u8 IOC_STATUS_FS2_READ_ONLY = 0x44;
constexpr u8 IOC_STATUS_FS2_NO_SPACE = 0x45;
constexpr u8 IOC_STATUS_FS2_NOT_DIR = 0x46;
constexpr u8 IOC_STATUS_FS2_IS_DIR = 0x47;
constexpr u8 IOC_STATUS_FS2_NO_HANDLE = 0x48;
constexpr u8 IOC_STATUS_FS2_STALE = 0x49;
constexpr u8 IOC_STATUS_FS2_RANGE = 0x4a;
constexpr u8 IOC_STATUS_FS2_NO_MEDIA = 0x4b;
constexpr u8 IOC_STATUS_FS2_IO = 0x4e;

// PING
constexpr u8 IOC_FW_LEVEL = 75;
constexpr unsigned IOC_OFF_PING_LEVEL = 20;
constexpr unsigned IOC_OFF_PING_POWER = 21;
constexpr unsigned IOC_OFF_PING_CALLS_LO = 22;
constexpr unsigned IOC_OFF_PING_ABORTS_LO = 24;
constexpr unsigned PING_ECHO_BYTES = 16;

// The PING power byte of a controller on a powered board with no shutdown
// request: /PWR_OFF high, latched high and driven; /SHUTDOWN_RQ high, no edge
// latched, its pull-up on (power_init); no stack-overflow reset.
constexpr u8 IOC_PING_PWR_OFF_PIN = 0x01;
constexpr u8 IOC_PING_PWR_OFF_LAT = 0x02;
constexpr u8 IOC_PING_PWR_OFF_DRIVEN = 0x04;
constexpr u8 IOC_PING_SHUTDOWN_PIN = 0x08;
constexpr u8 IOC_PING_SHUTDOWN_WPU = 0x20;
constexpr u8 IOC_PING_LINK_SYNCED = 0x40;
constexpr u8 PING_POWER_IDLE = IOC_PING_PWR_OFF_PIN | IOC_PING_PWR_OFF_LAT | IOC_PING_PWR_OFF_DRIVEN |
		IOC_PING_SHUTDOWN_PIN | IOC_PING_SHUTDOWN_WPU;

// READY and DONE
constexpr unsigned IOC_OFF_READY_XFER_ID = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_READY_DIRECTION = OFF_PAYLOAD + 1;
constexpr unsigned IOC_OFF_READY_LEN_LO = OFF_PAYLOAD + 2;
constexpr unsigned IOC_OFF_READY_LBA = OFF_PAYLOAD + 4;
constexpr u8 IOC_READY_PAYLOAD_LEN = 8;
constexpr unsigned IOC_OFF_DONE_XFER_ID = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_DONE_STATUS = OFF_PAYLOAD + 1;
constexpr u8 IOC_DONE_PAYLOAD_LEN = 4 + 8 + 8;
constexpr u8 BULK_DIR_MCU_TO_Z80 = 0x00;
constexpr u8 BULK_DIR_Z80_TO_MCU = 0x01;

// SD
constexpr unsigned IOC_SD_READ_BYTES = 16;
constexpr unsigned SD_TRACE_BYTES = 8;
constexpr unsigned IOC_SD_RECORD_BYTES = 128;
constexpr unsigned IOC_OFF_RECORD_0 = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_RECORD_UNIT = OFF_PAYLOAD + 4;
constexpr unsigned IOC_SD_RECORD_UNIT_LEN = 5;

// Volumes
constexpr unsigned IOC_OFF_VOL_REQ_UNIT = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_VOL_REQ_NAME = OFF_PAYLOAD + 1;
constexpr unsigned IOC_OFF_VOL_UNIT = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_VOL_MODE = OFF_PAYLOAD + 1;
constexpr unsigned IOC_OFF_VOL_GUARD = OFF_PAYLOAD + 22;
constexpr u8 IOC_VOL_INFO_LEN = 25;
constexpr u32 VOL_IMAGE_RECORDS = 65536;
constexpr u32 VOL_IMAGE_BLOCKS = VOL_IMAGE_RECORDS / 4;
constexpr u32 VOL_IMAGE_BYTES = VOL_IMAGE_RECORDS * 128;
constexpr u8 VOL_CPM_DIR[11] = { 'C', 'P', 'M', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' };
constexpr u8 VOL_DEFAULT[2][11] = {
	{ 'C', 'P', 'M', '_', '1', ' ', ' ', ' ', 'D', 'R', 'V' },
	{ 'C', 'P', 'M', '_', '2', ' ', ' ', ' ', 'D', 'R', 'V' } };
constexpr u8 VOL_RAW_NAME[11] = { '*', 'R', 'A', 'W', '*', ' ', ' ', ' ', ' ', ' ', ' ' };

// Shared folder
constexpr char SHARE_DIR[] = "/SHARED";
constexpr char SHARE_PREFIX[] = "/SHARED/";
constexpr unsigned SHARE_PREFIX_LEN = 8;
constexpr unsigned IOC_FS_CHUNK_MAX = 512;
constexpr unsigned IOC_OFF_FS_NAME = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS_OPEN_MODE = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS_OPEN_NAME = OFF_PAYLOAD + 1;
constexpr unsigned IOC_OFF_FS_XFER_HANDLE = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS_XFER_OFFSET = OFF_PAYLOAD + 1;
constexpr unsigned IOC_OFF_FS_XFER_LEN = OFF_PAYLOAD + 5;
constexpr unsigned IOC_OFF_FS_CLOSE_HANDLE = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS_CLOSE_SIZE = OFF_PAYLOAD + 1;
constexpr u8 IOC_FS_MODE_WRITE = 0x01;
constexpr unsigned IOC_OFF_FS_DIR_ATTR = OFF_PAYLOAD + 11;
constexpr unsigned IOC_OFF_FS_DIR_SIZE = OFF_PAYLOAD + 12;
constexpr unsigned IOC_OFF_FS_DIR_MORE = OFF_PAYLOAD + 16;
constexpr u8 IOC_FS_READDIR_LEN = 17;
constexpr u8 IOC_FS_OPEN_REPLY_LEN = 5;
constexpr u8 IOC_FS_STAT_REPLY_LEN = 5;
constexpr unsigned IOC_OFF_FS_SELF_MOUNTED = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS_SELF_FSTYPE = OFF_PAYLOAD + 1;
constexpr unsigned IOC_OFF_FS_SELF_CSIZE = OFF_PAYLOAD + 2;
constexpr unsigned IOC_OFF_FS_SELF_DATABASE = OFF_PAYLOAD + 4;
constexpr unsigned IOC_OFF_FS_SELF_FILES = OFF_PAYLOAD + 8;
constexpr unsigned IOC_OFF_FS_SELF_SUM = OFF_PAYLOAD + 10;
constexpr unsigned IOC_OFF_FS_SELF_BYTES = OFF_PAYLOAD + 12;
constexpr unsigned IOC_OFF_FS_SELF_FRESULT = OFF_PAYLOAD + 16;
constexpr u8 IOC_FS_SELFTEST_LEN = 17;

// FS2
constexpr u8 IOC_FS2_VERSION = 1;
constexpr u8 IOC_FS2_STATUS_VERSION = 1;
constexpr u16 IOC_FS2_CHUNK_MAX = 512;
constexpr u8 IOC_FS2_FILE_SLOTS = 2;
constexpr u8 IOC_FS2_DIR_SLOTS = 1;
constexpr u8 IOC_FS2_RESOLVER_COMPONENTS = 16;
constexpr u8 IOC_FS2_TOKEN_BYTES = 2;
constexpr u16 IOC_FS2_CAPS_FLAGS = 0x0002 | 0x0004 | 0x0008 | 0x0010 | 0x0020 | 0x1000 // offset, resolver, generation, stat, space, run
		| 0x0040 | 0x0080 | 0x0100 | 0x0200 | 0x0400 | 0x0800;                          // write, truncate, unlink, rename, mkdir, rmdir
constexpr u8 IOC_FS2_CAP_REPLY_LEN = 14;
constexpr u8 IOC_FS2_GENERATION_REPLY_LEN = 4;
constexpr unsigned IOC_OFF_FS2_NAME = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS2_OPEN_MODE = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS2_OPEN_NAME = OFF_PAYLOAD + 1;
constexpr u8 IOC_FS2_OPEN_UPDATE = 0;
constexpr u8 IOC_FS2_OPEN_CREATE_NEW = 1;
constexpr u8 IOC_FS2_OPEN_CREATE_ALWAYS = 2;
constexpr unsigned IOC_OFF_FS2_TOKEN = OFF_PAYLOAD + 0;
constexpr unsigned IOC_OFF_FS2_OFFSET = OFF_PAYLOAD + 2;
constexpr unsigned IOC_OFF_FS2_LENGTH = OFF_PAYLOAD + 6;
constexpr unsigned IOC_OFF_FS2_TRUNCATE_SIZE = OFF_PAYLOAD + 2;
constexpr unsigned IOC_OFF_FS2_RENAME_TO = OFF_PAYLOAD + 11;
constexpr u8 IOC_FS2_OPEN_REPLY_LEN = 7;
constexpr u8 IOC_FS2_DIR_REPLY_LEN = 2;
constexpr u8 IOC_FS2_DIRENT_REPLY_LEN = 16;
constexpr u8 IOC_FS2_STAT_REPLY_LEN = 5;
constexpr u8 IOC_FS2_SPACE_REPLY_LEN = 16;

// HID
constexpr unsigned IOC_OFF_HID_REQ_PAGE = OFF_PAYLOAD;
constexpr u8 IOC_HID_PAGE_GAMEPAD = 0x06;
constexpr u8 IOC_HID_PAGE_OTHER = 0x07;
constexpr u8 IOC_HID_STATUS_PAYLOAD_LEN = 26;
constexpr u8 IOC_HID_GAMEPAD_PAYLOAD_LEN = 26;
constexpr u8 IOC_HID_OTHER_PAYLOAD_LEN = 21;
constexpr unsigned IOC_OFF_HID_USB_INT = OFF_PAYLOAD + 2;
constexpr unsigned IOC_OFF_HID_DEV_COUNT = OFF_PAYLOAD + 13;
constexpr unsigned IOC_OFF_HID_KBD_ADDR = OFF_PAYLOAD + 14;
constexpr unsigned IOC_OFF_HID_REPORTS = OFF_PAYLOAD + 15;      // LO, HI
constexpr unsigned IOC_OFF_HID_LAST_REPORT = OFF_PAYLOAD + 17;  // 8 bytes
constexpr unsigned IOC_OFF_HID_KBD_SPEED = OFF_PAYLOAD + 25;
constexpr u8 IOC_HID_INPUT_META_LEN = 2;
constexpr u8 IOC_HID_MOUSE_PAYLOAD_LEN = 9;

u16 get16(u8 const *p) { return p[0] | (p[1] << 8); }
u32 get32(u8 const *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (u32(p[3]) << 24); }
void put16(u8 *p, u16 v) { p[0] = u8(v); p[1] = u8(v >> 8); }
void put32(u8 *p, u32 v) { p[0] = u8(v); p[1] = u8(v >> 8); p[2] = u8(v >> 16); p[3] = u8(v >> 24); }

void reply_init(zephyr80_ioc_core::frame const &request, zephyr80_ioc_core::frame &reply, u8 cls, u8 status, u8 len)
{
	reply.fill(0);
	reply[OFF_CLASS] = cls;
	reply[OFF_SEQ] = request[OFF_SEQ];
	reply[OFF_STATUS] = status;
	reply[OFF_LEN] = len;
}

// external_sync.c is_command_class (normal profile, IOC_FS_COMMANDS = 1).
bool is_command_class(u8 value)
{
	switch (value)
	{
	case CMD_PING: case CMD_RESET: case CMD_SD_READ: case CMD_BULK_TEST:
	case CMD_SD_READ_BULK: case CMD_SD_WRITE_BULK: case CMD_SD_READ_REC:
	case CMD_SD_READ_BLOCK: case CMD_SD_WRITE_REC: case CMD_SD_FLUSH:
	case CMD_PROFILE: case CMD_LINK_SYNC: case CMD_HID_STATUS: case CMD_HID_INPUT:
	case CMD_HID_MOUSE: case CMD_XFER_STATUS: case CMD_VOL_MOUNT: case CMD_VOL_INFO:
		return true;
	default:
		return (value >= CMD_FS_OPENDIR && value <= CMD_FS_SELFTEST) ||
				(value >= CMD_FS2_CAPS && value <= CMD_FS2_READ_RUN);
	}
}

// ioc_status.c ioc_status_from_sd
u8 status_from_card(zephyr80_ioc_storage::card st)
{
	using card = zephyr80_ioc_storage::card;
	switch (st)
	{
	case card::OK:             return IOC_STATUS_OK;
	case card::NO_CARD:        return IOC_STATUS_SD_NO_CARD;
	case card::NO_RESPONSE:    return IOC_STATUS_SD_NO_RESPONSE;
	case card::UNUSABLE:       return IOC_STATUS_SD_UNUSABLE;
	case card::NOT_READY:      return IOC_STATUS_SD_NOT_READY;
	case card::READ:           return IOC_STATUS_SD_READ_FAIL;
	case card::NO_TOKEN:       return IOC_STATUS_SD_NO_TOKEN;
	case card::CRC:            return IOC_STATUS_SD_CRC;
	case card::BUS:            return IOC_STATUS_SD_BUS;
	case card::WRITE:          return IOC_STATUS_SD_WRITE_FAIL;
	case card::WRITE_REJECTED: return IOC_STATUS_SD_WRITE_REJ;
	case card::WRITE_BUSY:     return IOC_STATUS_SD_WRITE_BUSY;
	default:                   return IOC_STATUS_ERROR;
	}
}

// fs_share.c fres_to_ioc
u8 share_status(zephyr80_ioc_storage::fr result)
{
	using fr = zephyr80_ioc_storage::fr;
	switch (result)
	{
	case fr::OK:            return IOC_STATUS_OK;
	case fr::NO_FILE:
	case fr::NO_PATH:       return IOC_STATUS_FS_NO_FILE;
	case fr::INVALID_NAME:  return IOC_STATUS_FS_BAD_NAME;
	case fr::NOT_ENABLED:
	case fr::NO_FILESYSTEM: return IOC_STATUS_FS_NOT_MOUNTED;
	case fr::DISK_ERR:
	case fr::NOT_READY:     return IOC_STATUS_SD_READ_FAIL;
	default:                return IOC_STATUS_FS_ERROR;
	}
}

// fs2.c legal_char
bool legal_char(u8 c)
{
	if (c < 0x21 || c > 0x7e)
		return false;
	return !std::strchr("\"*+,/:;<=>?[\\]|.~", c);
}

// fs2.c unpack_name: a packed 8.3 name (high bits ignored) to "NAME.EXT".
bool unpack_name(char *out, u8 const *name)
{
	unsigned n = 0;
	bool saw_space = false;
	for (unsigned i = 0; i < 8; ++i)
	{
		u8 const c = name[i] & 0x7f;
		if (c == ' ')
		{
			saw_space = true;
			continue;
		}
		if (saw_space || !legal_char(c))
			return false;
		out[n++] = char(c);
	}
	if (n == 0)
		return false;
	saw_space = false;
	if (name[8] != ' ')
	{
		out[n++] = '.';
		for (unsigned i = 8; i < 11; ++i)
		{
			u8 const c = name[i] & 0x7f;
			if (c == ' ')
			{
				saw_space = true;
				continue;
			}
			if (saw_space || !legal_char(c))
				return false;
			out[n++] = char(c);
		}
	}
	else if (name[9] != ' ' || name[10] != ' ')
	{
		return false;
	}
	out[n] = 0;
	return true;
}

// fs2.c pack_name: "NAME.EXT" to a packed, upper-case 8.3 name.
bool fs2_pack_name(u8 *out, char const *name)
{
	unsigned b = 0, e = 8;
	bool extension = false;
	std::fill_n(out, 11, ' ');
	for (u8 c; (c = u8(*name++)) != 0; )
	{
		if (c == '.')
		{
			if (extension || b == 0)
				return false;
			extension = true;
			continue;
		}
		if (!legal_char(c))
			return false;
		if (c >= 'a' && c <= 'z')
			c -= 'a' - 'A';
		if (!extension)
		{
			if (b >= 8)
				return false;
			out[b++] = c;
		}
		else
		{
			if (e >= 11)
				return false;
			out[e++] = c;
		}
	}
	return b != 0 && (!extension || e != 8);
}

// fs_share.c pack_name: no validation, truncating.
void share_pack_name(u8 *out, char const *fname)
{
	unsigned i = 0, n = 0;
	std::fill_n(out, 11, ' ');
	while (fname[i] && fname[i] != '.' && n < 8)
		out[n++] = u8(fname[i++]);
	while (fname[i] && fname[i] != '.')
		i++;
	if (fname[i] == '.')
	{
		i++;
		n = 8;
		while (fname[i] && n < 11)
			out[n++] = u8(fname[i++]);
	}
}

// fs_share.c share_path: "/SHARED/" + a packed name, rejecting anything
// that is not purely a name.
bool share_path(char *out, u8 const *name11)
{
	for (unsigned i = 0; i < 11; i++)
	{
		u8 const c = name11[i];
		if (c == ' ')
			continue;
		if (c < 0x21 || c > 0x7e || c == '/' || c == '\\' || c == ':' || c == '.')
			return false;
	}
	std::memcpy(out, SHARE_PREFIX, SHARE_PREFIX_LEN);
	unsigned n = SHARE_PREFIX_LEN;
	for (unsigned i = 0; i < 8 && name11[i] != ' '; i++)
		out[n++] = char(name11[i]);
	if (n == SHARE_PREFIX_LEN)
		return false;
	if (name11[8] != ' ')
	{
		out[n++] = '.';
		for (unsigned i = 8; i < 11 && name11[i] != ' '; i++)
			out[n++] = char(name11[i]);
	}
	out[n] = 0;
	return true;
}

} // anonymous namespace


zephyr80_ioc_core::zephyr80_ioc_core(link &transport)
	: m_link(transport)
	, m_storage(nullptr)
	, m_calls(0)
	, m_aborts(0)
	, m_hid(transport)
{
}

// The controller restarts with the board (main.c platform_init, hid_host_init):
// every protocol object is forgotten, the counters restart.
void zephyr80_ioc_core::reset()
{
	m_storage->controller_reset();
	m_calls = 0;
	m_aborts = 0;

	m_xfer_counter = 0;
	m_last_xfer_id = 0;
	m_last_status = 0;
	m_bulk_dir = BULK_DIR_MCU_TO_Z80;
	m_bulk_length = 0;
	m_bulk_buffer = BUF_CHUNK;
	m_bulk_offset = 0;
	m_bulk_id = m_bulk_type = m_bulk_seq = m_bulk_status = 0;
	m_bulk_commit = COMMIT_NONE;
	m_run_sent = m_run_length = 0;
	m_run_id = m_run_seq = m_run_type = 0;
	m_chunk.fill(0);
	m_block.fill(0);
	m_record.fill(0);
	m_write_record = 0;
	m_write_unit = 0;
	m_write_offset = 0;
	m_write_length = 0;
	m_write_slot = 0;

	for (unsigned u = 0; u < VOL_UNITS; u++)
		m_vol_open[u] = false;
	vol_init();

	m_share_dir_open = false;
	m_share_dir = 0;
	m_share_handle = 0;
	m_share_next = 1;
	m_share_writing = false;
	m_share_pos = 0;
	m_share_file = 0;

	// fs2_init
	for (unsigned i = 0; i < IOC_FS2_FILE_SLOTS; ++i)
		m_files[i] = fs2_file{ 0, 0, u8(i + 1), false, false, 0 };
	m_dir_generation = 0;
	m_dir_cookie = 1;
	m_dir_open = false;
	m_dir_object = 0;
	fs2_root_path();
	m_generation = 1;

	m_hid.reset();
}

u16 zephyr80_ioc_core::crc16(u16 crc, u8 data)
{
	crc ^= u16(data) << 8;
	for (int i = 0; i < 8; i++)
		crc = (crc & 0x8000) ? u16((crc << 1) ^ 0x1021) : u16(crc << 1);
	return crc;
}

// packet_at: the marker, a command-lane length, the whole packet inside the
// window, an admitted class, a zero request STATUS and the CRC.
bool zephyr80_ioc_core::packet_at(u8 const *packet, unsigned available, frame &request) const
{
	if (available < 2 + 2 + FIXED_LEN + 2 || packet[0] != SYNC0 || packet[1] != SYNC1)
		return false;
	unsigned const length = packet[2] | (packet[3] << 8);
	if (length < FIXED_LEN || length > FIXED_LEN + MAX_DATA || 2 + 2 + length + 2 > available)
		return false;
	u8 const type = packet[4];
	if (!is_command_class(type) || packet[6] != IOC_STATUS_OK)
		return false;

	u16 crc = 0;
	for (unsigned i = 2; i < 4 + length; i++)
		crc = crc16(crc, packet[i]);
	if (crc != ((packet[4 + length] << 8) | packet[5 + length]))
		return false;

	request.fill(0);
	request[OFF_CLASS] = type;
	request[OFF_SEQ] = packet[5];
	request[OFF_STATUS] = packet[6];
	request[OFF_LEN] = length - FIXED_LEN;
	std::copy_n(packet + 7, length - FIXED_LEN, request.begin() + OFF_PAYLOAD);
	return true;
}

// copy_received_frame: a window of marking idle holds no request; otherwise
// the first position where a whole valid packet starts.
bool zephyr80_ioc_core::decode(u8 const *window, unsigned length, frame &request) const
{
	if (std::all_of(window, window + length, [] (u8 b) { return b == 0xff; }))
		return false;
	for (unsigned start = 0; start < length; start++)
		if (packet_at(window + start, length - start, request))
			return true;
	return false;
}

// external_sync_send: the reply packet, CRC high byte first.
unsigned zephyr80_ioc_core::encode(frame const &reply, u8 *wire)
{
	unsigned const data = std::min<unsigned>(reply[OFF_LEN], MAX_DATA);
	unsigned const length = FIXED_LEN + data;
	unsigned n = 0;
	wire[n++] = SYNC0;
	wire[n++] = SYNC1;
	wire[n++] = u8(length);
	wire[n++] = u8(length >> 8);
	wire[n++] = reply[OFF_CLASS];
	wire[n++] = reply[OFF_SEQ];
	wire[n++] = reply[OFF_STATUS];
	for (unsigned i = 0; i < data; i++)
		wire[n++] = reply[OFF_PAYLOAD + i];
	u16 crc = 0;
	for (unsigned i = 2; i < n; i++)
		crc = crc16(crc, wire[i]);
	wire[n++] = u8(crc >> 8);
	wire[n++] = u8(crc);
	return n;
}

// dispatch.c dispatch_command, normal profile with the filesystem commands.
bool zephyr80_ioc_core::dispatch(frame const &request, frame &reply)
{
	switch (request[OFF_CLASS])
	{
	case CMD_FS2_CAPS:       fs2_caps(request, reply); break;
	case CMD_FS2_GENERATION: fs2_generation(request, reply); break;
	case CMD_FS2_RESET:      fs2_reset(request, reply); break;
	case CMD_FS2_ROOT:       fs2_root(request, reply); break;
	case CMD_FS2_PUSH:       fs2_push(request, reply); break;
	case CMD_FS2_OPEN_RO:    fs2_open_ro(request, reply); break;
	case CMD_FS2_READ:       fs2_read(request, reply, RSP_FS2_READ, IOC_FS2_CHUNK_MAX, 0); break;
	case CMD_FS2_READ_RUN:   fs2_read(request, reply, RSP_FS2_READ_RUN, RUN_MAX, IOC_FS2_CHUNK_MAX - 1); break;
	case CMD_FS2_CLOSE:      fs2_close(request, reply); break;
	case CMD_FS2_OPENDIR:    fs2_opendir(request, reply); break;
	case CMD_FS2_READDIR:    fs2_readdir(request, reply); break;
	case CMD_FS2_CLOSEDIR:   fs2_closedir(request, reply); break;
	case CMD_FS2_STAT:       fs2_stat(request, reply); break;
	case CMD_FS2_SPACE:      fs2_space(request, reply); break;
	case CMD_FS2_OPEN_RW:    fs2_open_rw(request, reply); break;
	case CMD_FS2_WRITE:      fs2_write(request, reply); break;
	case CMD_FS2_SYNC:       fs2_sync(request, reply); break;
	case CMD_FS2_TRUNCATE:   fs2_truncate(request, reply); break;
	case CMD_FS2_UNLINK:     fs2_namespace(request, reply, RSP_FS2_UNLINK, CMD_FS2_UNLINK); break;
	case CMD_FS2_RENAME:     fs2_rename(request, reply); break;
	case CMD_FS2_MKDIR:      fs2_namespace(request, reply, RSP_FS2_MKDIR, CMD_FS2_MKDIR); break;
	case CMD_FS2_RMDIR:      fs2_namespace(request, reply, RSP_FS2_RMDIR, CMD_FS2_RMDIR); break;
	case CMD_FS_OPENDIR:     fs_opendir(request, reply); break;
	case CMD_FS_READDIR:     fs_readdir(request, reply); break;
	case CMD_FS_OPEN:        fs_open(request, reply); break;
	case CMD_FS_READ:        fs_read(request, reply); break;
	case CMD_FS_WRITE:       fs_write(request, reply); break;
	case CMD_FS_CLOSE:       fs_close(request, reply); break;
	case CMD_FS_STAT:        fs_stat(request, reply); break;
	case CMD_FS_DELETE:      fs_delete(request, reply); break;
	case CMD_FS_SELFTEST:    fs_selftest(request, reply); break;
	case CMD_VOL_MOUNT:      vol_mount_cmd(request, reply); break;
	case CMD_VOL_INFO:       vol_info(request, reply); break;
	case CMD_PING:           ping(request, reply); break;
	case CMD_SD_READ:        sd_read(request, reply); break;
	case CMD_SD_READ_BLOCK:  sd_read_block(request, reply); break;
	case CMD_SD_READ_REC:    sd_read_rec(request, reply); break;
	case CMD_SD_WRITE_REC:   sd_write_rec(request, reply); break;
	case CMD_HID_STATUS:     hid_status(request, reply); break;
	case CMD_HID_INPUT:      hid_input(request, reply); break;
	case CMD_HID_MOUSE:      hid_mouse(request, reply); break;
	case CMD_SD_FLUSH:       sd_flush(request, reply); break;
	case CMD_XFER_STATUS:    xfer_status(request, reply); break;

	case CMD_LINK_SYNC:
		// handler_link_sync: both lanes, before the reply, so the reply
		// re-establishes the command lane's boundary.
		m_link.resync_lanes();
		reply_init(request, reply, RSP_LINK_SYNC, IOC_STATUS_OK, 0);
		break;

	case CMD_RESET:
		// handler_reset: the host reset pair, then the controller restarts;
		// it never replies.
		m_link.host_reset();
		return false;

	default:
		// CMD_BULK_TEST, CMD_SD_READ_BULK, CMD_SD_WRITE_BULK and CMD_PROFILE
		// are diagnostic-profile classes: admitted, then refused here.
		unknown(request, reply);
		break;
	}
	return true;
}

void zephyr80_ioc_core::unknown(frame const &request, frame &reply) const
{
	reply_init(request, reply, RSP_UNKNOWN_COMMAND, IOC_STATUS_UNKNOWN_CMD, 0);
}

void zephyr80_ioc_core::ping(frame const &request, frame &reply) const
{
	reply_init(request, reply, RSP_PING, IOC_STATUS_OK, MAX_DATA);
	std::copy_n(request.begin() + OFF_PAYLOAD, PING_ECHO_BYTES, reply.begin() + OFF_PAYLOAD);
	reply[IOC_OFF_PING_LEVEL] = IOC_FW_LEVEL;
	reply[IOC_OFF_PING_POWER] = PING_POWER_IDLE | (m_link.command_lane_synced() ? IOC_PING_LINK_SYNCED : 0);
	put16(&reply[IOC_OFF_PING_CALLS_LO], m_calls);
	put16(&reply[IOC_OFF_PING_ABORTS_LO], m_aborts);
	// Bytes 26-29 are diagnostic-profile edge counts and read zero.
}

// handler_xfer_status: the DONE record.  The peek, raw-window and sync
// fields are diagnostic-profile fields and read zero.
void zephyr80_ioc_core::xfer_status(frame const &request, frame &reply) const
{
	reply_init(request, reply, RSP_XFER_STATUS, IOC_STATUS_OK, IOC_DONE_PAYLOAD_LEN);
	reply[IOC_OFF_DONE_XFER_ID] = m_last_xfer_id;
	reply[IOC_OFF_DONE_STATUS] = m_last_status;
}

// The controller's idle branch (main.c): vol_service, the one auto-mount of
// the conventional images once the card is up by any path.
void zephyr80_ioc_core::idle()
{
	if (m_automount_done || !m_storage->card_ready())
		return;
	m_automount_done = true;
	for (unsigned u = 0; u < VOL_UNITS; u++)
		if (vol_mount(u, VOL_DEFAULT[u]) != card::OK)
			break; // no filesystem, or no image
}


//**************************************************************************
//  Bulk (bulk_channel.c)
//**************************************************************************

zephyr80_ioc_core::bulk_dir zephyr80_ioc_core::bulk_armed() const
{
	if (!m_bulk_length)
		return bulk_dir::NONE;
	return (m_bulk_dir == BULK_DIR_Z80_TO_MCU) ? bulk_dir::FROM_Z80 : bulk_dir::TO_Z80;
}

// bulk_channel_next_xfer_id: wraps, never 0.
u8 zephyr80_ioc_core::next_xfer_id()
{
	if (++m_xfer_counter == 0)
		m_xfer_counter = 1;
	return m_xfer_counter;
}

u8 *zephyr80_ioc_core::buffer(u8 which, u16 offset)
{
	switch (which)
	{
	case BUF_BLOCK:  return &m_block[0];
	case BUF_RECORD: return &m_record[0];
	default:         return &m_chunk[offset];
	}
}

u8 const *zephyr80_ioc_core::buffer(u8 which, u16 offset) const
{
	return const_cast<zephyr80_ioc_core *>(this)->buffer(which, offset);
}

void zephyr80_ioc_core::arm_send(u8 which, u16 offset, u16 length, u8 id, u8 type, u8 sequence)
{
	m_bulk_dir = BULK_DIR_MCU_TO_Z80;
	m_bulk_buffer = which;
	m_bulk_offset = offset;
	m_bulk_length = length;
	m_bulk_id = id;
	m_bulk_type = type;
	m_bulk_seq = sequence;
	m_bulk_status = IOC_STATUS_OK;
	m_bulk_commit = COMMIT_NONE;
}

void zephyr80_ioc_core::arm_receive(u8 which, u16 length, u8 id, u8 type, u8 sequence, u8 commit_kind)
{
	m_bulk_dir = BULK_DIR_Z80_TO_MCU;
	m_bulk_buffer = which;
	m_bulk_offset = 0;
	m_bulk_length = length;
	m_bulk_id = id;
	m_bulk_type = type;
	m_bulk_seq = sequence;
	m_bulk_status = IOC_STATUS_OK;
	m_bulk_commit = commit_kind;
}

void zephyr80_ioc_core::ready(frame &reply, u8 id, u8 direction, u16 length, u32 echo)
{
	reply[IOC_OFF_READY_XFER_ID] = id;
	reply[IOC_OFF_READY_DIRECTION] = direction;
	put16(&reply[IOC_OFF_READY_LEN_LO], length);
	put32(&reply[IOC_OFF_READY_LBA], echo);
}

// bulk_run_send: A5 5A LEN TYPE SEQ STATUS DATA and the trailer, which the
// deployed CRC bypass sends as two zero bytes.
unsigned zephyr80_ioc_core::bulk_packet(u8 *wire) const
{
	if (!m_bulk_length || m_bulk_dir != BULK_DIR_MCU_TO_Z80)
		return 0;
	unsigned const length = FIXED_LEN + m_bulk_length;
	unsigned n = 0;
	wire[n++] = SYNC0;
	wire[n++] = SYNC1;
	wire[n++] = u8(length);
	wire[n++] = u8(length >> 8);
	wire[n++] = m_bulk_type;
	wire[n++] = m_bulk_seq;
	wire[n++] = m_bulk_status;
	u8 const *data = buffer(m_bulk_buffer, m_bulk_offset);
	std::copy_n(data, m_bulk_length, wire + n);
	n += m_bulk_length;
	wire[n++] = 0;
	wire[n++] = 0;
	return n;
}

// The bytes are on the wire: for a read that is the whole transfer.
void zephyr80_ioc_core::bulk_sent()
{
	if (!m_bulk_length)
		return;
	m_bulk_length = 0;
	m_last_xfer_id = m_bulk_id;
	m_last_status = IOC_STATUS_OK;
}

// bulk_run_receive's capture window: the packet plus the late-start slack.
unsigned zephyr80_ioc_core::bulk_window() const
{
	if (!m_bulk_length || m_bulk_dir != BULK_DIR_Z80_TO_MCU)
		return 0;
	return std::min<unsigned>(m_bulk_length + (2 + 2 + FIXED_LEN + 2) + BULK_SEARCH + 2, BULK_WINDOW_MAX);
}

// bulk_run_receive after the window: find A5 5A within the search range,
// check the header against what READY armed, take the DATA, then commit.
// The deployed CRC bypass leaves the trailer unchecked.  The lane delivers
// whole characters, so the bit-phase search is a byte search here.
void zephyr80_ioc_core::bulk_received(u8 const *window, unsigned length)
{
	if (!m_bulk_length || m_bulk_dir != BULK_DIR_Z80_TO_MCU)
		return;
	unsigned const data = m_bulk_length;
	m_bulk_length = 0;
	m_last_xfer_id = m_bulk_id;

	unsigned start = 0;
	bool found = false;
	for (; start < BULK_SEARCH && start + 2 <= length; start++)
		if (window[start] == SYNC0 && window[start + 1] == SYNC1)
		{
			found = true;
			break;
		}
	unsigned const header = start + 2;
	if (!found || header + 5 + data > length)
	{
		m_last_status = IOC_STATUS_BULK_NO_SYNC;
		return;
	}
	unsigned const declared = window[header] | (window[header + 1] << 8);
	if (declared != FIXED_LEN + data || window[header + 2] != m_bulk_type || window[header + 3] != m_bulk_seq ||
			window[header + 4] != IOC_STATUS_OK)
	{
		m_last_status = IOC_STATUS_BULK_NO_SYNC;
		return;
	}
	std::copy_n(window + header + 5, data, buffer(m_bulk_buffer, 0));
	m_last_status = commit();
}

// bulk_run_if_armed when the host never asserted its bulk RTS.
void zephyr80_ioc_core::bulk_no_host()
{
	if (!m_bulk_length)
		return;
	m_bulk_length = 0;
	m_last_xfer_id = m_bulk_id;
	m_last_status = IOC_STATUS_BULK_NO_HOST;
}

u8 zephyr80_ioc_core::commit()
{
	switch (m_bulk_commit)
	{
	case COMMIT_RECORD: return vol_write_record(m_write_unit, m_write_record, &m_record[0]);
	case COMMIT_SHARE:  return share_commit();
	case COMMIT_FS2:    return fs2_commit();
	default:            return IOC_STATUS_OK;
	}
}

// fs2_run_next: arm the run's next packet, at most IOC_FS2_CHUNK_MAX bytes.
bool zephyr80_ioc_core::run_next()
{
	if (m_run_sent >= m_run_length)
	{
		m_run_length = 0;
		return false;
	}
	u16 const n = std::min<u16>(m_run_length - m_run_sent, IOC_FS2_CHUNK_MAX);
	arm_send(BUF_CHUNK, m_run_sent, n, m_run_id, m_run_type, m_run_seq);
	m_run_sent += n;
	return true;
}


//**************************************************************************
//  SD records and volumes (handlers.c, volume.c)
//**************************************************************************

namespace {

u32 decode_record(zephyr80_ioc_core::frame const &request) { return get32(&request[IOC_OFF_RECORD_0]); }

// The optional unit byte, read only when the frame carries it.
u8 decode_unit(zephyr80_ioc_core::frame const &request)
{
	return request[OFF_LEN] < IOC_SD_RECORD_UNIT_LEN ? 0 : request[IOC_OFF_RECORD_UNIT];
}

} // anonymous namespace

// handler_sd_read: the first IOC_SD_READ_BYTES of card block 0, or the SD
// trace.  No SPI trace is modelled: it reads as never recorded (zeros).
void zephyr80_ioc_core::sd_read(frame const &request, frame &reply)
{
	card st = m_storage->card_init();
	if (st == card::OK)
		st = m_storage->block_read(0, &m_block[0]);
	reply_init(request, reply, RSP_SD_READ, status_from_card(st), 0);
	if (st != card::OK)
	{
		reply[OFF_LEN] = SD_TRACE_BYTES;
		return;
	}
	reply[OFF_LEN] = IOC_SD_READ_BYTES;
	std::copy_n(m_block.begin(), IOC_SD_READ_BYTES, reply.begin() + OFF_PAYLOAD);
}

void zephyr80_ioc_core::sd_read_rec(frame const &request, frame &reply)
{
	u32 const record = decode_record(request);
	unsigned const unit = decode_unit(request);
	u8 status;
	card st = vol_ensure_mounted();
	if (st != card::OK)
		status = status_from_card(st);
	else if (!vol_map(unit, record >> 2) || record >= m_vol_records[unit])
		status = vol_map_failure(unit);
	else
	{
		status = vol_read_block(unit, record >> 2, &m_block[0]);
		if (status == IOC_STATUS_OK)
			std::copy_n(m_block.begin() + (record & 3) * IOC_SD_RECORD_BYTES, IOC_SD_RECORD_BYTES, m_record.begin());
	}
	reply_init(request, reply, RSP_SD_READ_REC, status, IOC_READY_PAYLOAD_LEN);
	if (status != IOC_STATUS_OK)
		return;
	u8 const id = next_xfer_id();
	ready(reply, id, BULK_DIR_MCU_TO_Z80, IOC_SD_RECORD_BYTES, record);
	arm_send(BUF_RECORD, 0, IOC_SD_RECORD_BYTES, id, RSP_SD_READ_REC, request[OFF_SEQ]);
}

void zephyr80_ioc_core::sd_read_block(frame const &request, frame &reply)
{
	u32 const block = decode_record(request);
	unsigned const unit = decode_unit(request);
	u8 status;
	card st = vol_ensure_mounted();
	if (st != card::OK)
		status = status_from_card(st);
	else if (block > (0xffffffffU >> 2) || !vol_map(unit, block))
		status = vol_map_failure(unit);
	else
		status = vol_read_block(unit, block, &m_block[0]);
	reply_init(request, reply, RSP_SD_READ_BLOCK, status, IOC_READY_PAYLOAD_LEN);
	if (status != IOC_STATUS_OK)
		return;
	u8 const id = next_xfer_id();
	ready(reply, id, BULK_DIR_MCU_TO_Z80, storage::BLOCK_BYTES, block);
	arm_send(BUF_BLOCK, 0, storage::BLOCK_BYTES, id, RSP_SD_READ_BLOCK, request[OFF_SEQ]);
}

// handler_sd_write_rec: READY to receive; the record is written by the
// commit after the bulk phase.
void zephyr80_ioc_core::sd_write_rec(frame const &request, frame &reply)
{
	m_write_record = decode_record(request);
	m_write_unit = decode_unit(request);
	reply_init(request, reply, RSP_SD_WRITE_REC, IOC_STATUS_OK, IOC_READY_PAYLOAD_LEN);
	u8 const id = next_xfer_id();
	ready(reply, id, BULK_DIR_Z80_TO_MCU, IOC_SD_RECORD_BYTES, m_write_record);
	arm_receive(BUF_RECORD, IOC_SD_RECORD_BYTES, id, CMD_SD_WRITE_REC, request[OFF_SEQ], COMMIT_RECORD);
}

void zephyr80_ioc_core::sd_flush(frame const &request, frame &reply)
{
	reply_init(request, reply, RSP_SD_FLUSH, status_from_card(m_storage->flush()), 0);
}

void zephyr80_ioc_core::vol_init()
{
	for (unsigned u = 0; u < VOL_UNITS; u++)
	{
		if (m_vol_open[u])
			m_storage->image_close(m_vol_image[u]);
		m_vol_open[u] = false;
		m_vol_image[u] = 0;
		m_vol_mode[u] = VOL_NONE;
		m_vol_extents[u] = 0;
		m_vol_base[u] = 0;
		m_vol_records[u] = 0;
		std::fill_n(m_vol_name[u], 11, ' ');
	}
	m_mount_attempted = false;
	m_automount_done = false;
}

// vol_ensure_mounted: the card up, and unit 0 in raw mode as the fallback.
// Media without card blocks get no raw unit.
zephyr80_ioc_core::card zephyr80_ioc_core::vol_ensure_mounted()
{
	if (m_mount_attempted && m_storage->card_ready())
		return card::OK;
	if (m_mount_attempted)
		vol_init();
	card const st = m_storage->card_init();
	if (st != card::OK)
		return st;
	m_mount_attempted = true;
	if (m_vol_mode[0] == VOL_NONE && m_storage->raw_blocks())
		mount_raw(0);
	return card::OK;
}

void zephyr80_ioc_core::mount_raw(unsigned unit)
{
	m_vol_mode[unit] = VOL_RAW;
	m_vol_extents[unit] = 1;
	m_vol_base[unit] = 0;
	m_vol_records[unit] = VOL_IMAGE_RECORDS;
	std::copy_n(VOL_RAW_NAME, 11, m_vol_name[unit]);
}

// vol_mount: the old mount goes first; the image is /CPM/<name>, exactly
// 8 MiB.  An unresolved image leaves the unit unmounted.
zephyr80_ioc_core::card zephyr80_ioc_core::vol_mount(unsigned unit, u8 const *name11)
{
	if (unit >= VOL_UNITS)
		return card::READ;
	if (m_vol_open[unit])
		m_storage->image_close(m_vol_image[unit]);
	m_vol_open[unit] = false;
	m_vol_mode[unit] = VOL_NONE;
	m_vol_extents[unit] = 0;
	m_vol_base[unit] = 0;
	m_vol_records[unit] = 0;
	std::fill_n(m_vol_name[unit], 11, ' ');

	storage::image info{ 0, 0 };
	storage::object img = 0;
	switch (m_storage->image_open(VOL_CPM_DIR, name11, VOL_IMAGE_BYTES, info, img))
	{
	case storage::image_status::OK:
		break;
	case storage::image_status::IO:
		return card::READ;
	case storage::image_status::NO_FS:
	case storage::image_status::NO_FILE:
		return card::NO_RESPONSE; // nothing to mount
	default:
		return card::UNUSABLE;    // wrong size, or too fragmented
	}
	m_vol_open[unit] = true;
	m_vol_image[unit] = img;
	std::copy_n(name11, 11, m_vol_name[unit]);
	m_vol_extents[unit] = info.extents;
	m_vol_base[unit] = info.base_lba;
	m_vol_records[unit] = VOL_IMAGE_RECORDS;
	m_vol_mode[unit] = VOL_FILE;
	return card::OK;
}

// vol_map's bound, for a block of the unit.
bool zephyr80_ioc_core::vol_map(unsigned unit, u32 block) const
{
	return unit < VOL_UNITS && m_vol_mode[unit] != VOL_NONE && block < (m_vol_records[unit] >> 2);
}

u8 zephyr80_ioc_core::vol_map_failure(unsigned unit) const
{
	return (unit >= VOL_UNITS || m_vol_mode[unit] == VOL_NONE) ? IOC_STATUS_VOL_UNMOUNTED : IOC_STATUS_VOL_RANGE;
}

u8 zephyr80_ioc_core::vol_read_block(unsigned unit, u32 block, u8 *dst)
{
	if (m_vol_mode[unit] == VOL_RAW)
		return status_from_card(m_storage->block_read(block, dst));
	return status_from_card(m_storage->image_read(m_vol_image[unit], block, dst));
}

// vol_write_record: the record into its containing block (sd_cache's
// read-modify-write; caching and write-back are the backend's).
u8 zephyr80_ioc_core::vol_write_record(unsigned unit, u32 record, u8 const *src)
{
	card const st = vol_ensure_mounted();
	if (st != card::OK)
		return status_from_card(st);
	if (unit >= VOL_UNITS || !vol_map(unit, record >> 2) || record >= m_vol_records[unit])
		return vol_map_failure(unit);
	u32 const block = record >> 2;
	u8 status = vol_read_block(unit, block, &m_block[0]);
	if (status != IOC_STATUS_OK)
		return status;
	std::copy_n(src, IOC_SD_RECORD_BYTES, m_block.begin() + (record & 3) * IOC_SD_RECORD_BYTES);
	if (m_vol_mode[unit] == VOL_RAW)
		return status_from_card(m_storage->block_write(block, &m_block[0]));
	return status_from_card(m_storage->image_write(m_vol_image[unit], block, &m_block[0]));
}

// fill_vol_info / vol_report.  Boot guard: the controller is not degraded
// and has counted no resets; no stack-overflow reset.
void zephyr80_ioc_core::vol_report(unsigned unit, frame &reply) const
{
	reply[IOC_OFF_VOL_UNIT] = u8(unit);
	u8 *out = &reply[IOC_OFF_VOL_MODE];
	std::fill_n(out, 10, 0);
	std::fill_n(out + 10, 11, ' ');
	if (unit < VOL_UNITS)
	{
		out[0] = m_vol_mode[unit];
		out[1] = m_vol_extents[unit];
		put32(out + 2, m_vol_base[unit]);
		put32(out + 6, m_vol_records[unit]);
		std::copy_n(m_vol_name[unit], 11, out + 10);
	}
	reply[IOC_OFF_VOL_GUARD] = 0;
	reply[IOC_OFF_VOL_GUARD + 1] = 0;
	reply[IOC_OFF_VOL_GUARD + 2] = 0;
}

void zephyr80_ioc_core::vol_mount_cmd(frame const &request, frame &reply)
{
	unsigned const unit = request[IOC_OFF_VOL_REQ_UNIT];
	if (unit >= VOL_UNITS)
	{
		reply_init(request, reply, RSP_VOL_MOUNT, IOC_STATUS_VOL_UNMOUNTED, IOC_VOL_INFO_LEN);
		reply[IOC_OFF_VOL_UNIT] = u8(unit);
		return;
	}
	card st = vol_ensure_mounted();
	if (st == card::OK)
		st = vol_mount(unit, &request[IOC_OFF_VOL_REQ_NAME]);
	u8 const status = (st == card::UNUSABLE) ? IOC_STATUS_VOL_BAD_IMAGE : status_from_card(st);
	reply_init(request, reply, RSP_VOL_MOUNT, status, IOC_VOL_INFO_LEN);
	vol_report(unit, reply);
}

// handler_vol_info: what is known now; no mount, no card access.
void zephyr80_ioc_core::vol_info(frame const &request, frame &reply)
{
	reply_init(request, reply, RSP_VOL_INFO, IOC_STATUS_OK, IOC_VOL_INFO_LEN);
	vol_report(request[IOC_OFF_VOL_REQ_UNIT], reply);
}


//**************************************************************************
//  Shared folder (fs_share.c)
//**************************************************************************

u8 zephyr80_ioc_core::share_need_fs()
{
	bool mounted = false;
	if (m_storage->fs_mount(mounted) != card::OK)
		return IOC_STATUS_SD_READ_FAIL;
	return mounted ? IOC_STATUS_OK : IOC_STATUS_FS_NOT_MOUNTED;
}

void zephyr80_ioc_core::share_close_current()
{
	if (m_share_handle)
	{
		m_storage->close(m_share_file);
		m_share_handle = 0;
		m_share_writing = false;
		m_share_pos = 0;
	}
}

zephyr80_ioc_core::fr zephyr80_ioc_core::share_seek(u32 offset)
{
	if (m_share_pos == offset)
		return fr::OK;
	fr const result = m_storage->seek(m_share_file, offset);
	if (result == fr::OK)
		m_share_pos = offset;
	return result;
}

u8 zephyr80_ioc_core::share_xfer_params(frame const &request, u32 &offset, u16 &length) const
{
	if (!m_share_handle || request[IOC_OFF_FS_XFER_HANDLE] != m_share_handle)
		return IOC_STATUS_FS_NO_HANDLE;
	offset = get32(&request[IOC_OFF_FS_XFER_OFFSET]);
	length = get16(&request[IOC_OFF_FS_XFER_LEN]);
	if (length == 0 || length > IOC_FS_CHUNK_MAX)
		return IOC_STATUS_FS_RANGE;
	return IOC_STATUS_OK;
}

void zephyr80_ioc_core::fs_opendir(frame const &request, frame &reply)
{
	u8 const status = share_need_fs();
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS_OPENDIR, status, 0);
	if (m_share_dir_open)
	{
		m_storage->closedir(m_share_dir);
		m_share_dir_open = false;
	}
	fr const result = m_storage->opendir(SHARE_DIR, m_share_dir);
	if (result != fr::OK)
		return reply_init(request, reply, RSP_FS_OPENDIR, share_status(result), 0);
	m_share_dir_open = true;
	reply_init(request, reply, RSP_FS_OPENDIR, IOC_STATUS_OK, 0);
}

// One file entry per call; directories skipped; the end closes the session.
void zephyr80_ioc_core::fs_readdir(frame const &request, frame &reply)
{
	if (!m_share_dir_open)
		return reply_init(request, reply, RSP_FS_READDIR, IOC_STATUS_FS_NO_HANDLE, 0);
	storage::entry info;
	for (;;)
	{
		fr const result = m_storage->readdir(m_share_dir, info);
		if (result != fr::OK)
			return reply_init(request, reply, RSP_FS_READDIR, share_status(result), 0);
		if (!info.name[0])
		{
			m_storage->closedir(m_share_dir);
			m_share_dir_open = false;
			return reply_init(request, reply, RSP_FS_READDIR, IOC_STATUS_OK, IOC_FS_READDIR_LEN);
		}
		if (!(info.attr & storage::ATTR_DIR))
			break;
	}
	reply_init(request, reply, RSP_FS_READDIR, IOC_STATUS_OK, IOC_FS_READDIR_LEN);
	share_pack_name(&reply[OFF_PAYLOAD], info.name);
	reply[IOC_OFF_FS_DIR_ATTR] = info.attr;
	put32(&reply[IOC_OFF_FS_DIR_SIZE], info.size);
	reply[IOC_OFF_FS_DIR_MORE] = 1;
}

void zephyr80_ioc_core::fs_open(frame const &request, frame &reply)
{
	u8 const mode = request[IOC_OFF_FS_OPEN_MODE];
	u8 const status = share_need_fs();
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS_OPEN, status, 0);
	share_close_current();
	char path[SHARE_PREFIX_LEN + 12 + 1];
	if (!share_path(path, &request[IOC_OFF_FS_OPEN_NAME]))
		return reply_init(request, reply, RSP_FS_OPEN, IOC_STATUS_FS_BAD_NAME, 0);
	u8 const flags = (mode == IOC_FS_MODE_WRITE) ? (storage::OPEN_WRITE | storage::OPEN_CREATE_ALWAYS) : storage::OPEN_READ;
	fr const result = m_storage->open(path, flags, m_share_file);
	if (result != fr::OK)
		return reply_init(request, reply, RSP_FS_OPEN, share_status(result), 0);
	m_share_handle = m_share_next++;
	if (!m_share_next)
		m_share_next = 1;
	m_share_writing = (mode == IOC_FS_MODE_WRITE);
	m_share_pos = 0;
	reply_init(request, reply, RSP_FS_OPEN, IOC_STATUS_OK, IOC_FS_OPEN_REPLY_LEN);
	reply[OFF_PAYLOAD] = m_share_handle;
	put32(&reply[OFF_PAYLOAD + 1], m_storage->size(m_share_file));
}

// The chunk is read before READY; a short read at the end of the file is
// not an error, and nothing read arms no transfer.
void zephyr80_ioc_core::fs_read(frame const &request, frame &reply)
{
	u32 offset = 0;
	u16 length = 0;
	u8 const status = share_xfer_params(request, offset, length);
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS_READ, status, 0);
	u16 got = 0;
	fr result = share_seek(offset);
	if (result == fr::OK)
		result = m_storage->read(m_share_file, &m_chunk[0], length, got);
	if (result != fr::OK)
	{
		m_share_pos = 0xffffffff;
		return reply_init(request, reply, RSP_FS_READ, share_status(result), 0);
	}
	m_share_pos = offset + got;
	reply_init(request, reply, RSP_FS_READ, IOC_STATUS_OK, IOC_READY_PAYLOAD_LEN);
	if (!got)
		return;
	u8 const id = next_xfer_id();
	ready(reply, id, BULK_DIR_MCU_TO_Z80, got, offset);
	arm_send(BUF_CHUNK, 0, got, id, RSP_FS_READ, request[OFF_SEQ]);
}

void zephyr80_ioc_core::fs_write(frame const &request, frame &reply)
{
	u32 offset = 0;
	u16 length = 0;
	u8 status = share_xfer_params(request, offset, length);
	if (status == IOC_STATUS_OK && !m_share_writing)
		status = IOC_STATUS_FS_NO_HANDLE;
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS_WRITE, status, 0);
	m_write_offset = offset;
	m_write_length = length;
	reply_init(request, reply, RSP_FS_WRITE, IOC_STATUS_OK, IOC_READY_PAYLOAD_LEN);
	u8 const id = next_xfer_id();
	ready(reply, id, BULK_DIR_Z80_TO_MCU, length, offset);
	arm_receive(BUF_CHUNK, length, id, CMD_FS_WRITE, request[OFF_SEQ], COMMIT_SHARE);
}

// commit_fs_write: a short write is the card or the volume full.
u8 zephyr80_ioc_core::share_commit()
{
	if (!m_share_handle)
		return IOC_STATUS_FS_NO_HANDLE;
	u16 put = 0;
	fr result = share_seek(m_write_offset);
	if (result == fr::OK)
		result = m_storage->write(m_share_file, &m_chunk[0], m_write_length, put);
	if (result != fr::OK)
	{
		m_share_pos = 0xffffffff;
		return share_status(result);
	}
	m_share_pos = m_write_offset + put;
	return (put != m_write_length) ? IOC_STATUS_FS_ERROR : IOC_STATUS_OK;
}

// The exact length arrives here: a file being written only ever shortens.
void zephyr80_ioc_core::fs_close(frame const &request, frame &reply)
{
	if (!m_share_handle || request[IOC_OFF_FS_CLOSE_HANDLE] != m_share_handle)
		return reply_init(request, reply, RSP_FS_CLOSE, IOC_STATUS_FS_NO_HANDLE, 0);
	u32 const final_size = get32(&request[IOC_OFF_FS_CLOSE_SIZE]);
	fr result = fr::OK;
	if (m_share_writing && final_size < m_storage->size(m_share_file))
	{
		result = share_seek(final_size);
		if (result == fr::OK)
			result = m_storage->truncate(m_share_file);
	}
	if (result == fr::OK)
		result = m_storage->close(m_share_file);
	else
		m_storage->close(m_share_file);
	m_share_handle = 0;
	m_share_writing = false;
	m_share_pos = 0;
	u8 status = share_status(result);
	if (status == IOC_STATUS_OK && m_storage->flush() != card::OK)
		status = IOC_STATUS_SD_WRITE_FAIL;
	reply_init(request, reply, RSP_FS_CLOSE, status, 0);
}

void zephyr80_ioc_core::fs_stat(frame const &request, frame &reply)
{
	u8 const status = share_need_fs();
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS_STAT, status, 0);
	char path[SHARE_PREFIX_LEN + 12 + 1];
	if (!share_path(path, &request[IOC_OFF_FS_NAME]))
		return reply_init(request, reply, RSP_FS_STAT, IOC_STATUS_FS_BAD_NAME, 0);
	storage::entry info;
	fr const result = m_storage->stat(path, info);
	if (result != fr::OK)
		return reply_init(request, reply, RSP_FS_STAT, share_status(result), 0);
	reply_init(request, reply, RSP_FS_STAT, IOC_STATUS_OK, IOC_FS_STAT_REPLY_LEN);
	put32(&reply[OFF_PAYLOAD], info.size);
	reply[OFF_PAYLOAD + 4] = info.attr;
}

// The open file is closed first: deleting it would strand the handle.
void zephyr80_ioc_core::fs_delete(frame const &request, frame &reply)
{
	u8 const status = share_need_fs();
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS_DELETE, status, 0);
	char path[SHARE_PREFIX_LEN + 12 + 1];
	if (!share_path(path, &request[IOC_OFF_FS_NAME]))
		return reply_init(request, reply, RSP_FS_DELETE, IOC_STATUS_FS_BAD_NAME, 0);
	share_close_current();
	fr const result = m_storage->unlink(path);
	if (result == fr::OK)
		m_storage->flush();
	reply_init(request, reply, RSP_FS_DELETE, share_status(result), 0);
}

// handler_fs_selftest: the mount, a walk of /SHARED and an additive sum of
// its first regular file, with objects of its own.
void zephyr80_ioc_core::fs_selftest(frame const &request, frame &reply)
{
	reply_init(request, reply, RSP_FS_SELFTEST, IOC_STATUS_OK, IOC_FS_SELFTEST_LEN);
	bool mounted = false;
	if (m_storage->fs_mount(mounted) != card::OK)
	{
		reply[OFF_STATUS] = IOC_STATUS_SD_READ_FAIL;
		return;
	}
	if (!mounted)
	{
		reply[OFF_STATUS] = IOC_STATUS_FS_NOT_MOUNTED;
		return;
	}
	storage::geometry geo{};
	m_storage->fs_geometry(geo);
	reply[IOC_OFF_FS_SELF_MOUNTED] = 1;
	reply[IOC_OFF_FS_SELF_FSTYPE] = geo.fs_type;
	put16(&reply[IOC_OFF_FS_SELF_CSIZE], geo.cluster_blocks);
	put32(&reply[IOC_OFF_FS_SELF_DATABASE], geo.data_base);

	storage::object dir = 0;
	fr result = m_storage->opendir(SHARE_DIR, dir);
	if (result != fr::OK)
	{
		reply[IOC_OFF_FS_SELF_FRESULT] = u8(result);
		reply[OFF_STATUS] = share_status(result);
		return;
	}
	u16 files = 0;
	bool have_file = false;
	char path[SHARE_PREFIX_LEN + 12 + 1];
	storage::entry info;
	for (;;)
	{
		result = m_storage->readdir(dir, info);
		if (result != fr::OK || !info.name[0])
			break;
		if (info.attr & storage::ATTR_DIR)
			continue;
		files++;
		if (!have_file)
		{
			size_t const n = std::min<size_t>(std::strlen(info.name), 12);
			std::memcpy(path, SHARE_PREFIX, SHARE_PREFIX_LEN);
			std::memcpy(path + SHARE_PREFIX_LEN, info.name, n);
			path[SHARE_PREFIX_LEN + n] = 0;
			have_file = true;
		}
	}
	m_storage->closedir(dir);
	put16(&reply[IOC_OFF_FS_SELF_FILES], files);

	u16 sum = 0;
	u32 bytes = 0;
	if (result == fr::OK && have_file)
	{
		storage::object file = 0;
		result = m_storage->open(path, storage::OPEN_READ, file);
		if (result == fr::OK)
		{
			for (;;)
			{
				u16 got = 0;
				result = m_storage->read(file, &m_chunk[0], IOC_FS_CHUNK_MAX, got);
				if (result != fr::OK || !got)
					break;
				for (unsigned i = 0; i < got; i++)
					sum += m_chunk[i];
				bytes += got;
			}
			m_storage->close(file);
		}
	}
	put16(&reply[IOC_OFF_FS_SELF_SUM], sum);
	put32(&reply[IOC_OFF_FS_SELF_BYTES], bytes);
	reply[IOC_OFF_FS_SELF_FRESULT] = u8(result);
	if (result != fr::OK)
		reply[OFF_STATUS] = share_status(result);
}


//**************************************************************************
//  FS2 (fs2.c)
//**************************************************************************

void zephyr80_ioc_core::fs2_root_path()
{
	m_resolver[0] = '/';
	m_resolver[1] = 0;
	m_resolver_components = 0;
}

void zephyr80_ioc_core::fs2_close_files()
{
	for (auto &file : m_files)
	{
		if (file.open)
			m_storage->close(file.object);
		file.open = false;
		file.writable = false;
		file.position = 0;
	}
}

// fs2_media_invalidated: every context closed, the generation advanced
// (never to zero), the resolver back at the root.
void zephyr80_ioc_core::fs2_media_invalidated()
{
	fs2_close_files();
	if (m_dir_open)
		m_storage->closedir(m_dir_object);
	m_dir_open = false;
	if (++m_generation == 0)
		m_generation = 1;
	fs2_root_path();
}

// sdfs_invalidate: the mount dropped, then FS2's contexts.
void zephyr80_ioc_core::sdfs_invalidate()
{
	m_storage->fs_unmount();
	fs2_media_invalidated();
}

// map_result: a media-level failure drops every context.
u8 zephyr80_ioc_core::fs2_map(fr result)
{
	u8 status;
	switch (result)
	{
	case fr::OK:                  status = IOC_STATUS_OK; break;
	case fr::NO_FILE:
	case fr::NO_PATH:             status = IOC_STATUS_FS2_NOT_FOUND; break;
	case fr::EXIST:               status = IOC_STATUS_FS2_EXISTS; break;
	case fr::INVALID_NAME:        status = IOC_STATUS_FS2_BAD_NAME; break;
	case fr::DENIED:
	case fr::WRITE_PROTECTED:     status = IOC_STATUS_FS2_READ_ONLY; break;
	case fr::NOT_ENOUGH_CORE:
	case fr::TOO_MANY_OPEN_FILES: status = IOC_STATUS_FS2_NO_HANDLE; break;
	case fr::INVALID_OBJECT:      status = IOC_STATUS_FS2_STALE; break;
	case fr::INVALID_PARAMETER:   status = IOC_STATUS_FS2_RANGE; break;
	case fr::NOT_READY:
	case fr::NOT_ENABLED:
	case fr::NO_FILESYSTEM:       status = IOC_STATUS_FS2_NO_MEDIA; break;
	default:                      status = IOC_STATUS_FS2_IO; break;
	}
	if (result == fr::DISK_ERR || result == fr::INT_ERR || result == fr::NOT_READY ||
			result == fr::NOT_ENABLED || result == fr::NO_FILESYSTEM)
		sdfs_invalidate();
	return status;
}

u8 zephyr80_ioc_core::fs2_need_fs()
{
	bool mounted = false;
	if (m_storage->fs_mount(mounted) != card::OK || !mounted)
		return IOC_STATUS_FS2_NO_MEDIA;
	return IOC_STATUS_OK;
}

// make_path: the resolver plus one component, bounded by FS2_PATH_MAX;
// `append` makes it the resolver.
bool zephyr80_ioc_core::fs2_make_path(char *out, u8 const *name, bool append)
{
	char component[13];
	if (!unpack_name(component, name))
		return false;
	size_t base = std::strlen(m_resolver);
	size_t const add = std::strlen(component);
	if (base + add + ((base == 1) ? 0 : 1) > RESOLVER_MAX)
		return false;
	std::memcpy(out, m_resolver, base);
	if (base != 1)
		out[base++] = '/';
	std::memcpy(out + base, component, add + 1);
	if (append)
	{
		std::memcpy(m_resolver, out, base + add + 1);
		++m_resolver_components;
	}
	return true;
}

namespace {
u16 token_for(unsigned slot, u8 cookie) { return u16(slot + 1) | (u16(cookie) << 8); }
}

zephyr80_ioc_core::fs2_file *zephyr80_ioc_core::fs2_file_for(u16 token, u8 &status)
{
	u8 const raw = u8(token);
	if (raw == 0 || raw > IOC_FS2_FILE_SLOTS)
	{
		status = IOC_STATUS_FS2_NO_HANDLE;
		return nullptr;
	}
	fs2_file &file = m_files[raw - 1];
	if (file.cookie != u8(token >> 8))
		status = IOC_STATUS_FS2_NO_HANDLE;
	else if (file.generation != m_generation)
		status = IOC_STATUS_FS2_STALE;
	else if (!file.open)
		status = IOC_STATUS_FS2_NO_HANDLE;
	else
	{
		status = IOC_STATUS_OK;
		return &file;
	}
	return nullptr;
}

void zephyr80_ioc_core::fs2_caps(frame const &request, frame &reply) const
{
	reply_init(request, reply, RSP_FS2_CAPS, IOC_STATUS_OK, IOC_FS2_CAP_REPLY_LEN);
	reply[OFF_PAYLOAD + 0] = IOC_FS2_VERSION;
	reply[OFF_PAYLOAD + 1] = IOC_FS2_STATUS_VERSION;
	put16(&reply[OFF_PAYLOAD + 2], IOC_FS2_CAPS_FLAGS);
	put32(&reply[OFF_PAYLOAD + 4], m_generation);
	put16(&reply[OFF_PAYLOAD + 8], IOC_FS2_CHUNK_MAX);
	reply[OFF_PAYLOAD + 10] = IOC_FS2_FILE_SLOTS;
	reply[OFF_PAYLOAD + 11] = IOC_FS2_DIR_SLOTS;
	reply[OFF_PAYLOAD + 12] = IOC_FS2_RESOLVER_COMPONENTS;
	reply[OFF_PAYLOAD + 13] = IOC_FS2_TOKEN_BYTES;
}

void zephyr80_ioc_core::fs2_generation(frame const &request, frame &reply) const
{
	reply_init(request, reply, RSP_FS2_GENERATION, IOC_STATUS_OK, IOC_FS2_GENERATION_REPLY_LEN);
	put32(&reply[OFF_PAYLOAD], m_generation);
}

void zephyr80_ioc_core::fs2_reset(frame const &request, frame &reply)
{
	fs2_media_invalidated();
	reply_init(request, reply, RSP_FS2_RESET, IOC_STATUS_OK, IOC_FS2_GENERATION_REPLY_LEN);
	put32(&reply[OFF_PAYLOAD], m_generation);
}

void zephyr80_ioc_core::fs2_root(frame const &request, frame &reply)
{
	fs2_root_path();
	reply_init(request, reply, RSP_FS2_ROOT, IOC_STATUS_OK, 0);
}

void zephyr80_ioc_core::fs2_push(frame const &request, frame &reply)
{
	char path[RESOLVER_MAX + 1];
	u8 status = fs2_need_fs();
	if (status == IOC_STATUS_OK)
	{
		storage::entry info;
		if (m_resolver_components >= IOC_FS2_RESOLVER_COMPONENTS)
			status = IOC_STATUS_FS2_RANGE;
		else if (!fs2_make_path(path, &request[IOC_OFF_FS2_NAME], false))
			status = IOC_STATUS_FS2_BAD_NAME;
		else
		{
			status = fs2_map(m_storage->stat(path, info));
			if (status == IOC_STATUS_OK && !(info.attr & storage::ATTR_DIR))
				status = IOC_STATUS_FS2_NOT_DIR;
			if (status == IOC_STATUS_OK)
				fs2_make_path(path, &request[IOC_OFF_FS2_NAME], true);
		}
	}
	reply_init(request, reply, RSP_FS2_PUSH, status, 0);
}

void zephyr80_ioc_core::fs2_open_ro(frame const &request, frame &reply)
{
	char path[RESOLVER_MAX + 1];
	u8 status = fs2_need_fs();
	unsigned slot = IOC_FS2_FILE_SLOTS;
	if (status == IOC_STATUS_OK && !fs2_make_path(path, &request[IOC_OFF_FS2_NAME], false))
		status = IOC_STATUS_FS2_BAD_NAME;
	if (status == IOC_STATUS_OK)
	{
		storage::entry info;
		status = fs2_map(m_storage->stat(path, info));
		if (status == IOC_STATUS_OK && (info.attr & storage::ATTR_DIR))
			status = IOC_STATUS_FS2_IS_DIR;
	}
	if (status == IOC_STATUS_OK)
	{
		for (slot = 0; slot < IOC_FS2_FILE_SLOTS && m_files[slot].open; ++slot) { }
		if (slot == IOC_FS2_FILE_SLOTS)
			status = IOC_STATUS_FS2_NO_HANDLE;
	}
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->open(path, storage::OPEN_READ, m_files[slot].object));
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_OPEN_RO, status, 0);
	fs2_file &file = m_files[slot];
	if (++file.cookie == 0)
		file.cookie = 1;
	file.open = true;
	file.writable = false;
	file.generation = m_generation;
	file.position = 0;
	reply_init(request, reply, RSP_FS2_OPEN_RO, IOC_STATUS_OK, IOC_FS2_OPEN_REPLY_LEN);
	put16(&reply[OFF_PAYLOAD], token_for(slot, file.cookie));
	put32(&reply[OFF_PAYLOAD + 2], m_storage->size(file.object));
	reply[OFF_PAYLOAD + 6] = 0;
}

// fs2_read_common: READ is one chunk, READ_RUN a block-aligned run read
// whole before READY and sent one IOC_FS2_CHUNK_MAX packet at a time.
void zephyr80_ioc_core::fs2_read(frame const &request, frame &reply, u8 rsp, u16 max, u16 align)
{
	u16 const token = get16(&request[IOC_OFF_FS2_TOKEN]);
	u32 const offset = get32(&request[IOC_OFF_FS2_OFFSET]);
	u16 const length = get16(&request[IOC_OFF_FS2_LENGTH]);
	u8 status;
	fs2_file *file = fs2_file_for(token, status);
	u16 got = 0;

	m_run_length = 0;
	if (status == IOC_STATUS_OK && (length == 0 || length > max || (u16(offset) & align) != 0))
		status = IOC_STATUS_FS2_RANGE;
	if (status == IOC_STATUS_OK && file->position != offset)
	{
		status = fs2_map(m_storage->seek(file->object, offset));
		if (status == IOC_STATUS_OK)
			file->position = offset;
	}
	if (status == IOC_STATUS_OK)
	{
		status = fs2_map(m_storage->read(file->object, &m_chunk[0], length, got));
		// A failure that dropped every context has already closed this one.
		if (status == IOC_STATUS_OK)
			file->position = offset + got;
		else
			file->position = 0xffffffff;
	}
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, rsp, status, 0);
	reply_init(request, reply, rsp, IOC_STATUS_OK, IOC_READY_PAYLOAD_LEN);
	if (!got)
		return;
	m_run_id = next_xfer_id();
	m_run_seq = request[OFF_SEQ];
	m_run_type = rsp;
	m_run_length = got;
	m_run_sent = 0;
	ready(reply, m_run_id, BULK_DIR_MCU_TO_Z80, got, offset);
	run_next(); // the first (a read's only) packet
}

void zephyr80_ioc_core::fs2_close(frame const &request, frame &reply)
{
	u8 status;
	fs2_file *file = fs2_file_for(get16(&request[IOC_OFF_FS2_TOKEN]), status);
	if (status == IOC_STATUS_OK)
	{
		fr const result = m_storage->close(file->object);
		file->open = false;
		file->writable = false;
		file->position = 0;
		status = fs2_map(result);
	}
	reply_init(request, reply, RSP_FS2_CLOSE, status, 0);
}

void zephyr80_ioc_core::fs2_opendir(frame const &request, frame &reply)
{
	u8 status = fs2_need_fs();
	if (status == IOC_STATUS_OK && m_dir_open)
		status = IOC_STATUS_FS2_NO_HANDLE;
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->opendir(m_resolver, m_dir_object));
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_OPENDIR, status, 0);
	if (++m_dir_cookie == 0)
		m_dir_cookie = 1;
	m_dir_open = true;
	m_dir_generation = m_generation;
	reply_init(request, reply, RSP_FS2_OPENDIR, IOC_STATUS_OK, IOC_FS2_DIR_REPLY_LEN);
	put16(&reply[OFF_PAYLOAD], token_for(0, m_dir_cookie));
}

u8 zephyr80_ioc_core::fs2_dir_status(u16 token) const
{
	if (u8(token) != 1 || u8(token >> 8) != m_dir_cookie)
		return IOC_STATUS_FS2_NO_HANDLE;
	if (m_dir_generation != m_generation)
		return IOC_STATUS_FS2_STALE;
	if (!m_dir_open)
		return IOC_STATUS_FS2_NO_HANDLE;
	return IOC_STATUS_OK;
}

// Names that do not pack as 8.3 are skipped; only AM_DIR is reported.
void zephyr80_ioc_core::fs2_readdir(frame const &request, frame &reply)
{
	u8 status = fs2_dir_status(get16(&request[IOC_OFF_FS2_TOKEN]));
	storage::entry info{};
	u8 packed[11];
	while (status == IOC_STATUS_OK)
	{
		status = fs2_map(m_storage->readdir(m_dir_object, info));
		if (status != IOC_STATUS_OK)
			break;
		if (!info.name[0])
		{
			status = IOC_STATUS_FS2_END;
			break;
		}
		if (fs2_pack_name(packed, info.name))
			break;
	}
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_READDIR, status, 0);
	reply_init(request, reply, RSP_FS2_READDIR, IOC_STATUS_OK, IOC_FS2_DIRENT_REPLY_LEN);
	std::copy_n(packed, 11, &reply[OFF_PAYLOAD]);
	reply[OFF_PAYLOAD + 11] = info.attr & storage::ATTR_DIR;
	put32(&reply[OFF_PAYLOAD + 12], info.size);
}

void zephyr80_ioc_core::fs2_closedir(frame const &request, frame &reply)
{
	u8 status = fs2_dir_status(get16(&request[IOC_OFF_FS2_TOKEN]));
	if (status == IOC_STATUS_OK)
	{
		fr const result = m_storage->closedir(m_dir_object);
		m_dir_open = false;
		status = fs2_map(result);
	}
	reply_init(request, reply, RSP_FS2_CLOSEDIR, status, 0);
}

void zephyr80_ioc_core::fs2_stat(frame const &request, frame &reply)
{
	char path[RESOLVER_MAX + 1];
	storage::entry info;
	u8 status = fs2_need_fs();
	if (status == IOC_STATUS_OK && !fs2_make_path(path, &request[IOC_OFF_FS2_NAME], false))
		status = IOC_STATUS_FS2_BAD_NAME;
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->stat(path, info));
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_STAT, status, 0);
	reply_init(request, reply, RSP_FS2_STAT, IOC_STATUS_OK, IOC_FS2_STAT_REPLY_LEN);
	put32(&reply[OFF_PAYLOAD], info.size);
	reply[OFF_PAYLOAD + 4] = info.attr & storage::ATTR_DIR;
}

namespace {

u32 byte_count(u32 clusters, u32 cluster_bytes)
{
	if (cluster_bytes && clusters > 0xffffffffU / cluster_bytes)
		return 0xffffffff;
	return clusters * cluster_bytes;
}

u32 kibibyte_count(u32 clusters, u32 cluster_bytes)
{
	u64 const kib = (u64(clusters) * cluster_bytes) / 1024;
	return kib > 0xffffffffU ? 0xffffffff : u32(kib);
}

} // anonymous namespace

void zephyr80_ioc_core::fs2_space(frame const &request, frame &reply)
{
	storage::geometry geo{};
	u8 status = fs2_need_fs();
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->fs_geometry(geo));
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_SPACE, status, 0);
	u32 const cluster_bytes = u32(geo.cluster_blocks) * 512;
	reply_init(request, reply, RSP_FS2_SPACE, IOC_STATUS_OK, IOC_FS2_SPACE_REPLY_LEN);
	put32(&reply[OFF_PAYLOAD + 0], byte_count(geo.free_clusters, cluster_bytes));
	put32(&reply[OFF_PAYLOAD + 4], byte_count(geo.clusters, cluster_bytes));
	put32(&reply[OFF_PAYLOAD + 8], kibibyte_count(geo.free_clusters, cluster_bytes));
	put32(&reply[OFF_PAYLOAD + 12], kibibyte_count(geo.clusters, cluster_bytes));
}

void zephyr80_ioc_core::fs2_open_rw(frame const &request, frame &reply)
{
	char path[RESOLVER_MAX + 1];
	u8 status = fs2_need_fs();
	u8 const mode = request[IOC_OFF_FS2_OPEN_MODE];
	unsigned slot = IOC_FS2_FILE_SLOTS;
	if (mode > IOC_FS2_OPEN_CREATE_ALWAYS)
		status = IOC_STATUS_FS2_RANGE;
	if (status == IOC_STATUS_OK && !fs2_make_path(path, &request[IOC_OFF_FS2_OPEN_NAME], false))
		status = IOC_STATUS_FS2_BAD_NAME;
	if (status == IOC_STATUS_OK)
	{
		storage::entry info;
		fr const result = m_storage->stat(path, info);
		if (result == fr::OK)
		{
			if (info.attr & storage::ATTR_DIR)
				status = IOC_STATUS_FS2_IS_DIR;
			else if (mode == IOC_FS2_OPEN_CREATE_NEW)
				status = IOC_STATUS_FS2_EXISTS;
		}
		else if (result == fr::NO_FILE)
		{
			if (mode == IOC_FS2_OPEN_UPDATE)
				status = IOC_STATUS_FS2_NOT_FOUND;
		}
		else
			status = fs2_map(result);
	}
	if (status == IOC_STATUS_OK)
	{
		for (slot = 0; slot < IOC_FS2_FILE_SLOTS && m_files[slot].open; ++slot) { }
		if (slot == IOC_FS2_FILE_SLOTS)
			status = IOC_STATUS_FS2_NO_HANDLE;
	}
	if (status == IOC_STATUS_OK)
	{
		u8 flags = storage::OPEN_READ | storage::OPEN_WRITE;
		if (mode == IOC_FS2_OPEN_CREATE_NEW)
			flags |= storage::OPEN_CREATE_NEW;
		else if (mode == IOC_FS2_OPEN_CREATE_ALWAYS)
			flags |= storage::OPEN_CREATE_ALWAYS;
		status = fs2_map(m_storage->open(path, flags, m_files[slot].object));
	}
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_OPEN_RW, status, 0);
	fs2_file &file = m_files[slot];
	if (++file.cookie == 0)
		file.cookie = 1;
	file.open = true;
	file.writable = true;
	file.generation = m_generation;
	file.position = 0;
	reply_init(request, reply, RSP_FS2_OPEN_RW, IOC_STATUS_OK, IOC_FS2_OPEN_REPLY_LEN);
	put16(&reply[OFF_PAYLOAD], token_for(slot, file.cookie));
	put32(&reply[OFF_PAYLOAD + 2], m_storage->size(file.object));
	reply[OFF_PAYLOAD + 6] = 0;
}

// commit_fs2_write: the handle must still be the open, writable one of this
// generation; a short write is no space.
u8 zephyr80_ioc_core::fs2_commit()
{
	if (m_write_slot >= IOC_FS2_FILE_SLOTS)
		return IOC_STATUS_FS2_STALE;
	fs2_file &file = m_files[m_write_slot];
	if (!file.open || !file.writable || file.generation != m_generation)
		return IOC_STATUS_FS2_STALE;
	if (file.position != m_write_offset)
	{
		u8 const status = fs2_map(m_storage->seek(file.object, m_write_offset));
		if (status != IOC_STATUS_OK)
			return status;
		file.position = m_write_offset;
	}
	u16 put = 0;
	u8 const status = fs2_map(m_storage->write(file.object, &m_chunk[0], m_write_length, put));
	if (status != IOC_STATUS_OK)
		return status;
	file.position = m_write_offset + put;
	return (put != m_write_length) ? IOC_STATUS_FS2_NO_SPACE : IOC_STATUS_OK;
}

void zephyr80_ioc_core::fs2_write(frame const &request, frame &reply)
{
	u32 const offset = get32(&request[IOC_OFF_FS2_OFFSET]);
	u16 const length = get16(&request[IOC_OFF_FS2_LENGTH]);
	u8 status;
	fs2_file *file = fs2_file_for(get16(&request[IOC_OFF_FS2_TOKEN]), status);
	if (status == IOC_STATUS_OK && !file->writable)
		status = IOC_STATUS_FS2_READ_ONLY;
	if (status == IOC_STATUS_OK && (length == 0 || length > IOC_FS2_CHUNK_MAX))
		status = IOC_STATUS_FS2_RANGE;
	if (status != IOC_STATUS_OK)
		return reply_init(request, reply, RSP_FS2_WRITE, status, 0);
	m_write_slot = u8(file - &m_files[0]);
	m_write_offset = offset;
	m_write_length = length;
	reply_init(request, reply, RSP_FS2_WRITE, IOC_STATUS_OK, IOC_READY_PAYLOAD_LEN);
	u8 const id = next_xfer_id();
	ready(reply, id, BULK_DIR_Z80_TO_MCU, length, offset);
	arm_receive(BUF_CHUNK, length, id, CMD_FS2_WRITE, request[OFF_SEQ], COMMIT_FS2);
}

// UNLINK, MKDIR, RMDIR.  FF_FS_LOCK is 0, so removing closes every open
// FS2 file first.
void zephyr80_ioc_core::fs2_namespace(frame const &request, frame &reply, u8 cls, u8 op)
{
	char path[RESOLVER_MAX + 1];
	u8 status = fs2_need_fs();
	if (status == IOC_STATUS_OK && !fs2_make_path(path, &request[IOC_OFF_FS2_NAME], false))
		status = IOC_STATUS_FS2_BAD_NAME;
	if (status == IOC_STATUS_OK)
	{
		if (op == CMD_FS2_MKDIR)
			status = fs2_map(m_storage->mkdir(path));
		else
		{
			fs2_close_files();
			status = fs2_map(m_storage->unlink(path));
		}
	}
	reply_init(request, reply, cls, status, 0);
}

// Both names in the current resolver directory; an existing destination is
// EXISTS, not the denied status FatFs would give.
void zephyr80_ioc_core::fs2_rename(frame const &request, frame &reply)
{
	char from[RESOLVER_MAX + 1], to[RESOLVER_MAX + 1];
	u8 status = fs2_need_fs();
	if (status == IOC_STATUS_OK &&
			(!fs2_make_path(from, &request[IOC_OFF_FS2_NAME], false) || !fs2_make_path(to, &request[IOC_OFF_FS2_RENAME_TO], false)))
		status = IOC_STATUS_FS2_BAD_NAME;
	storage::entry info;
	if (status == IOC_STATUS_OK && m_storage->stat(to, info) == fr::OK)
		status = IOC_STATUS_FS2_EXISTS;
	if (status == IOC_STATUS_OK)
	{
		fs2_close_files();
		status = fs2_map(m_storage->rename(from, to));
	}
	reply_init(request, reply, RSP_FS2_RENAME, status, 0);
}

void zephyr80_ioc_core::fs2_sync(frame const &request, frame &reply)
{
	u8 status;
	fs2_file *file = fs2_file_for(get16(&request[IOC_OFF_FS2_TOKEN]), status);
	if (status == IOC_STATUS_OK && !file->writable)
		status = IOC_STATUS_FS2_READ_ONLY;
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->sync(file->object));
	reply_init(request, reply, RSP_FS2_SYNC, status, 0);
}

// f_lseek to the size (which grows a file open for writing), then
// f_truncate there.
void zephyr80_ioc_core::fs2_truncate(frame const &request, frame &reply)
{
	u32 const size = get32(&request[IOC_OFF_FS2_TRUNCATE_SIZE]);
	u8 status;
	fs2_file *file = fs2_file_for(get16(&request[IOC_OFF_FS2_TOKEN]), status);
	if (status == IOC_STATUS_OK && !file->writable)
		status = IOC_STATUS_FS2_READ_ONLY;
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->seek(file->object, size));
	if (status == IOC_STATUS_OK)
		status = fs2_map(m_storage->truncate(file->object));
	if (status == IOC_STATUS_OK)
		file->position = size;
	reply_init(request, reply, RSP_FS2_TRUNCATE, status, 0);
}


//**************************************************************************
//  HID (handlers.c, ioc_hid.c)
//**************************************************************************

// Page 0 is the passive status; pages 6 and 7 are the passive gamepad and
// second-interface pages; every other page answers page 0 (normal profile).
// No USB host controller is modelled, so the bring-up fields say no probe
// ran: HID_HOST_NOT_STARTED, revision 0, /USB_INT idle.  The live USB fields
// (hid_host_usb_state) describe the HID module's keyboard: one device at
// its address, the reports received since the mount, the last report and
// full speed; with no keyboard mounted, none, 0 and speed FFh.  Pages 6 and
// 7 are the module's live state (hid_host_gamepad_state): the mounted
// devices, each pad slot with its latch byte, and the second interface with
// its raw report and the mouse's running totals.
void zephyr80_ioc_core::hid_status(frame const &request, frame &reply) const
{
	u8 const page = request[IOC_OFF_HID_REQ_PAGE];
	if (page == IOC_HID_PAGE_GAMEPAD || page == IOC_HID_PAGE_OTHER)
	{
		zephyr80_ioc_hid::gamepad_state state;
		m_hid.state(state);
		if (page == IOC_HID_PAGE_OTHER)
		{
			reply_init(request, reply, RSP_HID_STATUS, IOC_STATUS_OK, IOC_HID_OTHER_PAYLOAD_LEN);
			u8 *p = &reply[OFF_PAYLOAD];
			p[0] = IOC_HID_PAGE_OTHER;
			p[1] = state.other_addr;
			p[2] = state.other_instance;
			put16(p + 3, state.other_reports);
			p[5] = state.other_last_len;
			std::copy(state.other_last.begin(), state.other_last.end(), p + 6);
			p[14] = state.mouse_buttons;
			put16(p + 15, state.mouse_x);
			put16(p + 17, state.mouse_y);
			put16(p + 19, state.mouse_wheel);
			return;
		}
		reply_init(request, reply, RSP_HID_STATUS, IOC_STATUS_OK, IOC_HID_GAMEPAD_PAYLOAD_LEN);
		u8 *p = &reply[OFF_PAYLOAD];
		p[0] = IOC_HID_PAGE_GAMEPAD;
		p[1] = state.device_count;
		p[2] = state.mount_count;
		p[3] = state.unmount_count;
		p[4] = state.last_device_addr;
		put16(p + 5, state.last_device_vid);
		put16(p + 7, state.last_device_pid);
		p[9] = state.hid_mount_count;
		p[10] = state.last_hid_addr;
		for (unsigned pad = 0; pad < zephyr80_ioc_hid::GAMEPADS; pad++)
		{
			u8 *q = p + 11 + pad * 7;
			q[0] = state.gamepad_addr[pad];
			q[1] = state.gamepad_instance[pad];
			q[2] = state.gamepad_first_arm[pad];
			put16(q + 3, state.gamepad_reports[pad]);
			q[5] = state.gamepad_last_len[pad];
			q[6] = state.gamepad_latch[pad];
		}
		p[25] = state.last_hid_protocol;
		return;
	}
	reply_init(request, reply, RSP_HID_STATUS, IOC_STATUS_OK, IOC_HID_STATUS_PAYLOAD_LEN);
	reply[IOC_OFF_HID_USB_INT] = 1;
	reply[IOC_OFF_HID_DEV_COUNT] = m_hid.device_count();
	reply[IOC_OFF_HID_KBD_ADDR] = m_hid.keyboard_address();
	put16(&reply[IOC_OFF_HID_REPORTS], m_hid.report_count());
	std::copy_n(m_hid.last_report(), zephyr80_ioc_hid::REPORT, &reply[IOC_OFF_HID_LAST_REPORT]);
	reply[IOC_OFF_HID_KBD_SPEED] = m_hid.speed();
}

// handler_hid_input: up to `wanted` (at most 24) queued bytes, then the
// count still queued and the saturating drop count.
void zephyr80_ioc_core::hid_input(frame const &request, frame &reply)
{
	unsigned wanted = std::min<unsigned>(request[OFF_PAYLOAD], MAX_DATA - IOC_HID_INPUT_META_LEN);
	unsigned const count = std::min<unsigned>(m_hid.queued(), wanted);
	reply_init(request, reply, RSP_HID_INPUT, IOC_STATUS_OK, u8(IOC_HID_INPUT_META_LEN + count));
	for (unsigned i = 0; i < count; i++)
		reply[OFF_PAYLOAD + 2 + i] = m_hid.get();
	reply[OFF_PAYLOAD + 0] = m_hid.queued();
	reply[OFF_PAYLOAD + 1] = m_hid.dropped();
}

// handler_hid_mouse / hid_mouse_take: flags, the buttons held, the buttons
// pressed and the motion since the last read, then those cleared.  Flags are
// zero with no mouse mounted, and the reads are all zero.
void zephyr80_ioc_core::hid_mouse(frame const &request, frame &reply)
{
	zephyr80_ioc_hid::mouse_read const mouse = m_hid.mouse_take();
	reply_init(request, reply, RSP_HID_MOUSE, IOC_STATUS_OK, IOC_HID_MOUSE_PAYLOAD_LEN);
	reply[OFF_PAYLOAD + 0] = mouse.flags;
	reply[OFF_PAYLOAD + 1] = mouse.buttons;
	reply[OFF_PAYLOAD + 2] = mouse.pressed;
	put16(&reply[OFF_PAYLOAD + 3], u16(mouse.dx));
	put16(&reply[OFF_PAYLOAD + 5], u16(mouse.dy));
	put16(&reply[OFF_PAYLOAD + 7], u16(mouse.wheel));
}
