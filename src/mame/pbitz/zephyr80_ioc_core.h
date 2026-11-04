// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_CORE_H
#define MAME_PBITZ_ZEPHYR80_IOC_CORE_H

#pragma once

#include "zephyr80_ioc_storage.h"

#include <array>

// The IO controller's command and bulk protocol, independent of any
// transport: the request frame found in a captured command window, its
// dispatch, the reply packet, and the bulk transfer a reply arms.  The
// specification is the controller firmware (Code/MCU/IOController) as
// deployed: the normal profile with the filesystem commands and the bulk-lane
// CRC bypass (include/config.h, Makefile).  include/ioc_frame.h gives the
// packet, the classes, the layouts and the statuses; src/external_sync.c the
// request validation (packet_at, is_command_class); src/main.c
// service_command_request the service order; src/dispatch.c the class table;
// src/handlers.c, fs2.c, fs_share.c, volume.c and ioc_hid.c the replies and
// state; src/bulk_channel.c the bulk packet and its lifecycle.
//
//   A5 5A LEN_LO LEN_HI TYPE SEQ STATUS DATA... CRC_HI CRC_LO
//
// LEN counts TYPE + SEQ + STATUS + DATA (3..29 on the command lane); the CRC
// is CRC-16-CCITT (1021h, preset 0, MSB-first, no final XOR) over LEN_LO
// through the last DATA byte.  Frames travel as the firmware's 32-byte
// compatibility mailbox: TYPE, SEQ, STATUS, DATA length, DATA.  A window
// holding no valid request gets no reply, as on the board.
//
// Bulk: a handler that answers READY arms one transfer (direction, id, TYPE,
// SEQ, a buffer and its length).  The transport runs it after the reply:
// bulk_packet()/bulk_sent() towards the Z80, bulk_window()/bulk_received()
// from it, bulk_no_host() when the host never admits it, and run_more()/
// run_next() for the further packets of a FS2_READ_RUN.  A write's backend
// effect happens inside bulk_received(), so XFER_STATUS (DONE) reports it
// only after it completed.
//
// Media come from a zephyr80_ioc_storage backend, the card socket; an empty
// socket answers as the firmware does with no card in it.
class zephyr80_ioc_core
{
public:
	using frame = std::array<u8, 32>;
	using storage = zephyr80_ioc_storage;

	static constexpr unsigned FRAME_SIZE = 32;
	static constexpr unsigned MAX_DATA = 26;           // IOC_COMMAND_MAX_DATA
	static constexpr unsigned MAX_WIRE = 2 + 2 + 3 + MAX_DATA + 2;
	static constexpr unsigned BULK_MAX = 512;          // BULK_MAX_LENGTH, one packet
	static constexpr unsigned RUN_MAX = 2048;          // IOC_FS2_RUN_MAX
	static constexpr unsigned BULK_WIRE_MAX = 2 + 2 + 3 + BULK_MAX + 2;
	static constexpr unsigned BULK_SEARCH = 16;        // BULK_RX_SEARCH_BITS / 8
	static constexpr unsigned BULK_WINDOW_MAX = BULK_MAX + (2 + 2 + 3 + 2) + BULK_SEARCH + 2; // BULK_RX_WINDOW_BYTES
	static constexpr unsigned HID_QUEUE = 128;         // HID_INPUT_QUEUE_SIZE
	static constexpr unsigned RESOLVER_MAX = 239;          // FS2_PATH_MAX
	static constexpr unsigned VOL_UNITS = 2;

	// What the protocol asks of the transport it runs on.
	class link
	{
	public:
		// Both lanes drop their character boundary: the next transfer on
		// each re-establishes it (external_sync_request_resync and
		// bulk_channel_request_resync).
		virtual void resync_lanes() = 0;
		// The command lane's boundary is established (PING reports it).
		virtual bool command_lane_synced() const = 0;
		// /CTSB, the HID input doorbell: asserted while the queue holds bytes.
		virtual void hid_doorbell(bool asserted) = 0;
		// CMD_RESET (handler_reset): reset the machine; no reply follows.
		virtual void host_reset() = 0;

	protected:
		~link() = default;
	};

	enum class bulk_dir : u8 { NONE, TO_Z80, FROM_Z80 };

	explicit zephyr80_ioc_core(link &transport);

	// The backend serving the card socket; required before reset().
	void set_storage(storage &media) { m_storage = &media; }
	bool has_storage() const { return m_storage != nullptr; }

	void reset();
	// Calls save(item, name) for every persistent member.
	template <typename Save> void save_state(Save &&save);

	// One command service: begin() counts it, then either abort() (the
	// request was not released or did not decode) or decode() + dispatch()
	// + encode().  dispatch() returns false when no reply follows.
	void begin() { m_calls++; }
	void abort() { m_aborts++; }
	bool decode(u8 const *window, unsigned length, frame &request) const;
	bool dispatch(frame const &request, frame &reply);
	static unsigned encode(frame const &reply, u8 *wire);
	// The controller's idle pass between services (main.c): the one volume
	// auto-mount once the card is up.
	void idle();

	// The armed bulk transfer, run by the transport after the reply.
	bulk_dir bulk_armed() const;
	unsigned bulk_packet(u8 *wire) const;          // TO_Z80: the packet, at most BULK_WIRE_MAX bytes
	void bulk_sent();                              // TO_Z80: every byte clocked
	unsigned bulk_window() const;                  // FROM_Z80: bytes to clock, at most BULK_WINDOW_MAX
	void bulk_received(u8 const *window, unsigned length);
	void bulk_no_host();                           // the host never asserted its bulk RTS
	bool run_more() const { return m_run_length && m_run_sent < m_run_length; }
	bool run_next();                               // arm a run's next packet
	void run_end() { m_run_length = 0; }

	// The HID input queue (ioc_hid.c input_reserve/input_put): a key's bytes
	// enter whole or not at all; a refusal counts one drop.
	bool hid_input_put(u8 const *bytes, unsigned length);

	static u16 crc16(u16 crc, u8 data);

private:
	using card = storage::card;
	using fr = storage::fr;

	struct fs2_file
	{
		u32 generation;
		u32 position;
		u8 cookie;
		bool open;
		bool writable;
		storage::object object;
	};

	enum : u8 { COMMIT_NONE, COMMIT_RECORD, COMMIT_SHARE, COMMIT_FS2 };
	enum : u8 { BUF_CHUNK, BUF_BLOCK, BUF_RECORD };
	enum : u8 { VOL_NONE, VOL_RAW, VOL_FILE };

	bool packet_at(u8 const *packet, unsigned available, frame &request) const;
	void unknown(frame const &request, frame &reply) const;
	void ping(frame const &request, frame &reply) const;
	void xfer_status(frame const &request, frame &reply) const;
	void hid_status(frame const &request, frame &reply) const;
	void hid_input(frame const &request, frame &reply);
	void hid_mouse(frame const &request, frame &reply);

	// Bulk (bulk_channel.c)
	u8 next_xfer_id();
	void arm_send(u8 buffer, u16 offset, u16 length, u8 id, u8 type, u8 sequence);
	void arm_receive(u8 buffer, u16 length, u8 id, u8 type, u8 sequence, u8 commit);
	u8 *buffer(u8 which, u16 offset);
	u8 const *buffer(u8 which, u16 offset) const;
	u8 commit();
	static void ready(frame &reply, u8 id, u8 direction, u16 length, u32 echo);

	// SD records and volumes (handlers.c, volume.c)
	void sd_read(frame const &request, frame &reply);
	void sd_read_rec(frame const &request, frame &reply);
	void sd_read_block(frame const &request, frame &reply);
	void sd_write_rec(frame const &request, frame &reply);
	void sd_flush(frame const &request, frame &reply);
	void vol_mount_cmd(frame const &request, frame &reply);
	void vol_info(frame const &request, frame &reply);
	void vol_init();
	card vol_ensure_mounted();
	card vol_mount(unsigned unit, u8 const *name11);
	bool vol_map(unsigned unit, u32 block) const;
	u8 vol_map_failure(unsigned unit) const;
	u8 vol_read_block(unsigned unit, u32 block, u8 *dst);
	u8 vol_write_record(unsigned unit, u32 record, u8 const *src);
	void vol_report(unsigned unit, frame &reply) const;
	void mount_raw(unsigned unit);

	// Shared folder (fs_share.c)
	void fs_opendir(frame const &request, frame &reply);
	void fs_readdir(frame const &request, frame &reply);
	void fs_open(frame const &request, frame &reply);
	void fs_read(frame const &request, frame &reply);
	void fs_write(frame const &request, frame &reply);
	void fs_close(frame const &request, frame &reply);
	void fs_stat(frame const &request, frame &reply);
	void fs_delete(frame const &request, frame &reply);
	void fs_selftest(frame const &request, frame &reply);
	u8 share_need_fs();
	void share_close_current();
	fr share_seek(u32 offset);
	u8 share_xfer_params(frame const &request, u32 &offset, u16 &length) const;
	u8 share_commit();

	// FS2 (fs2.c)
	void fs2_caps(frame const &request, frame &reply) const;
	void fs2_generation(frame const &request, frame &reply) const;
	void fs2_reset(frame const &request, frame &reply);
	void fs2_root(frame const &request, frame &reply);
	void fs2_push(frame const &request, frame &reply);
	void fs2_open_ro(frame const &request, frame &reply);
	void fs2_read(frame const &request, frame &reply, u8 rsp, u16 max, u16 align);
	void fs2_close(frame const &request, frame &reply);
	void fs2_opendir(frame const &request, frame &reply);
	void fs2_readdir(frame const &request, frame &reply);
	void fs2_closedir(frame const &request, frame &reply);
	void fs2_stat(frame const &request, frame &reply);
	void fs2_space(frame const &request, frame &reply);
	void fs2_open_rw(frame const &request, frame &reply);
	void fs2_write(frame const &request, frame &reply);
	void fs2_sync(frame const &request, frame &reply);
	void fs2_truncate(frame const &request, frame &reply);
	void fs2_namespace(frame const &request, frame &reply, u8 cls, u8 op);
	void fs2_rename(frame const &request, frame &reply);
	void fs2_media_invalidated();
	void fs2_close_files();
	void fs2_root_path();
	u8 fs2_map(fr result);
	u8 fs2_need_fs();
	bool fs2_make_path(char *out, u8 const *name, bool append);
	fs2_file *fs2_file_for(u16 token, u8 &status);
	u8 fs2_dir_status(u16 token) const;
	u8 fs2_commit();
	void sdfs_invalidate();

	link &m_link;
	storage *m_storage;

	// Service counters (main.c svc_calls, svc_aborts)
	u16 m_calls;
	u16 m_aborts;

	// Bulk (bulk_channel.c)
	u8 m_xfer_counter;
	u8 m_last_xfer_id;
	u8 m_last_status;
	u8 m_bulk_dir;
	u16 m_bulk_length;       // armed DATA bytes; 0 = nothing armed
	u8 m_bulk_buffer;
	u16 m_bulk_offset;
	u8 m_bulk_id;
	u8 m_bulk_type;
	u8 m_bulk_seq;
	u8 m_bulk_status;
	u8 m_bulk_commit;
	u16 m_run_sent;          // fs2.c run_sent/run_length/...
	u16 m_run_length;
	u8 m_run_id;
	u8 m_run_seq;
	u8 m_run_type;
	std::array<u8, RUN_MAX> m_chunk;               // fs_bulk_chunk
	std::array<u8, storage::BLOCK_BYTES> m_block;  // xfer_block
	std::array<u8, 128> m_record;                  // xfer_record

	// Pending writes, staged by a READY for the commit
	u32 m_write_record;
	u8 m_write_unit;
	u32 m_write_offset;
	u16 m_write_length;
	u8 m_write_slot;

	// Volumes (volume.c); extents live in the backend's image object
	u8 m_vol_mode[VOL_UNITS];
	u8 m_vol_extents[VOL_UNITS];
	u32 m_vol_base[VOL_UNITS];
	u32 m_vol_records[VOL_UNITS];
	u8 m_vol_name[VOL_UNITS][11];
	storage::object m_vol_image[VOL_UNITS];
	bool m_vol_open[VOL_UNITS];
	bool m_mount_attempted;
	bool m_automount_done;

	// Shared folder (fs_share.c)
	bool m_share_dir_open;
	storage::object m_share_dir;
	u8 m_share_handle;       // 0 = nothing open
	u8 m_share_next;
	bool m_share_writing;
	u32 m_share_pos;
	storage::object m_share_file;

	// FS2 (fs2.c)
	std::array<fs2_file, 2> m_files;
	u32 m_dir_generation;
	u8 m_dir_cookie;
	bool m_dir_open;
	storage::object m_dir_object;
	char m_resolver[RESOLVER_MAX + 1];
	u8 m_resolver_components;
	u32 m_generation;

	// HID (ioc_hid.c): the input queue and the mouse read state
	std::array<u8, HID_QUEUE> m_hid_queue;
	u8 m_hid_head;
	u8 m_hid_tail;
	u8 m_hid_count;
	u8 m_hid_dropped;
	u8 m_mouse_flags;
	u8 m_mouse_buttons;
	u8 m_mouse_pressed;
	s16 m_mouse_dx;
	s16 m_mouse_dy;
	s16 m_mouse_wheel;
};

template <typename Save>
void zephyr80_ioc_core::save_state(Save &&save)
{
	save(m_calls, "core.m_calls");
	save(m_aborts, "core.m_aborts");
	save(m_xfer_counter, "core.m_xfer_counter");
	save(m_last_xfer_id, "core.m_last_xfer_id");
	save(m_last_status, "core.m_last_status");
	save(m_bulk_dir, "core.m_bulk_dir");
	save(m_bulk_length, "core.m_bulk_length");
	save(m_bulk_buffer, "core.m_bulk_buffer");
	save(m_bulk_offset, "core.m_bulk_offset");
	save(m_bulk_id, "core.m_bulk_id");
	save(m_bulk_type, "core.m_bulk_type");
	save(m_bulk_seq, "core.m_bulk_seq");
	save(m_bulk_status, "core.m_bulk_status");
	save(m_bulk_commit, "core.m_bulk_commit");
	save(m_run_sent, "core.m_run_sent");
	save(m_run_length, "core.m_run_length");
	save(m_run_id, "core.m_run_id");
	save(m_run_seq, "core.m_run_seq");
	save(m_run_type, "core.m_run_type");
	save(m_chunk, "core.m_chunk");
	save(m_block, "core.m_block");
	save(m_record, "core.m_record");
	save(m_write_record, "core.m_write_record");
	save(m_write_unit, "core.m_write_unit");
	save(m_write_offset, "core.m_write_offset");
	save(m_write_length, "core.m_write_length");
	save(m_write_slot, "core.m_write_slot");
	save(m_vol_mode, "core.m_vol_mode");
	save(m_vol_extents, "core.m_vol_extents");
	save(m_vol_base, "core.m_vol_base");
	save(m_vol_records, "core.m_vol_records");
	save(m_vol_name, "core.m_vol_name");
	save(m_vol_image, "core.m_vol_image");
	save(m_vol_open, "core.m_vol_open");
	save(m_mount_attempted, "core.m_mount_attempted");
	save(m_automount_done, "core.m_automount_done");
	save(m_share_dir_open, "core.m_share_dir_open");
	save(m_share_dir, "core.m_share_dir");
	save(m_share_handle, "core.m_share_handle");
	save(m_share_next, "core.m_share_next");
	save(m_share_writing, "core.m_share_writing");
	save(m_share_pos, "core.m_share_pos");
	save(m_share_file, "core.m_share_file");
	save(m_files, &fs2_file::generation, "core.m_files.generation");
	save(m_files, &fs2_file::position, "core.m_files.position");
	save(m_files, &fs2_file::cookie, "core.m_files.cookie");
	save(m_files, &fs2_file::open, "core.m_files.open");
	save(m_files, &fs2_file::writable, "core.m_files.writable");
	save(m_files, &fs2_file::object, "core.m_files.object");
	save(m_dir_generation, "core.m_dir_generation");
	save(m_dir_cookie, "core.m_dir_cookie");
	save(m_dir_open, "core.m_dir_open");
	save(m_dir_object, "core.m_dir_object");
	save(m_resolver, "core.m_resolver");
	save(m_resolver_components, "core.m_resolver_components");
	save(m_generation, "core.m_generation");
	save(m_hid_queue, "core.m_hid_queue");
	save(m_hid_head, "core.m_hid_head");
	save(m_hid_tail, "core.m_hid_tail");
	save(m_hid_count, "core.m_hid_count");
	save(m_hid_dropped, "core.m_hid_dropped");
	save(m_mouse_flags, "core.m_mouse_flags");
	save(m_mouse_buttons, "core.m_mouse_buttons");
	save(m_mouse_pressed, "core.m_mouse_pressed");
	save(m_mouse_dx, "core.m_mouse_dx");
	save(m_mouse_dy, "core.m_mouse_dy");
	save(m_mouse_wheel, "core.m_mouse_wheel");
}

#endif // MAME_PBITZ_ZEPHYR80_IOC_CORE_H
