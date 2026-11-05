// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_HID_H
#define MAME_PBITZ_ZEPHYR80_IOC_HID_H

#pragma once

#include <array>

#include "zephyr80_controller_latch.h"

// The IO controller's HID input: a behavioural model of the firmware's HID
// module (Code/MCU/IOController src/ioc_hid.c) above the HID report
// decode.  It takes boot-keyboard reports (modifier byte, reserved byte, six
// usages in press order) and keeps what the firmware keeps:
//
//   - the terminal input queue (input_reserve, input_put_unchecked,
//     hid_input_get): 128 bytes, a key's translation admitted whole or not
//     at all, a refusal counting one saturating drop; /CTSB, the doorbell,
//     asserted while the queue holds anything;
//   - translate_key: the TinyUSB US boot table (HID_KEYCODE_TO_ASCII) with
//     Shift and CapsLock, Ctrl letters and punctuation to control bytes, the
//     keypad's navigation layer while NumLock is off, VT100 arrows, Home/End
//     and CSI ~ editing keys, SS3/CSI function keys; CapsLock and NumLock
//     toggle their state and the LED report, nothing more;
//   - tuh_hid_report_received_cb's keyboard branch: Ctrl+Alt+Esc anywhere in
//     the report latches the reset request and nothing else of the report is
//     taken; only usages absent from the previous report are translated
//     (01h-03h, the rollover/error usages, never); the last such non-lock key
//     takes over the repeat with REPEAT_DELAY_MS, a released repeat key ends
//     it, a held one re-samples the modifiers;
//   - hid_host_task's repeat: due on the controller's millisecond clock,
//     re-armed REPEAT_PERIOD_MS after the clock it fired at, and translated
//     only while the queue holds less than half of its size;
//   - hid_host_usb_state for HID_STATUS page 0;
//   - the mouse, the dongle's second HID interface: 7-byte reports (ID 02,
//     buttons, X and Y as two 12-bit signed fields, wheel, horizontal
//     scroll) decoded into running totals, the buttons held and, since the
//     last take, the motion capped at +/-32767 and the buttons pressed
//     (hid_mouse_take, which clears them); an interface whose protocol is the
//     boot mouse is ignored at mount, as the firmware does;
//   - the F310 gamepads: a mount takes the first free of two slots, each slot
//     polled every GAMEPAD_POLL_MS after its last report, each report
//     decoded into the controller latch (zephyr80_controller_latch); the
//     keyboard's number keys are mirrored onto port 0's keypad lines on every
//     keyboard report (coleco_keypad_for_report);
//   - hid_host_gamepad_state for HID_STATUS pages 6 and 7.
//
// The clock is the caller's: the firmware's tusb_time_millis_api, which
// advances in TIMEBASE_TICK_MS steps.  Mounting is the moment TinyUSB
// enumeration completes (tuh_hid_mount_cb: CapsLock off, NumLock on, the LED
// report pushed); the PIC, the MAX3421E, USB transfers and enumeration
// itself are not modelled.  Plain C++: it builds outside MAME.
class zephyr80_ioc_hid
{
public:
	static constexpr unsigned QUEUE = 128;            // HID_INPUT_QUEUE_SIZE
	static constexpr unsigned REPORT = 8;             // a boot keyboard report
	static constexpr u32 REPEAT_DELAY_MS = 500;       // REPEAT_DELAY_MS
	static constexpr u32 REPEAT_PERIOD_MS = 60;       // REPEAT_PERIOD_MS
	static constexpr u32 TICK_MS = 10;                // TIMEBASE_TICK_MS
	static constexpr u8 KEYBOARD_ADDRESS = 1;         // the address enumeration gives the one keyboard
	static constexpr u8 LED_NUM_LOCK = 0x01;          // HID_LED_NUM_LOCK
	static constexpr u8 LED_CAPS_LOCK = 0x02;         // HID_LED_CAPS_LOCK
	static constexpr u32 GAMEPAD_POLL_MS = 10;        // GAMEPAD_POLL_MS
	static constexpr unsigned GAMEPADS = zephyr80_controller_latch::PORTS;
	static constexpr u8 MOUSE_REPORT_ID = 0x02;       // MOUSE_REPORT_ID
	static constexpr unsigned MOUSE_REPORT_LEN = 7;   // MOUSE_REPORT_LEN
	static constexpr u8 MOUSE_INSTANCE = 1;           // the dongle's second interface
	static constexpr u16 F310_VID = 0x046d, F310_PID = 0xc216;
	static constexpr u8 HID_MOUSE_MOUNTED = 0x01, HID_MOUSE_DECODED = 0x02;

	// HidMouseRead: what CMD_HID_MOUSE answers.
	struct mouse_read
	{
		u8 flags;
		u8 buttons;
		u8 pressed;
		s16 dx;
		s16 dy;
		s16 wheel;
	};

	// HidHostGamepadState (HID_STATUS pages 6 and 7).
	struct gamepad_state
	{
		u8 device_count;      // live mounted addresses 1..2
		u8 mount_count;
		u8 unmount_count;
		u8 last_device_addr;  // highest live non-hub address, or FFh
		u16 last_device_vid;
		u16 last_device_pid;
		u8 hid_mount_count;
		u8 last_hid_addr;
		u8 last_hid_protocol;
		u8 gamepad_addr[GAMEPADS];
		u8 gamepad_instance[GAMEPADS];
		u8 gamepad_first_arm[GAMEPADS];
		u16 gamepad_reports[GAMEPADS];
		u8 gamepad_last_len[GAMEPADS];
		u8 gamepad_latch[GAMEPADS];
		u8 other_addr;
		u8 other_instance;
		u16 other_reports;
		u8 other_last_len;
		std::array<u8, REPORT> other_last;
		u16 mouse_x;
		u16 mouse_y;
		u16 mouse_wheel;
		u8 mouse_buttons;
	};

	// /CTSB on SIO1/B (input_doorbell_update).
	class doorbell
	{
	public:
		virtual void hid_doorbell(bool asserted) = 0;

	protected:
		~doorbell() = default;
	};

	explicit zephyr80_ioc_hid(doorbell &line);

	// hid_host_init: empty queue, CapsLock off, NumLock on, no keyboard, no
	// reset request.
	void reset();
	// tuh_hid_mount_cb / tuh_hid_umount_cb for the keyboard interface.
	void mount();
	void unmount();
	bool mounted() const { return m_keyboard_addr != 0; }

	// tuh_hid_report_received_cb for the keyboard at `now_ms`.
	void report(u8 const *report, unsigned length, u32 now_ms);
	// hid_host_task's repeat at `now_ms`.
	void task(u32 now_ms);
	// When task() next has work: the repeat's due time.
	bool repeat_due(u32 &due_ms) const;
	// hid_reset_requested: Ctrl+Alt+Esc was seen.
	bool reset_requested() const { return m_reset_combo; }

	// The queue: put() is input_reserve plus input_put_unchecked for one
	// translated key; get() is hid_input_get (0 when empty).
	bool put(u8 const *bytes, unsigned length);
	u8 get();
	u8 queued() const { return m_count; }
	u8 dropped() const { return m_dropped; }

	// The mouse (the dongle's second HID interface).  mount_mouse is
	// tuh_hid_mount_cb for an interface of `protocol` (the boot mouse, 2, is
	// ignored); it needs the keyboard first, as a protocol-0 interface arriving
	// before it would be the keyboard.  unmount_mouse is tuh_hid_umount_cb: the buttons held, the
	// decode flag and the pending motion clear, the running totals stay.
	// mouse_report is tuh_hid_report_received_cb for that interface; take is
	// hid_mouse_take.
	bool mount_mouse(u8 protocol = 0);
	void unmount_mouse();
	bool mouse_mounted() const { return m_other_addr != 0; }
	void mouse_report(u8 const *report, unsigned length);
	mouse_read mouse_take();

	// The F310 pads.  mount_pad is tuh_mount_cb plus tuh_hid_mount_cb for an
	// F310 DirectInput interface: the slot (0 or 1) it took, or GAMEPAD_NONE
	// when both are taken.  unmount_pad is the device leaving (tuh_umount_cb);
	// the slot's latch returns to idle.  gamepad_due says whether the slot's
	// next poll has come; gamepad_report is the report that poll received.
	static constexpr u8 GAMEPAD_NONE = 0xff;
	u8 mount_pad(u8 &address);
	void unmount_pad(u8 slot);
	bool pad_mounted(unsigned slot) const { return slot < GAMEPADS && m_pad_addr[slot] != 0; }
	bool gamepad_due(unsigned slot, u32 now_ms) const;
	void gamepad_report(unsigned slot, u8 const *report, unsigned length, u32 now_ms);
	// When a gamepad poll is next due, if any pad is mounted.
	bool gamepad_next_due(u32 &due_ms) const;
	u8 pad_latch(unsigned slot) const { return m_latch.value(slot); }
	// The controller latch: what E0h-FFh reads present.
	zephyr80_controller_latch const &latch() const { return m_latch; }
	void state(gamepad_state &out) const;

	// hid_host_usb_state and the LED report.
	u8 device_count() const { return m_device_count; }
	u8 keyboard_address() const { return m_keyboard_addr; }
	u16 report_count() const { return m_reports; }
	u8 const *last_report() const { return m_last.data(); }
	u8 speed() const { return m_speed; }
	u8 leds() const { return m_leds; }
	bool caps_lock() const { return m_caps_lock; }
	bool num_lock() const { return m_num_lock; }

	// Calls save(item, name) for every persistent member.
	template <typename Save> void save_state(Save &&save);

private:
	void translate(u8 keycode, u8 modifier);
	void put_byte(u8 value);
	void put_escape(u8 introducer, u8 final);
	void put_csi_number(u8 tens, u8 ones);
	bool was_down(u8 keycode) const;
	void led_refresh();
	void usb_mounted(u8 address, u16 vid, u16 pid);
	void usb_unmounted();
	void hid_mounted(u8 address, u8 protocol);
	void pad_release(u8 slot);
	static u8 keypad_for_report(u8 const *report, unsigned length);

	doorbell &m_doorbell;

	// input_queue
	std::array<u8, QUEUE> m_queue;
	u8 m_head;
	u8 m_tail;
	u8 m_count;
	u8 m_dropped;

	// the keyboard
	u8 m_keyboard_addr;      // 0 = no keyboard mounted
	u8 m_speed;              // tusb_speed_t, FFh unknown
	u16 m_reports;
	std::array<u8, REPORT> m_last; // keyboard_last
	bool m_caps_lock;
	bool m_num_lock;
	u8 m_leds;               // led_report
	u8 m_repeat_key;         // 0 = nothing repeating
	u8 m_repeat_mod;
	u32 m_repeat_due;
	bool m_reset_combo;      // reset_combo_seen

	// tuh_mount_cb / tuh_hid_mount_cb bookkeeping (HID_STATUS page 6)
	u8 m_device_count;       // usb_device_count
	u8 m_mount_count;
	u8 m_unmount_count;
	u8 m_last_addr;          // usb_last_device_addr
	u16 m_last_vid;
	u16 m_last_pid;
	u8 m_hid_mounts;         // hid_mount_calls
	u8 m_last_hid_addr;      // hid_mount_daddr
	u8 m_last_hid_proto;     // hid_mount_proto

	// the gamepad slots
	std::array<u8, GAMEPADS> m_pad_addr;       // gamepad_addr, 0 = free
	std::array<u8, GAMEPADS> m_pad_first_arm;
	std::array<u16, GAMEPADS> m_pad_reports;
	std::array<u8, GAMEPADS> m_pad_last_len;
	std::array<u32, GAMEPADS> m_pad_next_poll; // gamepad_next_poll_ms
	zephyr80_controller_latch m_latch;

	// the second HID interface and the decoded mouse
	u8 m_other_addr;         // 0 = none
	u16 m_other_reports;
	u8 m_other_last_len;
	std::array<u8, REPORT> m_other_last;
	u16 m_mouse_x, m_mouse_y, m_mouse_wheel;   // running totals, wrapping
	u8 m_mouse_buttons;
	bool m_mouse_decoded;
	s16 m_pend_x, m_pend_y, m_pend_wheel;      // since the last take, capped
	u8 m_mouse_pressed;
};

template <typename Save>
void zephyr80_ioc_hid::save_state(Save &&save)
{
	save(m_queue, "hid.m_queue");
	save(m_head, "hid.m_head");
	save(m_tail, "hid.m_tail");
	save(m_count, "hid.m_count");
	save(m_dropped, "hid.m_dropped");
	save(m_keyboard_addr, "hid.m_keyboard_addr");
	save(m_speed, "hid.m_speed");
	save(m_reports, "hid.m_reports");
	save(m_last, "hid.m_last");
	save(m_caps_lock, "hid.m_caps_lock");
	save(m_num_lock, "hid.m_num_lock");
	save(m_leds, "hid.m_leds");
	save(m_repeat_key, "hid.m_repeat_key");
	save(m_repeat_mod, "hid.m_repeat_mod");
	save(m_repeat_due, "hid.m_repeat_due");
	save(m_reset_combo, "hid.m_reset_combo");
	save(m_device_count, "hid.m_device_count");
	save(m_mount_count, "hid.m_mount_count");
	save(m_unmount_count, "hid.m_unmount_count");
	save(m_last_addr, "hid.m_last_addr");
	save(m_last_vid, "hid.m_last_vid");
	save(m_last_pid, "hid.m_last_pid");
	save(m_hid_mounts, "hid.m_hid_mounts");
	save(m_last_hid_addr, "hid.m_last_hid_addr");
	save(m_last_hid_proto, "hid.m_last_hid_proto");
	save(m_pad_addr, "hid.m_pad_addr");
	save(m_pad_first_arm, "hid.m_pad_first_arm");
	save(m_pad_reports, "hid.m_pad_reports");
	save(m_pad_last_len, "hid.m_pad_last_len");
	save(m_pad_next_poll, "hid.m_pad_next_poll");
	m_latch.save_state(save);
	save(m_other_addr, "hid.m_other_addr");
	save(m_other_reports, "hid.m_other_reports");
	save(m_other_last_len, "hid.m_other_last_len");
	save(m_other_last, "hid.m_other_last");
	save(m_mouse_x, "hid.m_mouse_x");
	save(m_mouse_y, "hid.m_mouse_y");
	save(m_mouse_wheel, "hid.m_mouse_wheel");
	save(m_mouse_buttons, "hid.m_mouse_buttons");
	save(m_mouse_decoded, "hid.m_mouse_decoded");
	save(m_pend_x, "hid.m_pend_x");
	save(m_pend_y, "hid.m_pend_y");
	save(m_pend_wheel, "hid.m_pend_wheel");
	save(m_mouse_pressed, "hid.m_mouse_pressed");
}

#endif // MAME_PBITZ_ZEPHYR80_IOC_HID_H
