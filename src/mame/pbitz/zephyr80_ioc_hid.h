// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_HID_H
#define MAME_PBITZ_ZEPHYR80_IOC_HID_H

#pragma once

#include <array>

// The IO controller's keyboard input: a behavioural model of the firmware's
// HID module (Code/MCU/IOController src/ioc_hid.c) above the HID usages.  It
// takes boot-keyboard reports (modifier byte, reserved byte, six usages in
// press order) and keeps what the firmware keeps:
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
//   - hid_host_usb_state for HID_STATUS page 0.
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

	// hid_host_usb_state and the LED report.
	u8 device_count() const { return mounted() ? 1 : 0; }
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
}

#endif // MAME_PBITZ_ZEPHYR80_IOC_HID_H
