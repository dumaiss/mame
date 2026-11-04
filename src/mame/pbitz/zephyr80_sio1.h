// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_SIO1_H
#define MAME_PBITZ_ZEPHYR80_SIO1_H

#pragma once

#include "zephyr80_ioc_lane.h"

// SIO1, the IO-controller link: a Z80 SIO whose two channels run External
// Sync x1 lanes clocked by the controller.  This is a behavioural model of
// what the KonaOS IOC driver (drivers/ioc/ioc_boot.c, ioc_io.asm) programs and
// observes, at byte granularity:
//
//   - the WR0 pointer and commands (reset External/Status, channel reset,
//     error reset, Tx CRC reset, Tx underrun/EOM latch reset), WR1-WR7;
//   - RR0: Rx character available, Tx buffer empty, /DCD, Sync/Hunt, /CTS,
//     Tx underrun/EOM; /DCD and /CTS latch on a change until Reset
//     External/Status;  RR1: All Sent (always set) and Rx overrun;  RR2;
//   - the receiver: disabled, hunting or assembling; Hunt ends only on a
//     falling /SYNC edge (External Sync); an assembled character waits one
//     character time in the receive pipeline before it enters the
//     three-deep FIFO, and a fourth character overwrites the last;
//   - the transmitter: one buffer; on an empty buffer it underruns, sending
//     the Tx CRC (low byte first) when the EOM latch was reset with Tx CRC
//     enabled and WR7 fill otherwise, and sets the latch; disabled, it marks
//     (FFh) and drops its buffer;
//   - /RTS: asserted by WR5 D1; with the transmitter enabled, clearing D1
//     does not release it (an External Sync transmitter never empties), only
//     disabling the transmitter does.
//
// Not modelled: interrupts, the daisy chain and WAIT/READY (both guests keep
// WR1 at 00h; a write that enables any is logged), modes other than External
// Sync x1, Auto Enables, receive CRC, parity, break/abort, DTR, and bit-level
// timing (the /SYNC position inside a character).
class zephyr80_sio1_device : public device_t, public zephyr80_ioc_lane_port
{
public:
	zephyr80_sio1_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void set_client(zephyr80_ioc_lane_client &client) { m_client = &client; }

	// CPU side, addressed like z80sio's ba_cd: A1 channel (0 = A), A0 control.
	u8 ba_cd_r(offs_t offset);
	void ba_cd_w(offs_t offset, u8 data);

	// zephyr80_ioc_lane_port
	virtual u8 lane_clock(zephyr80_ioc_lane lane, u8 to_z80) override;
	virtual void lane_sync(zephyr80_ioc_lane lane, bool asserted) override;
	virtual void lane_dcd(zephyr80_ioc_lane lane, bool asserted) override;
	virtual void lane_cts(zephyr80_ioc_lane lane, bool asserted) override;
	virtual bool lane_rts(zephyr80_ioc_lane lane) const override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	u8 rr0(unsigned ch) const;
	u8 control_read(unsigned ch);
	void control_write(unsigned ch, u8 data);
	void write_register(unsigned ch, unsigned reg, u8 data);
	u8 data_read(unsigned ch);
	void channel_reset(unsigned ch);
	void receiver_restart(unsigned ch);
	void update_rts(unsigned ch);
	void lines_changed(unsigned ch);
	void crc_update(unsigned ch, u8 data);

	zephyr80_ioc_lane_client *m_client;

	u8 m_wr[2][8];
	u8 m_pointer[2];
	// receiver
	bool m_hunt[2];       // RR0 D4: no character boundary
	bool m_pipe_full[2];  // an assembled character awaits the next clock
	u8 m_pipe[2];
	u8 m_fifo[2][3];
	u8 m_fifo_depth[2];
	u8 m_data[2];         // the data port's last value
	bool m_overrun[2];    // RR1 D5
	// transmitter
	bool m_tx_full[2];
	u8 m_tx_buffer[2];
	bool m_eom[2];        // RR0 D6: Tx underrun/EOM latch
	u16 m_crc[2];
	u8 m_crc_left[2];     // CRC bytes still to send
	// lines (asserted levels)
	bool m_dcd[2];
	bool m_cts[2];
	bool m_sync[2];
	bool m_rts[2];
	bool m_latched[2];    // /DCD and /CTS latched since Reset External/Status
	u8 m_ext[2];          // their latched RR0 bits
	u8 m_warned;          // unmodelled features already logged
};

DECLARE_DEVICE_TYPE(ZEPHYR80_SIO1, zephyr80_sio1_device)

#endif // MAME_PBITZ_ZEPHYR80_SIO1_H
