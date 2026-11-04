// license:BSD-3-Clause
// copyright-holders:Kitamura
#ifndef MAME_PBITZ_ZEPHYR80_IOC_SPEED_H
#define MAME_PBITZ_ZEPHYR80_IOC_SPEED_H

#pragma once

// `-ioc_speed paced|fast`: how the IO controller's lanes keep time.  Paced
// (the default) clocks every byte on the controller's own byte period, so
// elapsed time follows the timing table in zephyr80_ioc.cpp.  Fast clocks a
// byte as soon as the other end can take it and falls back to the paced
// period only when it cannot; protocol outcomes are the same.
class zephyr80_ioc_speed_interface : public device_interface
{
public:
	virtual bool fast() const = 0;

protected:
	zephyr80_ioc_speed_interface(const machine_config &mconfig, device_t &device)
		: device_interface(device, "ioc_speed")
	{
	}
};

class zephyr80_ioc_speed_slot_device : public device_t, public device_single_card_slot_interface<zephyr80_ioc_speed_interface>
{
public:
	template <typename T>
	zephyr80_ioc_speed_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: zephyr80_ioc_speed_slot_device(mconfig, tag, owner, 0)
	{
		set_options(std::forward<T>(opts), dflt, false);
	}
	zephyr80_ioc_speed_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	bool fast() const { return m_mode && m_mode->fast(); }

protected:
	virtual void device_config_complete() override;
	virtual void device_start() override ATTR_COLD;

private:
	zephyr80_ioc_speed_interface *m_mode;
};

class zephyr80_ioc_paced_device : public device_t, public zephyr80_ioc_speed_interface
{
public:
	zephyr80_ioc_paced_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);
	virtual bool fast() const override { return false; }

protected:
	virtual void device_start() override ATTR_COLD;
};

class zephyr80_ioc_fast_device : public device_t, public zephyr80_ioc_speed_interface
{
public:
	zephyr80_ioc_fast_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);
	virtual bool fast() const override { return true; }

protected:
	virtual void device_start() override ATTR_COLD;
};

DECLARE_DEVICE_TYPE(ZEPHYR80_IOC_SPEED, zephyr80_ioc_speed_slot_device)
DECLARE_DEVICE_TYPE(ZEPHYR80_IOC_PACED, zephyr80_ioc_paced_device)
DECLARE_DEVICE_TYPE(ZEPHYR80_IOC_FAST, zephyr80_ioc_fast_device)

#endif // MAME_PBITZ_ZEPHYR80_IOC_SPEED_H
