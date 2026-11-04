/* license:BSD-3-Clause
 * copyright-holders:Kitamura
 *
 * FatFs (3rdparty/fatfs, the IO controller firmware's copy and ffconf.h)
 * built for the Zephyr-80 card socket.  Its media layer is the socket's
 * (zephyr80_card.cpp), under names of its own, and the window write-back is
 * reachable so that raw card access stays coherent with the FatFs window.
 * The mount counter is reachable for save states: a restored volume and its
 * objects keep their mount ID, and the next mount must not reuse it.
 * FatFs itself is unmodified and keeps its own license (fatfs/LICENSE.txt).
 */
#define disk_initialize zephyr80_card_disk_initialize
#define disk_status zephyr80_card_disk_status
#define disk_read zephyr80_card_disk_read
#define disk_write zephyr80_card_disk_write
#define disk_ioctl zephyr80_card_disk_ioctl

#include "ff.c"

FRESULT zephyr80_fatfs_sync_window(FATFS *fs);
WORD zephyr80_fatfs_mount_id(void);
void zephyr80_fatfs_set_mount_id(WORD id);

/* sync_window: a dirty window is written back (and to the second FAT). */
FRESULT zephyr80_fatfs_sync_window(FATFS *fs)
{
	return sync_window(fs);
}

/* Fsid: the last volume mount ID handed out. */
WORD zephyr80_fatfs_mount_id(void)
{
	return Fsid;
}

void zephyr80_fatfs_set_mount_id(WORD id)
{
	Fsid = id;
}
