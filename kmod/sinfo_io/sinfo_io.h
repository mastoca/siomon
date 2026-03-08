/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SINFO_IO_H
#define _SINFO_IO_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
typedef uint8_t  __u8;
typedef uint16_t __u16;
#endif

#define SINFO_IO_MAGIC 'S'
#define SINFO_IO_BATCH_MAX 32

struct sinfo_io_setup {
	__u16 hwm_base;
	__u16 reserved;
};

struct sinfo_io_reg {
	__u16 reg;      /* Input: banked addr (high=bank, low=offset) */
	__u8  value;    /* Output: register value */
	__u8  status;   /* Output: 0=success */
};

struct sinfo_io_batch {
	__u8  count;           /* Input: number of registers (1-32) */
	__u8  reserved[3];
	__u16 regs[SINFO_IO_BATCH_MAX];   /* Input: banked register addresses */
	__u8  values[SINFO_IO_BATCH_MAX]; /* Output: register values */
};

#define SINFO_IO_SETUP      _IOW(SINFO_IO_MAGIC,  0x01, struct sinfo_io_setup)
#define SINFO_IO_READ_REG   _IOWR(SINFO_IO_MAGIC, 0x02, struct sinfo_io_reg)
#define SINFO_IO_READ_BATCH _IOWR(SINFO_IO_MAGIC, 0x03, struct sinfo_io_batch)

#endif /* _SINFO_IO_H */
