// SPDX-License-Identifier: GPL-2.0-only
/*
 * sinfo_io - Atomic Super I/O register access for sinfo
 *
 * Provides a character device (/dev/sinfo_io) with ioctl commands for
 * atomic banked register reads on Nuvoton NCT67xx / ITE IT87xx Super I/O
 * hardware monitoring chips.
 *
 * This solves race conditions with kernel hwmon drivers (nct6775, it87)
 * that share the same I/O port bank-select register. By claiming exclusive
 * port ownership via request_region(), this module prevents the kernel
 * driver from loading, making sinfo the sole accessor.
 *
 * Read-only: no write support to HWM registers.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "sinfo_io.h"

#define DEVICE_NAME   "sinfo_io"
#define HWM_BASE_MIN  0x0200
#define HWM_BASE_MAX  0x0F00
#define REG_BANK      0x4E

struct sinfo_io_ctx {
	u16 addr_port;
	u16 data_port;
	struct resource *region;
	bool setup_done;
	spinlock_t lock;
};

static int sinfo_io_open(struct inode *inode, struct file *file)
{
	struct sinfo_io_ctx *ctx;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	spin_lock_init(&ctx->lock);
	file->private_data = ctx;
	return 0;
}

static int sinfo_io_release(struct inode *inode, struct file *file)
{
	struct sinfo_io_ctx *ctx = file->private_data;

	if (ctx->region)
		release_region(ctx->addr_port, 2);

	kfree(ctx);
	return 0;
}

static int handle_setup(struct sinfo_io_ctx *ctx, void __user *argp)
{
	struct sinfo_io_setup setup;

	if (ctx->setup_done)
		return -EALREADY;

	if (copy_from_user(&setup, argp, sizeof(setup)))
		return -EFAULT;

	if (setup.hwm_base < HWM_BASE_MIN || setup.hwm_base > HWM_BASE_MAX)
		return -EINVAL;
	if (setup.hwm_base & 0x0F)
		return -EINVAL;

	ctx->addr_port = setup.hwm_base + 5;
	ctx->data_port = setup.hwm_base + 6;

	ctx->region = request_region(ctx->addr_port, 2, DEVICE_NAME);
	if (!ctx->region)
		return -EBUSY;

	ctx->setup_done = true;
	pr_info("sinfo_io: claimed ports 0x%04x-0x%04x (HWM base 0x%04x)\n",
		ctx->addr_port, ctx->data_port, setup.hwm_base);
	return 0;
}

static int handle_read_reg(struct sinfo_io_ctx *ctx, void __user *argp)
{
	struct sinfo_io_reg r;
	unsigned long flags;
	u8 bank, offset;

	if (!ctx->setup_done)
		return -ENODEV;

	if (copy_from_user(&r, argp, sizeof(r)))
		return -EFAULT;

	bank = r.reg >> 8;
	offset = r.reg & 0xFF;

	spin_lock_irqsave(&ctx->lock, flags);
	outb(REG_BANK, ctx->addr_port);
	outb(bank, ctx->data_port);
	outb(offset, ctx->addr_port);
	r.value = inb(ctx->data_port);
	spin_unlock_irqrestore(&ctx->lock, flags);

	r.status = 0;

	if (copy_to_user(argp, &r, sizeof(r)))
		return -EFAULT;

	return 0;
}

static int handle_read_batch(struct sinfo_io_ctx *ctx, void __user *argp)
{
	struct sinfo_io_batch batch;
	unsigned long flags;
	u8 cur_bank = 0xFF;
	int i;

	if (!ctx->setup_done)
		return -ENODEV;

	if (copy_from_user(&batch, argp, sizeof(batch)))
		return -EFAULT;

	if (batch.count == 0 || batch.count > SINFO_IO_BATCH_MAX)
		return -EINVAL;

	spin_lock_irqsave(&ctx->lock, flags);
	for (i = 0; i < batch.count; i++) {
		u8 bank = batch.regs[i] >> 8;
		u8 offset = batch.regs[i] & 0xFF;

		if (bank != cur_bank) {
			outb(REG_BANK, ctx->addr_port);
			outb(bank, ctx->data_port);
			cur_bank = bank;
		}
		outb(offset, ctx->addr_port);
		batch.values[i] = inb(ctx->data_port);
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (copy_to_user(argp, &batch, sizeof(batch)))
		return -EFAULT;

	return 0;
}

static long sinfo_io_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct sinfo_io_ctx *ctx = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case SINFO_IO_SETUP:
		return handle_setup(ctx, argp);
	case SINFO_IO_READ_REG:
		return handle_read_reg(ctx, argp);
	case SINFO_IO_READ_BATCH:
		return handle_read_batch(ctx, argp);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations sinfo_io_fops = {
	.owner          = THIS_MODULE,
	.open           = sinfo_io_open,
	.release        = sinfo_io_release,
	.unlocked_ioctl = sinfo_io_ioctl,
};

static struct miscdevice sinfo_io_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = DEVICE_NAME,
	.fops  = &sinfo_io_fops,
	.mode  = 0600,
};

static int __init sinfo_io_init(void)
{
	pr_info("sinfo_io: loading\n");
	return misc_register(&sinfo_io_misc);
}

static void __exit sinfo_io_exit(void)
{
	misc_deregister(&sinfo_io_misc);
	pr_info("sinfo_io: unloaded\n");
}

module_init(sinfo_io_init);
module_exit(sinfo_io_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("sinfo project");
MODULE_DESCRIPTION("Atomic Super I/O register access for sinfo");
MODULE_VERSION("0.1.0");
