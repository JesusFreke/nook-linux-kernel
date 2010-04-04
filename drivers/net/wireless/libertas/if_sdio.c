/*
 *  linux/drivers/net/wireless/libertas/if_sdio.c
 *
 *  Copyright 2007-2008 Pierre Ossman
 *
 * Inspired by if_cs.c, Copyright 2007 Holger Schurig
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This hardware has more or less no CMD53 support, so all registers
 * must be accessed using sdio_readb()/sdio_writeb().
 *
 * Transfers must be in one transaction or the firmware goes bonkers.
 * This means that the transfer must either be small enough to do a
 * byte based transfer or it must be padded to a multiple of the
 * current block size.
 *
 * As SDIO is still new to the kernel, it is unfortunately common with
 * bugs in the host controllers related to that. One such bug is that
 * controllers cannot do transfers that aren't a multiple of 4 bytes.
 * If you don't have time to fix the host controller driver, you can
 * work around the problem by modifying if_sdio_host_to_card() and
 * if_sdio_card_to_host() to pad the data.
 */

#include <linux/moduleparam.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#ifdef CONFIG_MACH_BRAVO
#include <mach/bravo_gpio.h>
#endif

#include "host.h"
#include "decl.h"
#include "defs.h"
#include "dev.h"
#include "if_sdio.h"

static char *lbs_helper_name = NULL;
module_param_named(helper_name, lbs_helper_name, charp, 0644);

static char *lbs_fw_name = NULL;
module_param_named(fw_name, lbs_fw_name, charp, 0644);

static const struct sdio_device_id if_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_MARVELL, SDIO_DEVICE_ID_MARVELL_LIBERTAS) },
	{ /* end: all zeroes */						},
};


MODULE_DEVICE_TABLE(sdio, if_sdio_ids);

struct if_sdio_model {
	int model;
	const char *helper;
	const char *firmware;
};

static struct if_sdio_model if_sdio_models[] = {
	{
		/* 8385 */
		.model = 0x04,
		.helper = "sd8385_helper.bin",
		.firmware = "sd8385.bin",
	},
	{
		/* 8686 */
		.model = 0x0B,
		.helper = "/firmware/sd8686_helper.bin",
		.firmware = "/firmware/sd8686.bin",
	},
};

struct if_sdio_packet {
	struct if_sdio_packet	*next;
	u16			nb;
	u8			buffer[0] __attribute__((aligned(4)));
};

struct if_sdio_card {
	struct sdio_func	*func;
	struct lbs_private	*priv;

	int			model;
	unsigned long		ioport;

	const char		*helper;
	const char		*firmware;

	u8			buffer[65536];

	spinlock_t		lock;
	struct if_sdio_packet	*packets;
#ifndef CONFIG_MACH_BRAVO_WIFI_OPTIMIZE
	struct work_struct	packet_worker;
#endif
};

struct if_sdio_card *g_sdio_card = NULL;

/********************************************************************/
/* I/O                                                              */
/********************************************************************/
#ifdef CONFIG_MACH_BRAVO
int bravo_load_firmware(struct builtin_fw *fw, unsigned long *p_order)
{
  struct file *fp = NULL;
  unsigned long file_size = 0;
  unsigned char *buf = NULL;
  struct files_struct *files = current->files;
  int ret = 0;
 
  fp = filp_open(fw->name, O_RDONLY | O_LARGEFILE, 0);
  if (!fp)
  {
  	printk(KERN_NOTICE "bravo_load_firmware: OPEN file failed\n");
	return -EAGAIN;
  }
  
  fw->size = fp->f_mapping->host->i_size;
  printk(KERN_NOTICE "bravo_load_firmware: file_size(%d)\n", fw->size);

  fw->data = vmalloc(fw->size);
  if (!(fw->data))
  {
  	printk(KERN_NOTICE "system is running out of memory, allocate 128KB failed, try again later\n");
	filp_close(fp, files);
	return -ENOMEM;
  }
  
  ret = kernel_read(fp, 0, fw->data, fw->size);
  printk(KERN_NOTICE "bravo_load_firmware: filesize(%d), ret(%d)\n", fw->size, ret);
  
  filp_close(fp, files);

  return 0;
}
#endif

// Writes a value to the configuration register. Currently only used
// to wake the chip from deep sleep.
static int if_sdio_write_config(struct lbs_private* priv, u8 value)
{
    int ret = 0;
    struct if_sdio_card* card = (struct if_sdio_card*)priv->card;

	lbs_deb_enter(LBS_DEB_SDIO);
    if (!card) { return -1; }

    sdio_claim_host(card->func);
    sdio_writeb(card->func, value, IF_SDIO_CONFIG, &ret);
    sdio_release_host(card->func);

	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);
    return ret;
} 

static u16 if_sdio_read_scratch(struct if_sdio_card *card, int *err)
{
	int ret, reg;
	u16 scratch;

	if (card->model == 0x04)
		reg = IF_SDIO_SCRATCH_OLD;
	else
		reg = IF_SDIO_SCRATCH;

	scratch = sdio_readb(card->func, reg, &ret);
	if (!ret)
		scratch |= sdio_readb(card->func, reg + 1, &ret) << 8;

	//printk(KERN_NOTICE "if_sdio_read_scratch:scratch(0x%x)\n", scratch);

	if (err)
		*err = ret;

	if (ret)
		return 0xffff;

	return scratch;
}

static int if_sdio_handle_cmd(struct if_sdio_card *card,
		u8 *buffer, unsigned size)
{
	struct lbs_private *priv = card->priv;
	int ret;
	unsigned long flags;
	u8 i;

	lbs_deb_enter(LBS_DEB_SDIO);

	if (size > LBS_CMD_BUFFER_SIZE) {
		lbs_deb_sdio("response packet too large (%d bytes)\n",
			(int)size);
		ret = -E2BIG;
		goto out;
	}

	spin_lock_irqsave(&priv->driver_lock, flags);

	i = (priv->resp_idx == 0) ? 1 : 0;
#ifdef CONFIG_MACH_BRAVO
	{
	struct cmd_ctrl_node *curcmd = priv->cur_cmd;
	unsigned short cmdno = 0x0;
	
	if ( (curcmd) && (curcmd->cmdbuf) )
	{
		cmdno = curcmd->cmdbuf->command;
	}
	if (priv->resp_len[i])
	{
		if (cmdno)
			printk(KERN_NOTICE "IF_SDIO_HANDLE_CMD: CMD(%d) response buffer is occupied!!!\n", cmdno);
		else
			printk(KERN_NOTICE "IF_SDIO_HANDLE_CMD: CMD response buffer is occupied!!!\n");
	}
	}
#else
	BUG_ON(priv->resp_len[i]);
#endif

	priv->resp_len[i] = size;
	memcpy(priv->resp_buf[i], buffer, size);
	lbs_notify_command_response(priv, i);

	spin_unlock_irqrestore(&card->priv->driver_lock, flags);

	ret = 0;

out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);
	return ret;
}

static int if_sdio_handle_data(struct if_sdio_card *card,
		u8 *buffer, unsigned size)
{
	int ret;
	struct sk_buff *skb;
	char *data;

	lbs_deb_enter(LBS_DEB_SDIO);

	if (size > MRVDRV_ETH_RX_PACKET_BUFFER_SIZE) {
		lbs_deb_sdio("response packet too large (%d bytes)\n",
			(int)size);
		ret = -E2BIG;
		goto out;
	}

	skb = dev_alloc_skb(MRVDRV_ETH_RX_PACKET_BUFFER_SIZE + NET_IP_ALIGN);
	if (!skb) {
		ret = -ENOMEM;
		goto out;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	data = skb_put(skb, size);

	memcpy(data, buffer, size);

	lbs_process_rxed_packet(card->priv, skb);

	ret = 0;

out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_handle_event(struct if_sdio_card *card,
		u8 *buffer, unsigned size)
{
	int ret;
	u32 event;

	lbs_deb_enter(LBS_DEB_SDIO);

	if (card->model == 0x04) {
		event = sdio_readb(card->func, IF_SDIO_EVENT, &ret);
		if (ret)
		{
			printk(KERN_NOTICE "if_sdio_handle_event: ERROR sdio_readb()\n");
			goto out;
		}
	} else {
		if (size < 4) {
			lbs_deb_sdio("event packet too small (%d bytes)\n",(int)size);
			printk(KERN_NOTICE "event packet too small (%d bytes)\n",(int)size);
			ret = -EINVAL;
			goto out;
		}
		event = buffer[3] << 24;
		event |= buffer[2] << 16;
		event |= buffer[1] << 8;
		event |= buffer[0] << 0;
	}

	lbs_queue_event(card->priv, event & 0xFF);
	ret = 0;

out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_card_to_host(struct if_sdio_card *card)
{
	int ret;
	u8 status;
	u16 size, type, chunk;
	unsigned long timeout;

	lbs_deb_enter(LBS_DEB_SDIO);

	timeout = jiffies + HZ;
	while (1) {
		status = sdio_readb(card->func, IF_SDIO_STATUS, &ret);
		if (ret)
			goto out;
		if (status & IF_SDIO_IO_RDY)
			break;
		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			goto out;
		}
#ifdef CONFIG_MACH_BRAVO
		msleep(1);
#else
		mdelay(1);
#endif
	}

#ifdef CONFIG_MACH_BRAVO
	mutex_lock(&(card->priv->isr_lock));
	//printk(KERN_NOTICE "GET0\n");
#endif
	size = if_sdio_read_scratch(card, &ret);
	if (ret)
		goto out;

	if (size < 4) {
		lbs_deb_sdio("invalid packet size (%d bytes) from firmware\n",
			(int)size);
		ret = -EINVAL;
		goto out;
	}


	/*
	 * The transfer must be in one transaction or the firmware
	 * goes suicidal. There's no way to guarantee that for all
	 * controllers, but we can at least try.
	 */
	chunk = sdio_align_size(card->func, size);

	ret = sdio_readsb(card->func, card->buffer, card->ioport, chunk);
	if (ret)
		goto out;

	chunk = card->buffer[0] | (card->buffer[1] << 8);
	type = card->buffer[2] | (card->buffer[3] << 8);

	lbs_deb_sdio("packet of type %d and size %d bytes\n",
		(int)type, (int)chunk);

	if (chunk > size) {
		lbs_deb_sdio("packet fragment (%d > %d)\n",
			(int)chunk, (int)size);
		ret = -EINVAL;
		goto out;
	}

	if (chunk < size) {
		lbs_deb_sdio("packet fragment (%d < %d)\n",
			(int)chunk, (int)size);
	}

	switch (type) {
	case MVMS_CMD:
		//printk(KERN_NOTICE "HANDLE_CMD\n");
		ret = if_sdio_handle_cmd(card, card->buffer + 4, chunk - 4);
		if (ret)
			goto out;
		break;
	case MVMS_DAT:
		//printk(KERN_NOTICE "HANDLE_DAT\n");
		ret = if_sdio_handle_data(card, card->buffer + 4, chunk - 4);
		if (ret)
			goto out;
		break;
	case MVMS_EVENT:
		//printk(KERN_NOTICE "HANDLE_EVENT\n");
		ret = if_sdio_handle_event(card, card->buffer + 4, chunk - 4);
		if (ret)
			goto out;
		break;
	default:
		//printk(KERN_NOTICE "invalid type (%d) from firmware\n", (int)type);
		ret = -EINVAL;
		goto out;
	}

out:
#ifdef CONFIG_MACH_BRAVO
	mutex_unlock(&(card->priv->isr_lock));
#endif

	if (ret)
		lbs_pr_err("problem fetching packet from firmware\n");

	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

#ifdef CONFIG_MACH_BRAVO_WIFI_OPTIMIZE
static void if_sdio_host_to_card_worker(struct if_sdio_card *card)
{
	struct if_sdio_packet *packet;
	unsigned long timeout;
	u8 status;
	int ret;
	unsigned long flags;
	lbs_deb_enter(LBS_DEB_SDIO);
	
	while (1) {
		spin_lock_irqsave(&card->lock, flags);
		packet = card->packets;
		if (packet)
			card->packets = packet->next;
#ifdef CONFIG_MACH_BRAVO
		if (packet)
		{
			struct cmd_confirm_sleep *p_confirm_sleep = (struct cmd_confirm_sleep *)(&(packet->buffer[4]));
			if ( (p_confirm_sleep->hdr.command == cpu_to_le16(CMD_802_11_PS_MODE)) && (p_confirm_sleep->action == cpu_to_le16(CMD_SUBCMD_SLEEP_CONFIRMED)) )
			{
				if ( (card->priv->psstate != PS_STATE_PRE_SLEEP) && (card->priv->psstate != PS_STATE_SLEEP) )
				{
					printk(KERN_NOTICE "host_to_card_worker: IGNORE sleep confirm cmd\n");
					spin_unlock_irqrestore(&card->lock, flags);
					continue;
				}
			}
		}
#endif
		spin_unlock_irqrestore(&card->lock, flags);

		if (!packet)
			break;

		sdio_claim_host(card->func);

		timeout = jiffies + HZ;
		while (1) {
			status = sdio_readb(card->func, IF_SDIO_STATUS, &ret);
			if (ret)
				goto release;
			if (status & IF_SDIO_IO_RDY)
				break;
			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				printk(KERN_NOTICE "host_to_card_worker: IF_SDIO_STATUS[IF_SDIO_IO_RDY] is not set!!!\n");
				goto release;
			}
			mdelay(1);
		}

#ifdef CONFIG_MACH_BRAVO
		mutex_lock(&(card->priv->isr_lock));
		//printk(KERN_NOTICE "GET1\n");
#endif
		ret = sdio_writesb(card->func, card->ioport,
				packet->buffer, packet->nb);
#ifdef CONFIG_MACH_BRAVO
		mutex_unlock(&(card->priv->isr_lock));
		//printk(KERN_NOTICE "REL1\n");
#endif
		if (ret)
			goto release;
release:
		sdio_release_host(card->func);

		kfree(packet);
	}

	lbs_deb_leave(LBS_DEB_SDIO);
}
#else
static void if_sdio_host_to_card_worker(struct work_struct *work)
{
	struct if_sdio_card *card;
	struct if_sdio_packet *packet;
	unsigned long timeout;
	u8 status;
	int ret;
	unsigned long flags;
	lbs_deb_enter(LBS_DEB_SDIO);
	
	card = container_of(work, struct if_sdio_card, packet_worker);

	while (1) {
		spin_lock_irqsave(&card->lock, flags);
		packet = card->packets;
		if (packet)
			card->packets = packet->next;
#ifdef CONFIG_MACH_BRAVO
		if (packet)
		{
			struct cmd_confirm_sleep *p_confirm_sleep = (struct cmd_confirm_sleep *)(&(packet->buffer[4]));
			if ( (p_confirm_sleep->hdr.command == cpu_to_le16(CMD_802_11_PS_MODE)) && (p_confirm_sleep->action == cpu_to_le16(CMD_SUBCMD_SLEEP_CONFIRMED)) )
			{
				if (card->priv->psstate != PS_STATE_SLEEP)
				{
					printk(KERN_NOTICE "host_to_card_worker: IGNORE sleep confirm cmd\n");
					spin_unlock_irqrestore(&card->lock, flags);
					continue;
				}
			}
		}
#endif
		spin_unlock_irqrestore(&card->lock, flags);

		if (!packet)
			break;

		sdio_claim_host(card->func);

		timeout = jiffies + HZ;
		while (1) {
			status = sdio_readb(card->func, IF_SDIO_STATUS, &ret);
			if (ret)
				goto release;
			if (status & IF_SDIO_IO_RDY)
				break;
			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				printk(KERN_NOTICE "host_to_card_worker: IF_SDIO_STATUS[IF_SDIO_IO_RDY] is not set!!!\n");
				goto release;
			}
			mdelay(1);
		}

#ifdef CONFIG_MACH_BRAVO
		mutex_lock(&(card->priv->isr_lock));
		//printk(KERN_NOTICE "GET1\n");
#endif
		ret = sdio_writesb(card->func, card->ioport,
				packet->buffer, packet->nb);
#ifdef CONFIG_MACH_BRAVO
		mutex_unlock(&(card->priv->isr_lock));
		//printk(KERN_NOTICE "REL1\n");
#endif
		if (ret)
			goto release;
release:
		sdio_release_host(card->func);

		kfree(packet);
	}

	lbs_deb_leave(LBS_DEB_SDIO);
}
#endif  //CONFIG_MACH_BRAVO_WIFI_OPTIMIZE

/********************************************************************/
/* Firmware                                                         */
/********************************************************************/

static int if_sdio_prog_helper(struct if_sdio_card *card)
{
	int ret;
	u8 status;
#ifdef CONFIG_MACH_BRAVO
	struct builtin_fw fw;
	unsigned long order = 0;
#else
	const struct firmware *fw;
#endif
	unsigned long timeout;
	u8 *chunk_buffer;
	u32 chunk_size;
	const u8 *firmware;
	size_t size;

	lbs_deb_enter(LBS_DEB_SDIO);

#ifdef CONFIG_MACH_BRAVO
	fw.name = card->helper;
	ret = bravo_load_firmware(&fw, &order);
	if (ret) {
		lbs_pr_err("Failed to load helper firmware\n");
		goto out;
	}
#else
	ret = request_firmware(&fw, card->helper, &card->func->dev);
	if (ret) {
		lbs_pr_err("can't load helper firmware\n");
		goto out;
	}
#endif
	chunk_buffer = kzalloc(64, GFP_KERNEL);
	if (!chunk_buffer) {
		ret = -ENOMEM;
		goto release_fw;
	}

	sdio_claim_host(card->func);

	ret = sdio_set_block_size(card->func, 32);
	if (ret)
		goto release;

#ifdef CONFIG_MACH_BRAVO
	firmware = fw.data;
	size = fw.size;
#else
	firmware = fw->data;
	size = fw->size;
#endif

	while (size) {
		timeout = jiffies + HZ;
		while (1) {
			status = sdio_readb(card->func, IF_SDIO_STATUS, &ret);
			if (ret)
				goto release;
			if ((status & IF_SDIO_IO_RDY) &&
					(status & IF_SDIO_DL_RDY))
				break;
			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				goto release;
			}
			mdelay(1);
		}

		chunk_size = min(size, (size_t)60);

		*((__le32*)chunk_buffer) = cpu_to_le32(chunk_size);
		memcpy(chunk_buffer + 4, firmware, chunk_size);
/*
		lbs_deb_sdio("sending %d bytes chunk\n", chunk_size);
*/
		ret = sdio_writesb(card->func, card->ioport,
				chunk_buffer, 64);
		if (ret)
			goto release;

		firmware += chunk_size;
		size -= chunk_size;
	}

	/* an empty block marks the end of the transfer */
	memset(chunk_buffer, 0, 4);
	ret = sdio_writesb(card->func, card->ioport, chunk_buffer, 64);
	if (ret)
		goto release;

	lbs_deb_sdio("waiting for helper to boot...\n");

	/* wait for the helper to boot by looking at the size register */
	timeout = jiffies + HZ;
	while (1) {
		u16 req_size;

		req_size = sdio_readb(card->func, IF_SDIO_RD_BASE, &ret);
		if (ret)
			goto release;

		req_size |= sdio_readb(card->func, IF_SDIO_RD_BASE + 1, &ret) << 8;
		if (ret)
			goto release;

		if (req_size != 0)
			break;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			goto release;
		}

		msleep(10);
	}

	ret = 0;

release:
	sdio_set_block_size(card->func, 0);
	sdio_release_host(card->func);
	kfree(chunk_buffer);
release_fw:
#ifdef CONFIG_MACH_BRAVO
	vfree(fw.data);
#else
	release_firmware(fw);
#endif

out:
	if (ret)
		lbs_pr_err("failed to load helper firmware\n");

	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_prog_real(struct if_sdio_card *card)
{
	int ret;
	u8 status;
#ifdef CONFIG_MACH_BRAVO
	struct builtin_fw fw;
	unsigned long order = 0;
#else
	const struct firmware *fw;
#endif
	unsigned long timeout;
	u8 *chunk_buffer;
	u32 chunk_size;
	const u8 *firmware;
	size_t size, req_size;

	lbs_deb_enter(LBS_DEB_SDIO);

#ifdef CONFIG_MACH_BRAVO
	fw.name = card->firmware;
	ret = bravo_load_firmware(&fw, &order);
	if (ret) {
		lbs_pr_err("Failed to load helper firmware\n");
		goto out;
	}
#else
	ret = request_firmware(&fw, card->firmware, &card->func->dev);
	if (ret) {
		lbs_pr_err("can't load firmware\n");
		goto out;
	}
#endif
	chunk_buffer = kzalloc(512, GFP_KERNEL);
	if (!chunk_buffer) {
		ret = -ENOMEM;
		goto release_fw;
	}

	sdio_claim_host(card->func);

	ret = sdio_set_block_size(card->func, 32);
	if (ret)
		goto release;

#ifdef CONFIG_MACH_BRAVO
	firmware = fw.data;
	size = fw.size;
#else
	firmware = fw->data;
	size = fw->size;
#endif

	while (size) {
		timeout = jiffies + HZ;
		while (1) {
			status = sdio_readb(card->func, IF_SDIO_STATUS, &ret);
			if (ret)
				goto release;
			if ((status & IF_SDIO_IO_RDY) &&
					(status & IF_SDIO_DL_RDY))
				break;
			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				goto release;
			}
			mdelay(1);
		}

		req_size = sdio_readb(card->func, IF_SDIO_RD_BASE, &ret);
		if (ret)
			goto release;

		req_size |= sdio_readb(card->func, IF_SDIO_RD_BASE + 1, &ret) << 8;
		if (ret)
			goto release;
/*
		lbs_deb_sdio("firmware wants %d bytes\n", (int)req_size);
*/
		if (req_size == 0) {
			lbs_deb_sdio("firmware helper gave up early\n");
			ret = -EIO;
			goto release;
		}

		if (req_size & 0x01) {
			lbs_deb_sdio("firmware helper signalled error\n");
			ret = -EIO;
			goto release;
		}

		if (req_size > size)
			req_size = size;

		while (req_size) {
			chunk_size = min(req_size, (size_t)512);

			memcpy(chunk_buffer, firmware, chunk_size);
/*
			lbs_deb_sdio("sending %d bytes (%d bytes) chunk\n",
				chunk_size, (chunk_size + 31) / 32 * 32);
*/
			ret = sdio_writesb(card->func, card->ioport,
				chunk_buffer, (chunk_size + 31) / 32 * 32);
			if (ret)
				goto release;

			firmware += chunk_size;
			size -= chunk_size;
			req_size -= chunk_size;
		}
	}

	ret = 0;

	lbs_deb_sdio("waiting for firmware to boot...\n");

	/* wait for the firmware to boot */
	timeout = jiffies + HZ;
	while (1) {
		u16 scratch;

		scratch = if_sdio_read_scratch(card, &ret);
		if (ret)
			goto release;

		if (scratch == IF_SDIO_FIRMWARE_OK)
			break;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			goto release;
		}

		msleep(10);
	}

	ret = 0;

release:
	sdio_set_block_size(card->func, 0);
	sdio_release_host(card->func);
	kfree(chunk_buffer);
release_fw:
#ifdef CONFIG_MACH_BRAVO
	vfree(fw.data);
#else
	release_firmware(fw);
#endif

out:
	if (ret)
		lbs_pr_err("failed to load firmware\n");

	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

static int if_sdio_prog_firmware(struct if_sdio_card *card)
{
	int ret;
	u16 scratch;

	lbs_deb_enter(LBS_DEB_SDIO);

	sdio_claim_host(card->func);
	scratch = if_sdio_read_scratch(card, &ret);
	sdio_release_host(card->func);

	if (ret)
		goto out;

	if (scratch == IF_SDIO_FIRMWARE_OK) {
		lbs_deb_sdio("firmware already loaded\n");
		goto success;
	}

	ret = if_sdio_prog_helper(card);
	if (ret)
		goto out;

	ret = if_sdio_prog_real(card);
	if (ret)
		goto out;

success:
	ret = 0;

out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

/*******************************************************************/
/* Libertas callbacks                                              */
/*******************************************************************/

static int if_sdio_host_to_card(struct lbs_private *priv,
		u8 type, u8 *buf, u16 nb)
{
	int ret;
	struct if_sdio_card *card;
	struct if_sdio_packet *packet, *cur;
	u16 size;
	unsigned long flags;

	lbs_deb_enter_args(LBS_DEB_SDIO, "type %d, bytes %d", type, nb);

	card = priv->card;

	if (nb > (65536 - sizeof(struct if_sdio_packet) - 4)) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * The transfer must be in one transaction or the firmware
	 * goes suicidal. There's no way to guarantee that for all
	 * controllers, but we can at least try.
	 */
	size = sdio_align_size(card->func, nb + 4);

	packet = kzalloc(sizeof(struct if_sdio_packet) + size,
			GFP_ATOMIC);
	if (!packet) {
		ret = -ENOMEM;
		goto out;
	}

	packet->next = NULL;
	packet->nb = size;

	/*
	 * SDIO specific header.
	 */
	packet->buffer[0] = (nb + 4) & 0xff;
	packet->buffer[1] = ((nb + 4) >> 8) & 0xff;
	packet->buffer[2] = type;
	packet->buffer[3] = 0;

	memcpy(packet->buffer + 4, buf, nb);

	spin_lock_irqsave(&card->lock, flags);

	if (!card->packets)
		card->packets = packet;
	else {
		cur = card->packets;
		while (cur->next)
			cur = cur->next;
		cur->next = packet;
	}

	switch (type) {
	case MVMS_CMD:
		priv->dnld_sent = DNLD_CMD_SENT;
		break;
	case MVMS_DAT:
		priv->dnld_sent = DNLD_DATA_SENT;
		break;
	default:
		lbs_deb_sdio("unknown packet type %d\n", (int)type);
	}

	spin_unlock_irqrestore(&card->lock, flags);

#ifdef CONFIG_MACH_BRAVO_WIFI_OPTIMIZE
	if_sdio_host_to_card_worker(card);
#else
	schedule_work(&card->packet_worker);
#endif

	ret = 0;

out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

/*******************************************************************/
/* SDIO callbacks                                                  */
/*******************************************************************/

static void if_sdio_interrupt(struct sdio_func *func)
{
	int ret;
	struct if_sdio_card *card;
	u8 cause;

	lbs_deb_enter(LBS_DEB_SDIO);

	card = sdio_get_drvdata(func);

	cause = sdio_readb(card->func, IF_SDIO_H_INT_STATUS, &ret);
	if (ret)
		goto out;

	lbs_deb_sdio("interrupt: 0x%X\n", (unsigned)cause);
	//printk(KERN_NOTICE "if_sdio_interrupt(0): interrupt -- 0x%x\n", (unsigned)cause);

	sdio_writeb(card->func, ~cause, IF_SDIO_H_INT_STATUS, &ret);
	if (ret)
		goto out;

	/*
	 * Ignore the define name, this really means the card has
	 * successfully received the command.
	 */
	if (cause & IF_SDIO_H_INT_DNLD)
		lbs_host_to_card_done(card->priv);
	
	if (cause & IF_SDIO_H_INT_UPLD) {
		ret = if_sdio_card_to_host(card);
		if (ret)
			goto out;
	}

	ret = 0;
	//printk(KERN_NOTICE "if_sdio_interrup(1)\n");
out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

}

static void lbs_wait_on_mainthread(struct lbs_private *priv)
{
	while (1)
	{
		spin_lock_irq(&priv->driver_lock);
		if ( list_empty(&(priv->waitq.task_list)) )
		{
			spin_unlock_irq(&priv->driver_lock);
			schedule();
		}
		else
		{
			printk(KERN_NOTICE "LBS_THREAD has been scheduled in and put into INTERRUPTIBLE state\n");
			break;
		}
	}
}

static int if_sdio_probe(struct sdio_func *func,
		const struct sdio_device_id *id)
{
	struct if_sdio_card *card;
	struct lbs_private *priv;
	int ret, i;
	unsigned int model;
	struct if_sdio_packet *packet;

	lbs_deb_enter(LBS_DEB_SDIO);

	for (i = 0;i < func->card->num_info;i++) {
		if (sscanf(func->card->info[i],
				"802.11 SDIO ID: %x", &model) == 1)
			break;
		if (sscanf(func->card->info[i],
				"ID: %x", &model) == 1)
			break;
               if (!strcmp(func->card->info[i], "IBIS Wireless SDIO Card")) {
                       model = 4;
                       break;
               }
	}

	if (i == func->card->num_info) {
		lbs_pr_err("unable to identify card model\n");
		return -ENODEV;
	}

	if (!g_sdio_card)
	{
		g_sdio_card = kzalloc(sizeof(struct if_sdio_card), GFP_KERNEL);
		if (!g_sdio_card)
			return -ENOMEM;
	}
	else
		memset(g_sdio_card, 0x0, sizeof(struct if_sdio_card));
	
	card = g_sdio_card;

	card->func = func;
	card->model = model;
	spin_lock_init(&card->lock);
#ifndef CONFIG_MACH_BRAVO_WIFI_OPTIMIZE
	INIT_WORK(&card->packet_worker, if_sdio_host_to_card_worker);
#endif

	for (i = 0;i < ARRAY_SIZE(if_sdio_models);i++) {
		if (card->model == if_sdio_models[i].model)
			break;
	}

	if (i == ARRAY_SIZE(if_sdio_models)) {
		lbs_pr_err("unkown card model 0x%x\n", card->model);
		ret = -ENODEV;
		goto free;
	}

	card->helper = if_sdio_models[i].helper;
	card->firmware = if_sdio_models[i].firmware;

	if (lbs_helper_name) {
		lbs_deb_sdio("overriding helper firmware: %s\n",
			lbs_helper_name);
		card->helper = lbs_helper_name;
	}

	if (lbs_fw_name) {
		lbs_deb_sdio("overriding firmware: %s\n", lbs_fw_name);
		card->firmware = lbs_fw_name;
	}

	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret)
		goto release;

	ret = sdio_claim_irq(func, if_sdio_interrupt);
	if (ret)
		goto disable;

	card->ioport = sdio_readb(func, IF_SDIO_IOPORT, &ret);

	if (ret)
		goto release_int;

	card->ioport |= sdio_readb(func, IF_SDIO_IOPORT + 1, &ret) << 8;
	if (ret)
		goto release_int;

	card->ioport |= sdio_readb(func, IF_SDIO_IOPORT + 2, &ret) << 16;
	if (ret)
		goto release_int;

	sdio_release_host(func);

	sdio_set_drvdata(func, card);

	lbs_deb_sdio("class = 0x%X, vendor = 0x%X, "
			"device = 0x%X, model = 0x%X, ioport = 0x%X\n",
			func->class, func->vendor, func->device,
			model, (unsigned)card->ioport);

	ret = if_sdio_prog_firmware(card);
	if (ret)
		goto reclaim;

	priv = lbs_add_card(card, &func->dev);
	if (!priv) {
		ret = -ENOMEM;
		goto reclaim;
	}

	card->priv = priv;

	priv->card = card;
	priv->hw_host_to_card = if_sdio_host_to_card;
    priv->write_config = if_sdio_write_config;

#ifdef CONFIG_MACH_BRAVO
	lbs_wait_on_mainthread(priv);
#endif
    
	priv->fw_ready = 1;

	/*
	 * Enable interrupts now that everything is set up
	 */
	sdio_claim_host(func);
	sdio_writeb(func, 0x0f, IF_SDIO_H_INT_MASK, &ret);
	sdio_release_host(func);
	if (ret)
		goto reclaim;

	ret = lbs_start_card(priv);
	if (ret)
		goto err_activate_card;

#ifdef CONFIG_MACH_BRAVO
	priv->ps_supported = 1;
#else
	priv->ps_supported = 1;
#endif

out:
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;

err_activate_card:
	flush_scheduled_work();
	free_netdev(priv->dev);
	kfree(priv);
reclaim:
	sdio_claim_host(func);
release_int:
	sdio_release_irq(func);
disable:
	sdio_disable_func(func);
release:
	sdio_release_host(func);
free:
	while (card->packets) {
		packet = card->packets;
		card->packets = card->packets->next;
		kfree(packet);
	}

	//[ICS] --Don't release g_sdio_card -- we need contiguous pages for g_sdio_card->buffer(64KB),
	//The only safe solution is to allocate contiguous 64KB during system initialization.
	//(During system initialization, system memory is not granular)
	//kfree(card);

	goto out;
}

static void if_sdio_remove(struct sdio_func *func)
{
	struct if_sdio_card *card;
	struct if_sdio_packet *packet;

	printk(KERN_NOTICE "IF_SDIO_REMOVE get invoked!\n");

	lbs_deb_enter(LBS_DEB_SDIO);

	card = sdio_get_drvdata(func);

	card->priv->surpriseremoved = 1;

	lbs_deb_sdio("call remove card\n");
	lbs_stop_card(card->priv);
	lbs_remove_card(card->priv);

	flush_scheduled_work();

	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_irq(func);
	sdio_release_host(func);

	while (card->packets) {
		packet = card->packets;
		card->packets = card->packets->next;
		kfree(packet);
	}

	//kfree(card);
	memset(g_sdio_card, 0x0, sizeof(struct if_sdio_card));

	lbs_deb_leave(LBS_DEB_SDIO);
}
#ifndef CONFIG_BRAVO_WIFI_ONOFF
static struct sdio_driver if_sdio_driver = {
#else
struct sdio_driver if_sdio_driver = {
#endif
	.name		= "libertas_sdio",
	.id_table	= if_sdio_ids,
	.probe		= if_sdio_probe,
	.remove		= if_sdio_remove,
};

/*******************************************************************/
/* Module functions                                                */
/*******************************************************************/

static int __init if_sdio_init_module(void)
{
	int ret = 0;

	lbs_deb_enter(LBS_DEB_SDIO);

	printk(KERN_INFO "libertas_sdio: Libertas SDIO driver\n");
	printk(KERN_INFO "libertas_sdio: Copyright Pierre Ossman.\n");

	ret = sdio_register_driver(&if_sdio_driver);
	lbs_deb_leave_args(LBS_DEB_SDIO, "ret %d", ret);

	return ret;
}

static void __exit if_sdio_exit_module(void)
{
	lbs_deb_enter(LBS_DEB_SDIO);

	sdio_unregister_driver(&if_sdio_driver);

	lbs_deb_leave(LBS_DEB_SDIO);
}

module_init(if_sdio_init_module);
module_exit(if_sdio_exit_module);

MODULE_DESCRIPTION("Libertas SDIO WLAN Driver");
MODULE_AUTHOR("Pierre Ossman");
MODULE_LICENSE("GPL");
