// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

/* here we define and configure the fifo layout */

#include "mcp25xxfd_can.h"
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>

/* some module parameters that are currently not configurable via netlink */
unsigned int tx_fifos;
module_param(tx_fifos, uint, 0664);
MODULE_PARM_DESC(tx_fifos, "Number of tx-fifos to configure\n");

bool three_shot;
module_param(three_shot, bool, 0664);
MODULE_PARM_DESC(three_shot, "Use 3 shots when one-shot is requested");

#if defined(CONFIG_DEBUG_FS)
static
void mcp25xxfd_can_setup_fifo_debugfs_rxtx(struct net_device *net,
					   const char *name,
					   struct mcp25xxfd_fifo *desc)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;

	/* remove old debug directory */
	debugfs_remove_recursive(desc->debugfs_dir);

	/* create new one */
	desc->debugfs_dir = debugfs_create_dir(name, priv->debugfs_dir);

	/* and fill it */
	debugfs_create_u32("count",  0444, desc->debugfs_dir,
			   &desc->count);
	debugfs_create_u32("size",  0444, desc->debugfs_dir,
			   &desc->size);
	debugfs_create_u32("start",  0444, desc->debugfs_dir,
			   &desc->start);
	debugfs_create_u32("increment",  0444, desc->debugfs_dir,
			   &desc->increment);
	debugfs_create_u32("priority_start",  0444, desc->debugfs_dir,
			   &desc->priority_start);
	debugfs_create_u32("priority_increment",  0444, desc->debugfs_dir,
			   &desc->priority_increment);
}

static
void mcp25xxfd_can_setup_fifo_debugfs_link_rxtx(struct mcp25xxfd_fifo *desc,
						int index, int fifo)
{
	char name[4];
	char link[32];

	snprintf(name, sizeof(name), "%02i", index);
	snprintf(link, sizeof(link), "../can_fifos/%02i", fifo);

	debugfs_create_symlink(name, desc->debugfs_dir, link);
}

static
void mcp25xxfd_can_setup_fifo_debugfs_present_fifo(struct net_device *net,
						   int index)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct dentry *debugfs_dir;
	char name[4];

	snprintf(name, sizeof(name), "%02i", index);
	debugfs_dir = debugfs_create_dir(name, cpriv->fifos.debugfs_dir);

	/* and the entries */
	debugfs_create_u32("is_tx", 0444, debugfs_dir,
			   &cpriv->fifos.fifo_info[index].is_tx);
	debugfs_create_x32("offset", 0444, debugfs_dir,
			   &cpriv->fifos.fifo_reg[index].offset);
	debugfs_create_u32("priority", 0444, debugfs_dir,
			   &cpriv->fifos.fifo_info[index].priority);
	debugfs_create_u64("use_count", 0444, debugfs_dir,
			   &cpriv->fifos.fifo_info[index].use_count);
}

static
void mcp25xxfd_can_setup_fifo_debugfs_present_fifos(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	int i;

	/* create the directory if necessary */
	if (!cpriv->fifos.debugfs_dir)
		cpriv->fifos.debugfs_dir =
			debugfs_create_dir("can_fifos", priv->debugfs_dir);

	/* now present all fifos
	 * - note that there is no fifo 0 - that is the TX-queue!
	 */
	for (i = 1; i < 32; i++)
		mcp25xxfd_can_setup_fifo_debugfs_present_fifo(net, i);
}

static void mcp25xxfd_can_remove_fifo_debugfs(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	debugfs_remove_recursive(cpriv->fifos.debugfs_dir);
	cpriv->fifos.debugfs_dir = NULL;
	debugfs_remove_recursive(cpriv->fifos.rx.debugfs_dir);
	cpriv->fifos.rx.debugfs_dir = NULL;
	debugfs_remove_recursive(cpriv->fifos.tx.debugfs_dir);
	cpriv->fifos.tx.debugfs_dir = NULL;
}

#else

static
void mcp25xxfd_can_setup_fifo_debugfs_rxtx(struct net_device *net,
					   const char *name,
					   struct mcp25xxfd_fifo *desc)
{
}

static
void mcp25xxfd_can_setup_fifo_debugfs_link_rxtx(struct mcp25xxfd_fifo *desc,
						int index, int fifo)
{
}

static
void mcp25xxfd_can_setup_fifo_debugfs_present_fifos(struct net_device *net)
{
}

static void mcp25xxfd_can_remove_fifo_debugfs(struct net_device *net)
{
}
#endif

static int mcp25xxfd_can_get_fifo_address(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	int ret;

	/* write the config to the controller in one go */
	ret = mcp25xxfd_cmd_write_regs(spi, CAN_FIFOCON(0),
				       &cpriv->fifos.fifo_reg[0].control,
				       sizeof(cpriv->fifos.fifo_reg));
	if (ret)
		return ret;

	/* we need to move out of config mode to force address computation */
	ret = mcp25xxfd_can_switch_mode(spi, &cpriv->regs.con,
					CAN_CON_MODE_INTERNAL_LOOPBACK);
	if (ret)
		return ret;

	/* and get back into config mode */
	ret = mcp25xxfd_can_switch_mode(spi, &cpriv->regs.con,
					CAN_CON_MODE_CONFIG);
	if (ret)
		return ret;

	/* read address and config back in */
	return mcp25xxfd_cmd_read_regs(spi, CAN_FIFOCON(0),
				       &cpriv->fifos.fifo_reg[0].control,
				       sizeof(cpriv->fifos.fifo_reg));
}

static int mcp25xxfd_can_setup_fifo_config(struct net_device *net,
					   const char *name,
					   struct mcp25xxfd_fifo *desc,
					   u32 flags,
					   u32 flags_last)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	bool is_tx;
	u32 val;
	int index, prio, fifo, count;

	/* present fifo type info */
	mcp25xxfd_can_setup_fifo_debugfs_rxtx(net, name, desc);

	/* now setup the fifos themselves */
	for (index = 0,
	     fifo = desc->start,
	     count = desc->count,
	     prio = desc->priority_start;
	     count > 0;
	     index++,
	     fifo += desc->increment,
	     prio += desc->priority_increment,
	     count--) {
		/* select the effective value */
		val = (count > 1) ? flags : flags_last;

		/* are we in tx mode */
		is_tx = flags & CAN_FIFOCON_TXEN;
		cpriv->fifos.fifo_info[fifo].is_tx = is_tx;

		/* set priority if we are tx */
		if (is_tx) {
			cpriv->fifos.fifo_info[fifo].priority = prio;
			val |= (prio << CAN_FIFOCON_TXPRI_SHIFT);
		}

		/* setup interface */
		cpriv->fifos.fifo_reg[fifo].control = val;

		/* and link debugfs fifo */
		mcp25xxfd_can_setup_fifo_debugfs_link_rxtx(desc, index,
							   fifo);
	}

	return 0;
}

static int mcp25xxfd_can_setup_tx_fifo_config(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 tx_flags;

	/* TX Fifo configuration */
	tx_flags = CAN_FIFOCON_FRESET | /* reset FIFO */
		CAN_FIFOCON_TXEN | /* this is a TX_FIFO */
		CAN_FIFOCON_TXATIE | /* show up txatie flags in txatif reg */
		(cpriv->fifos.payload_mode << CAN_FIFOCON_PLSIZE_SHIFT) |
		(0 << CAN_FIFOCON_FSIZE_SHIFT); /* 1 FIFO only */
	if (cpriv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		if (three_shot)
			tx_flags |= CAN_FIFOCON_TXAT_THREE_SHOT <<
				CAN_FIFOCON_TXAT_SHIFT;
		else
			tx_flags |= CAN_FIFOCON_TXAT_ONE_SHOT <<
				CAN_FIFOCON_TXAT_SHIFT;
	else
		tx_flags |= CAN_FIFOCON_TXAT_UNLIMITED <<
			CAN_FIFOCON_TXAT_SHIFT;

	return mcp25xxfd_can_setup_fifo_config(net, "can_fifo_tx",
					       &cpriv->fifos.tx,
					       tx_flags, tx_flags);
}

static int mcp25xxfd_can_setup_rx_fifo_config(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 rx_flags, rx_flags_last;
	int ret;

	/* RX Fifo configuration */
	rx_flags = CAN_FIFOCON_FRESET | /* reset FIFO */
		CAN_FIFOCON_RXTSEN | /* RX timestamps */
		CAN_FIFOCON_TFERFFIE | /* FIFO Full */
		CAN_FIFOCON_TFHRFHIE | /* FIFO Half Full*/
		CAN_FIFOCON_TFNRFNIE | /* FIFO not empty */
		(cpriv->fifos.payload_mode << CAN_FIFOCON_PLSIZE_SHIFT) |
		(0 << CAN_FIFOCON_FSIZE_SHIFT); /* 1 FIFO only */
	rx_flags_last = rx_flags | CAN_FIFOCON_RXOVIE;

	/* configure the fifos */
	ret = mcp25xxfd_can_setup_fifo_config(net, "can_fifo_rx",
					      &cpriv->fifos.rx,
					      rx_flags, rx_flags_last);
	if (ret)
		return ret;

	mcp25xxfd_can_rx_fifo_debugfs(net);

	return 0;
}

static int mcp25xxfd_can_clear_rx_filter_masks(struct spi_device *spi)
{
	u32 data[2 * 32]; /* 2 registers (match and mask) per filter */

	memset(data, 0, sizeof(data));

	return mcp25xxfd_cmd_write_regs(spi, CAN_FLTOBJ(0),
					data, sizeof(data));
}

static int mcp25xxfd_can_setup_rx_filter_config(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	struct mcp25xxfd_fifo *desc = &cpriv->fifos.rx;
	u8 filter_con[32];
	int filter, fifo, ret;

	/* present all fifos */
	mcp25xxfd_can_setup_fifo_debugfs_present_fifos(net);

	/* clear the filter mask - match any frame with every filter */
	ret = mcp25xxfd_can_clear_rx_filter_masks(spi);
	if (ret)
		return ret;

	/* clear the filters and filter mappings for all filters */
	memset(filter_con, 0, sizeof(filter_con));

	/* and now set up the rx filters */
	for (filter = 0, fifo = desc->start;
	     filter < desc->count;
	     filter++, fifo += desc->increment) {
		/* set up filter config - we can use the mask of filter 0 */
		filter_con[filter] = CAN_FIFOCON_FLTEN(0) |
			(fifo << CAN_FILCON_SHIFT(0));
	}

	/* and set up filter control */
	return mcp25xxfd_cmd_write_regs(spi, CAN_FLTCON(0),
					(u32 *)filter_con,
					sizeof(filter_con));
}

static int mcp25xxfd_can_compute_fifos(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int tef_memory_used, tx_memory_used, rx_memory_available;

	/* default settings as per MTU/CANFD */
	switch (net->mtu) {
	case CAN_MTU:
		/* mtu is 8 */
		cpriv->fifos.payload_size = 8;
		cpriv->fifos.payload_mode = CAN_TXQCON_PLSIZE_8;

		/* 7 tx fifos */
		cpriv->fifos.tx.count = 7;

		break;
	case CANFD_MTU:
		/* wish there was a way to have hw filters
		 * that can separate based on length ...
		 */
		/* MTU is 64 */
		cpriv->fifos.payload_size = 64;
		cpriv->fifos.payload_mode = CAN_TXQCON_PLSIZE_64;

		/* 7 tx fifos */
		cpriv->fifos.tx.count = 7;

		break;
	default:
		return -EINVAL;
	}

	/* compute effective sizes */
	cpriv->fifos.tef.size = sizeof(struct mcp25xxfd_obj_tef);
	cpriv->fifos.tx.size = sizeof(struct mcp25xxfd_obj_tx) +
		cpriv->fifos.payload_size;
	cpriv->fifos.rx.size = sizeof(struct mcp25xxfd_obj_rx) +
		cpriv->fifos.payload_size;

	/* if defined as a module parameter modify the number of tx_fifos */
	if (tx_fifos) {
		netdev_info(net,
			    "Using %i tx-fifos as per module parameter\n",
			    tx_fifos);
		cpriv->fifos.tx.count = tx_fifos;
	}

	/* there can be at the most 30 tx fifos (TEF and at least 1 RX fifo */
	if (cpriv->fifos.tx.count > 30) {
		netdev_err(net,
			   "There is an absolute maximum of 30 tx-fifos\n");
		return -EINVAL;
	}

	/* set tef fifos to the number of tx fifos */
	cpriv->fifos.tef.count = cpriv->fifos.tx.count;

	/* compute size of the tx fifos and TEF */
	tx_memory_used = cpriv->fifos.tx.count * cpriv->fifos.tx.size;
	tef_memory_used = cpriv->fifos.tef.count * cpriv->fifos.tef.size;

	/* calculate evailable memory for RX_fifos */
	rx_memory_available = MCP25XXFD_BUFFER_TXRX_SIZE -
		tx_memory_used -
		tef_memory_used;

	/* we need at least one RX Frame */
	if (rx_memory_available < cpriv->fifos.rx.size) {
		netdev_err(net,
			   "Configured %i tx-fifos exceeds available memory already\n",
			   cpriv->fifos.tx.count);
		return -EINVAL;
	}

	/* calculate possible amount of RX fifos */
	cpriv->fifos.rx.count = rx_memory_available / cpriv->fifos.rx.size;

	/* so now calculate effective number of rx-fifos
	 * there are only 31 fifos available in total,
	 * so we need to limit ourselves
	 */
	if (cpriv->fifos.rx.count + cpriv->fifos.tx.count > 31)
		cpriv->fifos.rx.count = 31 - cpriv->fifos.tx.count;

	/* define the layout now that we have gotten everything */
	cpriv->fifos.tx.start = 1;
	cpriv->fifos.tx.increment = 1;
	/* highest priority is 31 */
	cpriv->fifos.tx.priority_start = 31;
	cpriv->fifos.tx.priority_increment = -1;

	cpriv->fifos.rx.start = 1 + cpriv->fifos.tx.count;
	cpriv->fifos.rx.increment = 1;
	/* rx fifos do not have a priority */
	cpriv->fifos.rx.priority_start = 0;
	cpriv->fifos.rx.priority_increment = 0;

	return 0;
}

int mcp25xxfd_can_setup_fifos(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	int ret;

	/* compute fifos counts */
	ret = mcp25xxfd_can_compute_fifos(net);
	if (ret)
		return ret;

	/* configure TEF */
	if (cpriv->fifos.tef.count)
		cpriv->regs.tefcon =
			CAN_TEFCON_FRESET |
			CAN_TEFCON_TEFNEIE |
			CAN_TEFCON_TEFTSEN |
			((cpriv->fifos.tef.count - 1) <<
			 CAN_TEFCON_FSIZE_SHIFT);
	else
		cpriv->regs.tefcon = 0;
	ret = mcp25xxfd_cmd_write(spi, CAN_TEFCON, cpriv->regs.tefcon);
	if (ret)
		return ret;

	/* clear fifo_reg and fifo_info */
	memset(cpriv->fifos.fifo_reg, 0, sizeof(cpriv->fifos.fifo_reg));
	memset(cpriv->fifos.fifo_info, 0, sizeof(cpriv->fifos.fifo_info));

	/* configure FIFOS themselves */
	ret = mcp25xxfd_can_setup_tx_fifo_config(net);
	if (ret)
		return ret;
	ret = mcp25xxfd_can_setup_rx_fifo_config(net);
	if (ret)
		return ret;

	/* get fifo addresses */
	ret = mcp25xxfd_can_get_fifo_address(net);
	if (ret)
		return ret;

	/* finally configure RX filters */
	ret = mcp25xxfd_can_setup_rx_filter_config(net);
	if (ret)
		return ret;

	/* setup tx_fifo_queue */
	return mcp25xxfd_can_tx_queue_alloc(net);
}

void mcp25xxfd_can_release_fifos(struct net_device *net)
{
	mcp25xxfd_can_tx_queue_free(net);
	mcp25xxfd_can_remove_fifo_debugfs(net);
}
