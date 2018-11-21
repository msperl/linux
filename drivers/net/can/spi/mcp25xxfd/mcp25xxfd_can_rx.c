// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 * implementation of can transmission
 *
 * Copyright 2017 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include "mcp25xxfd_can.h"
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

/* module parameters */
bool do_not_submit_rx;
module_param(do_not_submit_rx, bool, 0664);
MODULE_PARM_DESC(do_not_submit_rx,
		 "do not submit rx frames to can stack - used to test performance of the spi layer");

static
struct sk_buff *mcp25xxfd_can_submit_rx_normal_frame(struct net_device *net,
						     u32 id,
						     u32 dlc, u8 **data)
{
	struct can_frame *frame;
	struct sk_buff *skb;

	/* allocate frame */
	skb = alloc_can_skb(net, &frame);
	if (!skb)
		return NULL;

	/* set id, dlc and flags */
	frame->can_id = id;
	frame->can_dlc = dlc;

	/* and set the pointer to data */
	*data = frame->data;

	return skb;
}

/* it is almost identical except for the type of the frame... */
static
struct sk_buff *mcp25xxfd_can_submit_rx_fd_frame(struct net_device *net,
						 u32 id, u32 flags,
						 u32 len, u8 **data)
{
	struct canfd_frame *frame;
	struct sk_buff *skb;

	/* allocate frame */
	skb = alloc_canfd_skb(net, &frame);
	if (!skb)
		return NULL;

	/* set id, dlc and flags */
	frame->can_id = id;
	frame->len = len;
	frame->flags |= flags;

	/* and set the pointer to data */
	*data = frame->data;

	return skb;
}

int mcp25xxfd_can_submit_rx_frame(struct spi_device *spi, int fifo)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int addr = cpriv->fifos.fifo_reg[fifo].offset;
	struct mcp25xxfd_obj_rx *rx =
		(struct mcp25xxfd_obj_rx *)(cpriv->sram + addr);
	u8 *data = NULL;
	struct sk_buff *skb;
	u32 id, dlc, len, flags;

	/* compute the can_id */
	mcp25xxfd_mcpid_to_canid(rx->id, rx->flags, &id);

	/* and dlc */
	dlc = (rx->flags & CAN_OBJ_FLAGS_DLC_MASK) >>
		CAN_OBJ_FLAGS_DLC_SHIFT;
	len = can_dlc2len(dlc);

	/* update stats */
	priv->net->stats.rx_packets++;
	priv->net->stats.rx_bytes += len;
	cpriv->fifos.rx.dlc_usage[dlc]++;
	if (rx->flags & CAN_OBJ_FLAGS_FDF)
		cpriv->fifos.rx.fd_count++;

	/* allocate the skb buffer */
	if (rx->flags & CAN_OBJ_FLAGS_FDF) {
		flags = 0;

		flags |= (flags & CAN_OBJ_FLAGS_BRS) ? CANFD_BRS : 0;
		flags |= (flags & CAN_OBJ_FLAGS_ESI) ? CANFD_ESI : 0;
		skb = mcp25xxfd_can_submit_rx_fd_frame(net, id, flags,
						       dlc, &data);
	} else {
		skb = mcp25xxfd_can_submit_rx_normal_frame(net, id,
							   len, &data);
	}
	if (!skb) {
		dev_err(&spi->dev, "cannot allocate RX skb\n");
		priv->net->stats.rx_dropped++;
		return -ENOMEM;
	}

	/* copy the payload data */
	memcpy(data, rx->data, len);

	/* and submit the frame */
	if (do_not_submit_rx)
		consume_skb(skb);
	else
		netif_rx_ni(skb);

	return 0;
}

static int mcp25xxfd_can_read_rx_frame(struct spi_device *spi,
				       int fifo)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int addr = cpriv->fifos.fifo_reg[fifo].offset;
	struct mcp25xxfd_obj_rx *rx =
		(struct mcp25xxfd_obj_rx *)(cpriv->sram + addr);
	int len, ret;

	/* we read the header plus 8 data bytes for "standard frames" */
	ret = mcp25xxfd_cmd_readn(spi, MCP25XXFD_SRAM_ADDR(addr),
				  rx, sizeof(*rx) + 8);
	if (ret)
		return ret;

	/* transpose the headers to CPU format*/
	rx->id = le32_to_cpu(rx->id);
	rx->flags = le32_to_cpu(rx->flags);
	rx->ts = le32_to_cpu(rx->ts);

	/* compute len */
	len = can_dlc2len((rx->flags & CAN_OBJ_FLAGS_DLC_MASK) >>
			  CAN_OBJ_FLAGS_DLC_SHIFT);

	/* read the remaining data for canfd frames */
	if (net->mtu == CANFD_MTU && len > 8) {
		/* here the extra portion reading CanFD frames */
		ret = mcp25xxfd_cmd_readn(spi,
					  MCP25XXFD_SRAM_ADDR(addr) +
					  sizeof(*rx) + 8,
					  &rx->data[8], len - 8);
		if (ret)
			return ret;
	}

	/* clear the rest of the buffer - just to be safe */
	memset(rx->data + len, 0,
	       ((net->mtu == CANFD_MTU) ? 64 : 8) - len);

	/* increment the statistics counter */
	cpriv->fifos.fifo_info[fifo].use_count++;

	/* add the fifo to the process queues */
	mcp25xxfd_can_queue_frame(cpriv, fifo, rx->ts);

	/* and clear the interrupt flag for that fifo */
	return mcp25xxfd_cmd_write_mask(spi, CAN_FIFOCON(fifo),
					CAN_FIFOCON_FRESET,
					CAN_FIFOCON_FRESET);
}

int mcp25xxfd_can_read_rx_frames(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo;
	int ret;

	/* we can optimize here */
	for (i = 0, fifo = cpriv->fifos.rx.start;
	     i < cpriv->fifos.rx.count;
	     i++, fifo += cpriv->fifos.rx.increment) {
		if (cpriv->status.rxif & BIT(fifo)) {
			/* read the frame */
			ret = mcp25xxfd_can_read_rx_frame(spi, fifo);
			if (ret)
				return ret;
		}
	}

	return 0;
}
