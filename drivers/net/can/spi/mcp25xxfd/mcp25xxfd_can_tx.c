// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include "mcp25xxfd_can.h"
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

/* structure of a spi message that is prepared and can get deployed quickly */
struct mcp2517fd_tx_spi_message {
	/* the network device this is related to */
	struct net_device *net;
	/* the fifo this fills */
	u32 fifo;
	/* the xfer to fill in the fifo data */
	struct {
		struct spi_message msg;
		struct spi_transfer xfer;
		struct {
			u8 cmd[2];
			u8 header[sizeof(struct mcp25xxfd_obj_tx)];
			u8 data[64];
		} data;
	} fill_fifo;
	/* the xfer to enable transmission on the can bus */
	struct {
		struct spi_message msg;
		struct spi_transfer xfer;
		struct {
			u8 cmd[2];
			u8 data;
		} data;
	} trigger_fifo;
};

struct mcp2517fd_tx_spi_message_queue {
	/* spinlock protecting the bitmaps
	 * as well as state and the skb_echo_* functions
	 */
	spinlock_t lock;
	/* bitmap of which fifo is in which stage */
	u32 idle;
	u32 in_fill_fifo_transfer;
	u32 in_trigger_fifo_transfer;
	u32 in_can_transfer;
	u32 transferred;

	/* the queue state as seen per controller */
	int state;

	/* statistics */
	struct {
		u64 tef_reads;
		u64 tef_conservative_reads;
		u64 tef_opportunistic_reads;
		u64 tef_read_splits;
#define TEF_READ_STATS_BINS 8
		u64 tef_opportunistic_read_sizes[TEF_READ_STATS_BINS];
	} stats;

	/* spinlock protecting spi submission order */
	spinlock_t spi_lock;

	/* map each fifo to a mcp2517fd_tx_spi_message */
	struct mcp2517fd_tx_spi_message *fifo2message[32];

	/* the individual messages */
	struct mcp2517fd_tx_spi_message message[];
};

/* mostly bit manipulations to move between stages */
static
struct mcp2517fd_tx_spi_message *mcp25xxfd_can_tx_queue_first_spi_message(
	struct mcp2517fd_tx_spi_message_queue *queue, u32 *bitmap)
{
	u32 first = ffs(*bitmap);

	if (!first)
		return NULL;

	return queue->fifo2message[first - 1];
}

static
struct mcp2517fd_tx_spi_message *mcp25xxfd_can_tx_queue_last_spi_message(
	struct mcp2517fd_tx_spi_message_queue *queue, u32 *bitmap)
{
	u32 last = ffs(*bitmap);

	if (!last)
		return NULL;

	return queue->fifo2message[last - 1];
}

static void mcp25xxfd_can_tx_queue_remove_spi_message(u32 *bitmap, int fifo)
{
	*bitmap &= ~BIT(fifo);
}

static void mcp25xxfd_can_tx_queue_add_spi_message(u32 *bitmap, int fifo)
{
	*bitmap |= BIT(fifo);
}

static void mcp25xxfd_can_tx_queue_move_spi_message(u32 *src, u32 *dest,
						    int fifo)
{
	mcp25xxfd_can_tx_queue_remove_spi_message(src, fifo);
	mcp25xxfd_can_tx_queue_add_spi_message(dest, fifo);
}

static void mcp25xxfd_can_tx_spi_message_fill_fifo_complete(void *context)
{
	struct mcp2517fd_tx_spi_message *msg = context;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(msg->net);
	unsigned long flags;

	/* reset transfer length to without data (DLC = 0) */
	msg->fill_fifo.xfer.len = sizeof(msg->fill_fifo.data.cmd) +
		sizeof(msg->fill_fifo.data.header);

	/* we need to hold this lock to protect us from
	 * concurrent access by start_xmit
	 */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* move to in_trigger_fifo_transfer */
	mcp25xxfd_can_tx_queue_move_spi_message(
		&cpriv->fifos.tx_queue->in_fill_fifo_transfer,
		&cpriv->fifos.tx_queue->in_trigger_fifo_transfer,
		msg->fifo);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);
}

static void mcp25xxfd_can_tx_spi_message_trigger_fifo_complete(void *context)
{
	struct mcp2517fd_tx_spi_message *msg = context;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(msg->net);
	unsigned long flags;

	/* we need to hold this lock to protect us from
	 * concurrent access by the interrupt thread
	 */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* move to can_transfer */
	mcp25xxfd_can_tx_queue_move_spi_message(
		&cpriv->fifos.tx_queue->in_trigger_fifo_transfer,
		&cpriv->fifos.tx_queue->in_can_transfer,
		msg->fifo);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);
}

#if defined(CONFIG_DEBUG_FS)
static void mcp25xxfd_can_tx_queue_debugfs(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct dentry *dir = cpriv->fifos.tx.debugfs_dir;
	struct mcp2517fd_tx_spi_message_queue *queue = cpriv->fifos.tx_queue;
	char name[32];
	u64 *data;
	int i;

	debugfs_create_u32("netif_queue_state",  0444, dir,
			   &cpriv->fifos.tx_queue->state);

	/* and for each queue stage the states */
	debugfs_create_x32("fifos_idle", 0444, dir,
			   &queue->idle);
	debugfs_create_x32("fifos_in_fill_fifo_transfer", 0444, dir,
			   &queue->in_fill_fifo_transfer);
	debugfs_create_x32("fifos_in_trigger_fifo_transfer", 0444, dir,
			   &queue->in_trigger_fifo_transfer);
	debugfs_create_x32("fifos_in_can_transfer", 0444, dir,
			   &queue->in_can_transfer);
	debugfs_create_x32("fifos_transferred", 0444, dir,
			   &queue->transferred);

	/* add statistics on tef */
	debugfs_create_u64("tef_reads", 0444, dir,
			   &queue->stats.tef_reads);
	debugfs_create_u64("tef_conservative_reads", 0444, dir,
			   &queue->stats.tef_conservative_reads);
	debugfs_create_u64("tef_opportunistic_reads", 0444, dir,
			   &queue->stats.tef_opportunistic_reads);
	debugfs_create_u64("tef_read_splits", 0444, dir,
			   &queue->stats.tef_read_splits);
	for (i = 0; i < TEF_READ_STATS_BINS - 2; i++) {
		snprintf(name, sizeof(name),
			 "tef_opportunistic_reads_%i", i + 1);
		data = &queue->stats.tef_opportunistic_read_sizes[i];
		debugfs_create_u64(name, 0444, dir, data);
	}

	/* i is already set correctly in the loop above */
	snprintf(name, sizeof(name), "tef_opportunistic_reads_%i+", i + 1);
	debugfs_create_u64(name, 0444, dir,
			   &queue->stats.tef_opportunistic_read_sizes[i]);
}
#else
static void mcp25xxfd_can_tx_queue_debugfs(struct net_device *net)
{
}
#endif

static
void mcp25xxfd_can_tx_message_init(struct net_device *net,
				   struct mcp2517fd_tx_spi_message *msg,
				   int fifo)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	const u32 trigger = CAN_FIFOCON_TXREQ | CAN_FIFOCON_UINC;
	const int first_byte = mcp25xxfd_first_byte(trigger);
	u32 addr;

	/* and initialize the structure */
	msg->net = net;
	msg->fifo = fifo;

	/* init fill_fifo */
	spi_message_init(&msg->fill_fifo.msg);
	msg->fill_fifo.msg.complete =
		mcp25xxfd_can_tx_spi_message_fill_fifo_complete;
	msg->fill_fifo.msg.context = msg;

	msg->fill_fifo.xfer.speed_hz = priv->spi_use_speed_hz;
	msg->fill_fifo.xfer.tx_buf = msg->fill_fifo.data.cmd;
	msg->fill_fifo.xfer.len = sizeof(msg->fill_fifo.data.cmd) +
		sizeof(msg->fill_fifo.data.header);
	spi_message_add_tail(&msg->fill_fifo.xfer, &msg->fill_fifo.msg);

	addr = MCP25XXFD_SRAM_ADDR(cpriv->fifos.fifo_reg[fifo].offset);
	mcp25xxfd_calc_cmd_addr(INSTRUCTION_WRITE,
				addr, msg->fill_fifo.data.cmd);

	/* init trigger_fifo */
	spi_message_init(&msg->trigger_fifo.msg);
	msg->trigger_fifo.msg.complete =
		mcp25xxfd_can_tx_spi_message_trigger_fifo_complete;
	msg->trigger_fifo.msg.context = msg;

	msg->trigger_fifo.xfer.speed_hz = priv->spi_use_speed_hz;
	msg->trigger_fifo.xfer.tx_buf = msg->trigger_fifo.data.cmd;
	msg->trigger_fifo.xfer.len = sizeof(msg->trigger_fifo.data.cmd) +
		sizeof(msg->trigger_fifo.data.data);
	spi_message_add_tail(&msg->trigger_fifo.xfer, &msg->trigger_fifo.msg);

	mcp25xxfd_calc_cmd_addr(INSTRUCTION_WRITE,
				CAN_FIFOCON(fifo) + first_byte,
				msg->trigger_fifo.data.cmd);
	msg->trigger_fifo.data.data = trigger >> (8 * first_byte);

	/* and add to idle tx transfers */
	mcp25xxfd_can_tx_queue_add_spi_message(&cpriv->fifos.tx_queue->idle,
					       fifo);
}

int mcp25xxfd_can_tx_queue_alloc(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	size_t size = sizeof(struct mcp2517fd_tx_spi_message_queue) +
		cpriv->fifos.tx.count *
		sizeof(struct mcp2517fd_tx_spi_message);
	struct mcp2517fd_tx_spi_message *msg;
	int i, fifo;

	/* allocate the fifos as an array */
	cpriv->fifos.tx_queue = kzalloc(size, GFP_KERNEL);
	if (!cpriv->fifos.tx_queue)
		return -ENOMEM;

	/* initialize the tx_queue structure */
	spin_lock_init(&cpriv->fifos.tx_queue->lock);
	spin_lock_init(&cpriv->fifos.tx_queue->spi_lock);

	/* initialize the individual spi_message structures */
	for (i = 0, fifo = cpriv->fifos.tx.start;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment) {
		msg = &cpriv->fifos.tx_queue->message[i];
		cpriv->fifos.tx_queue->fifo2message[fifo] = msg;
		mcp25xxfd_can_tx_message_init(net, msg, fifo);
	}

	mcp25xxfd_can_tx_queue_debugfs(net);

	return 0;
}

void mcp25xxfd_can_tx_queue_free(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	/* eventually we may need to wait here
	 * for all transfers to have finished
	 */

	kfree(cpriv->fifos.tx_queue);
	cpriv->fifos.tx_queue = NULL;
}

void mcp25xxfd_can_tx_queue_manage_nolock(struct net_device *net, int state)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	/* skip early */
	if (state == cpriv->fifos.tx_queue->state)
		return;

	/* start/stop netif_queue if necessary */
	switch (cpriv->fifos.tx_queue->state) {
	case TX_QUEUE_STATE_RUNABLE:
		switch (state) {
		case TX_QUEUE_STATE_RESTART:
		case TX_QUEUE_STATE_STARTED:
			netif_wake_queue(net);
			cpriv->fifos.tx_queue->state =
				TX_QUEUE_STATE_STARTED;
			break;
		}
		break;
	case TX_QUEUE_STATE_STOPPED:
		switch (state) {
		case TX_QUEUE_STATE_STARTED:
			netif_wake_queue(net);
			cpriv->fifos.tx_queue->state = state;
			break;
		}
		break;
	case TX_QUEUE_STATE_STARTED:
		switch (state) {
		case TX_QUEUE_STATE_RUNABLE:
		case TX_QUEUE_STATE_STOPPED:
			netif_stop_queue(net);
			cpriv->fifos.tx_queue->state = state;
			break;
		}
		break;
	default:
		WARN(true, "Unsupported tx_queue state: %i\n",
		     cpriv->fifos.tx_queue->state);
		break;
	}
}

void mcp25xxfd_can_tx_queue_manage(struct net_device *net, int state)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	unsigned long flags;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	mcp25xxfd_can_tx_queue_manage_nolock(net, state);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);
}

void mcp25xxfd_can_tx_queue_restart(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	unsigned long flags;
	u32 mask;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* only move if there is nothing pending or idle */
	mask = cpriv->fifos.tx_queue->idle |
		cpriv->fifos.tx_queue->in_fill_fifo_transfer |
		cpriv->fifos.tx_queue->in_trigger_fifo_transfer |
		cpriv->fifos.tx_queue->in_can_transfer;
	if (mask)
		goto out;

	/* move all items from transferred to idle */
	cpriv->fifos.tx_queue->idle |= cpriv->fifos.tx_queue->transferred;
	cpriv->fifos.tx_queue->transferred = 0;

	/* and enable queue */
	mcp25xxfd_can_tx_queue_manage_nolock(net, TX_QUEUE_STATE_RESTART);
out:
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);
}

static int mcp25xxfd_can_int_handle_txatif_fifo(struct spi_device *spi,
						int fifo)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 val;
	unsigned long flags;
	int ret;

	/* read fifo status */
	ret = mcp25xxfd_cmd_read(spi, CAN_FIFOSTA(fifo), &val);
	if (ret)
		return ret;

	/* clear the relevant interrupt flags */
	ret = mcp25xxfd_cmd_write_mask(spi,
				       CAN_FIFOSTA(fifo),
				       0,
				       CAN_FIFOSTA_TXABT |
				       CAN_FIFOSTA_TXLARB |
				       CAN_FIFOSTA_TXERR |
				       CAN_FIFOSTA_TXATIF);
	if (ret)
		return ret;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);
	/* for specific cases we probably could trigger a retransmit
	 * instead of an abort.
	 */

	/* and we release it from the echo_skb buffer
	 * NOTE: this is one place where packet delivery will not
	 * be ordered, as we do not have any timing information
	 * when this occurred
	 */
	can_get_echo_skb(net, fifo);

	mcp25xxfd_can_tx_queue_move_spi_message(
		&cpriv->fifos.tx_queue->in_can_transfer,
		&cpriv->fifos.tx_queue->transferred,
		fifo);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	/* but we need to run a bit of cleanup */
	cpriv->status.txif &= ~BIT(fifo);
	net->stats.tx_aborted_errors++;

	/* handle all the known cases accordingly - ignoring FIFO full */
	val &= CAN_FIFOSTA_TXABT |
		CAN_FIFOSTA_TXLARB |
		CAN_FIFOSTA_TXERR;
	switch (val) {
	case CAN_FIFOSTA_TXERR:
		/* this indicates a possible bus error */
		break;
	default:
		dev_warn_ratelimited(&spi->dev,
				     "Unknown TX-Fifo abort condition: %08x - stopping tx-queue\n",
				     val);
		break;
	}

	return 0;
}

int mcp25xxfd_can_int_handle_txatif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo;
	int ret;

	/* if txatif is unset, then there are no
	 * can frames that have been transmitted
	 * and need to get reingested into the network stack
	 */
	if (!cpriv->status.txatif)
		return 0;
	cpriv->stats.int_txat_count++;

	/* process all the fifos with that flag set */
	for (i = 0, fifo = cpriv->fifos.tx.start;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment) {
		if (cpriv->status.txatif & BIT(fifo)) {
			ret = mcp25xxfd_can_int_handle_txatif_fifo(spi,
								   fifo);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/* submit the fifo back to the network stack */
int mcp25xxfd_can_submit_tx_frame(struct spi_device *spi, int fifo)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_obj_tx *tx = (struct mcp25xxfd_obj_tx *)
		(cpriv->sram + cpriv->fifos.fifo_reg[fifo].offset);
	int dlc = (tx->flags & CAN_OBJ_FLAGS_DLC_MASK) >>
		CAN_OBJ_FLAGS_DLC_SHIFT;
	unsigned long flags;

	/* update counters */
	net->stats.tx_packets++;
	cpriv->fifos.tx.dlc_usage[dlc]++;
	priv->net->stats.tx_bytes += can_dlc2len(dlc);
	if (tx->flags & CAN_OBJ_FLAGS_FDF)
		cpriv->stats.tx_fd_count++;
	if (tx->flags & CAN_OBJ_FLAGS_BRS)
		cpriv->stats.tx_brs_count++;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* release the echo buffer */
	can_get_echo_skb(priv->net, fifo);

	/* move from in_can_transfer to transferred */
	mcp25xxfd_can_tx_queue_move_spi_message(
		&cpriv->fifos.tx_queue->in_can_transfer,
		&cpriv->fifos.tx_queue->transferred,
		fifo);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	return 0;
}

static
int mcp25xxfd_can_tef_read(struct spi_device *spi, int start, int count)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 tef_offset = start * cpriv->fifos.tef.size;
	struct mcp25xxfd_obj_tef *tef =
		(struct mcp25xxfd_obj_tef *)(cpriv->sram + tef_offset);
	int last, read, ret;

	/* compute how many we can read in one go */
	last = start + count;
	read = (last > cpriv->fifos.tef.count) ?
		(cpriv->fifos.tef.count - start) :
		count;

	/* and read it */
	ret = mcp25xxfd_cmd_read_regs(spi, MCP25XXFD_SRAM_ADDR(tef_offset),
				      &tef->id, sizeof(*tef) * read);
	if (ret)
		return ret;

	/* and read a second part on wrap */
	if (read != count) {
		/* update stats */
		cpriv->fifos.tx_queue->stats.tef_read_splits++;
		/* compute the  addresses  */
		read = count - read;
		tef = (struct mcp25xxfd_obj_tef *)(cpriv->sram);
		/* and read again */
		ret = mcp25xxfd_cmd_read_regs(spi,
					      MCP25XXFD_SRAM_ADDR(0),
					      &tef->id,
					      sizeof(*tef) * read);
	}

	return ret;
}

static int mcp25xxfd_can_int_handle_tefif_fifo(struct spi_device *spi,
					       bool read_data)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 tef_offset = cpriv->fifos.tef.index * cpriv->fifos.tef.size;
	struct mcp25xxfd_obj_tef *tef =
		(struct mcp25xxfd_obj_tef *)(cpriv->sram + tef_offset);
	int fifo, ret;
	unsigned long flags;

	/* read the next TEF entry to get the transmit timestamp and fifo */
	tef = (struct mcp25xxfd_obj_tef *)(cpriv->sram + tef_offset);
	if (read_data) {
		ret = mcp25xxfd_can_tef_read(spi, cpriv->fifos.tef.index, 1);
		if (ret)
			return ret;
	}

	/* get the fifo from tef */
	fifo = (tef->flags & CAN_OBJ_FLAGS_SEQ_MASK) >>
		CAN_OBJ_FLAGS_SEQ_SHIFT;

	/* check that the fifo is valid */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);
	if ((cpriv->fifos.tx_queue->in_can_transfer & BIT(fifo)) == 0)
		netdev_err(net,
			   "tefif: fifo %i not pending - tef data: id: %08x flags: %08x, ts: %08x - this may be a problem with spi signal quality- try reducing spi-clock speed if this can get reproduced",
			   fifo, tef->id, tef->flags, tef->ts);
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	/* update stats */
	cpriv->fifos.tx_queue->stats.tef_reads++;

	/* now we can schedule the fifo for echo submission */
	mcp25xxfd_can_queue_frame(cpriv, -fifo, tef->ts);

	/* increment the tef index with wraparround */
	cpriv->fifos.tef.index++;
	if (cpriv->fifos.tef.index >= cpriv->fifos.tef.count)
		cpriv->fifos.tef.index = 0;

	/* finally just increment the TEF pointer */
	return mcp25xxfd_cmd_write_mask(spi, CAN_TEFCON,
				 CAN_TEFCON_UINC,
				 CAN_TEFCON_UINC);
}

/* reading TEF entries can be made even more efficient by reading
 * multiple TEF entries in one go.
 * Under the assumption that we have count(TEF) >= count(TX_FIFO)
 * we can even release TEFs early (before we read them)
 * (and potentially restarting the transmit-queue early aswell)
 */

static
int mcp25xxfd_can_int_handle_tefif_conservative(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 tefsta;
	int ret;

	/* read the TEF status */
	ret = mcp25xxfd_cmd_read_mask(spi, CAN_TEFSTA,
				      &tefsta, CAN_TEFSTA_TEFNEIF);
	if (ret)
		return ret;

	/* read the tef in an inefficient loop */
	while (tefsta & CAN_TEFSTA_TEFNEIF) {
		/* read one tef */
		ret = mcp25xxfd_can_int_handle_tefif_fifo(spi, true);
		if (ret)
			return ret;

		/* update stats */
		cpriv->fifos.tx_queue->stats.tef_conservative_reads++;

		/* read the TEF status */
		ret = mcp25xxfd_cmd_read_mask(spi, CAN_TEFSTA,
					      &tefsta, CAN_TEFSTA_TEFNEIF);
		if (ret)
			return ret;
	}

	return 0;
}

static
int mcp25xxfd_can_int_handle_tefif_oportunistic(struct spi_device *spi,
						u32 finished)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo, count, ret;

	/* count the number of fifos that have terminated */
	for (i = 0, fifo = cpriv->fifos.tx.start, count = 0;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment)
		if (finished & BIT(fifo))
			count++;

	/* read them in one go if possible
	 * we also assume that we have count(TEF) >= count(TX-FIFOS)
	 * this may require 2 reads when we wrap arround
	 * (that is unless count(TEF) == count(TX-FIFOS))
	 */
	ret = mcp25xxfd_can_tef_read(spi, cpriv->fifos.tef.index, count);
	if (ret)
		return ret;

	/* update stats */
	cpriv->fifos.tx_queue->stats.tef_opportunistic_reads++;
	i = min_t(int, TEF_READ_STATS_BINS - 1, count - 1);
	cpriv->fifos.tx_queue->stats.tef_opportunistic_read_sizes[i]++;

	/* now iterate over all of them without reading again */
	for (i = 0, fifo = cpriv->fifos.tx.start;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment) {
		if (finished & BIT(fifo)) {
			ret = mcp25xxfd_can_int_handle_tefif_fifo(spi, false);
			if (ret)
				return ret;
		}
	}

	return 0;
}

int mcp25xxfd_can_int_handle_tefif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	unsigned long flags;
	u32 finished;

	if (!(cpriv->status.intf & CAN_INT_TEFIF))
		return 0;
	cpriv->stats.int_tef_count++;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* compute finished fifos and clear them immediately */
	finished = (cpriv->fifos.tx_queue->in_can_transfer ^
		    cpriv->status.txreq) &
		cpriv->fifos.tx_queue->in_can_transfer;

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	/* in case of a strange situation run in safe mode */
	if (!finished) {
		netdev_warn(net,
			    "Something is wrong - we got a TEF interrupt but we were not able to detect a finished fifo\n");
		return mcp25xxfd_can_int_handle_tefif_conservative(spi);
	}

	/* otherwise run in oportunistic mode */
	return mcp25xxfd_can_int_handle_tefif_oportunistic(spi, finished);
}

static
void mcp25xxfd_can_tx_fill_fifo_common(struct net_device *net,
				       struct mcp2517fd_tx_spi_message *smsg,
				       struct mcp25xxfd_obj_tx *tx,
				       int dlc, u8 *data)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int len = can_dlc2len(dlc);

	/* update statistics */
	cpriv->fifos.tx.dlc_usage[dlc]++;
	cpriv->fifos.fifo_info[smsg->fifo].use_count++;

	/* add fifo number as seq */
	tx->flags |= smsg->fifo << CAN_OBJ_FLAGS_SEQ_SHIFT;

	/* copy data to tx->data for future reference */
	memcpy(tx->data, data, len);

	/* transform header to controller format */
	mcp25xxfd_convert_from_cpu(&tx->id, sizeof(*tx) / sizeof(u32));

	/* copy header + data to final location - we are not aligned */
	memcpy(smsg->fill_fifo.data.header, &tx->id, sizeof(*tx) + len);

	/* convert it back to CPU format */
	mcp25xxfd_convert_to_cpu(&tx->id, sizeof(*tx) / sizeof(u32));

	/* set up size of transfer */
	smsg->fill_fifo.xfer.len = sizeof(smsg->fill_fifo.data.cmd) +
		sizeof(smsg->fill_fifo.data.header) + len;
}

static
void mcp25xxfd_can_tx_fill_fifo_fd(struct net_device *net,
				   struct canfd_frame *frame,
				   struct mcp2517fd_tx_spi_message *smsg,
				   struct mcp25xxfd_obj_tx *tx)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int dlc = can_len2dlc(frame->len);

	/* update some statistics */
	cpriv->stats.tx_fd_count++;

	/* compute can id */
	mcp25xxfd_canid_to_mcpid(frame->can_id, &tx->id, &tx->flags);

	/* setup flags */
	tx->flags |= dlc << CAN_OBJ_FLAGS_DLC_SHIFT;
	tx->flags |= (frame->can_id & CAN_EFF_FLAG) ? CAN_OBJ_FLAGS_IDE : 0;
	tx->flags |= (frame->can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0;
	if (frame->flags & CANFD_BRS) {
		tx->flags |= CAN_OBJ_FLAGS_BRS;
		cpriv->stats.tx_brs_count++;
	}
	tx->flags |= (frame->flags & CANFD_ESI) ? CAN_OBJ_FLAGS_ESI : 0;
	tx->flags |= CAN_OBJ_FLAGS_FDF;

	/* and do common processing */
	mcp25xxfd_can_tx_fill_fifo_common(net, smsg, tx,
					  dlc, frame->data);
}

static
void mcp25xxfd_can_tx_fill_fifo(struct net_device *net,
				struct can_frame *frame,
				struct mcp2517fd_tx_spi_message *smsg,
				struct mcp25xxfd_obj_tx *tx)
{
	/* set frame to valid dlc */
	if (frame->can_dlc > 8)
		frame->can_dlc = 8;

	/* compute can id */
	mcp25xxfd_canid_to_mcpid(frame->can_id, &tx->id, &tx->flags);

	/* setup flags */
	tx->flags |= frame->can_dlc << CAN_OBJ_FLAGS_DLC_SHIFT;
	tx->flags |= (frame->can_id & CAN_EFF_FLAG) ? CAN_OBJ_FLAGS_IDE : 0;
	tx->flags |= (frame->can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0;

	/* and do common processing */
	mcp25xxfd_can_tx_fill_fifo_common(net, smsg, tx,
					  frame->can_dlc, frame->data);
}

static struct mcp2517fd_tx_spi_message
*mcp25xxfd_can_tx_queue_get_next_fifo(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp2517fd_tx_spi_message *smsg;
	unsigned long flags;

	/* we need to hold this lock to protect us against
	 * concurrent modifications of cpriv->fifos.tx_queue->idle
	 * in the interrupt thread
	 */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* get the first entry from idle */
	if (cpriv->fifos.tx.increment > 0)
		smsg = mcp25xxfd_can_tx_queue_first_spi_message(
			cpriv->fifos.tx_queue, &cpriv->fifos.tx_queue->idle);
	else
		smsg = mcp25xxfd_can_tx_queue_last_spi_message(
			cpriv->fifos.tx_queue, &cpriv->fifos.tx_queue->idle);
	if (!smsg)
		goto out_busy;

	/* and move the fifo to next stage */
	mcp25xxfd_can_tx_queue_move_spi_message(
		&cpriv->fifos.tx_queue->idle,
		&cpriv->fifos.tx_queue->in_fill_fifo_transfer,
		smsg->fifo);

	/* if queue is empty then stop the network queue immediately */
	if (!cpriv->fifos.tx_queue->idle)
		mcp25xxfd_can_tx_queue_manage_nolock(net,
						     TX_QUEUE_STATE_RUNABLE);
out_busy:
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	return smsg;
}

/* submit the can message to the can-bus */
netdev_tx_t mcp25xxfd_can_start_xmit(struct sk_buff *skb,
				     struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	struct mcp2517fd_tx_spi_message *smsg;
	struct mcp25xxfd_obj_tx *tx;
	int addr;
	unsigned long flags;
	int ret;

	/* invalid skb we can ignore */
	if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	/* acquire lock on spi so that we are are not risking
	 * some reordering of spi messages when we are running
	 * start_xmit in multiple threads (on multiple cores)
	 */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->spi_lock, flags);

	/* get the fifo message structure to process now */
	smsg = mcp25xxfd_can_tx_queue_get_next_fifo(net);
	if (!smsg)
		goto out_busy;

	/* compute the fifo in sram */
	addr = cpriv->fifos.fifo_reg[smsg->fifo].offset;
	tx = (struct mcp25xxfd_obj_tx *)(cpriv->sram + addr);

	/* fill in message from skb->data depending on can2.0 or canfd */
	if (can_is_canfd_skb(skb))
		mcp25xxfd_can_tx_fill_fifo_fd(net,
					      (struct canfd_frame *)skb->data,
					      smsg, tx);
	else
		mcp25xxfd_can_tx_fill_fifo(net,
					   (struct can_frame *)skb->data,
					   smsg, tx);

	/* submit the two messages asyncronously
	 * the reason why we separate transfers into two spi_messages is:
	 *  * because the spi framework (currently) does add a 10us delay
	 *    between 2 spi_transfers in a single spi_message when
	 *    change_cs is set - 2 consecutive spi messages show a shorter
	 *     cs disable phase increasing bus utilization
	 *  * this allows the interrupt handler to start spi messages earlier
	 *    so reducing latencies a bit and to allow for better concurrency
	 */
	ret = spi_async(spi, &smsg->fill_fifo.msg);
	if (ret)
		goto out_async_failed;
	ret = spi_async(spi, &smsg->trigger_fifo.msg);
	if (ret)
		goto out_async_failed;

	/* unlock the spi bus */
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->spi_lock, flags);

	/* keep it for reference until the message really got transmitted */
	can_put_echo_skb(skb, net, smsg->fifo);

	return NETDEV_TX_OK;
out_async_failed:
	netdev_err(net, "spi_async submission of fifo %i failed - %i\n",
		   smsg->fifo, ret);

out_busy:
	/* stop the queue */
	mcp25xxfd_can_tx_queue_manage_nolock(net, TX_QUEUE_STATE_STOPPED);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->spi_lock, flags);

	return NETDEV_TX_BUSY;
}
