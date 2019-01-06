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

unsigned int rx_prefetch_bytes = -1;
module_param(rx_prefetch_bytes, uint, 0664);
MODULE_PARM_DESC(rx_prefetch_bytes,
		 "number of bytes to blindly prefetch when reading a rx-fifo");

#if defined(CONFIG_DEBUG_FS)
void mcp25xxfd_can_rx_fifo_debugfs(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct dentry *dir = cpriv->fifos.rx.debugfs_dir;
	char name[32];
	int i;

	/* add statistics on tef */
	debugfs_create_u64("rx_reads", 0444, dir,
			   &cpriv->stats.rx_reads);
	debugfs_create_u64("rx_single_reads", 0444, dir,
			   &cpriv->stats.rx_single_reads);
	debugfs_create_u64("rx_bulk_reads", 0444, dir,
			   &cpriv->stats.rx_bulk_reads);
	debugfs_create_u64("rx_reads_prefetched_too_few", 0444, dir,
			   &cpriv->stats.rx_reads_prefetched_too_few);
	debugfs_create_u64("rx_reads_prefetched_too_few_bytes", 0444, dir,
			   &cpriv->stats.rx_reads_prefetched_too_few_bytes);
	debugfs_create_u64("rx_reads_prefetched_too_many", 0444, dir,
			   &cpriv->stats.rx_reads_prefetched_too_many);
	debugfs_create_u64("rx_reads_prefetched_too_many_bytes", 0444, dir,
			   &cpriv->stats.rx_reads_prefetched_too_many_bytes);
	debugfs_create_u32("rx_reads_prefetch_predicted", 0444, dir,
			   &cpriv->stats.rx_reads_prefetch_predicted);

	/* present the fifos */
	for (i = 0; i < RX_BULK_READ_STATS_BINS - 2; i++) {
		snprintf(name, sizeof(name),
			 "rx_bulk_reads_%i", i + 1);
		debugfs_create_u64(name, 0444, dir,
				   &cpriv->stats.rx_bulk_read_sizes[i]);
	}

	/* i is already set correctly in the loop above */
	snprintf(name, sizeof(name), "rx_bulk_reads_%i+", i + 1);
	debugfs_create_u64(name, 0444, dir,
			   &cpriv->stats.rx_bulk_read_sizes[i]);
}
#else
void mcp25xxfd_can_rx_fifo_debugfs(struct net_device *net)
{
}
#endif

static int mcp25xxfd_can_rx_predict_prefetch(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int dlc, i, highest;
	u8 histo[16];

	/* if we have a prfecth set then use that one */
	if (rx_prefetch_bytes != -1)
		return min_t(int, rx_prefetch_bytes,
			     (net->mtu == CANFD_MTU) ? 64 : 8);

	/* memset */
	memset(histo, 0, sizeof(histo));

	/* for all others compute the histogram */
	for (i = 0; i < RX_HISTORY_SIZE; i++)
		histo[cpriv->stats.rx_history_dlc[i]]++;

	/* and now find the highest fit */
	for (i = (net->mtu == CANFD_MTU) ? 15 : 8, dlc = 8, highest = 0;
	      i >= 0; i--) {
		if (highest < histo[i]) {
			highest = histo[i];
			dlc = i;
		}
	}

	/* compute length from dlc */
	cpriv->stats.rx_reads_prefetch_predicted = can_dlc2len(dlc);

	/* return the predicted length */
	return cpriv->stats.rx_reads_prefetch_predicted;
}

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

	/* add to rx_dlc_history */
	cpriv->stats.rx_history_dlc[cpriv->stats.rx_history_index] = dlc;
	cpriv->stats.rx_history_index++;
	if (cpriv->stats.rx_history_index >= RX_HISTORY_SIZE)
		cpriv->stats.rx_history_index = 0;

	/* allocate the skb buffer */
	if (rx->flags & CAN_OBJ_FLAGS_FDF) {
		flags = 0;
		flags |= (rx->flags & CAN_OBJ_FLAGS_BRS) ? CANFD_BRS : 0;
		flags |= (rx->flags & CAN_OBJ_FLAGS_ESI) ? CANFD_ESI : 0;
		skb = mcp25xxfd_can_submit_rx_fd_frame(net, id, flags,
						       len, &data);
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
				       int fifo, int prefetch_bytes,
				       bool read_data)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int addr = cpriv->fifos.fifo_reg[fifo].offset;
	struct mcp25xxfd_obj_rx *rx =
		(struct mcp25xxfd_obj_rx *)(cpriv->sram + addr);
	int dlc;
	int len, ret;

	/* we read the header plus prefetch_bytes */
	if (read_data) {
		cpriv->stats.rx_single_reads++;
		ret = mcp25xxfd_cmd_readn(spi, MCP25XXFD_SRAM_ADDR(addr),
					  rx, sizeof(*rx) + prefetch_bytes);
		if (ret)
			return ret;
	}

	/* transpose the headers to CPU format*/
	rx->id = le32_to_cpu(rx->id);
	rx->flags = le32_to_cpu(rx->flags);
	rx->ts = le32_to_cpu(rx->ts);

	/* compute len */
	dlc = (rx->flags & CAN_OBJ_FLAGS_DLC_MASK) >>
		CAN_OBJ_FLAGS_DLC_SHIFT;
	len = can_dlc2len(min_t(int, dlc, (net->mtu == CANFD_MTU) ? 15 : 8));

	/* read the remaining data for canfd frames */
	if (read_data && len > prefetch_bytes) {
		/* update stats */
		cpriv->stats.rx_reads_prefetched_too_few++;
		cpriv->stats.rx_reads_prefetched_too_few_bytes +=
			len - prefetch_bytes;
		/* here the extra portion reading data after prefetch */
		ret = mcp25xxfd_cmd_readn(spi,
					  MCP25XXFD_SRAM_ADDR(addr) +
					  sizeof(*rx) + prefetch_bytes,
					  &rx->data[prefetch_bytes],
					  len - prefetch_bytes);
		if (ret)
			return ret;
	}

	/* update stats */
	cpriv->stats.rx_reads++;
	if (len < prefetch_bytes) {
		cpriv->stats.rx_reads_prefetched_too_many++;
		cpriv->stats.rx_reads_prefetched_too_many_bytes +=
			prefetch_bytes - len;
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

static int mcp25xxfd_can_read_rx_frame_bulk(struct spi_device *spi,
					    int fifo_start,
					    int fifo_end)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int count = abs(fifo_end - fifo_start) + 1;
	int fifo_lowest = min_t(int, fifo_start, fifo_end);
	int addr = cpriv->fifos.fifo_reg[fifo_lowest].offset;
	struct mcp25xxfd_obj_rx *rx =
		(struct mcp25xxfd_obj_rx *)(cpriv->sram + addr);
	int len = (sizeof(*rx) + ((net->mtu == CANFD_MTU) ? 64 : 8)) * count;
	int fifo, i, ret;

	/* update stats */
	cpriv->stats.rx_bulk_reads++;
	i = min_t(int, RX_BULK_READ_STATS_BINS - 1, count - 1);
	cpriv->stats.rx_bulk_read_sizes[i]++;

	/* we read the header plus read_min data bytes */
	ret = mcp25xxfd_cmd_readn(spi, MCP25XXFD_SRAM_ADDR(addr),
				  rx, len);
	if (ret)
		return ret;

	/* now process all of them - no need to read... */
	for (fifo = fifo_start; count > 0;
	     fifo += cpriv->fifos.rx.increment, count--) {
		ret = mcp25xxfd_can_read_rx_frame(spi, fifo, 8, false);
		if (ret)
			return ret;
	}

	return 0;
}

/* at least in can2.0 mode we can read multiple RX-fifos in one go
 * in case they are ajactent to each other and thus we can reduce
 * the number of spi messages produced and this improves spi-bus
 * usage efficiency.
 * In canFD mode this may also be possible, but would need some
 * statistics to decide if it is worth reading a full 64 bytes
 * in one go.
 * But those statistics can get used to predict how many bytes
 * to read together with the can header (which is fixed to 8 at
 * this very moment.
 *
 * notes on the rational here:
 * * Reading just the CAN header info takes:
 *   * bytes read
 *     *  2 bytes command+address
 *     * 12 bytes data (id, flags, timestamp)
 *   * so that is at the very least 112 SCK (= 14 byte * 8 SCK/1 byte)
 *     - on a Raspberry pi 3 for such short requests actually
 *       126 SCK (=14 byte * 9 SCK/1 byte)
 *   * some SPI framework overhead which is observed to be 5-10 us
 *     on a raspberry pi 3 (time between SCK and stop SCK start)
 *   * with an effective 17.85 MHz SPI clock on a RPI it takes in total:
 *     it takes 12us = 6us + 6us
 * * now reading 8 bytes of CAN data (can2.0) takes:
 *   * bytes read
 *     *  2 bytes command+address
 *     *  8 bytes data
 *   * so that is at the very least 80 SCK (= 10 byte * 8 SCK/1 byte)
 *     - on a Raspberry pi 3 for such short requests actually
 *       90 SCK (= 10 byte * 9 SCK/1 byte)
 *   * some SPI framework overhead which is observed to be 5-10 us
 *     on a raspberry pi 3 (time between SCK and stop SCK start)
 *   * with an effective 17.85 MHz SPI clock on a RPI it takes in total:
 *     it takes 11us = 5.0us + 6us
 * * now reading CAN header plus 8 bytes of CAN data (can2.0) takes:
 *   * bytes read
 *     *  2 bytes command+address
 *     * 20 bytes data
 *   * so that is at the very least 176 SCK (= 22 byte * 8 SCK/1 byte)
 *     - on a Raspberry pi 3 for such short requests actually
 *       198 SCK (= 22 byte * 9 SCK/1 byte)
 *   * some SPI framework overhead which is observed to be 5-10 us
 *     on a raspberry pi 3 (time between SCK and stop SCK start)
 *   * with an effective 17.85 MHz SPI clock on a RPI it takes in total:
 *     it takes 17.1us = 11.1us + 6us
 *   * this is faster than the 2 individual SPI transfers for header
 *     and data which is in total 23us
 *     * this is even true for the case where we only have a single
 *       data byte (DLC=1) - the time here is 19.5us on a RPI3
 *     * the only time where we are less efficient is for the DLC=0 case.
 *       but the assumption here is that this is a rare case
 * To put it into perspective here the full table for a RPI3:
 * LE 2m  pr0 pr1 pr2 pr3 pr4 pr5  pr6  pr7  pr8 pr12 pr16 pr20 pr24 pr32 pr48
 *                                                                         pr64
 *  0  7.1 7.1
 *  1 14.6    7.6 8.1 8.6 9.1 9.6 10.1 10.6 11.1 13.1
 *  2 15.1        8.1 8.6 9.1 9.6 10.1 10.6 11.1 13.1
 *  3 15.6            8.6 9.1 9.6 10.1 10.6 11.1 13.1 15.1
 *  4 16.1                9.1 9.6 10.1 10.6 11.1 13.1 15.1
 *  5 16.6                    9.6 10.1 10.6 11.1 13.1 15.1
 *  6 17.1                        10.1 10.6 11.1 13.1 15.1
 *  7 17.6                             10.6 11.1 13.1 15.1 17.1
 *  8 18.1                                  11.1 13.1 15.1 17.1
 * 12 20.1                                       13.1 15.1 17.1 19.2
 * 16 22.1                                            15.1 17.1 19.2
 * 20 24.1                                                 17.1 19.2 23.2
 * 24 26.2                                                      19.2 23.2
 * 32 30.2                                                           23.2
 * 48 38.3                                                                31.3
 * 64 46.3                                                                 39.3
 * (Parameters: SPI Clock=17.8MHz, SCK/byte=9, overhead=6us)
 * Legend:
 *   LE = length,
 *   2m    = 2 SPI messages (header+data - except for LEN=0, only header)
 *  prX/pX = prefecth length times (only shown when < 2m and Len >= Prefetch)
 *
 * The diagonal schows the "optimal" time when the size of the Can frame would
 * be known ahead of time - i.e if it would be possible to define RX reception
 * filters based on can DLC values
 *
 * So for any Can frame except for LEN=0 the prefetch data solution is
 * better for prefetch of data=12 for CanFD.
 *
 * Here another table showing the optimal prefetch limits for SPI speeds
 * vs overhead_us at 8 or 9 SCLK/byte
 *
 * MHZ  2us@8   2us@9   4us@8   4us@9   6us@8   6us@9   8us@8   8us@9
 * 10.0 8b***   8b***   8b      8b*     12b**   8b*     12b     12b*
 * 12.5 8b**    8b***   12b***  8b      12b     12b*    16b*    16b**
 * 15.0 8b**    8b**    12b**   12b***  16b**   12b     20b**   16b
 * 17.5 8b*     8b*     12b*    12b**   16b     16b**   20b     20b**
 * 20.0 8b      8b*     16b***  12b*    20b**   16b     24b*    20b
 * (a * signifies not a full match, but for any length > count(*))
 *
 * So 8 bytes prefetch seems to be a very good tradeoff for can frame
 * except for DLC/LEN=0 frames.
 * The question here is mainly: how many frames do we have with DLC=0
 * vs all others.
 *
 * With some statistics of recent CAN frames this may be set dynamically
 * in the future.
 *
 * For this to work efficiently we would also need an estimate on
 * the SPI framework overhead, which is a function of the spi-bus-driver
 * implementation details, CPU type and speed as well as system load.
 * Also the effective SPI-clock speed is needed as well as the
 * number of spi clock cycles it takes for a single byte to get transferred
 * The bcm283x SOC for example pauses the SPI clock one cycle after
 * every byte it sends unless the data is fed to the controller by DMA.
 * (but for short transfers DMA mapping is very expensive and not worth
 * the effort. PIO and - in some situations - polling is used instead to
 * reduce the number of interrupts and the need for thread scheduling as
 * much as possible)
 *
 * This also means that for can2.0 only configured interfaces
 * reading multiple rx fifos is a realistic option of optimization
 */

static int mcp25xxfd_can_read_rx_frames_fd(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo, prefetch;
	int ret;

	/* calculate optimal prefetch to use */
	prefetch = mcp25xxfd_can_rx_predict_prefetch(net);

	/* loop all frames */
	for (i = 0, fifo = cpriv->fifos.rx.start;
	     i < cpriv->fifos.rx.count;
	     i++, fifo += cpriv->fifos.rx.increment) {
		if (cpriv->status.rxif & BIT(fifo)) {
			/* read the frame */
			ret = mcp25xxfd_can_read_rx_frame(spi, fifo,
							  prefetch, true);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/* right now we only optimize for sd (can2.0) frame case,
 * but in principle it could be also be valuable for CANFD
 * frames when we receive lots of 64 byte packets with BRS set
 * and a big difference between nominal and data bitrates
 */
static
int mcp25xxfd_can_read_rx_frames_sd(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo_start, fifo_end;
	int ret;

	/* iterate over fifos trying to find fifos next to each other */
	for (i = 0,
	     fifo_start = cpriv->fifos.rx.start,
	     fifo_end = fifo_start;
	     i < cpriv->fifos.rx.count;
	     i++,
	     fifo_end += cpriv->fifos.rx.increment,
	     fifo_start = fifo_end
	    ) {
		/* if bit is not set then continue */
		if (!(cpriv->status.rxif & BIT(fifo_start)))
			continue;
		/* find the last fifo with a bit set in sequence */
		for (fifo_end = fifo_start;
		     cpriv->status.rxif &
			     BIT(fifo_end + cpriv->fifos.rx.increment);
		     fifo_end += + cpriv->fifos.rx.increment)
			;
		/* and now read those fifos in bulk */
		ret = mcp25xxfd_can_read_rx_frame_bulk(spi,
						       fifo_start,
						       fifo_end);
		if (ret)
			return ret;
	}

	return 0;
}

int mcp25xxfd_can_read_rx_frames(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	if (net->mtu == CANFD_MTU)
		return mcp25xxfd_can_read_rx_frames_fd(spi);
	else
		return mcp25xxfd_can_read_rx_frames_sd(spi);
}
