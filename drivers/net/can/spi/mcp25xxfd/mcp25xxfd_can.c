// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

/* controller details
 *
 *  It has 32 FIFOs (of up to 32 CAN-frames).
 *
 * There are 4 Fifo types which can get configured:
 * * TEF - Transmission Event Fifo - which consumes FIFO 0
 *   even if it is not configured
 * * Tansmission Queue - for up to 32 Frames.
 *   this queue reorders CAN frames to get transmitted following the
 *   typical CAN dominant/recessive rules on the can bus itself.
 *   This FIFO is optional.
 * * TX FIFO: generic TX fifos that can contain arbitrary data
 *   and which come with a configurable priority for transmission
 *   It is also possible to have the Controller automatically trigger
 *   a transfer when a Filter Rule for a RTR frame matches.
 *   Each of these fifos in principle can get configured for distinct
 *   dlc sizes (8 thru 64 bytes)
 * * RX FIFO: generic RX fifo which is filled via filter-rules.
 *   Each of these fifos in principle can get configured for distinct
 *   dlc sizes (8 thru 64 bytes)
 *   Unfortunately there is no filter rule that would allow triggering
 *   on different frame sizes, so for all practical purposes the
 *   RX fifos have to be of the same size (unless one wants to experience
 *   lost data).
 * When a Can Frame is transmitted from the TX Queue or an individual
 * TX FIFO then a small TEF Frame can get added to the TEF FIFO queue
 * to log the Transmission of the frame - this includes ID, Flags
 * (including a custom identifier/index) and the timestamp (see below).
 *
 * The controller provides an optional free running counter with a divider
 * for timestamping of RX frames as well as for TEF entries.
 */

/* Implementation notes:
 *
 * * the can related portion of the driver is split out into
 *   * basic configuration (mcp25xxfd_can.c)
 *   * can fifo configuration (mcp25xxfd_can_fifo.c)
 *   * can interrupt handling (mcp25xxfd_can_int.c)
 *   * can reception including interrupt handling (mcp25xxfd_can_rx.c)
 *   * can transmission including interrupt handling (mcp25xxfd_can_tx.c)
 * * the interrupt handler trys to issue the smallest amount of
 *   spi messages to avoid spi stack overhead. for that it is more agressive
 *   when reading registers that are close to each other - e.g:
 *   * if there is one register that is not really needed then it will
 *     still read it to avoid the startup and teardown costs of the
 *     spi stack which results in latencies.
 *   * when in can2.0 mode it will read all 8 data bytes even if DLC = 0
 *     the header on its own is 12 bytes and then there are 2 bytes for
 *     the spi overhead so the effecitve worsted case overhead introduced
 *     is +57%
 * * due to the reordering inside the controller the Transmission queue
 *   feature is not used (it also makes the "next" frame freed less
 *   predictable which would complicate the code as well.
 *   the only advantage seen is that one could have more transmission
 *   slots in the case of CAN2.0 only.
 * * transmissions are instead handled by fixed priority fifos that can
 *   get filled and triggered asyncronously.
 *   The biggest drawback is that we are unable to sustain 100% CAN bus
 *   usage for more than the <number of can fifos allocated> frames.
 *   This means that we can have a duty cycle of about 7/8th of the CAN bus
 *   (unless more fifos are reserved for transmission)
 *   The only situation where this would be observable in real live
 *   would be big transfers (for firmware updates or similar)
 *   * this could have been better supported with a better
 *     layout of FIFOCON, so that TXREQ and TXPRIO can be written
 *     in a single transfer without a race condition
 * * can transmissions are handled by using spi_async for submission and
 *   scheduling. This somewhat impacts interrupt handling as some spi_sync
 *   calls will defer to the spi kthread resulting in some context switches.
 *   which introduces some latencies.
 * * some special features of the can controller can only get configured
 *   by the use of module parameters (some of which are changeable at
 *   runtime via sysfs) due to the fact that netlink does not support:
 *   * 3-shot submission
 *   * selection of number of fifos
 *   * configuarble delay between transmission of two can frames so that
 *     the can bus is not monopolized by this device.
 *   also some debug settings are controlable via modules.
 */

#include "mcp25xxfd_can.h"
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

/* module parameters */
unsigned int bw_sharing_log2bits;
module_param(bw_sharing_log2bits, uint, 0664);
MODULE_PARM_DESC(bw_sharing_log2bits,
		 "Delay between 2 transmissions in number of arbitration bit times\n");
bool enable_edge_filter;
module_param(enable_edge_filter, bool, 0664);
MODULE_PARM_DESC(enable_edge_filter,
		 "Enable ISO11898-1:2015 edge_filtering");
unsigned int tdc_mode = 2;
module_param(tdc_mode, uint, 0664);
MODULE_PARM_DESC(tdc_mode,
		 "Transmitter Delay Mode - 0 = disabled, 1 = fixed, 2 = auto\n");
unsigned int tdc_value;
module_param(tdc_value, uint, 0664);
MODULE_PARM_DESC(tdc_value,
		 "Transmission Delay Value - range: [0:63] SCLK");
int tdc_offset = 64; /* outside of range to use computed values */
module_param(tdc_offset, int, 0664);
MODULE_PARM_DESC(tdc_offset,
		 "Transmission Delay offset - range: [-64:63] SCLK");

/* everything related to bit timing */
static
const struct can_bittiming_const mcp25xxfd_can_nominal_bittiming_const = {
	.name           = DEVICE_NAME,
	.tseg1_min      = 2,
	.tseg1_max      = BIT(CAN_NBTCFG_TSEG1_BITS),
	.tseg2_min      = 1,
	.tseg2_max      = BIT(CAN_NBTCFG_TSEG2_BITS),
	.sjw_max        = BIT(CAN_NBTCFG_SJW_BITS),
	.brp_min        = 1,
	.brp_max        = BIT(CAN_NBTCFG_BRP_BITS),
	.brp_inc        = 1,
};

static
const struct can_bittiming_const mcp25xxfd_can_data_bittiming_const = {
	.name           = DEVICE_NAME,
	.tseg1_min      = 1,
	.tseg1_max      = BIT(CAN_DBTCFG_TSEG1_BITS),
	.tseg2_min      = 1,
	.tseg2_max      = BIT(CAN_DBTCFG_TSEG2_BITS),
	.sjw_max        = BIT(CAN_DBTCFG_SJW_BITS),
	.brp_min        = 1,
	.brp_max        = BIT(CAN_DBTCFG_BRP_BITS),
	.brp_inc        = 1,
};

static int mcp25xxfd_can_do_set_nominal_bittiming(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct can_bittiming *bt = &cpriv->can.bittiming;
	struct spi_device *spi = priv->spi;

	int sjw = bt->sjw;
	int pseg2 = bt->phase_seg2;
	int pseg1 = bt->phase_seg1;
	int propseg = bt->prop_seg;
	int brp = bt->brp;

	int tseg1 = propseg + pseg1;
	int tseg2 = pseg2;

	/* calculate nominal bit timing */
	cpriv->regs.nbtcfg = ((sjw - 1) << CAN_NBTCFG_SJW_SHIFT) |
		((tseg2 - 1) << CAN_NBTCFG_TSEG2_SHIFT) |
		((tseg1 - 1) << CAN_NBTCFG_TSEG1_SHIFT) |
		((brp - 1) << CAN_NBTCFG_BRP_SHIFT);

	return mcp25xxfd_cmd_write(spi, CAN_NBTCFG,
				   cpriv->regs.nbtcfg);
}

static int mcp25xxfd_can_do_set_data_bittiming(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct can_bittiming *bt = &cpriv->can.data_bittiming;
	struct spi_device *spi = priv->spi;

	int sjw = bt->sjw;
	int pseg2 = bt->phase_seg2;
	int pseg1 = bt->phase_seg1;
	int propseg = bt->prop_seg;
	int brp = bt->brp;

	int tseg1 = propseg + pseg1;
	int tseg2 = pseg2;

	int tdco;
	int ret;

	/* set up Transmitter delay compensation */
	cpriv->regs.tdc = 0;
	/* configure TDC mode */
	if (tdc_mode < 4)
		cpriv->regs.tdc = tdc_mode << CAN_TDC_TDCMOD_SHIFT;
	else
		cpriv->regs.tdc = CAN_TDC_TDCMOD_AUTO << CAN_TDC_TDCMOD_SHIFT;

	/* configure TDC offsets */
	if ((tdc_offset >= -64) && tdc_offset < 64)
		tdco = tdc_offset;
	else
		tdco = clamp_t(int, bt->brp * tseg1, -64, 63);
	cpriv->regs.tdc |= (tdco << CAN_TDC_TDCO_SHIFT) &
		CAN_TDC_TDCO_MASK;

	/* configure TDC value */
	if (tdc_value < 64)
		cpriv->regs.tdc |= tdc_value << CAN_TDC_TDCV_SHIFT;

	/* enable edge filtering */
	if (enable_edge_filter)
		cpriv->regs.tdc |= CAN_TDC_EDGFLTEN;

	/* set TDC */
	ret = mcp25xxfd_cmd_write(spi, CAN_TDC, cpriv->regs.tdc);
	if (ret)
		return ret;

	/* calculate data bit timing */
	cpriv->regs.dbtcfg = ((sjw - 1) << CAN_DBTCFG_SJW_SHIFT) |
		((tseg2 - 1) << CAN_DBTCFG_TSEG2_SHIFT) |
		((tseg1 - 1) << CAN_DBTCFG_TSEG1_SHIFT) |
		((brp - 1) << CAN_DBTCFG_BRP_SHIFT);

	return mcp25xxfd_cmd_write(spi, CAN_DBTCFG,
				   cpriv->regs.dbtcfg);
}

/* configuration functions that are used by the base during initialization */
int mcp25xxfd_can_get_mode(struct spi_device *spi, u32 *mode_data)
{
	int ret;

	ret = mcp25xxfd_cmd_read(spi, CAN_CON, mode_data);
	if (ret)
		return ret;

	return (*mode_data & CAN_CON_OPMOD_MASK) >> CAN_CON_OPMOD_SHIFT;
}

int mcp25xxfd_can_switch_mode_nowait(struct spi_device *spi,
				     u32 *mode_data, int mode)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int ret;

	/* get the current mode/register - if mode_data is -1
	 * this only happens during initialization phase
	 * when the can controller is not setup yet
	 * typically by calling mcp25xxfd_can_sleep_mode
	 */
	if (*mode_data == -1) {
		ret = mcp25xxfd_can_get_mode(spi, mode_data);
		if (ret < 0)
			return ret;
	}

	/* compute the effective mode in osc*/
	*mode_data &= ~(CAN_CON_REQOP_MASK | CAN_CON_OPMOD_MASK);
	*mode_data |= (mode << CAN_CON_REQOP_SHIFT) |
		(mode << CAN_CON_OPMOD_SHIFT);

	/* if the opmode is sleep then the oscilator will be disabled
	 * and also not ready, so fake this change
	 */
	if (mode == CAN_CON_MODE_SLEEP) {
		priv->regs.osc &= ~(MCP25XXFD_OSC_OSCRDY |
				    MCP25XXFD_OSC_PLLRDY |
				    MCP25XXFD_OSC_SCLKRDY);
		priv->regs.osc |= MCP25XXFD_OSC_OSCDIS;
	}

	/* request the mode switch */
	return mcp25xxfd_cmd_write(spi, CAN_CON, *mode_data);
}

int mcp25xxfd_can_switch_mode(struct spi_device *spi,
			      u32 *mode_data, int mode)
{
	int ret, i;

	/* trigger the mode switch itself */
	ret = mcp25xxfd_can_switch_mode_nowait(spi, mode_data, mode);
	if (ret)
		return ret;

	/* if we are in sleep mode then return immediately
	 * the controller does not respond back!
	 */
	if (mode == CAN_CON_MODE_SLEEP)
		return 0;

	/* wait for it to stabilize/switch mode
	 * we assume 256 rounds should be enough as this is > 12ms
	 * at 1MHz Can Bus speed without any extra overhead
	 *
	 * The assumption here is that it depends on bus activity
	 * how long it takes the controller to switch modes
	 */
	for (i = 0; i < 256; i++) {
		/* get the mode */
		ret = mcp25xxfd_can_get_mode(spi, mode_data);
		if (ret < 0)
			return ret;
		/* check that we have reached our mode */
		if (ret == mode)
			return 0;
	}

	dev_err(&spi->dev, "Failed to switch to mode %u in time\n", mode);
	return -ETIMEDOUT;
}

int mcp25xxfd_can_sleep_mode(struct spi_device *spi)
{
	u32 value = -1;

	return mcp25xxfd_can_switch_mode(spi, &value, CAN_CON_MODE_SLEEP);
}

int mcp25xxfd_can_hw_probe_modeswitch(struct spi_device *spi)
{
	u32 mode_data;
	int ret;

	/* so we should be in config mode now, so move to INTERNAL_LOOPBACK */
	ret = mcp25xxfd_can_switch_mode(spi, &mode_data,
					CAN_CON_MODE_INTERNAL_LOOPBACK);
	if (ret) {
		dev_err(&spi->dev,
			"Failed to switch into loopback mode\n");
		return ret;
	}

	/* and back into config mode */
	ret = mcp25xxfd_can_switch_mode(spi, &mode_data,
					CAN_CON_MODE_CONFIG);
	if (ret) {
		dev_err(&spi->dev,
			"Failed to switch back to config mode\n");
		return ret;
	}

	/* so we have checked basic functionality successfully */
	return 0;
}

int mcp25xxfd_can_hw_probe(struct spi_device *spi)
{
	u32 mode_data;
	int mode, ret;

	/* read TXQCON - the TXEN bit should always read as 1 */
	ret = mcp25xxfd_cmd_read(spi, CAN_TXQCON, &mode_data);
	if (ret)
		return ret;
	if ((mode_data & CAN_TXQCON_TXEN) == 0) {
		dev_err(&spi->dev,
			"Register TXQCON does not have bit TXEN set - reads as %08x - this may be a problem with spi bus signal quality - try reducing spi-clock speed if this can get reproduced",
			mode_data);
		return -EINVAL;
	}

	/* try to get the current mode */
	mode = mcp25xxfd_can_get_mode(spi, &mode_data);
	if (mode < 0)
		return mode;

	/* we would expect to be in config mode, as a SPI-reset should
	 * have moved us into config mode.
	 * But then the documentation says that SPI-reset may only work
	 * reliably when already in config mode
	 */

	/* so if we are in config mode then everything is fine
	 * and we check that a mode switch works propperly
	 */
	if (mode == CAN_CON_MODE_CONFIG)
		return mcp25xxfd_can_hw_probe_modeswitch(spi);

	/* if the bitfield is 0 then there is something is wrong */
	if (!mode_data) {
		dev_err(&spi->dev,
			"got controller config register reading as 0\n");
		return -EINVAL;
	}

	/* any other mode is unexpected */
	dev_err(&spi->dev,
		"Found controller in unexpected mode %i - register reads as %08x\n",
		mode, mode_data);

	/* so try to move to config mode
	 * if this fails, then everything is lost and the controller
	 * is not identified
	 * This action MAY be destructive if a different device is connected
	 * but note that the first hurdle (oscillator) was already
	 * successful - so we should be safe...
	 */
	ret = mcp25xxfd_can_switch_mode(spi, &mode_data,
					CAN_CON_MODE_CONFIG);
	if (ret) {
		dev_err(&spi->dev,
			"Mode did not switch to config as expected - could not identify controller - register reads as %08x\n",
			mode_data);
		return -EINVAL;
	}
	/* check that modeswitch is really working */
	return mcp25xxfd_can_hw_probe_modeswitch(spi);
}

/* debugfs related */
#if defined(CONFIG_DEBUG_FS)

static int mcp25xxfd_can_dump_all_regs(struct seq_file *file, void *offset)
{
	return mcp25xxfd_dump_regs_range(file, CAN_CON, CAN_FLTMASK(31));
}

static int mcp25xxfd_can_dump_regs(struct seq_file *file, void *offset)
{
	return mcp25xxfd_dump_regs_range(file, CAN_CON, CAN_TXQUA);
}

static void mcp25xxfd_can_debugfs_add(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(priv->net);
	struct dentry *root = priv->debugfs_dir;
	struct dentry *cregs, *cstatus, *cfifotef, *cstats;

	/* export config registers */
	cregs = debugfs_create_dir("can_regs", root);
	debugfs_create_x32("con",    0444, cregs, &cpriv->regs.con);
	debugfs_create_x32("tdc",    0444, cregs, &cpriv->regs.tdc);
	debugfs_create_x32("tscon",  0444, cregs, &cpriv->regs.tscon);
	debugfs_create_x32("tefcon", 0444, cregs, &cpriv->regs.tscon);
	debugfs_create_x32("nbtcfg", 0444, cregs, &cpriv->regs.nbtcfg);
	debugfs_create_x32("dbtcfg", 0444, cregs, &cpriv->regs.dbtcfg);

	/* export status registers */
	cstatus = debugfs_create_dir("can_status", root);
	debugfs_create_x32("intf",    0444, cstatus,
			   &cpriv->status.intf);
	debugfs_create_x32("rx_if",   0444, cstatus,
			   &cpriv->status.rxif);
	debugfs_create_x32("tx_if",   0444, cstatus,
			   &cpriv->status.txif);
	debugfs_create_x32("rx_ovif", 0444, cstatus,
			   &cpriv->status.rxovif);
	debugfs_create_x32("tx_atif", 0444, cstatus,
			   &cpriv->status.txatif);
	debugfs_create_x32("tx_req",  0444, cstatus,
			   &cpriv->status.txreq);
	debugfs_create_x32("trec",    0444, cstatus,
			   &cpriv->status.trec);

	cfifotef = debugfs_create_dir("can_tef", root);
	debugfs_create_u32("count",  0444, cfifotef,
			   &cpriv->fifos.tef.count);
	debugfs_create_u32("size",  0444, cfifotef,
			   &cpriv->fifos.tef.size);

	/* dump the controller registers themselves */
	debugfs_create_devm_seqfile(&priv->spi->dev, "can_regs_live_dump",
				    root, mcp25xxfd_can_dump_regs);
	debugfs_create_devm_seqfile(&priv->spi->dev, "can_regs_full_live_dump",
				    root, mcp25xxfd_can_dump_all_regs);

	/* and stats */
	cstats = debugfs_create_dir("can_stats", root);
# define DEBUGFS_CREATE(name, var) \
	debugfs_create_u64(name,  0444, cstats, &cpriv->stats.var)
	DEBUGFS_CREATE("int_calls",		 irq_calls);
	DEBUGFS_CREATE("int_loops",		 irq_loops);
	DEBUGFS_CREATE("int_system_error",	 int_serr_count);
	DEBUGFS_CREATE("int_system_error_tx",	 int_serr_tx_count);
	DEBUGFS_CREATE("int_system_error_rx",	 int_serr_rx_count);
	DEBUGFS_CREATE("int_mode_switch",	 int_mod_count);
	DEBUGFS_CREATE("int_rx",		 int_rx_count);
	DEBUGFS_CREATE("int_tx_attempt",	 int_txat_count);
	DEBUGFS_CREATE("int_tef",		 int_tef_count);
	DEBUGFS_CREATE("int_rx_overflow",	 int_rxov_count);
	DEBUGFS_CREATE("int_ecc_error",		 int_ecc_count);
	DEBUGFS_CREATE("int_rx_invalid_message", int_ivm_count);
	DEBUGFS_CREATE("int_crcerror",		 int_cerr_count);

	DEBUGFS_CREATE("tx_frames_fd",		 tx_fd_count);
	DEBUGFS_CREATE("tx_frames_brs",		 tx_brs_count);
#undef DEBUGFS_CREATE
}
#else
static void mcp25xxfd_can_debugfs_add(struct spi_device *spi)
{
}
#endif

int mcp25xxfd_clear_can_interrupts(struct spi_device *spi)
{
	return mcp25xxfd_cmd_write_mask(spi, CAN_INT, 0, CAN_INT_IF_MASK);
}

int mcp25xxfd_enable_can_interrupts(struct spi_device *spi, bool enable)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_can_priv *cpriv = priv->net ?
		netdev_priv(priv->net) : NULL;
	const u32 mask = CAN_INT_TEFIE |
		CAN_INT_RXIE |
		CAN_INT_MODIE |
		CAN_INT_SERRIE |
		CAN_INT_IVMIE |
		CAN_INT_CERRIE |
		CAN_INT_RXOVIE |
		CAN_INT_ECCIE;
	u32 value = cpriv ? cpriv->status.intf : 0;

	/* apply mask and */
	value &= ~(CAN_INT_IE_MASK);
	if (enable)
		value |= mask;
	/* write back if net is set up */
	if (cpriv)
		cpriv->status.intf = value;

	/* and write to int register */
	return mcp25xxfd_cmd_write_mask(spi, CAN_INT, value, mask);
}

static int mcp25xxfd_can_config(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	int ret;

	/* setup value of con_register */
	cpriv->regs.con = CAN_CON_STEF; /* enable TEF, disable TXQUEUE */

	/* transmission bandwidth sharing bits */
	if (bw_sharing_log2bits > 12)
		bw_sharing_log2bits = 12;
	cpriv->regs.con |= bw_sharing_log2bits << CAN_CON_TXBWS_SHIFT;

	/* non iso FD mode */
	if (!(cpriv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO))
		cpriv->regs.con |= CAN_CON_ISOCRCEN;

	/* one shot */
	if (cpriv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		cpriv->regs.con |= CAN_CON_RTXAT;

	/* apply it now together with a mode switch */
	ret = mcp25xxfd_can_switch_mode(spi, &cpriv->regs.con,
					CAN_CON_MODE_CONFIG);
	if (ret)
		return 0;

	/* time stamp control register - 1ns resolution */
	cpriv->regs.tscon = 0;
	ret = mcp25xxfd_cmd_write(spi, CAN_TBC, 0);
	if (ret)
		return ret;

	cpriv->regs.tscon = CAN_TSCON_TBCEN |
		((cpriv->can.clock.freq / 1000000)
		 << CAN_TSCON_TBCPRE_SHIFT);
	ret = mcp25xxfd_cmd_write(spi, CAN_TSCON, cpriv->regs.tscon);
	if (ret)
		return ret;

	/* setup fifos */
	ret = mcp25xxfd_can_setup_fifos(net);
	if (ret)
		return ret;

	/* setup can bittiming now - the do_set_bittiming methods
	 * are not used as they get callled before open
	 */
	ret = mcp25xxfd_can_do_set_nominal_bittiming(net);
	if (ret)
		return ret;
	ret = mcp25xxfd_can_do_set_data_bittiming(net);
	if (ret)
		return ret;

	return ret;
}

static void mcp25xxfd_can_shutdown(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;

	/* switch us to CONFIG mode - this disables the controller */
	mcp25xxfd_can_switch_mode(spi, &cpriv->regs.con,
				  CAN_CON_MODE_CONFIG);
}

/* all ops */

/* open and stop */
static int mcp25xxfd_can_open(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	int ret;

	ret = open_candev(net);
	if (ret) {
		netdev_err(net, "unable to set initial baudrate!\n");
		return ret;
	}

	/* clear those statistics */
	memset(&cpriv->stats, 0, sizeof(cpriv->stats));

	/* request an IRQ but keep disabled for now */
	ret = request_threaded_irq(spi->irq, NULL,
				   mcp25xxfd_can_int,
				   IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				   DEVICE_NAME, priv);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d - %i\n",
			spi->irq, ret);
		goto out_candev;
	}
	disable_irq(spi->irq);
	priv->irq.enabled = false;

	/* enable power to the transceiver */
	ret = mcp25xxfd_power_enable(cpriv->transceiver, 1);
	if (ret)
		goto out_irq;

	/* enable clock (so that spi works) */
	ret = mcp25xxfd_start_clock(spi, MCP25XXFD_CLK_USER_CAN);
	if (ret)
		goto out_transceiver;

	/* configure controller for reception */
	ret = mcp25xxfd_can_config(net);
	if (ret)
		goto out_canclock;

	/* setting up state */
	cpriv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable interrupts */
	ret = mcp25xxfd_enable_interrupts(spi, true);
	if (ret)
		goto out_canconfig;

	/* switch to active mode */
	ret = mcp25xxfd_can_switch_mode(spi, &cpriv->regs.con,
					(net->mtu == CAN_MTU) ?
					CAN_CON_MODE_CAN2_0 :
					CAN_CON_MODE_MIXED);
	if (ret)
		goto out_int;

	/* start the tx_queue */
	mcp25xxfd_can_tx_queue_manage(net, TX_QUEUE_STATE_STARTED);

	return 0;

out_int:
	mcp25xxfd_enable_interrupts(spi, false);
out_canconfig:
	mcp25xxfd_can_release_fifos(net);
out_canclock:
	mcp25xxfd_stop_clock(spi, MCP25XXFD_CLK_USER_CAN);
out_transceiver:
	mcp25xxfd_power_enable(cpriv->transceiver, 0);
out_irq:
	free_irq(spi->irq, priv);
out_candev:
	close_candev(net);
	return ret;
}

static int mcp25xxfd_can_stop(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;

	/* stop transmit queue */
	mcp25xxfd_can_tx_queue_manage(net, TX_QUEUE_STATE_STOPPED);

	/* release debugfs */
	mcp25xxfd_can_release_fifos(net);

	/* shutdown the can controller */
	mcp25xxfd_can_shutdown(net);

	/* disable inerrupts on controller */
	mcp25xxfd_enable_interrupts(spi, false);

	/* stop the clock */
	mcp25xxfd_stop_clock(spi, MCP25XXFD_CLK_USER_CAN);

	/* and disable the transceiver */
	mcp25xxfd_power_enable(cpriv->transceiver, 0);

	/* disable interrupt on host */
	free_irq(spi->irq, priv);
	priv->irq.enabled = false;

	/* close the can_decice */
	close_candev(net);

	return 0;
}

/* mode setting */
static int mcp25xxfd_can_do_set_mode(struct net_device *net,
				     enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* binary error counters */
static int mcp25xxfd_can_get_berr_counter(const struct net_device *net,
					  struct can_berr_counter *bec)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	bec->txerr = (cpriv->status.trec & CAN_TREC_TEC_MASK) >>
		CAN_TREC_TEC_SHIFT;
	bec->rxerr = (cpriv->status.trec & CAN_TREC_REC_MASK) >>
		CAN_TREC_REC_SHIFT;

	return 0;
}

static const struct net_device_ops mcp25xxfd_netdev_ops = {
	.ndo_open = mcp25xxfd_can_open,
	.ndo_stop = mcp25xxfd_can_stop,
	.ndo_start_xmit = mcp25xxfd_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

/* probe and remove */
int mcp25xxfd_can_setup(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct mcp25xxfd_can_priv *cpriv;
	struct net_device *net;
	struct regulator *transceiver;
	int ret;

	/* get transceiver power regulator*/
	transceiver = devm_regulator_get_optional(&spi->dev,
						  "xceiver");
	if (PTR_ERR(transceiver) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	/* allocate can device */
	net = alloc_candev(sizeof(*cpriv), TX_ECHO_SKB_MAX);
	if (!net)
		return -ENOMEM;

	/* and do some cross-asignments */
	cpriv = netdev_priv(net);
	cpriv->priv = priv;
	priv->net = net;
	SET_NETDEV_DEV(net, &spi->dev);

	net->netdev_ops = &mcp25xxfd_netdev_ops;
	net->flags |= IFF_ECHO;

	cpriv->transceiver = transceiver;

	cpriv->can.clock.freq = priv->clock_freq;

	cpriv->can.bittiming_const =
		&mcp25xxfd_can_nominal_bittiming_const;
	cpriv->can.data_bittiming_const =
		&mcp25xxfd_can_data_bittiming_const;
	/* we are not setting bit-timing methods here as they get
	 * called by the framework before open so the controller is
	 * still in sleep mode, which does not help
	 */
	cpriv->can.do_set_mode =
		mcp25xxfd_can_do_set_mode;
	cpriv->can.do_get_berr_counter =
		mcp25xxfd_can_get_berr_counter;
	cpriv->can.ctrlmode_supported =
		CAN_CTRLMODE_FD |
		CAN_CTRLMODE_FD_NON_ISO |
		CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY |
		CAN_CTRLMODE_BERR_REPORTING |
		CAN_CTRLMODE_ONE_SHOT;

	ret = register_candev(net);
	if (ret) {
		dev_err(&spi->dev, "Failed to register can device\n");
		goto out;
	}

	mcp25xxfd_can_debugfs_add(spi);

	return 0;
out:
	free_candev(net);

	return ret;
}

void mcp25xxfd_can_remove(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);

	if (priv->net) {
		unregister_candev(priv->net);
		free_candev(priv->net);
		priv->net = NULL;
	}
}
