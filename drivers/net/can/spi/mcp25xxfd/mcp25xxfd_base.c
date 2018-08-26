// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "mcp25xxfd.h"

/* device description and rational:
 *
 * the mcp25xxfd is a CanFD controller that also supports can2.0 only
 * modes.
 * It is connected via spi to the host and requires at minimum a single
 * irq line in addition to the SPI lines - it is not mentioned explicitly
 * in the documentation but in principle SPI 3-wire should be possible.
 *
 * The clock connected is typically 4MHz, 20MHz or 40MHz.
 * When using a 4MHz clock the controller can use an integrated PLL to
 * get 40MHz.
 *
 * The controller itself has 2KB of SRAM for CAN-data.
 * ECC can get enabled for SRAM.
 * CRC-16 checksumming of SPI transfers can get implemented
 *   - some optimization options may not be efficient in such a situation.
 *   - more SPI bus bandwidth is used for transfer of CRCs and
 *     transfer length information
 *
 * It also contains 2 GPIO pins that can get used either as interrupt lines
 * or GPIO IN or Out or STANDBY flags.
 * In addition there is a PIN that allows output of a (divided) clock out
 * or as a SOF (Start of Can FRAME) interrupt line - e.g for wakeup.
 */

/* known hardware issues and workarrounds in this driver:
 *
 * * There is one situation where the controller will require a full POR
 *   (total power off) to recover from a bad Clock configuration.
 *   This happens when the wrong clock is configured in the device tree
 *   (say 4MHz are configured, while 40MHz is the actual clock frequency
 *   of the HW).
 *   In such a situation the driver tries to enable the PLL, which will
 *   never synchronize and the controller becomes unresponsive to further
 *   spi requests until a full POR.
 *
 *   Mitigation:
 *     none as of now
 *
 *   Possible implementation of a mitigation/sanity check:
 *     during initialization:
 *       * try to identify the HW at 1MHz:
 *         on success:
 *           * controller is identified
 *         on failure:
 *           * controller is absent - fail
 *       * force controller clock to run with disabled PLL
 *       * try to identify the HW at 2MHz:
 *         on success:
 *           * controller clock is >= 4 MHz
 *           * this may be 4MHz
 *         on failure:
 *           * controller clock is < 4 MHz
 *       * try to identify the HW at 2.5MHz:
 *         on success:
 *           * controller clock is >= 5 MHz
 *           * this may not be 4MHz
 *         on failure:
 *           * controller clock is 4 MHz
 *           * enable PLL
 *           * exit successfully (or run last test for verification purposes)
 *       * try to identify the HW at <dt-clock/2> MHz:
 *         on success:
 *           * controller clock is >= <dt-clock/2> MHz
 *              (it could be higher though)
 *         on failure:
 *           * the controller is not running at the
 *             clock rate configured in the DT
 *           * if PLL is enabled warn about requirements of POR
 *           * fail
 *
 *   Side-effects:
 *     * longer initialization time
 *
 *   Possible issues with mitigation:
 *     * possibly miss-identification because the SPI block may work
 *       "somewhat" at frequencies > < clock / 2 + delta f>
 *       this may be especially true for the situation where we test if
 *       2.5MHz SPI-Clock works.
 *     * also SPI HW-clock dividers may do a round down to fixed frequencies
 *       which is not propperly reported and may result in false positives
 *       because a frequency lower than expected is used.
 *
 *   This is the reason why only simple testing is enabled at the risk of
 *   the need for a POR.
 */

/* SPI helper */

/* wrapper arround spi_sync, that sets speed_hz */
static int mcp25xxfd_sync_transfer(struct spi_device *spi,
				   struct spi_transfer *xfer,
				   unsigned int xfers)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int i;

	for (i = 0; i < xfers; i++)
		xfer[i].speed_hz = priv->spi_use_speed_hz;

	return spi_sync_transfer(spi, xfer, xfers);
}

/* simple spi_write wrapper with speed_hz */
static int mcp25xxfd_write(struct spi_device *spi,
			   const void *tx_buf,
			   unsigned int tx_len)
{
	struct spi_transfer xfer;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = tx_buf;
	xfer.len = tx_len;

	return mcp25xxfd_sync_transfer(spi, &xfer, 1);
}

/* an optimization of spi_write_then_read that merges the transfers */
static int mcp25xxfd_write_then_read(struct spi_device *spi,
				     const void *tx_buf,
				     unsigned int tx_len,
				     void *rx_buf,
				     unsigned int rx_len)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct spi_transfer xfer[2];
	u8 single_reg_data_tx[6];
	u8 single_reg_data_rx[6];
	int ret;

	memset(xfer, 0, sizeof(xfer));

	/* when using a halfduplex controller or to big for buffer */
	if ((spi->master->flags & SPI_MASTER_HALF_DUPLEX) ||
	    (tx_len + rx_len > sizeof(priv->spi_tx))) {
		xfer[0].tx_buf = tx_buf;
		xfer[0].len = tx_len;

		xfer[1].rx_buf = rx_buf;
		xfer[1].len = rx_len;

		return mcp25xxfd_sync_transfer(spi, xfer, 2);
	}

	/* full duplex optimization */
	xfer[0].len = tx_len + rx_len;
	if (xfer[0].len > sizeof(single_reg_data_tx)) {
		mutex_lock(&priv->spi_rxtx_lock);
		xfer[0].tx_buf = priv->spi_tx;
		xfer[0].rx_buf = priv->spi_rx;
	} else {
		xfer[0].tx_buf = single_reg_data_tx;
		xfer[0].rx_buf = single_reg_data_rx;
	}

	/* copy and clean */
	memcpy((u8 *)xfer[0].tx_buf, tx_buf, tx_len);
	memset((u8 *)xfer[0].tx_buf + tx_len, 0, rx_len);

	ret = mcp25xxfd_sync_transfer(spi, xfer, 1);
	if (!ret)
		memcpy(rx_buf, xfer[0].rx_buf + tx_len, rx_len);

	if (xfer[0].len > sizeof(single_reg_data_tx))
		mutex_unlock(&priv->spi_rxtx_lock);

	return ret;
}

static int mcp25xxfd_write_then_write(struct spi_device *spi,
				      const void *tx_buf,
				      unsigned int tx_len,
				      const void *tx2_buf,
				      unsigned int tx2_len)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct spi_transfer xfer;
	u8 single_reg_data[6];
	int ret;

	if (tx_len + tx2_len > MCP25XXFD_BUFFER_TXRX_SIZE)
		return -EINVAL;

	memset(&xfer, 0, sizeof(xfer));

	xfer.len = tx_len + tx2_len;
	if (xfer.len > sizeof(single_reg_data)) {
		mutex_lock(&priv->spi_rxtx_lock);
		xfer.tx_buf = priv->spi_tx;
	} else {
		xfer.tx_buf = single_reg_data;
	}

	memcpy((u8 *)xfer.tx_buf, tx_buf, tx_len);
	memcpy((u8 *)xfer.tx_buf + tx_len, tx2_buf, tx2_len);

	ret = mcp25xxfd_sync_transfer(spi, &xfer, 1);

	if (xfer.len > sizeof(single_reg_data))
		mutex_unlock(&priv->spi_rxtx_lock);

	return ret;
}

/* mcp25xxfd spi command/protocol helper */

/* read multiple bytes, transform some registers */
int mcp25xxfd_cmd_readn(struct spi_device *spi, u32 reg,
			void *data, int n)
{
	u8 cmd[2];
	int ret;

	mcp25xxfd_calc_cmd_addr(INSTRUCTION_READ, reg, cmd);

	ret = mcp25xxfd_write_then_read(spi, &cmd, 2, data, n);
	if (ret)
		return ret;

	return 0;
}

/* read a register, but we are only interrested in a few bytes */
int mcp25xxfd_cmd_read_mask(struct spi_device *spi, u32 reg,
			    u32 *data, u32 mask)
{
	int first_byte, last_byte, len_byte;
	int ret;

	/* check that at least one bit is set */
	if (!mask)
		return -EINVAL;

	/* calculate first and last byte used */
	first_byte = mcp25xxfd_first_byte(mask);
	last_byte = mcp25xxfd_last_byte(mask);
	len_byte = last_byte - first_byte + 1;

	/* do a partial read */
	*data = 0;
	ret = mcp25xxfd_cmd_readn(spi, reg + first_byte,
				  ((void *)data + first_byte), len_byte);
	if (ret)
		return ret;

	mcp25xxfd_convert_to_cpu(data, 1);

	return 0;
}

static int mcp25xxfd_cmd_reset(struct spi_device *spi)
{
	u8 cmd[2];

	mcp25xxfd_calc_cmd_addr(INSTRUCTION_RESET, 0, cmd);

	/* write the reset command */
	return mcp25xxfd_write(spi, cmd, 2);
}

int mcp25xxfd_cmd_writen(struct spi_device *spi, u32 reg,
			 void *data, int n)
{
	u8 cmd[2];
	int ret;

	mcp25xxfd_calc_cmd_addr(INSTRUCTION_WRITE, reg, cmd);

	ret = mcp25xxfd_write_then_write(spi, &cmd, 2, data, n);
	if (ret)
		return ret;

	return 0;
}

/* read a register, but we are only interrested in a few bytes */
int mcp25xxfd_cmd_write_mask(struct spi_device *spi, u32 reg,
			     u32 data, u32 mask)
{
	int first_byte, last_byte, len_byte;
	u8 cmd[2];

	/* check that at least one bit is set */
	if (!mask)
		return -EINVAL;

	/* calculate first and last byte used */
	first_byte = mcp25xxfd_first_byte(mask);
	last_byte = mcp25xxfd_last_byte(mask);
	len_byte = last_byte - first_byte + 1;

	/* prepare buffer */
	mcp25xxfd_calc_cmd_addr(INSTRUCTION_WRITE, reg + first_byte, cmd);
	data = cpu_to_le32(data);

	return mcp25xxfd_write_then_write(spi,
					  cmd, sizeof(cmd),
					  ((void *)&data + first_byte),
					  len_byte);
}

int mcp25xxfd_cmd_write_regs(struct spi_device *spi, u32 reg,
			     u32 *data, u32 bytes)
{
	int ret;

	/* first transpose to controller format */
	mcp25xxfd_convert_from_cpu(data, bytes / sizeof(u32));

	/* now write it */
	ret = mcp25xxfd_cmd_writen(spi, reg, data, bytes);

	/* and convert it back to cpu format even if it fails */
	mcp25xxfd_convert_to_cpu(data, bytes / sizeof(u32));

	return ret;
}

int mcp25xxfd_cmd_read_regs(struct spi_device *spi, u32 reg,
			    u32 *data, u32 bytes)
{
	int ret;

	/* read it */
	ret = mcp25xxfd_cmd_readn(spi, reg, data, bytes);

	/* and convert it to cpu format */
	mcp25xxfd_convert_to_cpu((u32 *)data, bytes / sizeof(u32));

	return ret;
}

/* debugfs related */
#if defined(CONFIG_DEBUG_FS)

int mcp25xxfd_dump_regs_range(struct seq_file *file, u32 start, u32 end)
{
	struct spi_device *spi = file->private;
	u32 data[32];
	int bytes = end - start + sizeof(u32);
	int i, l, count, ret;

	for (count =  bytes / sizeof(u32); count > 0; count -= 32) {
		/* read up to 32 registers in one go */
		l = min(count, 32);
		ret = mcp25xxfd_cmd_read_regs(spi, start,
					      data, l * sizeof(u32));
		if (ret)
			return ret;
		/* dump those read registers */
		for (i = 0; i < l; i++, start += sizeof(u32))
			seq_printf(file, "Reg 0x%03x = 0x%08x\n",
				   start, data[i]);
	}

	return 0;
}

static int mcp25xxfd_dump_regs(struct seq_file *file, void *offset)
{
	return mcp25xxfd_dump_regs_range(file,
					 MCP25XXFD_OSC,
					 MCP25XXFD_ECCSTAT);
}

static int mcp25xxfd_debugfs_setup(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct dentry *root, *regs;
	char name[64];

	/* the base directory */
	snprintf(name, sizeof(name),
		 DEVICE_NAME "-%s",
		 dev_name(&priv->spi->dev));
	priv->debugfs_dir = debugfs_create_dir(name, NULL);
	root = priv->debugfs_dir;

	/* expose some parameters related to clocks */
	debugfs_create_u32("spi_setup_speed_hz", 0444, root,
			   &priv->spi_setup_speed_hz);
	debugfs_create_u32("spi_normal_speed_hz", 0444, root,
			   &priv->spi_normal_speed_hz);
	debugfs_create_u32("spi_use_speed_hz", 0444, root,
			   &priv->spi_use_speed_hz);
	debugfs_create_u32("clk_user_mask", 0444, root, &priv->clk_user_mask);

	/* expose the system registers */
	priv->debugfs_regs_dir = debugfs_create_dir("regs", root);
	regs = priv->debugfs_regs_dir;
	debugfs_create_x32("osc", 0444, regs, &priv->regs.osc);
	debugfs_create_x32("iocon", 0444, regs, &priv->regs.iocon);
	debugfs_create_x32("crc", 0444, regs, &priv->regs.crc);
	debugfs_create_x32("ecccon", 0444, regs, &priv->regs.ecccon);

	/* dump the controller registers themselves */
	debugfs_create_devm_seqfile(&priv->spi->dev, "regs_live_dump",
				    root, mcp25xxfd_dump_regs);

	return 0;
}

static void mcp25xxfd_debugfs_remove(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);

	debugfs_remove_recursive(priv->debugfs_dir);
	priv->debugfs_dir = NULL;
}
#else
int mcp25xxfd_dump_regs_range(struct seq_file *file, u32 start, u32 end)
{
	return 0;
}

static int mcp25xxfd_debugfs_setup(struct mcp25xxfd_priv *priv)
{
	return 0;
}

static void mcp25xxfd_debugfs_remove(struct mcp25xxfd_priv *priv)
{
}
#endif

/* HW Related */
int mcp25xxfd_clear_ecc_interrupts(struct spi_device *spi)
{
	u32 val, addr;
	int ret;

	/* first report the error address */
	ret = mcp25xxfd_cmd_read(spi, MCP25XXFD_ECCSTAT, &val);
	if (ret)
		return ret;

	/* if no flags are set then nothing to do */
	if (!(val & (MCP25XXFD_ECCSTAT_SECIF | MCP25XXFD_ECCSTAT_DEDIF)))
		return 0;

	addr = (val & MCP25XXFD_ECCSTAT_ERRADDR_MASK) >>
		MCP25XXFD_ECCSTAT_ERRADDR_SHIFT;

	dev_err_ratelimited(&spi->dev,
			    "ECC %s bit error at %03x\n",
			    (val & MCP25XXFD_ECCSTAT_DEDIF) ?
			    "double" : "single",
			    addr);

	/* and clear the error */
	return mcp25xxfd_cmd_write_mask(spi, MCP25XXFD_ECCSTAT, 0,
					MCP25XXFD_ECCSTAT_SECIF |
					MCP25XXFD_ECCSTAT_DEDIF);
}

int mcp25xxfd_clear_crc_interrupts(struct spi_device *spi)
{
	return mcp25xxfd_cmd_write_mask(spi, MCP25XXFD_CRC, 0,
					MCP25XXFD_CRC_CRCERRIF |
					MCP25XXFD_CRC_FERRIF);
}

int mcp25xxfd_clear_interrupts(struct spi_device *spi)
{
	mcp25xxfd_clear_ecc_interrupts(spi);
	mcp25xxfd_clear_crc_interrupts(spi);

	return 0;
}

static int mcp25xxfd_enable_ecc_interrupts(struct spi_device *spi,
					   bool enable)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	u32 mask = MCP25XXFD_ECCCON_SECIE | MCP25XXFD_ECCCON_DEDIE;

	priv->regs.ecccon &= ~mask;
	priv->regs.ecccon |= MCP25XXFD_ECCCON_ECCEN | (enable ? mask : 0);

	return mcp25xxfd_cmd_write_mask(spi, MCP25XXFD_ECCCON,
					priv->regs.ecccon,
					MCP25XXFD_ECCCON_ECCEN | mask);
}

static int mcp25xxfd_enable_crc_interrupts(struct spi_device *spi,
					   bool enable)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	u32 mask = MCP25XXFD_CRC_CRCERRIE | MCP25XXFD_CRC_FERRIE;

	priv->regs.crc &= ~mask;
	priv->regs.crc |= enable ? mask : 0;

	return mcp25xxfd_cmd_write_mask(spi, MCP25XXFD_CRC,
					priv->regs.crc, mask);
}

int mcp25xxfd_enable_interrupts(struct spi_device *spi,
				bool enable)
{
	/* error handling only on enable for this function */
	int ret = 0;

	/* if we enable clear interrupt flags first */
	if (enable)
		ret = mcp25xxfd_clear_interrupts(spi);
	if (enable && ret)
		goto out;

	ret = mcp25xxfd_enable_ecc_interrupts(spi, enable);
	if (enable && ret)
		goto out;

	ret = mcp25xxfd_enable_crc_interrupts(spi, enable);
	if (enable && ret)
		goto out_ecc;

	/* if we disable interrupts clear interrupt flags last */
	if (!enable)
		mcp25xxfd_clear_interrupts(spi);

	return 0;

out_ecc:
	mcp25xxfd_enable_ecc_interrupts(spi, false);
out:
	return ret;
}

static int _mcp25xxfd_waitfor_osc(struct spi_device *spi,
				  u32 waitfor,
				  u32 mask)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	unsigned long timeout;
	int ret;

	/* wait for synced pll/osc/sclk */
	timeout = jiffies + MCP25XXFD_OSC_POLLING_JIFFIES;
	while (time_before_eq(jiffies, timeout)) {
		ret = mcp25xxfd_cmd_read(spi, MCP25XXFD_OSC,
					 &priv->regs.osc);
		if (ret)
			return ret;
		/* check for expected bits to be set/unset */
		if ((priv->regs.osc & mask) == waitfor)
			return 0;
	}

	return -ETIMEDOUT;
}

static int mcp25xxfd_configure_osc(struct spi_device *spi,
				   u32 value,
				   u32 waitfor,
				   u32 mask)
{
	int ret;

	/* write the osc value to the controller - waking it if necessary */
	ret = mcp25xxfd_cmd_write(spi, MCP25XXFD_OSC, value);
	if (ret)
		return ret;

	/* wait for the clock to stabelize */
	ret = _mcp25xxfd_waitfor_osc(spi, waitfor, mask);

	/* on timeout try again setting the register */
	if (ret == -ETIMEDOUT) {
		/* write the clock to the controller */
		ret = mcp25xxfd_cmd_write(spi, MCP25XXFD_OSC, value);
		if (ret)
			return ret;

		/* wait for the clock to stabelize */
		ret = _mcp25xxfd_waitfor_osc(spi, waitfor, mask);
	}

	/* handle timeout special - report the fact */
	if (ret == -ETIMEDOUT)
		dev_err(&spi->dev,
			"Clock did not switch within the timeout period\n");

	return ret;
}

static u32 _mcp25xxfd_clkout_mask(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	u32 value = 0;

	if (priv->config.clock_div2)
		value |= MCP25XXFD_OSC_SCLKDIV;

	switch (priv->config.clock_odiv) {
	case 0:
		break;
	case 1:
		value |= MCP25XXFD_OSC_CLKODIV_1 <<
			MCP25XXFD_OSC_CLKODIV_SHIFT;
		break;
	case 2:
		value |= MCP25XXFD_OSC_CLKODIV_2 <<
			MCP25XXFD_OSC_CLKODIV_SHIFT;
		break;
	case 4:
		value |= MCP25XXFD_OSC_CLKODIV_4 <<
			MCP25XXFD_OSC_CLKODIV_SHIFT;
		break;
	case 10:
		value |= MCP25XXFD_OSC_CLKODIV_10 <<
			MCP25XXFD_OSC_CLKODIV_SHIFT;
		break;
	default:
		/* this should never happen but is error-handled
		 * by the dt-parsing
		 */
		break;
	}

	return value;
}

static int _mcp25xxfd_start_clock(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	u32 value   = _mcp25xxfd_clkout_mask(spi);
	u32 waitfor = MCP25XXFD_OSC_OSCRDY;
	u32 mask    = waitfor |
		MCP25XXFD_OSC_OSCDIS |
		MCP25XXFD_OSC_PLLRDY |
		MCP25XXFD_OSC_PLLEN;

	/* enable PLL as well - set expectations */
	if (priv->config.clock_pll) {
		value   |= MCP25XXFD_OSC_PLLEN;
		waitfor |= MCP25XXFD_OSC_PLLRDY | MCP25XXFD_OSC_PLLEN;
	}

	/* set the oscilator now */
	return mcp25xxfd_configure_osc(spi, value, waitfor, mask);
}

static int _mcp25xxfd_stop_clock(struct spi_device *spi)
{
	u32 value   = _mcp25xxfd_clkout_mask(spi);
	u32 waitfor = 0;
	u32 mask    = MCP25XXFD_OSC_OSCDIS |
		MCP25XXFD_OSC_PLLRDY |
		MCP25XXFD_OSC_PLLEN;
	int ret;

	ret = mcp25xxfd_configure_osc(spi, value, waitfor, mask);
	if (ret)
		return ret;

	/* finally switch the controller mode to sleep
	 * by this time the controller should be in config mode already
	 * this way we wake to config mode again
	 */
	return mcp25xxfd_can_sleep_mode(spi);
}

int mcp25xxfd_start_clock(struct spi_device *spi, int requestor_mask)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int ret = 0;

	/* without a clock there is nothing we can do... */
	if (IS_ERR(priv->clk))
		return IS_ERR(priv->clk);

	mutex_lock(&priv->clk_user_lock);

	/* if clock is already started, then skip */
	if (priv->clk_user_mask & requestor_mask)
		goto out;

	/* enable the clock on the host side*/
	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto out;

	/* enable the clock on the controller side */
	ret = _mcp25xxfd_start_clock(spi);
	if (ret)
		goto out;

	/* mark the clock for the specific component as started */
	priv->clk_user_mask |= requestor_mask;

	/* and now we use the normal spi speed */
	priv->spi_use_speed_hz = priv->spi_normal_speed_hz;

out:
	mutex_unlock(&priv->clk_user_lock);

	return ret;
}

int mcp25xxfd_stop_clock(struct spi_device *spi, int requestor_mask)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int ret;

	/* without a clock there is nothing we can do... */
	if (IS_ERR(priv->clk))
		return IS_ERR(priv->clk);

	mutex_lock(&priv->clk_user_lock);

	/* if the mask is empty then skip, as the clock is stopped */
	if (!priv->clk_user_mask)
		goto out;

	/* clear the clock mask */
	priv->clk_user_mask &= ~requestor_mask;

	/* if the mask is not empty then skip, as the clock is needed */
	if (priv->clk_user_mask)
		goto out;

	/* and now we use the setup spi speed */
	priv->spi_use_speed_hz = priv->spi_setup_speed_hz;

	/* stop the clock on the controller */
	ret = _mcp25xxfd_stop_clock(spi);

	/* and we stop the clock on the host*/
	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);
out:
	mutex_unlock(&priv->clk_user_lock);

	return 0;
}

static int mcp25xxfd_enable_ecc(struct spi_device *spi)
{
	u8 buffer[256];
	int i;
	int ret;

	/* set up RAM ECC - enable interrupts sets it as well */
	ret = mcp25xxfd_enable_ecc_interrupts(spi, false);
	if (ret)
		return ret;

	/* and clear SRAM so that no read fails from now on */
	memset(buffer, 0, sizeof(buffer));
	for (i = 0; i < MCP25XXFD_SRAM_SIZE; i += sizeof(buffer)) {
		ret = mcp25xxfd_cmd_writen(spi,
					   MCP25XXFD_SRAM_ADDR(i),
					   buffer, sizeof(buffer));
		if (ret)
			return ret;
	}

	return 0;
}

static int mcp25xxfd_hw_wake(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int ret = 0;

	/* enable power to controller */
	mcp25xxfd_power_enable(priv->power, 1);

	/* if there is no sleep mask, then there is nothing to wake */
	if (!priv->clk_sleep_mask)
		return 0;

	/* start the clocks */
	ret = mcp25xxfd_start_clock(spi, priv->clk_sleep_mask);
	if (ret)
		return 0;

	/* clear the sleep mask */
	mutex_lock(&priv->clk_user_lock);
	priv->clk_sleep_mask = 0;
	mutex_unlock(&priv->clk_user_lock);

	/* enable the interrupts again */
	return mcp25xxfd_enable_interrupts(spi, true);
}

static int mcp25xxfd_hw_sleep(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);

	mutex_lock(&priv->clk_user_lock);
	priv->clk_sleep_mask = priv->clk_user_mask;
	mutex_unlock(&priv->clk_user_lock);

	/* disable interrupts */
	mcp25xxfd_enable_interrupts(spi, false);

	/* stop the clocks */
	mcp25xxfd_stop_clock(spi, priv->clk_sleep_mask);

	/* disable power to controller */
	return mcp25xxfd_power_enable(priv->power, 0);
}

static int mcp25xxfd_hw_probe(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int ret;

	/* Wait for oscillator startup timer after power up */
	mdelay(MCP25XXFD_OST_DELAY_MS);

	/* send a "blind" reset, hoping we are in Config mode */
	mcp25xxfd_cmd_reset(spi);

	/* Wait for oscillator startup again */
	mdelay(MCP25XXFD_OST_DELAY_MS);

	/* check clock register that the clock is ready or disabled */
	ret = mcp25xxfd_cmd_read(spi, MCP25XXFD_OSC,
				 &priv->regs.osc);
	if (ret)
		return ret;

	/* there can only be one... */
	switch (priv->regs.osc &
		(MCP25XXFD_OSC_OSCRDY | MCP25XXFD_OSC_OSCDIS)) {
	case MCP25XXFD_OSC_OSCRDY: /* either the clock is ready */
		break;
	case MCP25XXFD_OSC_OSCDIS: /* or the clock is disabled */
		break;
	default:
		/* otherwise there is no valid device (or in strange state)
		 *
		 * if PLL is enabled but not ready, then there may be
		 * something "fishy"
		 * this happened during driver development
		 * (enabling pll, when when on wrong clock), so best warn
		 * about such a possibility
		 */
		if ((priv->regs.osc &
		     (MCP25XXFD_OSC_PLLEN | MCP25XXFD_OSC_PLLRDY))
		    == MCP25XXFD_OSC_PLLEN)
			dev_err(&spi->dev,
				"mcp25xxfd may be in a strange state - a power disconnect may be required\n");

		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_OF_DYNAMIC
int mcp25xxfd_of_parse(struct mcp25xxfd_priv *priv)
{
	struct spi_device *spi = priv->spi;
	const struct device_node *np = spi->dev.of_node;
	u32 val;
	int ret;

	priv->config.clock_div2 =
		of_property_read_bool(np, "microchip,clock-div2");

	ret = of_property_read_u32_index(np, "microchip,clock-out-div",
					 0, &val);
	if (!ret) {
		switch (val) {
		case 0:
		case 1:
		case 2:
		case 4:
		case 10:
			priv->config.clock_odiv = val;
			break;
		default:
			dev_err(&spi->dev,
				"Invalid value in device tree for microchip,clock_out_div: %u - valid values: 0, 1, 2, 4, 10\n",
				val);
			return -EINVAL;
		}
	}

	return 0;
}
#else
int mcp25xxfd_of_parse(struct mcp25xxfd_priv *priv)
{
	return 0;
}
#endif

static const struct of_device_id mcp25xxfd_of_match[] = {
	{
		.compatible	= "microchip,mcp2517fd",
		.data		= (void *)CAN_MCP2517FD,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mcp25xxfd_of_match);

static int mcp25xxfd_probe(struct spi_device *spi)
{
	const struct of_device_id *of_id =
		of_match_device(mcp25xxfd_of_match, &spi->dev);
	struct mcp25xxfd_priv *priv;
	struct clk *clk;
	int ret, freq;

	/* as irq_create_fwspec_mapping() can return 0, check for it */
	if (spi->irq <= 0) {
		dev_err(&spi->dev, "no valid irq line defined: irq = %i\n",
			spi->irq);
		return -EINVAL;
	}

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	freq = clk_get_rate(clk);
	if (freq < MCP25XXFD_MIN_CLOCK_FREQUENCY ||
	    freq > MCP25XXFD_MAX_CLOCK_FREQUENCY) {
		dev_err(&spi->dev,
			"Clock frequency %i is not in range [%i:%i]\n",
			freq,
			MCP25XXFD_MIN_CLOCK_FREQUENCY,
			MCP25XXFD_MAX_CLOCK_FREQUENCY);
		return -ERANGE;
	}

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* assign some data */
	if (of_id)
		priv->model = (enum mcp25xxfd_model)of_id->data;
	else
		priv->model = spi_get_device_id(spi)->driver_data;

	spi_set_drvdata(spi, priv);
	priv->spi = spi;

	mutex_init(&priv->clk_user_lock);

	/* enable the clock and mark as enabled */
	ret = clk_prepare_enable(clk);
	if (ret)
		goto out_free;

	/* do not use the SCK clock divider of 2 */
	priv->config.clock_div2 = false;

	/* clock output is divided by 10 */
	priv->config.clock_odiv = 10;

	/* if we have a clock that is <= 4MHz, enable the pll */
	priv->config.clock_pll =
		(freq <= MCP25XXFD_AUTO_PLL_MAX_CLOCK_FREQUENCY);

	/* check in device tree for overrrides */
	ret = mcp25xxfd_of_parse(priv);
	if (ret)
		return ret;

	/* decide on the effective clock rate */
	priv->clock_freq = freq;
	if (priv->config.clock_pll)
		priv->clock_freq *= MCP25XXFD_PLL_MULTIPLIER;
	if (priv->config.clock_div2)
		priv->clock_freq /= MCP25XXFD_SCLK_DIVIDER;

	/* calculate the clock frequencies to use
	 *
	 * setup clock speed is at most 1/4 the input clock speed
	 * the reason for the factor of 4 is that there is
	 * a clock divider in the controller that MAY be enabled in some
	 * circumstances so we may find a controller with that enabled
	 * during probe phase
	 */
	priv->spi_setup_speed_hz = freq / 4;

	/* normal operation clock speeds */
	priv->spi_normal_speed_hz = priv->clock_freq / 2;
	if (priv->config.clock_div2) {
		priv->spi_setup_speed_hz /= MCP25XXFD_SCLK_DIVIDER;
		priv->spi_normal_speed_hz /= MCP25XXFD_SCLK_DIVIDER;
	}
	/* set limit on speed */
	if (spi->max_speed_hz) {
		priv->spi_setup_speed_hz = min_t(int,
						 priv->spi_setup_speed_hz,
						 spi->max_speed_hz);
		priv->spi_normal_speed_hz = min_t(int,
						  priv->spi_normal_speed_hz,
						  spi->max_speed_hz);
	}

	/* use setup speed by default
	 * - this is switched when clock is enabled/disabled
	 */
	priv->spi_use_speed_hz = priv->spi_setup_speed_hz;

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		goto out_clk;

	priv->power = devm_regulator_get_optional(&spi->dev, "vdd");
	if (PTR_ERR(priv->power) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		goto out_clk;
	}

	ret = mcp25xxfd_power_enable(priv->power, 1);
	if (ret)
		goto out_clk;

	/* this will also enable the MCP25XXFD_CLK_USER_CAN clock */
	ret = mcp25xxfd_hw_probe(spi);

	/* on error retry a second time */
	if (ret == -ENODEV) {
		ret = mcp25xxfd_hw_probe(spi);
		if (!ret)
			dev_info(&spi->dev,
				 "found device only during retry\n");
	}
	if (ret) {
		if (ret == -ENODEV)
			dev_err(&spi->dev,
				"Cannot initialize MCP%x. Wrong wiring? (oscilator register reads as %08x)\n",
				priv->model, priv->regs.osc);
		goto out_probe;
	}

	/* enable the can controller clock */
	ret = mcp25xxfd_start_clock(spi, MCP25XXFD_CLK_USER_CAN);
	if (ret)
		goto out_probe;

	/* try to identify the can-controller - we need the clock here */
	ret = mcp25xxfd_can_hw_probe(spi);
	if (ret)
		goto out_ctlclk;

	/* add debugfs */
	ret = mcp25xxfd_debugfs_setup(spi);
	if (ret)
		goto out_ctlclk;

	/* disable interrupts */
	ret = mcp25xxfd_enable_interrupts(spi, false);
	if (ret)
		goto out_debugfs;

	/* setup ECC for SRAM */
	ret = mcp25xxfd_enable_ecc(spi);
	if (ret)
		goto out_debugfs;

	/* setting up GPIO */
	ret = mcp25xxfd_gpio_setup(spi);
	if (ret)
		goto out_debugfs;

	/* and put controller to sleep by stopping the can clock */
	ret = mcp25xxfd_stop_clock(spi, MCP25XXFD_CLK_USER_CAN);
	if (ret)
		goto out_gpio;

	dev_info(&spi->dev,
		 "MCP%x successfully initialized.\n", priv->model);
	return 0;

out_gpio:
	mcp25xxfd_gpio_remove(spi);
out_debugfs:
	mcp25xxfd_debugfs_remove(spi);
out_ctlclk:
	mcp25xxfd_stop_clock(spi, MCP25XXFD_CLK_USER_CAN);
out_probe:
	mcp25xxfd_power_enable(priv->power, 0);
out_clk:
	mcp25xxfd_stop_clock(spi, MCP25XXFD_CLK_USER_CAN);
out_free:
	mcp25xxfd_debugfs_remove(spi);
	dev_err(&spi->dev, "Probe failed, err=%d\n", -ret);
	return ret;
}

static int mcp25xxfd_remove(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);

	/* remove gpio */
	mcp25xxfd_gpio_remove(spi);

	/* clear all running clocks */
	mcp25xxfd_stop_clock(spi, priv->clk_user_mask);

	mcp25xxfd_debugfs_remove(spi);

	mcp25xxfd_power_enable(priv->power, 0);

	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

	return 0;
}

static int __maybe_unused mcp25xxfd_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);

	return mcp25xxfd_hw_sleep(spi);

	return 0;
}

static int __maybe_unused mcp25xxfd_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);

	return mcp25xxfd_hw_wake(spi);
}

static SIMPLE_DEV_PM_OPS(mcp25xxfd_pm_ops, mcp25xxfd_suspend,
	mcp25xxfd_resume);

static const struct spi_device_id mcp25xxfd_id_table[] = {
	{
		.name		= "mcp2517fd",
		.driver_data	= (kernel_ulong_t)CAN_MCP2517FD,
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, mcp25xxfd_id_table);

static struct spi_driver mcp25xxfd_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = mcp25xxfd_of_match,
		.pm = &mcp25xxfd_pm_ops,
	},
	.id_table = mcp25xxfd_id_table,
	.probe = mcp25xxfd_probe,
	.remove = mcp25xxfd_remove,
};
module_spi_driver(mcp25xxfd_can_driver);

MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_DESCRIPTION("Microchip 25XXFD CAN driver");
MODULE_LICENSE("GPL v2");
