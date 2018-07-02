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
 * Right now we only use the CAN controller block to put us into deep sleep
 * this means that the oscillator clock is turned off.
 * So this is the only thing that we implement here right now
 */

#include "mcp25xxfd_can.h"
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

static int mcp25xxfd_can_get_mode(struct spi_device *spi,
				  u32 *mode_data)
{
	int ret;

	ret = mcp25xxfd_cmd_read(spi, CAN_CON, mode_data);
	if (ret)
		return ret;

	return (*mode_data & CAN_CON_OPMOD_MASK) >> CAN_CON_OPMOD_SHIFT;
}

int mcp25xxfd_can_switch_mode(struct spi_device *spi,
			      u32 *mode_data, int mode)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	int ret, i;

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
	ret = mcp25xxfd_cmd_write(spi, CAN_CON, *mode_data);
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
