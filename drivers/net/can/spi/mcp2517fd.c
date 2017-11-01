/*
 * CAN bus driver for Microchip 2517FD CAN Controller with SPI Interface
 *
 * Copyright 2017 Martin Sperl
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 *
 */

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

#define DEVICE_NAME "mcp2517fd"

#define MCP2517FD_OST_DELAY_MS		3
#define MCP2517FD_MIN_CLOCK_FREQUENCY	1000000
#define MCP2517FD_MAX_CLOCK_FREQUENCY	40000000
#define MCP2517FD_PLL_MULTIPLIER	10
#define MCP2517FD_AUTO_PLL_MAX_CLOCK_FREQUENCY \
	(MCP2517FD_MAX_CLOCK_FREQUENCY / MCP2517FD_PLL_MULTIPLIER)
#define MCP2517FD_SCLK_DIVIDER		2

#define MCP2517FD_OSC_POLLING_JIFFIES	(HZ / 2)

#define TX_ECHO_SKB_MAX	32

#define INSTRUCTION_RESET		0x0000
#define INSTRUCTION_READ		0x3000
#define INSTRUCTION_WRITE		0x2000
#define INSTRUCTION_READ_CRC		0xB000
#define INSTRUCTION_WRITE_CRC		0xA000
#define INSTRUCTION_WRITE_SAVE		0xC000

#define ADDRESS_MASK 			0x0fff

#define MCP2517FD_SFR_BASE(x)		(0xE00 + (x))
#define MCP2517FD_OSC			MCP2517FD_SFR_BASE(0x00)
#  define MCP2517FD_OSC_PLLEN		BIT(0)
#  define MCP2517FD_OSC_OSCDIS		BIT(2)
#  define MCP2517FD_OSC_SCLKDIV		BIT(4)
#  define MCP2517FD_OSC_CLKODIV_BITS	2
#  define MCP2517FD_OSC_CLKODIV_SHIFT	5
#  define MCP2517FD_OSC_CLKODIV_MASK			\
	GENMASK(MCP2517FD_OSC_CLKODIV_SHIFT		\
		+ MCP2517FD_OSC_CLKODIV_BITS - 1,	\
		MCP2517FD_OSC_CLKODIV_SHIFT)
#  define MCP2517FD_OSC_CLKODIV_10	3
#  define MCP2517FD_OSC_CLKODIV_4	2
#  define MCP2517FD_OSC_CLKODIV_2	1
#  define MCP2517FD_OSC_CLKODIV_1	0
#  define MCP2517FD_OSC_PLLRDY		BIT(8)
#  define MCP2517FD_OSC_OSCRDY		BIT(10)
#  define MCP2517FD_OSC_SCLKRDY		BIT(12)
#define MCP2517FD_IOCON			MCP2517FD_SFR_BASE(0x04)
#  define MCP2517FD_IOCON_TRIS0		BIT(0)
#  define MCP2517FD_IOCON_TRIS1		BIT(1)
#  define MCP2517FD_IOCON_XSTBYEN	BIT(6)
#  define MCP2517FD_IOCON_LAT0		BIT(8)
#  define MCP2517FD_IOCON_LAT1		BIT(9)
#  define MCP2517FD_IOCON_GPIO0		BIT(16)
#  define MCP2517FD_IOCON_GPIO1		BIT(17)
#  define MCP2517FD_IOCON_PM0		BIT(24)
#  define MCP2517FD_IOCON_PM1		BIT(25)
#  define MCP2517FD_IOCON_TXCANOD	BIT(28)
#  define MCP2517FD_IOCON_SOF		BIT(29)
#  define MCP2517FD_IOCON_INTOD		BIT(29)
#define MCP2517FD_CRC			MCP2517FD_SFR_BASE(0x08)
#  define MCP2517FD_CRC_MASK		GENMASK(15, 0)
#  define MCP2517FD_CRC_CRCERRIE	BIT(16)
#  define MCP2517FD_CRC_FERRIE		BIT(17)
#  define MCP2517FD_CRC_CRCERRIF	BIT(24)
#  define MCP2517FD_CRC_FERRIF		BIT(25)
#define MCP2517FD_ECCCON		MCP2517FD_SFR_BASE(0x0C)
#  define MCP2517FD_ECCCON_ECCEN	BIT(0)
#  define MCP2517FD_ECCCON_SECIE	BIT(1)
#  define MCP2517FD_ECCCON_DEDIE	BIT(2)
#  define MCP2517FD_ECCCON_PARITY_BITS 6
#  define MCP2517FD_ECCCON_PARITY_SHIFT 8
#  define MCP2517FD_ECCCON_PARITY_MASK			\
	GENMASK(MCP2517FD_ECCCON_PARITY_SHIFT		\
		+ MCP2517FD_ECCCON_PARITY_BITS - 1,	\
		MCP2517FD_ECCCON_PARITY_SHIFT)
#define MCP2517FD_ECCSTAT		MCP2517FD_SFR_BASE(0x10)
#  define MCP2517FD_ECCSTAT_SECIF	BIT(1)
#  define MCP2517FD_ECCSTAT_DEDIF	BIT(2)
#  define MCP2517FD_ECCSTAT_ERRADDR_SHIFT 8
#  define MCP2517FD_ECCSTAT_ERRADDR_MASK	      \
	GENMASK(MCP2517FD_ECCSTAT_ERRADDR_SHIFT + 11, \
		MCP2517FD_ECCSTAT_ERRADDR_SHIFT)

#define CAN_SFR_BASE(x)			(0x000 + (x))
#define CAN_CON				CAN_SFR_BASE(0x00)
#  define CAN_CON_DNCNT_BITS		5
#  define CAN_CON_DNCNT_SHIFT		0
#  define CAN_CON_DNCNT_MASK					\
	GENMASK(CAN_CON_DNCNT_SHIFT + CAN_CON_DNCNT_BITS - 1,	\
		CAN_CON_DNCNT_SHIFT)
#  define CAN_CON_ISOCRCEN		BIT(5)
#  define CAN_CON_PXEDIS		BIT(6)
#  define CAN_CON_WAKFIL		BIT(8)
#  define CAN_CON_WFT_BITS		2
#  define CAN_CON_WFT_SHIFT		9
#  define CAN_CON_WFT_MASK					\
	GENMASK(CAN_CON_WFT_SHIFT + CAN_CON_WFT_BITS - 1,	\
		CAN_CON_WFT_SHIFT)
#  define CAN_CON_BUSY			BIT(11)
#  define CAN_CON_BRSDIS		BIT(12)
#  define CAN_CON_RTXAT			BIT(16)
#  define CAN_CON_ESIGM			BIT(17)
#  define CAN_CON_SERR2LOM		BIT(18)
#  define CAN_CON_STEF			BIT(19)
#  define CAN_CON_TXQEN			BIT(20)
#  define CAN_CON_OPMODE_BITS		3
#  define CAN_CON_OPMOD_SHIFT		21
#  define CAN_CON_OPMOD_MASK					\
	GENMASK(CAN_CON_OPMOD_SHIFT + CAN_CON_OPMODE_BITS -1,	\
		CAN_CON_OPMOD_SHIFT)
#  define CAN_CON_REQOP_BITS		3
#  define CAN_CON_REQOP_SHIFT		24
#  define CAN_CON_REQOP_MASK					\
	GENMASK(CAN_CON_REQOP_SHIFT + CAN_CON_REQOP_BITS - 1,	\
		CAN_CON_REQOP_SHIFT)
#    define CAN_CON_MODE_MIXED			0
#    define CAN_CON_MODE_SLEEP			1
#    define CAN_CON_MODE_INTERNAL_LOOPBACK	2
#    define CAN_CON_MODE_LISTENONLY		3
#    define CAN_CON_MODE_CONFIG			4
#    define CAN_CON_MODE_EXTERNAL_LOOPBACK	5
#    define CAN_CON_MODE_CAN2_0			6
#    define CAN_CON_MODE_RESTRICTED		7
#  define CAN_CON_ABAT			BIT(27)
#  define CAN_CON_TXBWS_BITS		3
#  define CAN_CON_TXBWS_SHIFT		28
#  define CAN_CON_TXBWS_MASK					\
	GENMASK(CAN_CON_TXBWS_SHIFT + CAN_CON_TXBWS_BITS - 1,	\
		CAN_CON_TXBWS_SHIFT)
#  define CAN_CON_DEFAULT					\
	( CAN_CON_ISOCRCEN					\
	  | CAN_CON_PXEDIS					\
	  | CAN_CON_WAKFIL					\
	  | (3 << CAN_CON_WFT_SHIFT)				\
	  | CAN_CON_STEF					\
	  | CAN_CON_TXQEN					\
	  | (CAN_CON_MODE_CONFIG << CAN_CON_OPMOD_SHIFT)	\
	  | (CAN_CON_MODE_CONFIG << CAN_CON_REQOP_SHIFT)	\
	)
#  define CAN_CON_DEFAULT_MASK			\
	( CAN_CON_DNCNT_MASK			\
	  |CAN_CON_ISOCRCEN			\
	  | CAN_CON_PXEDIS			\
	  | CAN_CON_WAKFIL			\
	  | CAN_CON_WFT_MASK			\
	  | CAN_CON_BRSDIS			\
	  | CAN_CON_RTXAT			\
	  | CAN_CON_ESIGM			\
	  | CAN_CON_SERR2LOM			\
	  | CAN_CON_STEF			\
	  | CAN_CON_TXQEN			\
	  | CAN_CON_OPMOD_MASK			\
	  | CAN_CON_REQOP_MASK			\
	  | CAN_CON_ABAT			\
	  | CAN_CON_TXBWS_MASK			\
	)
#define CAN_NBTCFG			CAN_SFR_BASE(0x04)
#  define CAN_NBTCFG_SJW_BITS		7
#  define CAN_NBTCFG_SJW_SHIFT		0
#  define CAN_NBTCFG_SJW_MASK					\
	GENMASK(CAN_NBTCFG_SJW_SHIFT + CAN_NBTCFG_SJW_BITS - 1, \
		CAN_NBTCFG_SJW_SHIFT)
#  define CAN_NBTCFG_TSEG2_BITS		7
#  define CAN_NBTCFG_TSEG2_SHIFT	8
#  define CAN_NBTCFG_TSEG2_MASK					    \
	GENMASK(CAN_NBTCFG_TSEG2_SHIFT + CAN_NBTCFG_TSEG2_BITS - 1, \
		CAN_NBTCFG_TSEG2_SHIFT)
#  define CAN_NBTCFG_TSEG1_BITS		8
#  define CAN_NBTCFG_TSEG1_SHIFT	16
#  define CAN_NBTCFG_TSEG1_MASK					    \
	GENMASK(CAN_NBTCFG_TSEG1_SHIFT + CAN_NBTCFG_TSEG1_BITS - 1, \
		CAN_NBTCFG_TSEG1_SHIFT)
#  define CAN_NBTCFG_BRP_BITS		8
#  define CAN_NBTCFG_BRP_SHIFT		24
#  define CAN_NBTCFG_BRP_MASK					\
	GENMASK(CAN_NBTCFG_BRP_SHIFT + CAN_NBTCFG_BRP_BITS - 1, \
		CAN_NBTCFG_BRP_SHIFT)
#define CAN_DBTCFG			CAN_SFR_BASE(0x08)
#  define CAN_DBTCFG_SJW_BITS		4
#  define CAN_DBTCFG_SJW_SHIFT		0
#  define CAN_DBTCFG_SJW_MASK					\
	GENMASK(CAN_DBTCFG_SJW_SHIFT + CAN_DBTCFG_SJW_BITS - 1, \
		CAN_DBTCFG_SJW_SHIFT)
#  define CAN_DBTCFG_TSEG2_BITS		4
#  define CAN_DBTCFG_TSEG2_SHIFT	8
#  define CAN_DBTCFG_TSEG2_MASK					    \
	GENMASK(CAN_DBTCFG_TSEG2_SHIFT + CAN_DBTCFG_TSEG2_BITS - 1, \
		CAN_DBTCFG_TSEG2_SHIFT)
#  define CAN_DBTCFG_TSEG1_BITS		5
#  define CAN_DBTCFG_TSEG1_SHIFT	16
#  define CAN_DBTCFG_TSEG1_MASK					    \
	GENMASK(CAN_DBTCFG_TSEG1_SHIFT + CAN_DBTCFG_TSEG1_BITS - 1, \
		CAN_DBTCFG_TSEG1_SHIFT)
#  define CAN_DBTCFG_BRP_BITS		8
#  define CAN_DBTCFG_BRP_SHIFT		24
#  define CAN_DBTCFG_BRP_MASK					\
	GENMASK(CAN_DBTCFG_BRP_SHIFT + CAN_DBTCFG_BRP_BITS - 1, \
		CAN_DBTCFG_BRP_SHIFT)
#define CAN_TDC				CAN_SFR_BASE(0x0C)
#  define CAN_TDC_TDCV_BITS		5
#  define CAN_TDC_TDCV_SHIFT		0
#  define CAN_TDC_TDCV_MASK					\
	GENMASK(CAN_TDC_TDCV_SHIFT + CAN_TDC_TDCV_BITS - 1, \
		CAN_TDC_TDCV_SHIFT)
#  define CAN_TDC_TDCO_BITS		5
#  define CAN_TDC_TDCO_SHIFT		8
#  define CAN_TDC_TDCO_MASK					\
	GENMASK(CAN_TDC_TDCO_SHIFT + CAN_TDC_TDCO_BITS - 1, \
		CAN_TDC_TDCO_SHIFT)
#  define CAN_TDC_TDCMOD_BITS		2
#  define CAN_TDC_TDCMOD_SHIFT		16
#  define CAN_TDC_TDCMOD_MASK					\
	GENMASK(CAN_TDC_TDCMOD_SHIFT + CAN_TDC_TDCMOD_BITS - 1, \
		CAN_TDC_TDCMOD_SHIFT)
#  define CAN_TDC_SID11EN		BIT(24)
#  define CAN_TDC_EDGFLTEN		BIT(25)
#define CAN_TBC				CAN_SFR_BASE(0x10)
#define CAN_TSCON			CAN_SFR_BASE(0x14)
#  define CAN_TSCON_TBCPRE_BITS		10
#  define CAN_TSCON_TBCPRE_SHIFT	0
#  define CAN_TSCON_TBCPRE_MASK					    \
	GENMASK(CAN_TSCON_TBCPRE_SHIFT + CAN_TSCON_TBCPRE_BITS - 1, \
		CAN_TSCON_TBCPRE_SHIFT)
#  define CAN_TSCON_TBCEN		BIT(24)
#  define CAN_TSCON_TSEOF		BIT(25)
#  define CAN_TSCON_TSRES		BIT(26)
#define CAN_VEC				CAN_SFR_BASE(0x18)
#  define CAN_VEC_ICODE_BITS		7
#  define CAN_VEC_ICODE_SHIFT		0
#  define CAN_VEC_ICODE_MASK					    \
	GENMASK(CAN_VEC_ICODE_SHIFT + CAN_VEC_ICODE_BITS - 1,	    \
		CAN_VEC_ICODE_SHIFT)
#  define CAN_VEC_FILHIT_BITS		5
#  define CAN_VEC_FILHIT_SHIFT		8
#  define CAN_VEC_FILHIT_MASK					\
	GENMASK(CAN_VEC_FILHIT_SHIFT + CAN_VEC_FILHIT_BITS - 1, \
		CAN_VEC_FILHIT_SHIFT)
#  define CAN_VEC_TXCODE_BITS		7
#  define CAN_VEC_TXCODE_SHIFT		16
#  define CAN_VEC_TXCODE_MASK					\
	GENMASK(CAN_VEC_TXCODE_SHIFT + CAN_VEC_TXCODE_BITS - 1, \
		CAN_VEC_TXCODE_SHIFT)
#  define CAN_VEC_RXCODE_BITS		7
#  define CAN_VEC_RXCODE_SHIFT		24
#  define CAN_VEC_RXCODE_MASK					\
	GENMASK(CAN_VEC_RXCODE_SHIFT + CAN_VEC_RXCODE_BITS - 1, \
		CAN_VEC_RXCODE_SHIFT)
#define CAN_INT				CAN_SFR_BASE(0x1C)
#  define CAN_INT_TXIF			BIT(0)
#  define CAN_INT_RXIF			BIT(1)
#  define CAN_INT_TBCIF			BIT(2)
#  define CAN_INT_MODIF			BIT(3)
#  define CAN_INT_TEFIF			BIT(4)
#  define CAN_INT_ECCIF			BIT(8)
#  define CAN_INT_SPICRCIF		BIT(9)
#  define CAN_INT_TXATIF		BIT(10)
#  define CAN_INT_RXOVIF		BIT(11)
#  define CAN_INT_SERRIF		BIT(12)
#  define CAN_INT_CERRIF		BIT(13)
#  define CAN_INT_WAKIF			BIT(14)
#  define CAN_INT_IVMIF			BIT(15)
#  define CAN_INT_TXIE			BIT(16)
#  define CAN_INT_RXIE			BIT(17)
#  define CAN_INT_TBCIE			BIT(18)
#  define CAN_INT_MODIE			BIT(19)
#  define CAN_INT_TEFIE			BIT(20)
#  define CAN_INT_ECCIE			BIT(24)
#  define CAN_INT_SPICRCIE		BIT(25)
#  define CAN_INT_TXATIE		BIT(26)
#  define CAN_INT_RXOVIE		BIT(27)
#  define CAN_INT_SERRIE		BIT(29)
#  define CAN_INT_WAKIE			BIT(30)
#  define CAN_INT_IVMIE			BIT(31)
#define CAN_RXIF			CAN_SFR_BASE(0x20)
#define CAN_TXIF			CAN_SFR_BASE(0x24)
#define CAN_RXOVIF			CAN_SFR_BASE(0x28)
#define CAN_TXATIF			CAN_SFR_BASE(0x2C)
#define CAN_TXREQ			CAN_SFR_BASE(0x30)
#define CAN_TREC			CAN_SFR_BASE(0x34)
#  define CAN_TREC_REC_BITS		8
#  define CAN_TREC_REC_SHIFT		0
#  define CAN_TREC_REC_MASK				    \
	GENMASK(CAN_TREC_REC_SHIFT + CAN_TREC_REC_BITS - 1, \
		CAN_TREC_REC_SHIFT)
#  define CAN_TREC_TEC_BITS		8
#  define CAN_TREC_TEC_SHIFT		8
#  define CAN_TREC_TEC_MASK				    \
	GENMASK(CAN_TREC_TEC_SHIFT + CAN_TREC_TEC_BITS - 1, \
		CAN_TREC_TEC_SHIFT)
#  define CAN_TREC_EWARN		BIT(16)
#  define CAN_TREC_RXWARN		BIT(17)
#  define CAN_TREC_TXWARN		BIT(18)
#  define CAN_TREC_RXBP			BIT(19)
#  define CAN_TREC_TXBP			BIT(20)
#  define CAN_TREC_TXBO			BIT(21)
#define CAN_BDIAG0			CAN_SFR_BASE(0x38)
#  define CAN_BDIAG0_NRERRCNT_BITS	8
#  define CAN_BDIAG0_NRERRCNT_SHIFT	0
#  define CAN_BDIAG0_NRERRCNT_MASK				\
	GENMASK(CAN_BDIAG0_NRERRCNT_SHIFT + CAN_BDIAG0_NRERRCNT_BITS - 1, \
		CAN_BDIAG0_NRERRCNT_SHIFT)
#  define CAN_BDIAG0_NTERRCNT_BITS	8
#  define CAN_BDIAG0_NTERRCNT_SHIFT	8
#  define CAN_BDIAG0_NTERRCNT_MASK					\
	GENMASK(CAN_BDIAG0_NTERRCNT_SHIFT + CAN_BDIAG0_NTERRCNT_BITS - 1, \
		CAN_BDIAG0_NTERRCNT_SHIFT)
#  define CAN_BDIAG0_DRERRCNT_BITS	8
#  define CAN_BDIAG0_DRERRCNT_SHIFT	16
#  define CAN_BDIAG0_DRERRCNT_MASK					\
	GENMASK(CAN_BDIAG0_DRERRCNT_SHIFT + CAN_BDIAG0_DRERRCNT_BITS - 1, \
		CAN_BDIAG0_DRERRCNT_SHIFT)
#  define CAN_BDIAG0_DTERRCNT_BITS	8
#  define CAN_BDIAG0_DTERRCNT_SHIFT	24
#  define CAN_BDIAG0_DTERRCNT_MASK					\
	GENMASK(CAN_BDIAG0_DTERRCNT_SHIFT + CAN_BDIAG0_DTERRCNT_BITS - 1, \
		CAN_BDIAG0_DTERRCNT_SHIFT)
#define CAN_BDIAG1			CAN_SFR_BASE(0x3C)
#  define CAN_BDIAG1_EFMSGCNT_BITS	16
#  define CAN_BDIAG1_EFMSGCNT_SHIFT	0
#  define CAN_BDIAG1_EFMSGCNT_MASK					\
	GENMASK(CAN_BDIAG1_EFMSGCNT_SHIFT + CAN_BDIAG1_EFMSGCNT_BITS - 1, \
		CAN_BDIAG1_EFMSGCNT_SHIFT)
#  define CAN_BDIAG1_NBIT0ERR		BIT(16)
#  define CAN_BDIAG1_NBIT1ERR		BIT(17)
#  define CAN_BDIAG1_NACKERR		BIT(18)
#  define CAN_BDIAG1_NSTUFERR		BIT(19)
#  define CAN_BDIAG1_NFORMERR		BIT(20)
#  define CAN_BDIAG1_NCRCERR		BIT(21)
#  define CAN_BDIAG1_TXBOERR		BIT(23)
#  define CAN_BDIAG1_DBIT0ERR		BIT(24)
#  define CAN_BDIAG1_DBIT1ERR		BIT(25)
#  define CAN_BDIAG1_DFORMERR		BIT(27)
#  define CAN_BDIAG1_STUFERR		BIT(28)
#  define CAN_BDIAG1_DCRCERR		BIT(29)
#  define CAN_BDIAG1_ESI		BIT(30)
#  define CAN_BDIAG1_DLCMM		BIT(31)
#define CAN_TEFCON			CAN_SFR_BASE(0x40)
#  define CAN_TEFCON_TEFNEIE		BIT(0)
#  define CAN_TEFCON_TEFHIE		BIT(1)
#  define CAN_TEFCON_TEFFIE		BIT(2)
#  define CAN_TEFCON_TEFOVIE		BIT(3)
#  define CAN_TEFCON_TEFTSEN		BIT(5)
#  define CAN_TEFCON_UINC		BIT(8)
#  define CAN_TEFCON_FRESET		BIT(10)
#  define CAN_TEFCON_FSIZE_BITS		5
#  define CAN_TEFCON_FSIZE_SHIFT	24
#  define CAN_TEFCON_FSIZE_MASK					    \
	GENMASK(CAN_TEFCON_FSIZE_SHIFT + CAN_TEFCON_FSIZE_BITS - 1, \
		CAN_TEFCON_FSIZE_SHIFT)
#define CAN_TEFSTA			CAN_SFR_BASE(0x44)
#  define CAN_TEFSTA_TEFNEIF		BIT(0)
#  define CAN_TEFSTA_TEFHIF		BIT(1)
#  define CAN_TEFSTA_TEFFIF		BIT(2)
#  define CAN_TEFSTA_TEVOVIF		BIT(3)
#define CAN_TEFUA			CAN_SFR_BASE(0x48)
#define CAN_RESERVED			CAN_SFR_BASE(0x4C)
#define CAN_TXQCON			CAN_SFR_BASE(0x50)
#  define CAN_TXQCON_TXQNIE		BIT(0)
#  define CAN_TXQCON_TXQEIE		BIT(2)
#  define CAN_TXQCON_TXATIE		BIT(4)
#  define CAN_TXQCON_TXEN		BIT(7)
#  define CAN_TXQCON_UINC		BIT(8)
#  define CAN_TXQCON_TXREQ		BIT(9)
#  define CAN_TXQCON_FRESET		BIT(10)
#  define CAN_TXQCON_TXPRI_BITS		5
#  define CAN_TXQCON_TXPRI_SHIFT	16
#  define CAN_TXQCON_TXPRI_MASK					    \
	GENMASK(CAN_TXQCON_TXPRI_SHIFT + CAN_TXQCON_TXPRI_BITS - 1, \
		CAN_TXQCON_TXPRI_SHIFT)
#  define CAN_TXQCON_TXAT_BITS		2
#  define CAN_TXQCON_TXAT_SHIFT		21
#  define CAN_TXQCON_TXAT_MASK					    \
	GENMASK(CAN_TXQCON_TXAT_SHIFT + CAN_TXQCON_TXAT_BITS - 1, \
		CAN_TXQCON_TXAT_SHIFT)
#  define CAN_TXQCON_FSIZE_BITS		5
#  define CAN_TXQCON_FSIZE_SHIFT	24
#  define CAN_TXQCON_FSIZE_MASK					    \
	GENMASK(CAN_TXQCON_FSIZE_SHIFT + CAN_TXQCON_FSIZE_BITS - 1, \
		CAN_TXQCON_FSIZE_SHIFT)
#  define CAN_TXQCON_PLSIZE_BITS	3
#  define CAN_TXQCON_PLSIZE_SHIFT	29
#  define CAN_TXQCON_PLSIZE_MASK				      \
	GENMASK(CAN_TXQCON_PLSIZE_SHIFT + CAN_TXQCON_PLSIZE_BITS - 1, \
		CAN_TXQCON_PLSIZE_SHIFT)
#    define CAN_TXQCON_PLSIZE_8		0
#    define CAN_TXQCON_PLSIZE_12	1
#    define CAN_TXQCON_PLSIZE_16	2
#    define CAN_TXQCON_PLSIZE_20	3
#    define CAN_TXQCON_PLSIZE_24	4
#    define CAN_TXQCON_PLSIZE_32	5
#    define CAN_TXQCON_PLSIZE_48	6
#    define CAN_TXQCON_PLSIZE_64	7

#define CAN_TXQSTA			CAN_SFR_BASE(0x54)
#  define CAN_TXQSTA_TXQNIF		BIT(0)
#  define CAN_TXQSTA_TXQEIF		BIT(2)
#  define CAN_TXQSTA_TXATIF		BIT(4)
#  define CAN_TXQSTA_TXERR		BIT(5)
#  define CAN_TXQSTA_TXLARB		BIT(6)
#  define CAN_TXQSTA_TXABT		BIT(7)
#  define CAN_TXQSTA_TXQCI_BITS		5
#  define CAN_TXQSTA_TXQCI_SHIFT	8
#  define CAN_TXQSTA_TXQCI_MASK					    \
	GENMASK(CAN_TXQSTA_TXQCI_SHIFT + CAN_TXQSTA_TXQCI_BITS - 1, \
		CAN_TXQSTA_TXQCI_SHIFT)

#define CAN_TXQUA			CAN_SFR_BASE(0x58)
#define CAN_FIFOCON(x)			CAN_SFR_BASE(0x5C + 12 * (x - 1))
#define CAN_FIFOCON_TFNRFNIE		BIT(0)
#define CAN_FIFOCON_TFHRFHIE		BIT(1)
#define CAN_FIFOCON_TFERFFIE		BIT(2)
#define CAN_FIFOCON_RXOVIE		BIT(3)
#define CAN_FIFOCON_TXATIE		BIT(4)
#define CAN_FIFOCON_RXTSEN		BIT(5)
#define CAN_FIFOCON_RTREN		BIT(6)
#define CAN_FIFOCON_TXEN		BIT(7)
#define CAN_FIFOCON_UINC		BIT(8)
#define CAN_FIFOCON_TXREQ		BIT(9)
#define CAN_FIFOCON_FRESET		BIT(10)
#  define CAN_FIFOCON_TXPRI_BITS	5
#  define CAN_FIFOCON_TXPRI_SHIFT	16
#  define CAN_FIFOCON_TXPRI_MASK					\
	GENMASK(CAN_FIFOCON_TXPRI_SHIFT + CAN_FIFOCON_TXPRI_BITS - 1,	\
		CAN_FIFOCON_TXPRI_SHIFT)
#  define CAN_FIFOCON_TXAT_BITS		2
#  define CAN_FIFOCON_TXAT_SHIFT	21
#  define CAN_FIFOCON_TXAT_MASK					    \
	GENMASK(CAN_FIFOCON_TXAT_SHIFT + CAN_FIFOCON_TXAT_BITS - 1, \
		CAN_FIFOCON_TXAT_SHIFT)
#  define CAN_FIFOCON_FSIZE_BITS	5
#  define CAN_FIFOCON_FSIZE_SHIFT	24
#  define CAN_FIFOCON_FSIZE_MASK					\
	GENMASK(CAN_FIFOCON_FSIZE_SHIFT + CAN_FIFOCON_FSIZE_BITS - 1,	\
		CAN_FIFOCON_FSIZE_SHIFT)
#  define CAN_FIFOCON_PLSIZE_BITS	3
#  define CAN_FIFOCON_PLSIZE_SHIFT	29
#  define CAN_FIFOCON_PLSIZE_MASK					\
	GENMASK(CAN_FIFOCON_PLSIZE_SHIFT + CAN_FIFOCON_PLSIZE_BITS - 1, \
		CAN_FIFOCON_PLSIZE_SHIFT)
#define CAN_FIFOSTA(x)			CAN_SFR_BASE(0x60 + 12 * (x - 1))
#  define CAN_FIFOSTA_TFNRFNIE		BIT(0)
#  define CAN_FIFOSTA_TFHRFHIE		BIT(1)
#  define CAN_FIFOSTA_TFERFFIE		BIT(2)
#  define CAN_FIFOSTA_RXOVIE		BIT(3)
#  define CAN_FIFOSTA_TXATIE		BIT(4)
#  define CAN_FIFOSTA_RXTSEN		BIT(5)
#  define CAN_FIFOSTA_RTREN		BIT(6)
#  define CAN_FIFOSTA_TXEN		BIT(7)
#  define CAN_FIFOSTA_FIFOCI_BITS	5
#  define CAN_FIFOSTA_FIFOCI_SHIFT	8
#  define CAN_FIFOSTA_FIFOCI_MASK					\
	GENMASK(CAN_FIFOSTA_FIFOCI_SHIFT + CAN_FIFOSTA_FIFOCI_BITS - 1, \
		CAN_FIFOSTA_FIFOCI_SHIFT)
#define CAN_FIFOUA(x)			CAN_SFR_BASE(0x64 + 12 * (x - 1))
#define CAN_FLTCON(x)			CAN_SFR_BASE(0x1D0 + (x >> 2))
#  define CAN_FILCON_SHIFT(x)		((x & 3) * 8)
#  define CAN_FILCON_BITS(x)		4
#  define CAN_FILCON_MASK(x)					\
	GENMASK(CAN_FILCON_SHIFT(x) + CAN_FILCON_BITS(x) - 1,	\
		CAN_FILCON_SHIFT(x))
#  define CAN_FIFOCON_FLTEN(x)		BIT(7 + CAN_FILCON_SHIFT(x))
#define CAN_FLTOBJ(x)			CAN_SFR_BASE(0x1F0 + 8 * x)
#  define CAN_FILOBJ_SID_BITS		11
#  define CAN_FILOBJ_SID_SHIFT		0
#  define CAN_FILOBJ_SID_MASK					\
	GENMASK(CAN_FILOBJ_SID_SHIFT + CAN_FILOBJ_SID_BITS - 1, \
		CAN_FILOBJ_SID_SHIFT)
#  define CAN_FILOBJ_EID_BITS		18
#  define CAN_FILOBJ_EID_SHIFT		12
#  define CAN_FILOBJ_EID_MASK					\
	GENMASK(CAN_FILOBJ_EID_SHIFT + CAN_FILOBJ_EID_BITS - 1, \
		CAN_FILOBJ_EID_SHIFT)
#  define CAN_FILOBJ_SID11		BIT(29)
#  define CAN_FILOBJ_EXIDE		BIT(30)
#define CAN_FLTMASK(x)			CAN_SFR_BASE(0x1F4 + 8 * x)
#  define CAN_FILMASK_MSID_BITS		11
#  define CAN_FILMASK_MSID_SHIFT	0
#  define CAN_FILMASK_MSID_MASK					\
	GENMASK(CAN_FILMASK_MSID_SHIFT + CAN_FILMASK_MSID_BITS - 1, \
		CAN_FILMASK_MSID_SHIFT)
#  define CAN_FILMASK_MEID_BITS		18
#  define CAN_FILMASK_MEID_SHIFT	12
#  define CAN_FILMASK_MEID_MASK					\
	GENMASK(CAN_FILMASK_MEID_SHIFT + CAN_FILMASK_MEID_BITS - 1, \
		CAN_FILMASK_MEID_SHIFT)
#  define CAN_FILMASK_MSID11		BIT(29)
#  define CAN_FILMASK_MIDE		BIT(30)

#define CAN_OBJ_ID_SID_BITS		11
#define CAN_OBJ_ID_SID_SHIFT		0
#define CAN_OBJ_ID_SID_MASK				\
	GENMASK(CAN_ID_SID_SHIFT + CAN_ID_SID_BITS - 1, \
		CAN_ID_SID_SHIFT)
#define CAN_OBJ_ID_EID_BITS		18
#define CAN_OBJ_ID_EID_SHIFT		11
#define CAN_OBJ_ID_EID_MASK				\
	GENMASK(CAN_ID_EID_SHIFT + CAN_ID_EID_BITS - 1, \
		CAN_ID_EID_SHIFT)
#define CAN_OBJ_ID_SID_BIT11		BIT(29)

#define CAN_OBJ_FLAGS_DLC_BITS		4
#define CAN_OBJ_FLAGS_DLC_SHIFT		0
#define CAN_OBJ_FLAGS_DLC_MASK				      \
	GENMASK(CAN_FLAGS_DLC_SHIFT + CAN_FLAGS_DLC_BITS - 1, \
		CAN_FLAGS_DLC_SHIFT)
#define CAN_OBJ_FLAGS_IDE		BIT(4)
#define CAN_OBJ_FLAGS_RTR		BIT(5)
#define CAN_OBJ_FLAGS_BRS		BIT(6)
#define CAN_OBJ_FLAGS_FDF		BIT(7)
#define CAN_OBJ_FLAGS_ESI		BIT(8)
#define CAN_OBJ_FLAGS_SEQ_BITS		7
#define CAN_OBJ_FLAGS_SEQ_SHIFT		9
#define CAN_OBJ_FLAGS_SEQ_MASK				      \
	GENMASK(CAN_FLAGS_SEQ_SHIFT + CAN_FLAGS_SEQ_BITS - 1, \
		CAN_FLAGS_SEQ_SHIFT)
#define CAN_OBJ_FLAGS_FILHIT_BITS	11
#define CAN_OBJ_FLAGS_FILHIT_SHIFT	5
#define CAN_OBJ_FLAGS_FILHIT_MASK				      \
	GENMASK(CAN_FLAGS_SEQ_SHIFT + CAN_FLAGS_SEQ_BITS - 1, \
		CAN_FLAGS_SEQ_SHIFT)

struct mcp2517fd_obj_tef {
	u32 id;
	u32 flags;
	u32 ts;
};

struct mcp2517fd_obj_tx {
	u32 id;
	u32 flags;
	u32 data[];
};

struct mcp2517fd_obj_rx {
	u32 id;
	u32 flags;
	u32 ts;
	u8 data[];
};

/* the prepared transmit spi message */
struct mcp2517fd_obj_tx_msg {
	int skb_idx;
	unsigned int min_length;
	struct {
		struct spi_message msg;
		struct spi_transfer xfer;
		u16 cmd_addr;
		u8 obj[sizeof(struct mcp2517fd_obj_tx)];
		u8 payload[64];
	} data;
	struct {
		struct spi_message msg;
		struct spi_transfer xfer;
		u16 cmd_addr;
		u8 data;
	} trigger;
};

#define FIFO_DATA(x)			(0x400 + (x))
#define FIFO_DATA_SIZE			0x800

#define RX_FIFO			       1
#define TX_FIFO(i)		       (2+i)

static const struct can_bittiming_const mcp2517fd_nominal_bittiming_const = {
	.name		= DEVICE_NAME,
	.tseg1_min	= 2,
	.tseg1_max	= BIT(CAN_NBTCFG_TSEG1_BITS),
	.tseg2_min	= 1,
	.tseg2_max	= BIT(CAN_NBTCFG_TSEG2_BITS),
	.sjw_max	= BIT(CAN_NBTCFG_SJW_BITS),
	.brp_min	= 1,
	.brp_max	= BIT(CAN_NBTCFG_BRP_BITS),
	.brp_inc	= 1,
};

static const struct can_bittiming_const mcp2517fd_data_bittiming_const = {
	.name		= DEVICE_NAME,
	.tseg1_min	= 1,
	.tseg1_max	= BIT(CAN_DBTCFG_TSEG1_BITS),
	.tseg2_min	= 1,
	.tseg2_max	= BIT(CAN_DBTCFG_TSEG2_BITS),
	.sjw_max	= BIT(CAN_DBTCFG_SJW_BITS),
	.brp_min	= 1,
	.brp_max	= BIT(CAN_DBTCFG_BRP_BITS),
	.brp_inc	= 1,
};

enum mcp2517fd_model {
	CAN_MCP2517FD	= 0x2517,
};

enum mcp2517fd_gpio_mode {
	gpio_mode_int		= 0,
	gpio_mode_standby	= MCP2517FD_IOCON_XSTBYEN,
	gpio_mode_out_low	= MCP2517FD_IOCON_PM0,
	gpio_mode_out_high	= MCP2517FD_IOCON_PM0 | MCP2517FD_IOCON_LAT0,
	gpio_mode_in		= MCP2517FD_IOCON_PM0 | MCP2517FD_IOCON_TRIS0
};

struct mcp2517fd_priv {
	struct can_priv	   can;
	struct net_device *net;
	struct spi_device *spi;

	enum mcp2517fd_model model;
	bool clock_pll;
	bool clock_div2;
	int  clock_odiv;

	enum mcp2517fd_gpio_mode  gpio0_mode;
	enum mcp2517fd_gpio_mode  gpio1_mode;
	bool gpio_opendrain;

	/* flags that should stay in the con_register */
	u32 con_val;

	int spi_max_speed_hz;
	int spi_setup_speed_hz;
	int spi_normal_speed_hz;

	struct mutex mcp_lock; /* SPI device lock */

	int payload_size;
	u8 payload_mode;

	struct mutex txfifo_lock;
	u8 tx_fifos;
	struct mcp2517fd_obj_tx_msg tx_msg[32];
	u32 tx_pending_mask;

	u32 tef_address_start;
	u32 tef_address_end;
	u32 tef_address;

	u8 rx_fifos;
	u32 rx_address_start;
	u32 rx_address_inc;
	u32 rx_address_end;
	u32 rx_address;

	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
#define AFTER_SUSPEND_POWER 4
#define AFTER_SUSPEND_RESTART 8
	int restart_tx;
	struct regulator *power;
	struct regulator *transceiver;
	struct clk *clk;
};

static void mcp2517fd_calc_cmd_addr(u16 cmd, u16 addr, u8 *data)
{
	cmd = cmd | (addr & ADDRESS_MASK);

	data[0] = (cmd >> 8) & 0xff;
	data[1] = (cmd >> 0) & 0xff;
}

static int mcp2517fd_cmd_reset(struct spi_device *spi)
{
	u8 tx_buf[2];

	mcp2517fd_calc_cmd_addr(INSTRUCTION_RESET, 0, tx_buf);

	/* write the reset command */
	return spi_write(spi, &tx_buf, 2);
}

/*
#define DEBUG
*/

/* read a register, but we are only interrested in a few bytes */
static int mcp2517fd_cmd_read_mask(struct spi_device *spi, u32 reg,
				   u32 *data, u32 mask)
{
	int first_byte, last_byte, len_byte;
	int ret;
	u8 cmd[2];

	/* check that at least one bit is set */
	if (! mask)
		return -EINVAL;

	/* calculate first and last byte used */
	first_byte = (ffs(mask) - 1)  >> 3;
	last_byte = (fls(mask) - 1)  >> 3;
	len_byte = last_byte - first_byte +1;

	/* prepare command */
	mcp2517fd_calc_cmd_addr(INSTRUCTION_READ, reg + first_byte, cmd);

	/* now execute the command */
	*data = 0;
	ret = spi_write_then_read(spi,
				  &cmd, 2,
				  data + first_byte,
				  last_byte - first_byte +1);
	if (ret)
		return ret;

	/* convert it to correct cpu ordering */
	*data = le32_to_cpu(*data);
#ifdef DEBUG
	dev_err(&spi->dev, "read_mask: %02x%02x = 0x%08x\n",
		cmd[0],cmd[1], *data);
	dev_err(&spi->dev, "\t\tmask: %08x [%i - %i] %i\n",
		mask, first_byte, last_byte, len_byte);
#endif
	return 0;
}

static int mcp2517fd_cmd_read(struct spi_device *spi, u32 reg, u32 *data)
{
	return mcp2517fd_cmd_read_mask(spi, reg, data, -1);
}

/* read a register, but we are only interrested in a few bytes */
static int mcp2517fd_cmd_write_mask(struct spi_device *spi, u32 reg,
				    u32 data, u32 mask)
{
	int first_byte, last_byte, len_byte;
	u8 txdata[6];

	/* check that at least one bit is set */
	if (! mask)
		return -EINVAL;

	/* calculate first and last byte used */
	first_byte = (ffs(mask) - 1)  >> 3;
	last_byte = (fls(mask) - 1)  >> 3;
	len_byte = last_byte - first_byte + 1;

	/* prepare buffer */
	mcp2517fd_calc_cmd_addr(INSTRUCTION_WRITE, reg + first_byte, txdata);
	data = cpu_to_le32(data);
	memcpy(txdata + 2, &data + first_byte, len_byte);
#ifdef DEBUG
	dev_err(&spi->dev, "write_mask 0x%02x%02x = 0x%08x\n",
		txdata[0],txdata[1], data);
	dev_err(&spi->dev, "\t\tmask: %08x [%i - %i] %i\n",
		mask, first_byte, last_byte, len_byte);
#endif
	/* now execute the command */
	return spi_write(spi, txdata, len_byte + 2);
}

static int mcp2517fd_cmd_write(struct spi_device *spi, u32 reg, u32 data)
{
	return mcp2517fd_cmd_write_mask(spi, reg, data, -1);
}

static void dump_reg(struct spi_device *spi)
{
	int i;
	u32 val;
	for (i=0; i < 4096; i += 4) {
		mcp2517fd_cmd_read(spi,i, &val);
		if (val)
			dev_err(&spi->dev, "  REG %03x = %08x\n", i, val);
	}
}

static int mcp2517fd_transmit_message_common(
	struct spi_device *spi, struct mcp2517fd_obj_tx_msg *msg,
	struct mcp2517fd_obj_tx *obj, int len, u8 *data)
{
	int ret;

	/* transform to le32*/
	dev_err(&spi->dev, "XXX - %08x - %08x\n", obj->id, obj->flags);
	obj->id = cpu_to_le32(obj->id);
	obj->flags = cpu_to_le32(obj->flags);
	dev_err(&spi->dev, "YYY - %08x - %08x\n", obj->id, obj->flags);

	/* copy data to non-aligned - without leaking heap data */
	memset(msg->data.payload, 0, sizeof(*msg->data.payload));
	memcpy(msg->data.obj, obj, sizeof(*obj));
	memcpy(msg->data.payload, data, len);

	/* transfers to FIFO RAM have to be multiple of 4 */
	msg->data.xfer.len = msg->min_length + ALIGN(len, 4);

	/*
	 *  and submit async transfers
	 * - surprizingly 2 separate work quicker than 1
	 */
	ret = spi_async(spi, &msg->data.msg);
	dev_err(&spi->dev," spi_async: %i - %i", ret, msg->data.xfer.speed_hz);
	if (ret)
		return NETDEV_TX_BUSY;
	ret = spi_async(spi, &msg->trigger.msg);
	dev_err(&spi->dev," spi_async: %i", ret);
	if (ret)
		return NETDEV_TX_BUSY;

	return NETDEV_TX_OK;
}

static int mcp2517fd_transmit_fdmessage(struct spi_device *spi, int prio,
				      struct canfd_frame *frame)
{
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	struct mcp2517fd_obj_tx_msg * msg = &priv->tx_msg[prio];
	struct mcp2517fd_obj_tx obj;
	int dlc = can_len2dlc(frame->len);

	frame->len = can_dlc2len(dlc);

	if (frame->can_id & CAN_EFF_FLAG) {
		obj.id = frame->can_id & CAN_EFF_MASK;
		obj.flags = CAN_OBJ_FLAGS_IDE;
	} else {
		obj.id = frame->can_id & CAN_SFF_MASK;
		obj.flags = 0;
	}

	obj.flags |= (dlc << CAN_OBJ_FLAGS_DLC_SHIFT) |
		(prio << CAN_OBJ_FLAGS_SEQ_SHIFT) |
		(frame->can_id & CAN_EFF_FLAG) ? CAN_OBJ_FLAGS_IDE : 0 |
		(frame->can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0 |
		(frame->flags & CANFD_BRS) ? CAN_OBJ_FLAGS_BRS : 0 |
		(frame->flags & CANFD_ESI) ? CAN_OBJ_FLAGS_ESI : 0 |
		CAN_OBJ_FLAGS_FDF;

	return mcp2517fd_transmit_message_common(
		spi, msg, &obj, frame->len, frame->data);
}

static int mcp2517fd_transmit_message(struct spi_device *spi, int prio,
				      struct can_frame *frame)
{
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	struct mcp2517fd_obj_tx_msg * msg = &priv->tx_msg[prio];
	struct mcp2517fd_obj_tx obj;

	if (frame->can_dlc > 8)
		frame->can_dlc = 8;

	if (frame->can_id & CAN_EFF_FLAG) {
		obj.id = frame->can_id & CAN_EFF_MASK;
		obj.flags = CAN_OBJ_FLAGS_IDE;
	} else {
		obj.id = frame->can_id & CAN_SFF_MASK;
		obj.flags = 0;
	}

	dev_err(&spi->dev, "AAA %08x - %i\n", obj.flags, frame->can_dlc);

	obj.flags |=
		(frame->can_dlc << CAN_OBJ_FLAGS_DLC_SHIFT);
	dev_err(&spi->dev, "AAA %08x - %i\n", obj.flags, frame->can_dlc);
	obj.flags |=
		(prio << CAN_OBJ_FLAGS_SEQ_SHIFT) |
		(frame->can_id & CAN_EFF_FLAG) ? CAN_OBJ_FLAGS_IDE : 0 |
		(frame->can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0;

	dev_err(&spi->dev, "AAA %08x - %i\n", obj.flags, frame->can_dlc);

	return mcp2517fd_transmit_message_common(
		spi, msg, &obj, frame->can_dlc, frame->data);
}

static void mcp2517fd_hw_sleep(struct spi_device *spi)
{
}

static int mcp2517fd_power_enable(struct regulator *reg, int enable)
{
	if (IS_ERR_OR_NULL(reg))
		return 0;

	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static int mcp2517fd_do_set_mode(struct net_device *net, enum can_mode mode)
{

	switch (mode) {
	case CAN_MODE_START:
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mcp2517fd_do_set_nominal_bittiming(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct spi_device *spi = priv->spi;


	/* calculate nominal bit timing */
	u32 val = ((bt->sjw - 1) << CAN_NBTCFG_SJW_SHIFT)
		| ((bt->phase_seg2 - 1) << CAN_NBTCFG_TSEG2_SHIFT)
		| ((bt->phase_seg1 + bt->prop_seg - 1)
		   << CAN_NBTCFG_TSEG1_SHIFT)
		| ((bt->brp) << CAN_NBTCFG_BRP_SHIFT);

	return mcp2517fd_cmd_write(spi, CAN_NBTCFG, val);
}

static int mcp2517fd_do_set_data_bittiming(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct can_bittiming *bt = &priv->can.data_bittiming;
	struct spi_device *spi = priv->spi;

	/* calculate data bit timing */
	u32 val = ((bt->sjw - 1) << CAN_DBTCFG_SJW_SHIFT)
		| ((bt->phase_seg2 - 1) << CAN_DBTCFG_TSEG2_SHIFT)
		| ((bt->phase_seg1 + bt->prop_seg - 1)
		   << CAN_DBTCFG_TSEG1_SHIFT)
		| ((bt->brp) << CAN_DBTCFG_BRP_SHIFT);

	return mcp2517fd_cmd_write(spi, CAN_DBTCFG, val);
}

static netdev_tx_t mcp2517fd_start_xmit(struct sk_buff *skb,
					struct net_device *net)
{
        struct mcp2517fd_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
	unsigned long prio;
	int ret;

	if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	dev_err(&spi->dev, "start_xmit:\n\tmask: %08x\n", priv->tx_pending_mask);

	/* decide on fifo to assign */
	mutex_lock(&priv->txfifo_lock);
	prio = fls(priv->tx_pending_mask);
	if (prio >= priv->tx_fifos) {
		mutex_unlock(&priv->txfifo_lock);
		return NETDEV_TX_BUSY;
	}
	/* mark as pending */
	priv->tx_pending_mask |= BIT(prio);
	mutex_unlock(&priv->txfifo_lock);

	dev_err(&spi->dev, "\tprio: %lu of %i\n", prio, priv->tx_fifos);
	dev_err(&spi->dev, "\tmask: %08x\n", priv->tx_pending_mask);

	/* if we are the last one, then disable the queue */
	if (prio == priv->tx_fifos - 1) {
		netif_stop_queue(net);
		dump_reg(spi);
	}

	/* now process it for real */
	if (can_is_canfd_skb(skb))
		ret = mcp2517fd_transmit_fdmessage(
			spi, prio, (struct canfd_frame *)skb->data);
	else
		ret = mcp2517fd_transmit_message(
			spi, prio, (struct can_frame *)skb->data);

	/* keep it for reference until the message really got transmitted */
	if (ret == NETDEV_TX_OK)
		can_put_echo_skb(skb, net, prio);
	return ret;
}

static void mcp2517fd_open_clean(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	free_irq(spi->irq, priv);
	mcp2517fd_hw_sleep(spi);
	mcp2517fd_power_enable(priv->transceiver, 0);
	close_candev(net);
}

static int mcp2517fd_hw_probe(struct spi_device *spi)
{
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	u32 val;
	int ret;

	/* setup the SPI clock speed for setup */
	spi->max_speed_hz = priv->spi_setup_speed_hz;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	/* Wait for oscillator startup timer after power up */
	mdelay(MCP2517FD_OST_DELAY_MS);

	/* send a "blind" reset, hoping we are in Config mode */
	mcp2517fd_cmd_reset(spi);

	/* Wait for oscillator startup again */
	mdelay(MCP2517FD_OST_DELAY_MS);

	/* check clock register that the clock is ready or disabled */
	ret = mcp2517fd_cmd_read(spi, MCP2517FD_OSC, &val);
	if (ret)
		return ret;

	dev_err(&spi->dev, "Osc reg: %08x\n", val);

	/* there can only be one... */
	switch (val & (MCP2517FD_OSC_OSCRDY | MCP2517FD_OSC_OSCDIS)) {
	case MCP2517FD_OSC_OSCRDY: /* either the clock is ready */
		break;
	case MCP2517FD_OSC_OSCDIS: /* or the clock is disabled */
		/* setup clock with defaults - only CLOCKDIV 10 */
		ret = mcp2517fd_cmd_write(
			spi, MCP2517FD_OSC,
			MCP2517FD_OSC_CLKODIV_10
			<< MCP2517FD_OSC_CLKODIV_SHIFT);
		if (ret)
			return ret;
		break;
	default:
		/* otherwise it is no valid device (or in strange state) */

		/*
		 * if PLL is enabled but not ready, then there may be
		 * something "fishy"
		 * this happened during driver development
		 * (enabling pll, when when on wrong clock), so best warn about
		 * such a possibility
		 */
		if ((val & (MCP2517FD_OSC_PLLEN | MCP2517FD_OSC_PLLRDY))
		    == MCP2517FD_OSC_PLLEN)
			dev_err(&spi->dev,
				"mcp2517fd may be in a strange state"
				" - a power disconnect may be required\n");

		return -ENODEV;
		break;
	}

	/* check if we are in config mode already*/

	/* read CON register and match */
	ret = mcp2517fd_cmd_read(spi, CAN_CON, &val);
	if (ret)
		return ret;
	dev_dbg(&spi->dev, "CAN_CON 0x%08x\n",val);

	/* apply mask and check */
	if ((val & CAN_CON_DEFAULT_MASK) == CAN_CON_DEFAULT)
		return 0;

	/*
	 * as per datasheet a reset only works in Config Mode
	 * so as we have in principle no knowledge of the current
	 * mode that the controller is in we have no safe way
	 * to detect the device correctly
	 * hence we need to "blindly" put the controller into
	 * config mode.
	 * on the "save" side, the OSC reg has to be valid already,
	 * so there is a chance we got the controller...
	 */

	/* blindly force it into config mode */
	ret = mcp2517fd_cmd_write(spi, CAN_CON, CAN_CON_DEFAULT);
	if (ret)
		return ret;

	/* delay some time */
	mdelay(MCP2517FD_OST_DELAY_MS);

	/* reset can controller */
	mcp2517fd_cmd_reset(spi);

	/* delay some time */
	mdelay(MCP2517FD_OST_DELAY_MS);

	/* read CON register and match a final time */
	ret = mcp2517fd_cmd_read(spi, CAN_CON, &val);
	if (ret)
		return ret;
	dev_dbg(&spi->dev, "CAN_CON 0x%08x\n",val);

	/* apply mask and check */
	return ((val & CAN_CON_DEFAULT_MASK) != CAN_CON_DEFAULT) ?
		-ENODEV : 0;
}

static int mcp2517fd_set_normal_mode(struct spi_device *spi)
{
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	int type = 0;
	int ret;

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		type = CAN_CON_MODE_EXTERNAL_LOOPBACK;
	else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		type = CAN_CON_MODE_LISTENONLY;
	else if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		type = CAN_CON_MODE_MIXED;
	else
		type = CAN_CON_MODE_CAN2_0;

	/* set mode to normal */
	ret = mcp2517fd_cmd_write(spi, CAN_CON,
				     priv->con_val
				     | (type << CAN_CON_REQOP_SHIFT));
	if (ret)
		return ret;

	/* switch spi speed to "normal" */
	spi->max_speed_hz = priv->spi_normal_speed_hz;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	dev_err(&spi->dev, "TODO: mcp2517fd_set_normal_mode");
	return 0;
}

static int mcp2517fd_setup_osc(struct spi_device *spi)
{
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	int val = ((priv->clock_pll) ? MCP2517FD_OSC_PLLEN : 0)
		| ((priv->clock_div2) ? MCP2517FD_OSC_SCLKDIV : 0);
	int waitfor = ((priv->clock_pll) ? MCP2517FD_OSC_PLLRDY : 0)
		| ((priv->clock_div2) ? MCP2517FD_OSC_SCLKRDY : 0)
		| MCP2517FD_OSC_OSCRDY;
	int ret;
	unsigned long timeout;

	/* manage clock_out divider */
	switch(priv->clock_odiv) {
	case 10:
		val |= (MCP2517FD_OSC_CLKODIV_10)
			<< MCP2517FD_OSC_CLKODIV_SHIFT;
		break;
	case 4:
		val |= (MCP2517FD_OSC_CLKODIV_4)
			<< MCP2517FD_OSC_CLKODIV_SHIFT;
		break;
	case 2:
		val |= (MCP2517FD_OSC_CLKODIV_2)
			<< MCP2517FD_OSC_CLKODIV_SHIFT;
		break;
	case 1:
		val |= (MCP2517FD_OSC_CLKODIV_1)
			<< MCP2517FD_OSC_CLKODIV_SHIFT;
		break;
	case 0:
		/* this means implicitly SOF output */
		val |= (MCP2517FD_OSC_CLKODIV_10)
			<< MCP2517FD_OSC_CLKODIV_SHIFT;
		break;
	default:
		dev_err(&spi->dev,
			"Unsupported output clock divider %i\n",
			priv->clock_odiv);
		return -EINVAL;
	}

	/* write clock */
	ret = mcp2517fd_cmd_write(spi, MCP2517FD_OSC, val);
	if (ret)
		return ret;

	/* wait for synced pll/osc/sclk */
	timeout = jiffies + MCP2517FD_OSC_POLLING_JIFFIES;
	while(jiffies <= timeout) {
		ret = mcp2517fd_cmd_read(spi, MCP2517FD_OSC, &val);
		if (ret)
			return ret;
		dev_err(&spi->dev,
			"Read OSC 0x%08x - wait 0x%08x\n",val,waitfor);
		if ((val & waitfor) == waitfor)
			return 0;
	}

	dev_err(&spi->dev,
		"Clock did not lock within the timeout period\n");

	/* we timed out */
	return -ENODEV;
}

static int mcp2517fd_setup_fifo(struct net_device *net,
				struct mcp2517fd_priv *priv,
				struct spi_device *spi)
{
	u32 con_val = priv->con_val;
	struct mcp2517fd_obj_tx_msg *msg;
	u32 val;
	int ret;
	int i;

	/* decide on TEF, tx and rx FIFOS */
	switch (net->mtu) {
	case CAN_MTU:
		priv->payload_size = 8;
		priv->payload_mode = CAN_TXQCON_PLSIZE_8;
		priv->rx_fifos = 32;
		priv->tx_fifos = 30;
		priv->rx_address_inc = sizeof(struct mcp2517fd_obj_rx) + 8;
		break;
	case CANFD_MTU:
		priv->payload_size = 64;
		priv->payload_mode = CAN_TXQCON_PLSIZE_64;
		priv->rx_fifos = 17;
		priv->tx_fifos = 8;
		priv->rx_address_inc = sizeof(struct mcp2517fd_obj_rx) + 64;
		break;
	default:
		return -EINVAL;
	}

	/* set up TEF SIZE to the number of tx_fifos */
	ret = mcp2517fd_cmd_write(
		spi, CAN_TEFCON,
		CAN_TEFCON_TEFTSEN |
		((priv->tx_fifos - 1) << CAN_TEFCON_FSIZE_SHIFT));
	if (ret)
		return ret;

	/* now set up RX FIFO */
	ret = mcp2517fd_cmd_write(
		spi, CAN_FIFOCON(RX_FIFO),
		(priv->payload_mode << CAN_FIFOCON_PLSIZE_SHIFT) |
		((priv->rx_fifos - 1) << CAN_FIFOCON_FSIZE_SHIFT) |
		CAN_FIFOCON_RXTSEN | /* RX timestamps */
		CAN_FIFOCON_FRESET | /* reset FIFO */
		CAN_FIFOCON_TFERFFIE | /* FIFO Full Interrupt enable */
		CAN_FIFOCON_TFHRFHIE | /* FIFO Half full Interrupt enable */
		CAN_FIFOCON_TFNRFNIE
		);
	if (ret)
		return ret;

	/* clear all filter */
	for (i = 0; i < 32; i++) {
		ret = mcp2517fd_cmd_write(spi, CAN_FLTOBJ(i), 0);
		if (ret)
			return ret;
		ret = mcp2517fd_cmd_write(spi, CAN_FLTMASK(i), 0);
		if (ret)
			return ret;
		ret = mcp2517fd_cmd_write(spi, CAN_FLTCON(i), 0);
		if (ret)
			return ret;
	}

	/* and the RX filter that is required */
	ret = mcp2517fd_cmd_write_mask(
		spi, CAN_FLTCON(0),
		CAN_FIFOCON_FLTEN(0) | (1 << CAN_FILCON_SHIFT(0)),
		CAN_FILCON_MASK(0) | CAN_FIFOCON_FLTEN(0));
	if (ret)
		return ret;

	/* now setup the TX FIFOS */
	for (i = 0; i < priv->tx_fifos; i++) {
		ret = mcp2517fd_cmd_write(
			spi, CAN_FIFOCON(TX_FIFO(i)),
			CAN_FIFOCON_FRESET | /* reset FIFO */
			(priv->payload_mode << CAN_FIFOCON_PLSIZE_SHIFT) |
			(0 << CAN_FIFOCON_FSIZE_SHIFT) | /* 1 FIFO only */
			((i) << CAN_FIFOCON_TXPRI_SHIFT) | /* priority */
			CAN_FIFOCON_TXEN);
		if (ret)
			return ret;
	}

	/* we need to move out of CONFIG mode shortly to get pointers */
	ret = mcp2517fd_cmd_write(
		spi, CAN_CON, con_val |
		(CAN_CON_MODE_INTERNAL_LOOPBACK << CAN_CON_REQOP_SHIFT));
	if (ret)
		return ret;

	/* get all the relevant addresses for the transmit fifos */
	for (i = priv->tx_fifos - 1; i >= 0; i--) {
		ret = mcp2517fd_cmd_read(spi, CAN_FIFOUA(TX_FIFO(i)),
					    &val);
		if (ret)
			return ret;
		/* normalize val to RAM address */
		val = FIFO_DATA(val);
		/* assign as rx_address_end - we go top to buttom, so its ok */
		priv->rx_address_end = val;

		/* clear tx messages */
		msg = &priv->tx_msg[i];
		memset(msg, 0, sizeof(msg));
		/* prepare the spi messages to submit to fifo */
		spi_message_init_with_transfers(
			&msg->data.msg, &msg->data.xfer, 1);
		msg->data.xfer.tx_buf = &msg->data.cmd_addr;
		msg->min_length = sizeof(msg->data.cmd_addr) +
			sizeof(msg->data.obj);
		mcp2517fd_calc_cmd_addr(
			INSTRUCTION_WRITE, val,
			(u8 *)&msg->data.cmd_addr);
		/* and the trigger */
		spi_message_init_with_transfers(
			&msg->trigger.msg, &msg->trigger.xfer, 1);
		msg->trigger.xfer.tx_buf = &msg->trigger.cmd_addr;
		msg->trigger.xfer.len = 3;
		mcp2517fd_calc_cmd_addr(
			INSTRUCTION_WRITE, CAN_FIFOCON(TX_FIFO(i)) + 1,
			(u8 *)&msg->trigger.cmd_addr);
		msg->trigger.data =
			(CAN_FIFOCON_TXREQ | CAN_FIFOCON_UINC) >> 8;

		dev_err(&spi->dev," TX-FIFO%02i: %04x\n",
			i, val);
	}

	/* for the rx fifo */
	ret = mcp2517fd_cmd_read(spi, CAN_FIFOUA(RX_FIFO), &val);
	if (ret)
		return ret;
	val = FIFO_DATA(val);
	priv->rx_address_start = val;
	priv->rx_address = val;
	dev_err(&spi->dev," RX-FIFO: %03x - %03x\n",
		priv->rx_address, priv->rx_address_end);

	/* for the TEF fifo */
	ret = mcp2517fd_cmd_read(spi, CAN_TEFUA, &val);
	if (ret)
		return ret;
	priv->tef_address_start = 0x400 + val;
	priv->tef_address = 0x400 + val;
	priv->tef_address_end = priv->rx_address_start;
	dev_err(&spi->dev," TEF-FIFO: %03x - %03x\n",
		priv->tef_address, priv->tef_address_end);

	/* now get back into config mode */
	ret = mcp2517fd_cmd_write(
		spi, CAN_CON, con_val |
		(CAN_CON_MODE_CONFIG << CAN_CON_REQOP_SHIFT));
	if (ret)
		return ret;

	return 0;
}

static int mcp2517fd_clear_ram(struct spi_device *spi)
{
	struct spi_transfer t[2];
	u8 tx_buf[2];

	*((u16 *)tx_buf) =
		cpu_to_be16(INSTRUCTION_WRITE | 0x400);

	memset(t, 0, sizeof(t));
	t[0].tx_buf = tx_buf;
	t[0].len = sizeof(tx_buf);
	t[1].len = 2048;

	return spi_sync_transfer(spi, t, 2);
}

static int mcp2517fd_setup(struct net_device *net,
			   struct mcp2517fd_priv *priv,
			   struct spi_device *spi)
{
	u32 val;
	int ret;

	dev_err(&spi->dev, "Start_setup\n");

	ret = mcp2517fd_clear_ram(spi);
	if (ret)
		return ret;

	/* set up pll/clock if required */
	ret = mcp2517fd_setup_osc(spi);
	if (ret)
		return ret;

	/* set up RAM ECC (but for now without interrupts) */
	ret = mcp2517fd_cmd_write(spi, MCP2517FD_ECCCON,
				      MCP2517FD_ECCCON_ECCEN);
	if (ret)
		return ret;

	/* GPIO handling - could expose this as gpios*/
	val = 0; /* INT PUSHPULL INT , TXCAN PUSH/PULL, no Standby */
	val |= MCP2517FD_IOCON_TXCANOD; /* OpenDrain TXCAN */
	val |= MCP2517FD_IOCON_INTOD; /* OpenDrain INT pins */

	/* SOF/CLOCKOUT pin 3 */
	if (priv->clock_odiv < 0)
		val |= MCP2517FD_IOCON_SOF;
	/* GPIO0 - pin 9 */
	switch (priv->gpio0_mode) {
	case gpio_mode_standby:
	case gpio_mode_int: /* asserted low on TXIF */
	case gpio_mode_out_low:
	case gpio_mode_out_high:
	case gpio_mode_in:
		val |= priv->gpio0_mode;
		break;
	default: /* GPIO IN */
		dev_err(&spi->dev,
			"GPIO1 does not support mode %08x\n",
			priv->gpio0_mode);
		return -EINVAL;
	}
	/* GPIO1 - pin 8 */
	switch (priv->gpio1_mode) {
	case gpio_mode_standby:
		dev_err(&spi->dev,
			"GPIO1 does not support transciever standby\n");
		return -EINVAL;
	case gpio_mode_int: /* asserted low on RXIF */
	case gpio_mode_out_low:
	case gpio_mode_out_high:
	case gpio_mode_in:
		val |= priv->gpio1_mode << 1;
		break;
	default:
		dev_err(&spi->dev,
			"GPIO1 does not support mode %08x\n",
			priv->gpio0_mode);
		return -EINVAL;
	}
	/* INT/GPIO pins as open drain */
	if (priv->gpio_opendrain)
		val |= MCP2517FD_IOCON_INTOD;

	ret = mcp2517fd_cmd_write(spi, MCP2517FD_IOCON, val);
	if (ret)
		return ret;

	/* set up Transmitter Delay compensation */
	ret = mcp2517fd_cmd_write(spi, CAN_TDC, CAN_TDC_EDGFLTEN);
	if (ret)
		return ret;

	/* time stamp control register - 1ns resolution, but disabled */
	ret = mcp2517fd_cmd_write(spi, CAN_TBC, 0);
	if (ret)
		return ret;
	ret = mcp2517fd_cmd_write(spi, CAN_TSCON,
				      CAN_TSCON_TBCEN |
				      ((priv->can.clock.freq / 1000000)
				       << CAN_TSCON_TBCPRE_SHIFT));
	if (ret)
		return ret;

	/* interrupt configuration */
	ret = mcp2517fd_cmd_write(spi, CAN_INT, 0);
	if (ret)
		return ret;

	/* setup value of con_register */
	priv->con_val = CAN_CON_STEF /* enable TEF */;
	/* non iso FD mode */
	if (!(priv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO))
		priv->con_val |= CAN_CON_ISOCRCEN;
	/* one shot */
	if (!(priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT))
		priv->con_val |= CAN_CON_RTXAT;

	/* setup fifos - this also puts the system into sleep mode */
	ret = mcp2517fd_setup_fifo(net, priv, spi);

	return ret;
}

static int mcp2517fd_disable_interrupts(struct spi_device *spi)
{
	disable_irq(spi->irq);
	return 0;
}

static int mcp2517fd_enable_interrupts(struct spi_device *spi)
{
	enable_irq(spi->irq);
	return 0;
}

static irqreturn_t mcp2517fd_can_ist(int irq, void *dev_id)
{
	struct mcp2517fd_priv *priv = dev_id;
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	u32 iflags;
	int ret;

	/* read interrupt status flags */
	ret = mcp2517fd_cmd_read(spi, CAN_INT, &iflags);

	/* */

	/* enable irq */
	ret = mcp2517fd_disable_interrupts(spi);


	return IRQ_HANDLED;
}

static int mcp2517fd_open(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	int ret;

	ret = open_candev(net);
	if (ret) {
		dev_err(&spi->dev, "unable to set initial baudrate!\n");
		return ret;
	}

	mcp2517fd_power_enable(priv->transceiver, 1);

	mutex_lock(&priv->mcp_lock);
	priv->force_quit = 0;

	ret = request_threaded_irq(spi->irq, NULL,
				   mcp2517fd_can_ist,
				   IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				   DEVICE_NAME, priv);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
		mcp2517fd_power_enable(priv->transceiver, 0);
		close_candev(net);
		goto open_unlock;
	}

	ret = mcp2517fd_hw_probe(spi);
	if (ret) {
		mcp2517fd_open_clean(net);
		goto open_unlock;
	}

	ret = mcp2517fd_setup(net, priv, spi);
	if (ret) {
		mcp2517fd_open_clean(net);
		goto open_unlock;
	}

	mcp2517fd_do_set_nominal_bittiming(net);

	ret = mcp2517fd_set_normal_mode(spi);
	if (ret) {
		mcp2517fd_open_clean(net);
		goto open_unlock;
	}

	can_led_event(net, CAN_LED_EVENT_OPEN);

	netif_wake_queue(net);

open_unlock:
	mutex_unlock(&priv->mcp_lock);
	return ret;
}

static void mcp2517fd_clean(struct net_device *net)
{
        struct mcp2517fd_priv *priv = netdev_priv(net);
	int i;

	for (i = 0; i < priv->tx_fifos; i++) {
		if (priv->tx_pending_mask & BIT(i)) {
			can_free_echo_skb(priv->net, 0);
			net->stats.tx_errors++;
		}
	}

	priv->tx_pending_mask = 0;
}

static int mcp2517fd_stop(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	close_candev(net);

	priv->force_quit = 1;
	free_irq(spi->irq, priv);

	/* Disable and clear pending interrupts */
	/*
	mcp251x_write_reg(spi, CANINTE, 0x00);
	mcp251x_write_reg(spi, CANINTF, 0x00);

	mcp251x_write_reg(spi, TXBCTRL(0), 0);
	*/
	mcp2517fd_clean(net);
	/*
	mcp251x_hw_sleep(spi);
	*/
	mcp2517fd_power_enable(priv->transceiver, 0);

	priv->can.state = CAN_STATE_STOPPED;

	can_led_event(net, CAN_LED_EVENT_STOP);

	return 0;
}

static const struct net_device_ops mcp2517fd_netdev_ops = {
	.ndo_open = mcp2517fd_open,
	.ndo_stop = mcp2517fd_stop,
	.ndo_start_xmit = mcp2517fd_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static const struct of_device_id mcp2517fd_of_match[] = {
	{
		.compatible	= "microchip,mcp2517fd",
		.data		= (void *)CAN_MCP2517FD,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mcp2517fd_of_match);

static const struct spi_device_id mcp2517fd_id_table[] = {
	{
		.name		= "mcp2517fd",
		.driver_data	= (kernel_ulong_t)CAN_MCP2517FD,
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, mcp2517fd_id_table);

static int mcp2517fd_can_probe(struct spi_device *spi)
{
	const struct of_device_id *of_id =
		of_match_device(mcp2517fd_of_match, &spi->dev);
	struct net_device *net;
	struct mcp2517fd_priv *priv;
	struct clk *clk;
	int ret, freq;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		return PTR_ERR(clk);
	} else {
		freq = clk_get_rate(clk);
	}

	if (freq < MCP2517FD_MIN_CLOCK_FREQUENCY
	    || freq > MCP2517FD_MAX_CLOCK_FREQUENCY) {
		dev_err(&spi->dev,
			"Clock frequency %i is not in range\n", freq);
		return -ERANGE;
	}

	/* Allocate can/net device */
	net = alloc_candev(sizeof(*priv), TX_ECHO_SKB_MAX);
	if (!net)
		return -ENOMEM;

	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret)
			goto out_free;
	}

	net->netdev_ops = &mcp2517fd_netdev_ops;
	net->flags |= IFF_ECHO;

	priv = netdev_priv(net);
	priv->can.bittiming_const = &mcp2517fd_nominal_bittiming_const;
	priv->can.do_set_bittiming = &mcp2517fd_do_set_nominal_bittiming;
	priv->can.data_bittiming_const = &mcp2517fd_data_bittiming_const;
	priv->can.do_set_data_bittiming = &mcp2517fd_do_set_data_bittiming;
	priv->can.do_set_mode = mcp2517fd_do_set_mode;

	priv->can.ctrlmode_supported =
		CAN_CTRLMODE_FD |
		CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY;
	/* CAN_CTRLMODE_BERR_REPORTING */

	if (of_id)
		priv->model = (enum mcp2517fd_model)of_id->data;
	else
		priv->model = spi_get_device_id(spi)->driver_data;
	priv->net = net;
	priv->clk = clk;

	spi_set_drvdata(spi, priv);

	/* set up gpio modes as GPIO in*/
	priv->gpio0_mode = gpio_mode_in;
	priv->gpio1_mode = gpio_mode_in;

	/* if we have a clock that is smaller then 4MHz, then enable the pll */
	priv->clock_pll = (freq <= MCP2517FD_AUTO_PLL_MAX_CLOCK_FREQUENCY);
	/* do not use the SCK clock divider */
	priv->clock_div2 = false;
	/* clock output is divided by 10 - maybe expose this as a clock ?*/
	priv->clock_odiv = 10;

	/* decide on real can clock rate */
	priv->can.clock.freq = freq;
	if (priv->clock_pll) {
		priv->can.clock.freq *= MCP2517FD_PLL_MULTIPLIER;
		if (priv->can.clock.freq > MCP2517FD_MAX_CLOCK_FREQUENCY) {
			dev_err(&spi->dev,
				"PLL clock frequency %i would exceed limit\n",
				priv->can.clock.freq
				);
			return -EINVAL;
		}
	}
	if (priv->clock_div2)
		priv->can.clock.freq /= MCP2517FD_SCLK_DIVIDER;

	/* calclculate the clock frequencies to use */
	priv->spi_max_speed_hz = spi->max_speed_hz;
	priv->spi_setup_speed_hz = freq / 2;
	priv->spi_normal_speed_hz = priv->can.clock.freq / 2;
	if (priv->clock_div2) {
		priv->spi_setup_speed_hz /= MCP2517FD_SCLK_DIVIDER;
		priv->spi_normal_speed_hz /= MCP2517FD_SCLK_DIVIDER;
	}

	if (priv->spi_max_speed_hz) {
		priv->spi_setup_speed_hz = min_t(int,
						 priv->spi_setup_speed_hz,
						 priv->spi_max_speed_hz);
		priv->spi_normal_speed_hz = min_t(int,
						  priv->spi_normal_speed_hz,
						  priv->spi_max_speed_hz);
	}

	/* Configure the SPI bus */
	spi->max_speed_hz = priv->spi_setup_speed_hz;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		goto out_clk;

	priv->power = devm_regulator_get_optional(&spi->dev, "vdd");
	priv->transceiver = devm_regulator_get_optional(&spi->dev, "xceiver");
	if ((PTR_ERR(priv->power) == -EPROBE_DEFER) ||
	    (PTR_ERR(priv->transceiver) == -EPROBE_DEFER)) {
		ret = -EPROBE_DEFER;
		goto out_clk;
	}

	ret = mcp2517fd_power_enable(priv->power, 1);
	if (ret)
		goto out_clk;

	priv->spi = spi;
	mutex_init(&priv->mcp_lock);
	mutex_init(&priv->txfifo_lock);

	SET_NETDEV_DEV(net, &spi->dev);

	ret = mcp2517fd_hw_probe(spi);
	if (ret) {
		if (ret == -ENODEV)
			dev_err(&spi->dev,
				"Cannot initialize MCP%x. Wrong wiring?\n",
				priv->model);
		goto error_probe;
	}

	mcp2517fd_hw_sleep(spi);

	ret = register_candev(net);
	if (ret)
		goto error_probe;

	devm_can_led_init(net);

	netdev_info(net, "MCP%x successfully initialized.\n", priv->model);
	return 0;

error_probe:
	mcp2517fd_power_enable(priv->power, 0);

out_clk:
	if (!IS_ERR(clk))
		clk_disable_unprepare(clk);

out_free:
	free_candev(net);
	dev_err(&spi->dev, "Probe failed, err=%d\n", -ret);
	return ret;
}

static int mcp2517fd_can_remove(struct spi_device *spi)
{
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	unregister_candev(net);

	mcp2517fd_power_enable(priv->power, 0);

	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

	free_candev(net);

	/* restore max_speed_hz */
	spi->max_speed_hz = priv->spi_max_speed_hz;
	spi_setup(spi);

	return 0;
}

static int __maybe_unused mcp2517fd_can_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	priv->force_quit = 1;
	disable_irq(spi->irq);

	if (netif_running(net)) {
		netif_device_detach(net);

		mcp2517fd_hw_sleep(spi);
		mcp2517fd_power_enable(priv->transceiver, 0);
		priv->after_suspend = AFTER_SUSPEND_UP;
	} else {
		priv->after_suspend = AFTER_SUSPEND_DOWN;
	}

	if (!IS_ERR_OR_NULL(priv->power)) {
		regulator_disable(priv->power);
		priv->after_suspend |= AFTER_SUSPEND_POWER;
	}

	return 0;
}

static int __maybe_unused mcp2517fd_can_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mcp2517fd_priv *priv = spi_get_drvdata(spi);

	if (priv->after_suspend & AFTER_SUSPEND_POWER)
		mcp2517fd_power_enable(priv->power, 1);

	if (priv->after_suspend & AFTER_SUSPEND_UP) {
		mcp2517fd_power_enable(priv->transceiver, 1);
	} else {
		priv->after_suspend = 0;
	}

	priv->force_quit = 0;

	enable_irq(spi->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mcp2517fd_can_pm_ops, mcp2517fd_can_suspend,
	mcp2517fd_can_resume);

static struct spi_driver mcp2517fd_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = mcp2517fd_of_match,
		.pm = &mcp2517fd_can_pm_ops,
	},
	.id_table = mcp2517fd_id_table,
	.probe = mcp2517fd_can_probe,
	.remove = mcp2517fd_can_remove,
};
module_spi_driver(mcp2517fd_can_driver);

MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_DESCRIPTION("Microchip 2517FD CAN driver");
MODULE_LICENSE("GPL v2");
