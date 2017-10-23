/*
 * CAN bus driver for Microchip 2517FD CAN Controller with SPI Interface
 *
 * Copyright 2017 Martin Sperl
 *
 * Based on CAN bus driver for the mcp251x controller
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

#define MCP2517FD_OST_DELAY_MS	(5)

#define TX_ECHO_SKB_MAX	1

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
#    define CAN_CON_MODE_MIXED		0
#    define CAN_CON_MODE_SLEEP		1
#    define CAN_CON_MODE_INTERNAL_LOOPB	2
#    define CAN_CON_MODE_LISTEN_ONLY	3
#    define CAN_CON_MODE_CONFIG		4
#    define CAN_CON_MODE_EXTERNAL_LOOPB	5
#    define CAN_CON_MODE_CAN2_0		6
#    define CAN_CON_MODE_RESTRICTED	7
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
#define CAN_BDIAG1			CAN_SFR_BASE(0x40)
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
#define CAN_TEFCON			CAN_SFR_BASE(0x44)
#  define CAN_TEFCON_TEFNEIE		BIT(0)
#  define CAN_TEFCON_TEFHIE		BIT(1)
#  define CAN_TEFCON_TEFFIE		BIT(2)
#  define CAN_TEFCON_TEVOVIE		BIT(3)
#  define CAN_TEFCON_TEVTSEN		BIT(5)
#  define CAN_TEFCON_UINC		BIT(8)
#  define CAN_TEFCON_FRESET		BIT(10)
#  define CAN_TEFCON_FSIZE_BITS		5
#  define CAN_TEFCON_FSIZE_SHIFT	24
#  define CAN_TEFCON_FSIZE_MASK					    \
	GENMASK(CAN_TEFCON_FSIZE_SHIFT + CAN_TEFCON_FSIZE_BITS - 1, \
		CAN_TEFCON_FSIZE_SHIFT)
#define CAN_TEFSTA			CAN_SFR_BASE(0x48)
#  define CAN_TEFSTA_TEFNEIF		BIT(0)
#  define CAN_TEFSTA_TEFHIF		BIT(1)
#  define CAN_TEFSTA_TEFFIF		BIT(2)
#  define CAN_TEFSTA_TEVOVIF		BIT(3)
#define CAN_TEFUA			CAN_SFR_BASE(0x4C)
#define CAN_RESERVED			CAN_SFR_BASE(0x50)
#define CAN_TXQCON			CAN_SFR_BASE(0x54)
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
#  define CAN_FIFOCON_TXPRI_MASK					\
	GENMASK(CAN_FIFOCON_TXPRI_SHIFT + CAN_FIFOCON_TXPRI_BITS - 1,	\
		CAN_FIFOCON_TXPRI_SHIFT)
#  define CAN_FIFOCON_TXAT_BITS		2
#  define CAN_FIFOCON_TXAT_SHIFT		21
#  define CAN_FIFOCON_TXAT_MASK					    \
	GENMASK(CAN_FIFOCON_TXAT_SHIFT + CAN_FIFOCON_TXAT_BITS - 1, \
		CAN_FIFOCON_TXAT_SHIFT)
#  define CAN_FIFOCON_FSIZE_BITS		5
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
#define CAN_FLTCON_BYTE(x)		CAN_SFR_BASE(0x1D0 + x)
#define CAN_FILCON_BYTE_SHIFT(x)	0
#define CAN_FILCON_BYTE_BITS(x)		4
#  define CAN_FILCON_BYTE_MASK(x)					\
	GENMASK(CAN_FILCON_BYTE_SHIFT(x) + CAN_FILCON_BYTE_BITS(x) - 1,	\
		CAN_FILCON_BYTE_SHIFT(x))
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

#define FIFO_DATA(x)			(0x400 + (x))
#define FIFO_DATA_SIZE			0x800

static const struct can_bittiming_const mcp2517fd_nominal_bittiming_const = {
	.name		= DEVICE_NAME,
	.tseg1_min	= 1,
	.tseg1_max	= BIT(CAN_NBTCFG_TSEG1_BITS),
	.tseg2_min	= 1,
	.tseg2_max	= BIT(CAN_NBTCFG_TSEG1_BITS),
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
	.tseg2_max	= BIT(CAN_DBTCFG_TSEG1_BITS),
	.sjw_max	= BIT(CAN_DBTCFG_SJW_BITS),
	.brp_min	= 1,
	.brp_max	= BIT(CAN_DBTCFG_BRP_BITS),
	.brp_inc	= 1,
};

enum mcp2517fd_model {
	CAN_MCP2517FD	= 0x2517,
};

struct mcp2517fd_priv {
	struct can_priv	   can;
	struct net_device *net;
	struct spi_device *spi;
	enum mcp2517fd_model model;

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

static u32 mcp2517fd_cmd_reset(struct spi_device *spi)
{
	u16 tx_buf = cpu_to_be16(INSTRUCTION_RESET);

	/* write the reset command */
	return spi_write(spi, &tx_buf, 2);
}

static u32 mcp2517fd_cmd_readreg(struct spi_device *spi, u32 reg)
{
	u16 tx_buf = cpu_to_be16(INSTRUCTION_READ | (reg & ADDRESS_MASK));
	u32 rx_buf = 0;

	/* read the register */
	spi_write_then_read(spi, &tx_buf, 2, &rx_buf, 4);

	/* transform to cpu order and return */
	return le32_to_cpup(&rx_buf);
}

static u32 mcp2517fd_cmd_writereg(struct spi_device *spi, u32 reg, u32 val)
{
	u8 tx_buf[6];

	/* fill the buffer */
	*((u16 *)tx_buf) =
		cpu_to_be16(INSTRUCTION_READ | (reg & ADDRESS_MASK));
	*(u32 *)&tx_buf[2] = val;

	/* read the register */
	return spi_write(spi, &tx_buf, 6);
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
	struct mcp2517fd_priv *priv = netdev_priv(net);

	switch (mode) {
	case CAN_MODE_START:
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mcp2517fd_do_set_bittiming(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct can_bittiming *nbt = &priv->can.bittiming;
	struct can_bittiming *dbt = &priv->can.data_bittiming;
	struct spi_device *spi = priv->spi;

	return 0;
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
	u32 reg;
	/* reset can controller */
	mcp2517fd_cmd_reset(spi);

	/* read CON register and match */
	reg = mcp2517fd_cmd_readreg(spi, CAN_CON);
	dev_dbg(&spi->dev, "CAN_CON 0x%08x\n",reg);

	/* apply mask and check */
	if ((reg & CAN_CON_DEFAULT_MASK) != CAN_CON_DEFAULT)
		return -ENODEV;

	return 0;
}

static int mcp2517fd_hw_reset(struct spi_device *spi)
{

	/* Wait for oscillator startup timer after power up */
	mdelay(MCP2517FD_OST_DELAY_MS);

	mcp2517fd_cmd_reset(spi);

	/* Wait for oscillator startup timer after power up */
	mdelay(MCP2517FD_OST_DELAY_MS);


	return -ENODEV;
}

static int mcp2517fd_hw_open_clean(struct spi_device *spi)
{
	return -ENODEV;
}

static int mcp2517fd_set_normal_mode(struct spi_device *spi)
{
	return -ENODEV;
}

static int mcp2517fd_setup(struct net_device *net,
			   struct mcp2517fd_priv *priv,
			   struct spi_device *spi)
{
	mcp2517fd_do_set_bittiming(net);

	return 0;
}

static int mcp2517fd_open(struct net_device *net)
{
	struct mcp2517fd_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
	int ret;

	ret = open_candev(net);
	if (ret) {
		dev_err(&spi->dev, "unable to set initial baudrate!\n");
		return ret;
	}

	mcp2517fd_power_enable(priv->transceiver, 1);

	priv->force_quit = 0;

	ret = mcp2517fd_hw_reset(spi);
	if (ret) {
		mcp2517fd_open_clean(net);
		goto open_unlock;
	}
	ret = mcp2517fd_setup(net, priv, spi);
	if (ret) {
		mcp2517fd_open_clean(net);
		goto open_unlock;
	}
	ret = mcp2517fd_set_normal_mode(spi);
	if (ret) {
		mcp2517fd_open_clean(net);
		goto open_unlock;
	}

	can_led_event(net, CAN_LED_EVENT_OPEN);

	netif_wake_queue(net);

open_unlock:
	return ret;
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
	mcp251x_clean(net);

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
//	.ndo_start_xmit = mcp2517fd_hard_start_xmit,
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

	if (freq < 1000000 || freq > 40000000)
		return -ERANGE;

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
	priv->can.data_bittiming_const = &mcp2517fd_data_bittiming_const;
	priv->can.do_set_mode = mcp2517fd_do_set_mode;

	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;
	if (of_id)
		priv->model = (enum mcp2517fd_model)of_id->data;
	else
		priv->model = spi_get_device_id(spi)->driver_data;
	priv->net = net;
	priv->clk = clk;

	spi_set_drvdata(spi, priv);

	/* TODO - decide on real clock rate */
	priv->can.clock.freq = freq;

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	spi->max_speed_hz = spi->max_speed_hz ? : freq / 2;
	spi->max_speed_hz = min_t(int, spi->max_speed_hz, freq / 2);
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
