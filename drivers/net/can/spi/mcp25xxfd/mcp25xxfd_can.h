/* SPDX-License-Identifier: GPL-2.0 */

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include "mcp25xxfd.h"
#include <linux/bitops.h>
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

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
	GENMASK(CAN_CON_OPMOD_SHIFT + CAN_CON_OPMODE_BITS - 1,	\
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
#  define CAN_CON_DEFAULT				\
	(CAN_CON_ISOCRCEN |				\
	 CAN_CON_PXEDIS |				\
	 CAN_CON_WAKFIL |				\
	 (3 << CAN_CON_WFT_SHIFT) |			\
	 CAN_CON_STEF |					\
	 CAN_CON_TXQEN |				\
	 (CAN_CON_MODE_CONFIG << CAN_CON_OPMOD_SHIFT) |	\
	 (CAN_CON_MODE_CONFIG << CAN_CON_REQOP_SHIFT))
#  define CAN_CON_DEFAULT_MASK	\
	(CAN_CON_DNCNT_MASK |	\
	 CAN_CON_ISOCRCEN |	\
	 CAN_CON_PXEDIS |	\
	 CAN_CON_WAKFIL |	\
	 CAN_CON_WFT_MASK |	\
	 CAN_CON_BRSDIS |	\
	 CAN_CON_RTXAT |	\
	 CAN_CON_ESIGM |	\
	 CAN_CON_SERR2LOM |	\
	 CAN_CON_STEF |		\
	 CAN_CON_TXQEN |	\
	 CAN_CON_OPMOD_MASK |	\
	 CAN_CON_REQOP_MASK |	\
	 CAN_CON_ABAT |		\
	 CAN_CON_TXBWS_MASK)
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
	GENMASK(CAN_TDC_TDCV_SHIFT + CAN_TDC_TDCV_BITS - 1,	\
		CAN_TDC_TDCV_SHIFT)
#  define CAN_TDC_TDCO_BITS		5
#  define CAN_TDC_TDCO_SHIFT		8
#  define CAN_TDC_TDCO_MASK					\
	GENMASK(CAN_TDC_TDCO_SHIFT + CAN_TDC_TDCO_BITS - 1,	\
		CAN_TDC_TDCO_SHIFT)
#  define CAN_TDC_TDCMOD_BITS		2
#  define CAN_TDC_TDCMOD_SHIFT		16
#  define CAN_TDC_TDCMOD_MASK					\
	GENMASK(CAN_TDC_TDCMOD_SHIFT + CAN_TDC_TDCMOD_BITS - 1, \
		CAN_TDC_TDCMOD_SHIFT)
#  define CAN_TDC_TDCMOD_DISABLED	0
#  define CAN_TDC_TDCMOD_MANUAL		1
#  define CAN_TDC_TDCMOD_AUTO		2
#  define CAN_TDC_SID11EN		BIT(24)
#  define CAN_TDC_EDGFLTEN		BIT(25)
#define CAN_TBC				CAN_SFR_BASE(0x10)
#define CAN_TSCON			CAN_SFR_BASE(0x14)
#  define CAN_TSCON_TBCPRE_BITS		10
#  define CAN_TSCON_TBCPRE_SHIFT	0
#  define CAN_TSCON_TBCPRE_MASK					    \
	GENMASK(CAN_TSCON_TBCPRE_SHIFT + CAN_TSCON_TBCPRE_BITS - 1, \
		CAN_TSCON_TBCPRE_SHIFT)
#  define CAN_TSCON_TBCEN		BIT(16)
#  define CAN_TSCON_TSEOF		BIT(17)
#  define CAN_TSCON_TSRES		BIT(18)
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
#  define CAN_INT_IF_SHIFT		0
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
#  define CAN_INT_IF_MASK		\
	(CAN_INT_TXIF |			\
	 CAN_INT_RXIF |			\
	 CAN_INT_TBCIF	|		\
	 CAN_INT_MODIF	|		\
	 CAN_INT_TEFIF	|		\
	 CAN_INT_ECCIF	|		\
	 CAN_INT_SPICRCIF |		\
	 CAN_INT_TXATIF |		\
	 CAN_INT_RXOVIF |		\
	 CAN_INT_CERRIF |		\
	 CAN_INT_SERRIF |		\
	 CAN_INT_WAKIF |		\
	 CAN_INT_IVMIF)
#  define CAN_INT_IF_CLEAR_MASK		\
	(CAN_INT_TBCIF	|		\
	 CAN_INT_MODIF	|		\
	 CAN_INT_CERRIF |		\
	 CAN_INT_SERRIF |		\
	 CAN_INT_WAKIF |		\
	 CAN_INT_IVMIF)
#  define CAN_INT_IE_SHIFT		16
#  define CAN_INT_TXIE			(CAN_INT_TXIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_RXIE			(CAN_INT_RXIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_TBCIE			(CAN_INT_TBCIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_MODIE			(CAN_INT_MODIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_TEFIE			(CAN_INT_TEFIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_ECCIE			(CAN_INT_ECCIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_SPICRCIE		\
	(CAN_INT_SPICRCIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_TXATIE		(CAN_INT_TXATIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_RXOVIE		(CAN_INT_RXOVIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_CERRIE		(CAN_INT_CERRIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_SERRIE		(CAN_INT_SERRIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_WAKIE			(CAN_INT_WAKIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_IVMIE			(CAN_INT_IVMIF << CAN_INT_IE_SHIFT)
#  define CAN_INT_IE_MASK		\
	(CAN_INT_TXIE |			\
	 CAN_INT_RXIE |			\
	 CAN_INT_TBCIE	|		\
	 CAN_INT_MODIE	|		\
	 CAN_INT_TEFIE	|		\
	 CAN_INT_ECCIE	|		\
	 CAN_INT_SPICRCIE |		\
	 CAN_INT_TXATIE |		\
	 CAN_INT_RXOVIE |		\
	 CAN_INT_CERRIE |		\
	 CAN_INT_SERRIE |		\
	 CAN_INT_WAKIE |		\
	 CAN_INT_IVMIE)
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
#  define CAN_BDIAG0_NRERRCNT_MASK					\
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
#  define CAN_BDIAG1_DSTUFERR		BIT(28)
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
	GENMASK(CAN_TXQCON_TXAT_SHIFT + CAN_TXQCON_TXAT_BITS - 1,   \
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
#define CAN_FIFOCON(x)			CAN_SFR_BASE(0x5C + 12 * ((x) - 1))
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
#  define CAN_FIFOCON_TXAT_ONE_SHOT	0
#  define CAN_FIFOCON_TXAT_THREE_SHOT	1
#  define CAN_FIFOCON_TXAT_UNLIMITED	2
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
#define CAN_FIFOSTA(x)			CAN_SFR_BASE(0x60 + 12 * ((x) - 1))
#  define CAN_FIFOSTA_TFNRFNIF		BIT(0)
#  define CAN_FIFOSTA_TFHRFHIF		BIT(1)
#  define CAN_FIFOSTA_TFERFFIF		BIT(2)
#  define CAN_FIFOSTA_RXOVIF		BIT(3)
#  define CAN_FIFOSTA_TXATIF		BIT(4)
#  define CAN_FIFOSTA_TXERR		BIT(5)
#  define CAN_FIFOSTA_TXLARB		BIT(6)
#  define CAN_FIFOSTA_TXABT		BIT(7)
#  define CAN_FIFOSTA_FIFOCI_BITS	5
#  define CAN_FIFOSTA_FIFOCI_SHIFT	8
#  define CAN_FIFOSTA_FIFOCI_MASK					\
	GENMASK(CAN_FIFOSTA_FIFOCI_SHIFT + CAN_FIFOSTA_FIFOCI_BITS - 1, \
		CAN_FIFOSTA_FIFOCI_SHIFT)
#define CAN_FIFOUA(x)			CAN_SFR_BASE(0x64 + 12 * ((x) - 1))
#define CAN_FLTCON(x)			CAN_SFR_BASE(0x1D0 + ((x) & 0x1c))
#  define CAN_FILCON_SHIFT(x)		(((x) & 3) * 8)
#  define CAN_FILCON_BITS(x)		CAN_FILCON_BITS_
#  define CAN_FILCON_BITS_		4
	/* avoid macro reuse warning, so do not use GENMASK as above */
#  define CAN_FILCON_MASK(x)					\
	(GENMASK(CAN_FILCON_BITS_ - 1, 0) << CAN_FILCON_SHIFT(x))
#  define CAN_FIFOCON_FLTEN(x)		BIT(7 + CAN_FILCON_SHIFT(x))
#define CAN_FLTOBJ(x)			CAN_SFR_BASE(0x1F0 + 8 * (x))
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
#define CAN_FLTMASK(x)			CAN_SFR_BASE(0x1F4 + 8 * (x))
#  define CAN_FILMASK_MSID_BITS		11
#  define CAN_FILMASK_MSID_SHIFT	0
#  define CAN_FILMASK_MSID_MASK					    \
	GENMASK(CAN_FILMASK_MSID_SHIFT + CAN_FILMASK_MSID_BITS - 1, \
		CAN_FILMASK_MSID_SHIFT)
#  define CAN_FILMASK_MEID_BITS		18
#  define CAN_FILMASK_MEID_SHIFT	12
#  define CAN_FILMASK_MEID_MASK					    \
	GENMASK(CAN_FILMASK_MEID_SHIFT + CAN_FILMASK_MEID_BITS - 1, \
		CAN_FILMASK_MEID_SHIFT)
#  define CAN_FILMASK_MSID11		BIT(29)
#  define CAN_FILMASK_MIDE		BIT(30)

#define CAN_OBJ_ID_SID_BITS		11
#define CAN_OBJ_ID_SID_SHIFT		0
#define CAN_OBJ_ID_SID_MASK					\
	GENMASK(CAN_OBJ_ID_SID_SHIFT + CAN_OBJ_ID_SID_BITS - 1, \
		CAN_OBJ_ID_SID_SHIFT)
#define CAN_OBJ_ID_EID_BITS		18
#define CAN_OBJ_ID_EID_SHIFT		11
#define CAN_OBJ_ID_EID_MASK					\
	GENMASK(CAN_OBJ_ID_EID_SHIFT + CAN_OBJ_ID_EID_BITS - 1, \
		CAN_OBJ_ID_EID_SHIFT)
#define CAN_OBJ_ID_SID_BIT11		BIT(29)

#define CAN_OBJ_FLAGS_DLC_BITS		4
#define CAN_OBJ_FLAGS_DLC_SHIFT		0
#define CAN_OBJ_FLAGS_DLC_MASK					      \
	GENMASK(CAN_OBJ_FLAGS_DLC_SHIFT + CAN_OBJ_FLAGS_DLC_BITS - 1, \
		CAN_OBJ_FLAGS_DLC_SHIFT)
#define CAN_OBJ_FLAGS_IDE		BIT(4)
#define CAN_OBJ_FLAGS_RTR		BIT(5)
#define CAN_OBJ_FLAGS_BRS		BIT(6)
#define CAN_OBJ_FLAGS_FDF		BIT(7)
#define CAN_OBJ_FLAGS_ESI		BIT(8)
#define CAN_OBJ_FLAGS_SEQ_BITS		7
#define CAN_OBJ_FLAGS_SEQ_SHIFT		9
#define CAN_OBJ_FLAGS_SEQ_MASK					      \
	GENMASK(CAN_OBJ_FLAGS_SEQ_SHIFT + CAN_OBJ_FLAGS_SEQ_BITS - 1, \
		CAN_OBJ_FLAGS_SEQ_SHIFT)
#define CAN_OBJ_FLAGS_FILHIT_BITS	11
#define CAN_OBJ_FLAGS_FILHIT_SHIFT	5
#define CAN_OBJ_FLAGS_FILHIT_MASK				      \
	GENMASK(CAN_FLAGS_FILHIT_SHIFT + CAN_FLAGS_FILHIT_BITS - 1,   \
		CAN_FLAGS_FILHIT_SHIFT)

#define TX_ECHO_SKB_MAX	32

struct mcp25xxfd_fifo {
	u32 count;
	u32 start;
	u32 increment;
	u32 size;
	u32 priority_start;
	u32 priority_increment;
	u64 dlc_usage[16];
	u64 fd_count;

	struct dentry *debugfs_dir;
};

struct mcp25xxfd_obj_ts {
	/* using signed here to handle rollover correctly */
	s32 ts;
	/* positive fifos are rx, negative tx, 0 is not a valid fifo */
	s32 fifo;
};

struct mcp25xxfd_can_priv {
	/* can_priv has to be the first one to be usable with alloc_candev
	 * which expects struct can_priv to be right at the start of the
	 * priv structure
	 */
	struct can_priv can;
	struct mcp25xxfd_priv *priv;
	struct regulator *transceiver;

	/* the can mode currently active */
	int mode;

	/* can config registers */
	struct {
		u32 con;
		u32 tdc;
		u32 tscon;
		u32 tefcon;
		u32 nbtcfg;
		u32 dbtcfg;
	} regs;

	/* can status registers (mostly) - read in one go
	 * bdiag0 and bdiag1 are optional, but when
	 * berr counters are requested on a regular basis
	 * during high CAN-bus load this would trigger the fact
	 * that spi_sync would get queued for execution in the
	 * spi thread and the spi handler would not get
	 * called inline in the interrupt thread without any
	 * context switches or wakeups...
	 */
	struct {
		u32 intf;
		/* ASSERT(CAN_INT + 4 == CAN_RXIF) */
		u32 rxif;
		/* ASSERT(CAN_RXIF + 4 == CAN_TXIF) */
		u32 txif;
		/* ASSERT(CAN_TXIF + 4 == CAN_RXOVIF) */
		u32 rxovif;
		/* ASSERT(CAN_RXOVIF + 4 == CAN_TXATIF) */
		u32 txatif;
		/* ASSERT(CAN_TXATIF + 4 == CAN_TXREQ) */
		u32 txreq;
		/* ASSERT(CAN_TXREQ + 4 == CAN_TREC) */
		u32 trec;
	} status;

	/* information of fifo setup */
	struct {
		/* define payload size and mode */
		u32 payload_size;
		u32 payload_mode;

		/* infos on fifo layout */
		struct {
			u32 count;
			u32 size;
			u32 index;
		} tef;
		struct mcp25xxfd_fifo tx;
		struct mcp25xxfd_fifo rx;

		/* the address and priority of all fifos */
		struct {
			u32 control;
			u32 status;
			u32 offset;
		} fifo_reg[32];
		struct {
			u32 is_tx;
			u32 priority;
			u64 use_count;
		} fifo_info[32];

		/* queue of can frames that need to get submitted
		 * to the network stack during an interrupt loop in one go
		 * (this gets sorted by timestamp before submission
		 * and contains both rx frames as well tx frames that have
		 * gone over the CAN bus successfully
		 */
		struct mcp25xxfd_obj_ts submit_queue[32];
		int  submit_queue_count;

		/* the tx queue of spi messages */
		struct mcp2517fd_tx_spi_message_queue *tx_queue;

		/* the directory entry of the can_fifo debugfs directory */
		struct dentry *debugfs_dir;
	} fifos;

	/* statistics */
	struct {
		u64 irq_calls;
		u64 irq_loops;

		u64 int_serr_count;
		u64 int_serr_rx_count;
		u64 int_serr_tx_count;
		u64 int_mod_count;
		u64 int_rx_count;
		u64 int_txat_count;
		u64 int_tef_count;
		u64 int_rxov_count;
		u64 int_ecc_count;
		u64 int_ivm_count;
		u64 int_cerr_count;

		u64 tx_fd_count;
		u64 tx_brs_count;

		u64 rx_reads;
		u64 rx_single_reads;
		u64 rx_reads_prefetched_too_few;
		u64 rx_reads_prefetched_too_few_bytes;
		u64 rx_reads_prefetched_too_many;
		u64 rx_reads_prefetched_too_many_bytes;
		u64 rx_bulk_reads;
#define RX_BULK_READ_STATS_BINS 8
		u64 rx_bulk_read_sizes[RX_BULK_READ_STATS_BINS];
#define RX_HISTORY_SIZE 32
		u8 rx_history_dlc[RX_HISTORY_SIZE];
		u8 rx_history_brs[RX_HISTORY_SIZE];
		u8 rx_history_index;
		u32 rx_reads_prefetch_predicted;
	} stats;

	/* bus state */
	struct {
		u32 state;
		u32 new_state;

		u32 bdiag[2];
	} bus;

	/* can merror messages */
	struct {
		u32 id;
		u8  data[8];
	} error_frame;

	/* a sram equivalent */
	u8 sram[MCP25XXFD_SRAM_SIZE];
};

struct mcp25xxfd_obj_tef {
	u32 id;
	u32 flags;
	u32 ts;
};

struct mcp25xxfd_obj_tx {
	u32 id;
	u32 flags;
	u8 data[];
};

struct mcp25xxfd_obj_rx {
	u32 id;
	u32 flags;
	u32 ts;
	u8 data[];
};

int mcp25xxfd_can_get_mode(struct spi_device *spi, u32 *mode_data);
int mcp25xxfd_can_switch_mode(struct spi_device *spi, u32 *mode_data,
			      int mode);
irqreturn_t mcp25xxfd_can_int(int irq, void *dev_id);
netdev_tx_t mcp25xxfd_can_start_xmit(struct sk_buff *skb,
				     struct net_device *net);
int mcp25xxfd_can_setup_fifos(struct net_device *net);
void mcp25xxfd_can_release_fifos(struct net_device *net);

/* ideally these would be defined in uapi/linux/can.h */
#define CAN_EFF_SID_SHIFT		(CAN_EFF_ID_BITS - CAN_SFF_ID_BITS)
#define CAN_EFF_SID_BITS		CAN_SFF_ID_BITS
#define CAN_EFF_SID_MASK				      \
	GENMASK(CAN_EFF_SID_SHIFT + CAN_EFF_SID_BITS - 1,     \
		CAN_EFF_SID_SHIFT)
#define CAN_EFF_EID_SHIFT		0
#define CAN_EFF_EID_BITS		CAN_EFF_SID_SHIFT
#define CAN_EFF_EID_MASK				      \
	GENMASK(CAN_EFF_EID_SHIFT + CAN_EFF_EID_BITS - 1,     \
		CAN_EFF_EID_SHIFT)

static inline
void mcp25xxfd_mcpid_to_canid(u32 mcp_id, u32 mcp_flags, u32 *can_id)
{
	u32 sid = (mcp_id & CAN_OBJ_ID_SID_MASK) >> CAN_OBJ_ID_SID_SHIFT;
	u32 eid = (mcp_id & CAN_OBJ_ID_EID_MASK) >> CAN_OBJ_ID_EID_SHIFT;

	/* select normal or extended ids */
	if (mcp_flags & CAN_OBJ_FLAGS_IDE) {
		*can_id = (eid << CAN_EFF_EID_SHIFT) |
			(sid << CAN_EFF_SID_SHIFT) |
			CAN_EFF_FLAG;
	} else {
		*can_id = sid;
	}
	/* handle rtr */
	*can_id |= (mcp_flags & CAN_OBJ_FLAGS_RTR) ? CAN_RTR_FLAG : 0;
}

static inline
void mcp25xxfd_canid_to_mcpid(u32 can_id, u32 *id, u32 *flags)
{
	/* depending on can_id flag compute extended or standard ids */
	if (can_id & CAN_EFF_FLAG) {
		int sid = (can_id & CAN_EFF_SID_MASK) >> CAN_EFF_SID_SHIFT;
		int eid = (can_id & CAN_EFF_EID_MASK) >> CAN_EFF_EID_SHIFT;
		*id = (eid << CAN_OBJ_ID_EID_SHIFT) |
			(sid << CAN_OBJ_ID_SID_SHIFT);
		*flags = CAN_OBJ_FLAGS_IDE;
	} else {
		*id = can_id & CAN_SFF_MASK;
		*flags = 0;
	}

	/* Handle RTR */
	*flags |= (can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0;
}

static inline
void mcp25xxfd_can_queue_frame(struct mcp25xxfd_can_priv *cpriv,
			       s32 fifo, s32 ts)
{
	int idx = cpriv->fifos.submit_queue_count;

	cpriv->fifos.submit_queue[idx].fifo = fifo;
	cpriv->fifos.submit_queue[idx].ts = ts;

	cpriv->fifos.submit_queue_count++;
}

int mcp25xxfd_can_read_rx_frames(struct spi_device *spi);
int mcp25xxfd_can_submit_rx_frame(struct spi_device *spi, int fifo);
int mcp25xxfd_can_submit_tx_frame(struct spi_device *spi, int fifo);

int mcp25xxfd_can_int_handle_txatif(struct spi_device *spi);
int mcp25xxfd_can_int_handle_tefif(struct spi_device *spi);

int mcp25xxfd_can_tx_queue_alloc(struct net_device *net);
void mcp25xxfd_can_tx_queue_free(struct net_device *net);
void mcp25xxfd_can_tx_queue_restart(struct net_device *net);
void mcp25xxfd_can_tx_queue_manage(struct net_device *net, int state);
void mcp25xxfd_can_tx_queue_manage_nolock(struct net_device *net, int state);
# define TX_QUEUE_STATE_STOPPED  0
# define TX_QUEUE_STATE_STARTED  1
# define TX_QUEUE_STATE_RUNABLE 2
# define TX_QUEUE_STATE_RESTART  3

int mcp25xxfd_can_switch_mode_nowait(struct spi_device *spi,
				     u32 *mode_data, int mode);

void mcp25xxfd_can_rx_fifo_debugfs(struct net_device *net);
