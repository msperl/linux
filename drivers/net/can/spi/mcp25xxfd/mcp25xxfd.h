/* SPDX-License-Identifier: GPL-2.0 */

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 *
 * Copyright 2019 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

/* some constants derived from the datasheets */
#define MCP25XXFD_OST_DELAY_MS		3
#define MCP25XXFD_MIN_CLOCK_FREQUENCY	1000000
#define MCP25XXFD_MAX_CLOCK_FREQUENCY	40000000
#define MCP25XXFD_PLL_MULTIPLIER	10
#define MCP25XXFD_AUTO_PLL_MAX_CLOCK_FREQUENCY				\
	(MCP25XXFD_MAX_CLOCK_FREQUENCY / MCP25XXFD_PLL_MULTIPLIER)
#define MCP25XXFD_SCLK_DIVIDER		2

/* address defines for SRAM */
#define MCP25XXFD_SRAM_SIZE 2048
#define MCP25XXFD_SRAM_ADDR(x) (0x400 + (x))

/* some defines for the driver */
#define MCP25XXFD_BUFFER_TXRX_SIZE (MCP25XXFD_SRAM_SIZE)
#define MCP25XXFD_OSC_POLLING_JIFFIES	(HZ / 2)

/* SPI commands */
#define INSTRUCTION_RESET		0x0000
#define INSTRUCTION_READ		0x3000
#define INSTRUCTION_WRITE		0x2000
#define INSTRUCTION_READ_CRC		0xB000
#define INSTRUCTION_WRITE_CRC		0xA000
#define INSTRUCTION_WRITE_SAVE		0xC000

#define ADDRESS_MASK			0x0fff

/* Register definitions */
#define MCP25XXFD_SFR_BASE(x)		(0xE00 + (x))
#define MCP25XXFD_OSC			MCP25XXFD_SFR_BASE(0x00)
#  define MCP25XXFD_OSC_PLLEN		BIT(0)
#  define MCP25XXFD_OSC_OSCDIS		BIT(2)
#  define MCP25XXFD_OSC_SCLKDIV		BIT(4)
#  define MCP25XXFD_OSC_CLKODIV_BITS	2
#  define MCP25XXFD_OSC_CLKODIV_SHIFT	5
#  define MCP25XXFD_OSC_CLKODIV_MASK			\
	GENMASK(MCP25XXFD_OSC_CLKODIV_SHIFT		\
		+ MCP25XXFD_OSC_CLKODIV_BITS - 1,	\
		MCP25XXFD_OSC_CLKODIV_SHIFT)
#  define MCP25XXFD_OSC_CLKODIV_10	3
#  define MCP25XXFD_OSC_CLKODIV_4	2
#  define MCP25XXFD_OSC_CLKODIV_2	1
#  define MCP25XXFD_OSC_CLKODIV_1	0
#  define MCP25XXFD_OSC_PLLRDY		BIT(8)
#  define MCP25XXFD_OSC_OSCRDY		BIT(10)
#  define MCP25XXFD_OSC_SCLKRDY		BIT(12)
#define MCP25XXFD_IOCON			MCP25XXFD_SFR_BASE(0x04)
#  define MCP25XXFD_IOCON_TRIS0		BIT(0)
#  define MCP25XXFD_IOCON_TRIS1		BIT(1)
#  define MCP25XXFD_IOCON_XSTBYEN	BIT(6)
#  define MCP25XXFD_IOCON_LAT0		BIT(8)
#  define MCP25XXFD_IOCON_LAT1		BIT(9)
#  define MCP25XXFD_IOCON_GPIO0		BIT(16)
#  define MCP25XXFD_IOCON_GPIO1		BIT(17)
#  define MCP25XXFD_IOCON_PM0		BIT(24)
#  define MCP25XXFD_IOCON_PM1		BIT(25)
#  define MCP25XXFD_IOCON_TXCANOD	BIT(28)
#  define MCP25XXFD_IOCON_SOF		BIT(29)
#  define MCP25XXFD_IOCON_INTOD		BIT(30)
#define MCP25XXFD_CRC			MCP25XXFD_SFR_BASE(0x08)
#  define MCP25XXFD_CRC_MASK		GENMASK(15, 0)
#  define MCP25XXFD_CRC_CRCERRIE	BIT(16)
#  define MCP25XXFD_CRC_FERRIE		BIT(17)
#  define MCP25XXFD_CRC_CRCERRIF	BIT(24)
#  define MCP25XXFD_CRC_FERRIF		BIT(25)
#define MCP25XXFD_ECCCON		MCP25XXFD_SFR_BASE(0x0C)
#  define MCP25XXFD_ECCCON_ECCEN	BIT(0)
#  define MCP25XXFD_ECCCON_SECIE	BIT(1)
#  define MCP25XXFD_ECCCON_DEDIE	BIT(2)
#  define MCP25XXFD_ECCCON_PARITY_BITS 6
#  define MCP25XXFD_ECCCON_PARITY_SHIFT 8
#  define MCP25XXFD_ECCCON_PARITY_MASK			\
	GENMASK(MCP25XXFD_ECCCON_PARITY_SHIFT		\
		+ MCP25XXFD_ECCCON_PARITY_BITS - 1,	\
		MCP25XXFD_ECCCON_PARITY_SHIFT)
#define MCP25XXFD_ECCSTAT		MCP25XXFD_SFR_BASE(0x10)
#  define MCP25XXFD_ECCSTAT_SECIF	BIT(1)
#  define MCP25XXFD_ECCSTAT_DEDIF	BIT(2)
#  define MCP25XXFD_ECCSTAT_ERRADDR_SHIFT 8
#  define MCP25XXFD_ECCSTAT_ERRADDR_MASK	      \
	GENMASK(MCP25XXFD_ECCSTAT_ERRADDR_SHIFT + 11, \
		MCP25XXFD_ECCSTAT_ERRADDR_SHIFT)

#define DEVICE_NAME "mcp25xxfd"

enum mcp25xxfd_model {
	CAN_MCP2517FD	= 0x2517,
};

struct mcp25xxfd_priv {
	struct spi_device *spi;
	struct gpio_chip *gpio;
	struct clk *clk;

	/* the actual model of the mcp25xxfd */
	enum mcp25xxfd_model model;

	/* everything clock related */
	int clock_freq;
	struct {
		/* clock configuration */
		int clock_pll;
		int clock_div2;
		int  clock_odiv;
	} config;

	/* lock for enabling/disabling the clock */
	struct mutex clk_user_lock;
	u32 clk_user_mask;
	u32 clk_sleep_mask;
#define MCP25XXFD_CLK_USER_CAN BIT(0)
#define MCP25XXFD_CLK_USER_GPIO0 BIT(1)
#define MCP25XXFD_CLK_USER_GPIO1 BIT(2)
#define MCP25XXFD_CLK_USER_CLKOUT BIT(3)

	/* power related */
	struct regulator *power;

	/* the distinct spi_speeds to use for spi communication */
	u32 spi_setup_speed_hz;
	u32 spi_normal_speed_hz;
	u32 spi_use_speed_hz;

	/* spi-tx/rx buffers for efficient transfers
	 * used during setup and irq
	 */
	struct mutex spi_rxtx_lock;
	u8 spi_tx[MCP25XXFD_BUFFER_TXRX_SIZE];
	u8 spi_rx[MCP25XXFD_BUFFER_TXRX_SIZE];

	/* configuration registers */
	struct {
		u32 osc;
		u32 iocon;
		u32 crc;
		u32 ecccon;
	} regs;

	/* debugfs related */
#if defined(CONFIG_DEBUG_FS)
	struct dentry *debugfs_dir;
	struct dentry *debugfs_regs_dir;
#endif
};

static inline int mcp25xxfd_power_enable(struct regulator *reg, int enable)
{
	if (IS_ERR_OR_NULL(reg))
		return 0;

	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static inline void mcp25xxfd_convert_to_cpu(u32 *data, int n)
{
	int i;

	for (i = 0; i < n; i++)
		data[i] = le32_to_cpu(data[i]);
}

static inline void mcp25xxfd_convert_from_cpu(u32 *data, int n)
{
	int i;

	for (i = 0; i < n; i++)
		data[i] = cpu_to_le32(data[i]);
}

static inline void mcp25xxfd_calc_cmd_addr(u16 cmd, u16 addr, u8 *data)
{
	cmd = cmd | (addr & ADDRESS_MASK);

	data[0] = (cmd >> 8) & 0xff;
	data[1] = (cmd >> 0) & 0xff;
}

static inline int mcp25xxfd_first_byte(u32 mask)
{
	return (mask & 0x0000ffff) ?
		((mask & 0x000000ff) ? 0 : 1) :
		((mask & 0x00ff0000) ? 2 : 3);
}

static inline int mcp25xxfd_last_byte(u32 mask)
{
	return (mask & 0xffff0000) ?
		((mask & 0xff000000) ? 3 : 2) :
		((mask & 0x0000ff00) ? 1 : 0);
}

int mcp25xxfd_cmd_readn(struct spi_device *spi, u32 reg,
			void *data, int n);
int mcp25xxfd_cmd_read_mask(struct spi_device *spi, u32 reg,
			    u32 *data, u32 mask);
static inline int mcp25xxfd_cmd_read(struct spi_device *spi, u32 reg,
				     u32 *data)
{
	return mcp25xxfd_cmd_read_mask(spi, reg, data, -1);
}

int mcp25xxfd_cmd_read_regs(struct spi_device *spi, u32 reg,
			    u32 *data, u32 bytes);

int mcp25xxfd_cmd_writen(struct spi_device *spi, u32 reg,
			 void *data, int n);
int mcp25xxfd_cmd_write_mask(struct spi_device *spi, u32 reg,
			     u32 data, u32 mask);
static inline int mcp25xxfd_cmd_write(struct spi_device *spi, u32 reg,
				      u32 data)
{
	return mcp25xxfd_cmd_write_mask(spi, reg, data, -1);
}

int mcp25xxfd_cmd_write_regs(struct spi_device *spi, u32 reg,
			     u32 *data, u32 bytes);

int mcp25xxfd_dump_regs_range(struct seq_file *file, u32 start, u32 end);

int mcp25xxfd_cmd_write_regs(struct spi_device *spi, u32 reg,
			     u32 *data, u32 bytes);

int mcp25xxfd_dump_regs_range(struct seq_file *file, u32 start, u32 end);

/* shared (internal) clock control */
int mcp25xxfd_stop_clock(struct spi_device *spi, int requestor_mask);
int mcp25xxfd_start_clock(struct spi_device *spi, int requestor_mask);

/* to put us to sleep fully we need the CAN controller to enter sleep mode */
int mcp25xxfd_can_sleep_mode(struct spi_device *spi);
/* probe the can controller */
int mcp25xxfd_can_hw_probe(struct spi_device *spi);

/* clearing interrupt flags */
int mcp25xxfd_clear_crc_interrupts(struct spi_device *spi);
int mcp25xxfd_clear_ecc_interrupts(struct spi_device *spi);
int mcp25xxfd_clear_interrupts(struct spi_device *spi);

/* enabling interrupts */
int mcp25xxfd_enable_interrupts(struct spi_device *spi, bool enable);

/* gpiolib support */
int mcp25xxfd_gpio_setup(struct spi_device *spi);
void mcp25xxfd_gpio_remove(struct spi_device *spi);
