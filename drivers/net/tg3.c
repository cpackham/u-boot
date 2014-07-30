/*
 * tg3.c: Broadcom Tigon3 ethernet driver.
 *
 * This is a port of the Tigon3 driver from Linux to U-Boot.
 *
 * Support for the following has been removed to simplify the code:
 *   - Jumbo frames, TSO, TSS, RSS, ASF, EEE, Flow control, VPD, APE, NVRAM
 *       -> These features are not very useful for a bootloader.
 *   - Interrupts, Timers
 *       -> These do not fit in with how things are done in U-Boot
 *   - Firmware
 *      -> May effect CHIPREV_ID_5701_A0 / ASIC_REV_57766.
 *   - USE_PHYLIB
 *      -> Used by ASIC_REV_5785 / ASIC_REV_57780
 *   - Subsys vendors
 *   - Setting the 10_100_ONLY flag
 *      -> In some cases, this value came from the driver_data value
 *         (Linux PCI).
 *      -> Will need to be re-implemented if your card needs this.
 *   - PCIe-only / PCI-X-only code.
 *   - Checking of pci_channel_offline.
 *      -> On Linux does this: (pdev->error_state != pci_channel_io_normal);
 *
 * Development and testing was done using the following card:
 *      [partno(BCM95718) rev 5717100] (PCI Express)
 *      vendor=14e4, device=1656
 *      PHY is 5718C (10/100/1000Base-T Ethernet)
 *
 * Other Notes:
 *
 *  - The DMA alloc function I'm using in here is borrowed from ARM code,
 *    as it was not defined for PPC.
 */

/*
 * tg3.c: Broadcom Tigon3 ethernet driver. (Linux)
 *
 * Copyright (C) 2001, 2002, 2003, 2004 David S. Miller (davem@redhat.com)
 * Copyright (C) 2001, 2002, 2003 Jeff Garzik (jgarzik@pobox.com)
 * Copyright (C) 2004 Sun Microsystems Inc.
 * Copyright (C) 2005-2014 Broadcom Corporation.
 */

#include <asm/byteorder.h>
#include <common.h>
#include <errno.h>
#include <malloc.h>
#include <net.h>
#include <netdev.h>
#include <asm/io.h>
#include <pci.h>
#include <linux/mii.h>
#include <phy.h>

/* TODO: Investigate the producur ring size. Will a smaller size work
 * on this card?  Do other cards require a larger size? */

#define TG3_TX_RING_SIZE		 32
#define TG3_RX_PRODUCER_RING_SIZE       512
#define TG3_RX_RETURN_RING_SIZE		 32
#define TG3_TX_BUFFER_COUNT		  1
#define TG3_RX_BUFFER_COUNT		 32
#define TG3_RX_FRAME_SIZE	       1536

#define  TG3_INIT_STATE_NONE 0
#define  TG3_INIT_STATE_PRE  1
#define  TG3_INIT_STATE_DONE 2

#include "tg3.h"

/* compatibility definition */
#define smp_mb() mb()

/* TODO: Find out why these aren't being pulled in from mii.h */
#define CTL1000_AS_MASTER	0x0800
#define CTL1000_ENABLE_MASTER	0x1000

/* TODO:  Borrowing this from ARM... */
static void *dma_alloc_coherent(size_t len, dma_addr_t *handle)
{
	*handle = (dma_addr_t)memalign(ARCH_DMA_MINALIGN, len);
	return (void *)*handle;
}

static struct pci_device_id tg3_supported[] = {
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5752 },
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5752M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5700},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5701},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5702},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5703},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5704},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5704S_2},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5702FE},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5705},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5705_2},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5719},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5721},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5722},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5723},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5705M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5705M_2},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5714},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5714S},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5780},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5780S},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5705F},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5754M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5755M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5756},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5750},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5751},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5715},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5715S},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5754},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5755},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5751M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5751F},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5787F},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5761E},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5761},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5764},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5787M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5782},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5784},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5786},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5787},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5788},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5789},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5702X},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5703X},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5704S},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5702A3},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5703A3},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5781},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5753},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5753M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5753F},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5901},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5901_2},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5906},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5906M},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5785_G},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5785_F},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57780},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57760},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57790},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57788},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5717},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5717_C},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5718},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57781},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57785},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57761},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57765},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57791},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57795},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5719},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5720},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57762},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57766},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5762},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5725},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_5727},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57764},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57767},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57787},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57782},
	{TG3PCI_VENDOR_BROADCOM, TG3PCI_DEVICE_TIGON3_57786},
	{PCI_VENDOR_ID_SYSKONNECT, PCI_DEVICE_ID_SYSKONNECT_9DXX},
	{PCI_VENDOR_ID_SYSKONNECT, PCI_DEVICE_ID_SYSKONNECT_9MXX},
	{PCI_VENDOR_ID_ALTIMA, PCI_DEVICE_ID_ALTIMA_AC1000},
	{PCI_VENDOR_ID_ALTIMA, PCI_DEVICE_ID_ALTIMA_AC1001},
	{PCI_VENDOR_ID_ALTIMA, PCI_DEVICE_ID_ALTIMA_AC1003},
	{PCI_VENDOR_ID_ALTIMA, PCI_DEVICE_ID_ALTIMA_AC9100},
	{PCI_VENDOR_ID_APPLE, PCI_DEVICE_ID_APPLE_TIGON3},
	{0x10cf, 0x11a2}, /* Fujitsu 1000base-SX with BCM5703SKHB */
	{}
};

/* Functions & macros to verify TG3_FLAGS types */

static inline int _tg3_flag(enum TG3_FLAGS flag, unsigned long *bits)
{
	return test_bit(flag, bits);
}

static inline void _tg3_flag_set(enum TG3_FLAGS flag, unsigned long *bits)
{
	set_bit(flag, bits);
}

static inline void _tg3_flag_clear(enum TG3_FLAGS flag, unsigned long *bits)
{
	clear_bit(flag, bits);
}

#define tg3_flag(tp, flag)				\
	_tg3_flag(TG3_FLAG_##flag, (tp)->tg3_flags)
#define tg3_flag_set(tp, flag)				\
	_tg3_flag_set(TG3_FLAG_##flag, (tp)->tg3_flags)
#define tg3_flag_clear(tp, flag)			\
	_tg3_flag_clear(TG3_FLAG_##flag, (tp)->tg3_flags)

#define RESET_KIND_SHUTDOWN	 0
#define RESET_KIND_INIT		 1
#define RESET_KIND_SUSPEND       2

#define TG3_DEF_RX_MODE		 0
#define TG3_DEF_TX_MODE		 0

#define TG3_GRC_LCLCTL_PWRSW_DELAY	100

/* hardware minimum and maximum for a single frame's data payload */
#define TG3_MAX_MTU(tp) 1500

/* Do not place this n-ring entries value into the tp struct itself,
 * we really want to expose these constants to GCC so that modulo et
 * al.  operations are done with shifts and masks instead of with
 * hw multiply/modulo instructions.  Another solution would be to
 * replace things like '% foo' with '& (foo - 1)'.
 */

#define TG3_RX_STD_RING_BYTES \
	(sizeof(struct tg3_rx_buffer_desc) * TG3_RX_PRODUCER_RING_SIZE)
#define TG3_RX_RCB_RING_BYTES \
	(sizeof(struct tg3_rx_buffer_desc) * TG3_RX_RETURN_RING_SIZE)
#define TG3_TX_RING_BYTES \
	(sizeof(struct tg3_tx_buffer_desc) * TG3_TX_RING_SIZE)

#define TG3_TX_BD_DMA_MAX_2K		2048
#define TG3_TX_BD_DMA_MAX_4K		4096

static void tg3_write32(struct tg3_priv *tp, u32 off, u32 val)
{
	writel(val, tp->regs + off);
}

static u32 tg3_read32(struct tg3_priv *tp, u32 off)
{
	return readl(tp->regs + off);
}

static void tg3_write_indirect_reg32(struct tg3_priv *tp, u32 off, u32 val)
{
	pci_write_config_dword(tp->devno, TG3PCI_REG_BASE_ADDR, off);
	pci_write_config_dword(tp->devno, TG3PCI_REG_DATA, val);
}

static void tg3_write_flush_reg32(struct tg3_priv *tp, u32 off, u32 val)
{
	writel(val, tp->regs + off);
	readl(tp->regs + off);
}

static u32 tg3_read_indirect_reg32(struct tg3_priv *tp, u32 off)
{
	u32 val;
	pci_write_config_dword(tp->devno, TG3PCI_REG_BASE_ADDR, off);
	pci_read_config_dword(tp->devno, TG3PCI_REG_DATA, &val);
	return val;
}

static void tg3_write_indirect_mbox(struct tg3_priv *tp, u32 off, u32 val)
{
	if (off == (MAILBOX_RCVRET_CON_IDX_0 + TG3_64BIT_REG_LOW)) {
		pci_write_config_dword(tp->devno, TG3PCI_RCV_RET_RING_CON_IDX +
				       TG3_64BIT_REG_LOW, val);
		return;
	}
	if (off == TG3_RX_STD_PROD_IDX_REG) {
		pci_write_config_dword(tp->devno, TG3PCI_STD_RING_PROD_IDX +
				       TG3_64BIT_REG_LOW, val);
		return;
	}

	pci_write_config_dword(tp->devno, TG3PCI_REG_BASE_ADDR, off + 0x5600);
	pci_write_config_dword(tp->devno, TG3PCI_REG_DATA, val);

	/* In indirect mode when disabling interrupts, we also need
	 * to clear the interrupt bit in the GRC local ctrl register.
	 */
	if ((off == (MAILBOX_INTERRUPT_0 + TG3_64BIT_REG_LOW)) &&
	    (val == 0x1)) {
		pci_write_config_dword(tp->devno, TG3PCI_MISC_LOCAL_CTRL,
				       tp->grc_local_ctrl|GRC_LCLCTRL_CLEARINT);
	}
}

static u32 tg3_read_indirect_mbox(struct tg3_priv *tp, u32 off)
{
	u32 val;
	pci_write_config_dword(tp->devno, TG3PCI_REG_BASE_ADDR, off + 0x5600);
	pci_read_config_dword(tp->devno, TG3PCI_REG_DATA, &val);
	return val;
}

/* usec_wait specifies the wait time in usec when writing to certain registers
 * where it is unsafe to read back the register without some delay.
 * GRC_LOCAL_CTRL is one example if the GPIOs are toggled to switch power.
 * TG3PCI_CLOCK_CTRL is another example if the clock frequencies are changed.
 */
static void _tw32_flush(struct tg3_priv *tp, u32 off, u32 val, u32 usec_wait)
{
	if (tg3_flag(tp, PCIX_TARGET_HWBUG) || tg3_flag(tp, ICH_WORKAROUND))
		/* Non-posted methods */
		tp->write32(tp, off, val);
	else {
		/* Posted method */
		tg3_write32(tp, off, val);
		if (usec_wait)
			udelay(usec_wait);
		tp->read32(tp, off);
	}
	/* Wait again after the read for the posted method to guarantee that
	 * the wait time is met.
	 */
	if (usec_wait)
		udelay(usec_wait);
}

static inline void tw32_mailbox_flush(struct tg3_priv *tp, u32 off, u32 val)
{
	tp->write32_mbox(tp, off, val);
	if (tg3_flag(tp, FLUSH_POSTED_WRITES) ||
	    (!tg3_flag(tp, MBOX_WRITE_REORDER) &&
	     !tg3_flag(tp, ICH_WORKAROUND)))
		tp->read32_mbox(tp, off);
}

static void tg3_write32_tx_mbox(struct tg3_priv *tp, u32 off, u32 val)
{
	void __iomem *mbox = tp->regs + off;
	writel(val, mbox);
	if (tg3_flag(tp, TXD_MBOX_HWBUG))
		writel(val, mbox);
	if (tg3_flag(tp, MBOX_WRITE_REORDER) ||
	    tg3_flag(tp, FLUSH_POSTED_WRITES))
		readl(mbox);
}

static u32 tg3_read32_mbox_5906(struct tg3_priv *tp, u32 off)
{
	return readl(tp->regs + off + GRCMBOX_BASE);
}

static void tg3_write32_mbox_5906(struct tg3_priv *tp, u32 off, u32 val)
{
	writel(val, tp->regs + off + GRCMBOX_BASE);
}

#define tw32_mailbox(reg, val)		tp->write32_mbox(tp, reg, val)
#define tw32_mailbox_f(reg, val)	tw32_mailbox_flush(tp, (reg), (val))
#define tw32_rx_mbox(reg, val)		tp->write32_rx_mbox(tp, reg, val)
#define tw32_tx_mbox(reg, val)		tp->write32_tx_mbox(tp, reg, val)
#define tr32_mailbox(reg)		tp->read32_mbox(tp, reg)

#define tw32(reg, val)			tp->write32(tp, reg, val)
#define tw32_f(reg, val)		_tw32_flush(tp, (reg), (val), 0)
#define tw32_wait_f(reg, val, us)	_tw32_flush(tp, (reg), (val), (us))
#define tr32(reg)			tp->read32(tp, reg)

static void tg3_write_mem(struct tg3_priv *tp, u32 off, u32 val)
{
	if (tg3_asic_rev(tp) == ASIC_REV_5906 && (off >= NIC_SRAM_STATS_BLK) &&
	    (off < NIC_SRAM_TX_BUFFER_DESC))
		return;

	if (tg3_flag(tp, SRAM_USE_CONFIG)) {
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_BASE_ADDR,
				       off);
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_DATA, val);

		/* Always leave this as zero. */
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_BASE_ADDR, 0);
	} else {
		tw32_f(TG3PCI_MEM_WIN_BASE_ADDR, off);
		tw32_f(TG3PCI_MEM_WIN_DATA, val);

		/* Always leave this as zero. */
		tw32_f(TG3PCI_MEM_WIN_BASE_ADDR, 0);
	}
}

static void tg3_read_mem(struct tg3_priv *tp, u32 off, u32 *val)
{
	if (tg3_asic_rev(tp) == ASIC_REV_5906 &&
	    (off >= NIC_SRAM_STATS_BLK) && (off < NIC_SRAM_TX_BUFFER_DESC)) {
		*val = 0;
		return;
	}

	if (tg3_flag(tp, SRAM_USE_CONFIG)) {
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_BASE_ADDR, off);
		pci_read_config_dword(tp->devno, TG3PCI_MEM_WIN_DATA, val);

		/* Always leave this as zero. */
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_BASE_ADDR, 0);
	} else {
		tw32_f(TG3PCI_MEM_WIN_BASE_ADDR, off);
		*val = tr32(TG3PCI_MEM_WIN_DATA);

		/* Always leave this as zero. */
		tw32_f(TG3PCI_MEM_WIN_BASE_ADDR, 0);
	}
}

static void tg3_disable_ints(struct tg3_priv *tp)
{
	tw32(TG3PCI_MISC_HOST_CTRL, (tp->misc_host_ctrl | MISC_HOST_CTRL_MASK_PCI_INT));
	tw32_mailbox_f(tp->napi[0].int_mbox, 0x00000001);
}

static void tg3_switch_clocks(struct tg3_priv *tp)
{
	u32 clock_ctrl; u32
	orig_clock_ctrl;

	if (tg3_flag(tp, CPMU_PRESENT) || tg3_flag(tp, 5780_CLASS))
		return;

	clock_ctrl = tr32(TG3PCI_CLOCK_CTRL);

	orig_clock_ctrl = clock_ctrl;
	clock_ctrl &= (CLOCK_CTRL_FORCE_CLKRUN |
		       CLOCK_CTRL_CLKRUN_OENABLE |
		       0x1f);
	tp->pci_clock_ctrl = clock_ctrl;

	if (tg3_flag(tp, 5705_PLUS)) {
		if (orig_clock_ctrl & CLOCK_CTRL_625_CORE) {
			tw32_wait_f(TG3PCI_CLOCK_CTRL,
				    clock_ctrl | CLOCK_CTRL_625_CORE, 40);
		}
	} else if ((orig_clock_ctrl & CLOCK_CTRL_44MHZ_CORE) != 0) {
		tw32_wait_f(TG3PCI_CLOCK_CTRL,
			    clock_ctrl |
			    (CLOCK_CTRL_44MHZ_CORE | CLOCK_CTRL_ALTCLK),
			    40);
		tw32_wait_f(TG3PCI_CLOCK_CTRL,
			    clock_ctrl | (CLOCK_CTRL_ALTCLK),
			    40);
	}
	tw32_wait_f(TG3PCI_CLOCK_CTRL, clock_ctrl, 40);
}

#define PHY_BUSY_LOOPS	5000

static int __tg3_readphy(struct tg3_priv *tp, unsigned int phy_addr, int reg,
			 u32 *val)
{
	u32 frame_val;
	unsigned int loops;
	int ret;

	if ((tp->mi_mode & MAC_MI_MODE_AUTO_POLL) != 0) {
		tw32_f(MAC_MI_MODE,
		       (tp->mi_mode & ~MAC_MI_MODE_AUTO_POLL));
		udelay(80);
	}

	*val = 0x0;

	frame_val  = ((phy_addr << MI_COM_PHY_ADDR_SHIFT) &
		      MI_COM_PHY_ADDR_MASK);
	frame_val |= ((reg << MI_COM_REG_ADDR_SHIFT) &
		      MI_COM_REG_ADDR_MASK);
	frame_val |= (MI_COM_CMD_READ | MI_COM_START);

	tw32_f(MAC_MI_COM, frame_val);

	loops = PHY_BUSY_LOOPS;
	while (loops != 0) {
		udelay(10);
		frame_val = tr32(MAC_MI_COM);

		if ((frame_val & MI_COM_BUSY) == 0) {
			udelay(5);
			frame_val = tr32(MAC_MI_COM);
			break;
		}
		loops -= 1;
	}

	ret = -EBUSY;
	if (loops != 0) {
		*val = frame_val & MI_COM_DATA_MASK;
		ret = 0;
	}

	if ((tp->mi_mode & MAC_MI_MODE_AUTO_POLL) != 0) {
		tw32_f(MAC_MI_MODE, tp->mi_mode);
		udelay(80);
	}

	return ret;
}

static int tg3_readphy(struct tg3_priv *tp, int reg, u32 *val)
{
	return __tg3_readphy(tp, tp->phy_addr, reg, val);
}

static int __tg3_writephy(struct tg3_priv *tp, unsigned int phy_addr, int reg,
			  u32 val)
{
	u32 frame_val;
	unsigned int loops;
	int ret;

	if ((tp->phy_flags & TG3_PHYFLG_IS_FET) &&
	    (reg == MII_CTRL1000 || reg == MII_TG3_AUX_CTRL))
		return 0;

	if ((tp->mi_mode & MAC_MI_MODE_AUTO_POLL) != 0) {
		tw32_f(MAC_MI_MODE,
		       (tp->mi_mode & ~MAC_MI_MODE_AUTO_POLL));
		udelay(80);
	}

	frame_val  = ((phy_addr << MI_COM_PHY_ADDR_SHIFT) &
		      MI_COM_PHY_ADDR_MASK);
	frame_val |= ((reg << MI_COM_REG_ADDR_SHIFT) &
		      MI_COM_REG_ADDR_MASK);
	frame_val |= (val & MI_COM_DATA_MASK);
	frame_val |= (MI_COM_CMD_WRITE | MI_COM_START);

	tw32_f(MAC_MI_COM, frame_val);

	loops = PHY_BUSY_LOOPS;
	while (loops != 0) {
		udelay(10);
		frame_val = tr32(MAC_MI_COM);
		if ((frame_val & MI_COM_BUSY) == 0) {
			udelay(5);
			frame_val = tr32(MAC_MI_COM);
			break;
		}
		loops -= 1;
	}

	ret = -EBUSY;
	if (loops != 0)
		ret = 0;

	if ((tp->mi_mode & MAC_MI_MODE_AUTO_POLL) != 0) {
		tw32_f(MAC_MI_MODE, tp->mi_mode);
		udelay(80);
	}

	return ret;
}

static int tg3_writephy(struct tg3_priv *tp, int reg, u32 val)
{
	return __tg3_writephy(tp, tp->phy_addr, reg, val);
}

static int tg3_phydsp_write(struct tg3_priv *tp, u32 reg, u32 val)
{
	int err;

	err = tg3_writephy(tp, MII_TG3_DSP_ADDRESS, reg);
	if (!err)
		err = tg3_writephy(tp, MII_TG3_DSP_RW_PORT, val);

	return err;
}

static int tg3_phy_auxctl_read(struct tg3_priv *tp, int reg, u32 *val)
{
	int err;

	err = tg3_writephy(tp, MII_TG3_AUX_CTRL,
			   (reg << MII_TG3_AUXCTL_MISC_RDSEL_SHIFT) |
			   MII_TG3_AUXCTL_SHDWSEL_MISC);
	if (!err)
		err = tg3_readphy(tp, MII_TG3_AUX_CTRL, val);

	return err;
}

static int tg3_phy_auxctl_write(struct tg3_priv *tp, int reg, u32 set)
{
	if (reg == MII_TG3_AUXCTL_SHDWSEL_MISC)
		set |= MII_TG3_AUXCTL_MISC_WREN;

	return tg3_writephy(tp, MII_TG3_AUX_CTRL, set | reg);
}

static int tg3_phy_toggle_auxctl_smdsp(struct tg3_priv *tp, bool enable)
{
	u32 val;
	int err;

	err = tg3_phy_auxctl_read(tp, MII_TG3_AUXCTL_SHDWSEL_AUXCTL, &val);

	if (err)
		return err;

	if (enable)
		val |= MII_TG3_AUXCTL_ACTL_SMDSP_ENA;
	else
		val &= ~MII_TG3_AUXCTL_ACTL_SMDSP_ENA;

	err = tg3_phy_auxctl_write((tp), MII_TG3_AUXCTL_SHDWSEL_AUXCTL,
				   val | MII_TG3_AUXCTL_ACTL_TX_6DB);

	return err;
}

static int tg3_phy_shdw_write(struct tg3_priv *tp, int reg, u32 val)
{
	return tg3_writephy(tp, MII_TG3_MISC_SHDW,
			    reg | val | MII_TG3_MISC_SHDW_WREN);
}

static int tg3_bmcr_reset(struct tg3_priv *tp)
{
	u32 phy_control;
	int limit, err;

	/* OK, reset it, and poll the BMCR_RESET bit until it
	 * clears or we time out.
	 */
	phy_control = BMCR_RESET;
	err = tg3_writephy(tp, MII_BMCR, phy_control);
	if (err != 0)
		return -EBUSY;

	limit = 5000;
	while (limit--) {
		err = tg3_readphy(tp, MII_BMCR, &phy_control);
		if (err != 0)
			return -EBUSY;

		if ((phy_control & BMCR_RESET) == 0) {
			udelay(40);
			break;
		}
		udelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static void tg3_mdio_start(struct tg3_priv *tp)
{
	tp->mi_mode &= ~MAC_MI_MODE_AUTO_POLL;
	tw32_f(MAC_MI_MODE, tp->mi_mode);
	udelay(80);
}

static int tg3_mdio_init(struct tg3_priv *tp)
{
	if (tg3_flag(tp, 5717_PLUS)) {
		u32 is_serdes;

		tp->phy_addr = tp->pci_fn + 1;

		if (tg3_chip_rev_id(tp) != CHIPREV_ID_5717_A0)
			is_serdes = tr32(SG_DIG_STATUS) & SG_DIG_IS_SERDES;
		else
			is_serdes = tr32(TG3_CPMU_PHY_STRAP) &
				    TG3_CPMU_PHY_STRAP_IS_SERDES;
		if (is_serdes)
			tp->phy_addr += 7;
	} else {
		tp->phy_addr = TG3_PHY_MII_ADDR;
	}

	tg3_mdio_start(tp);

	return 0;
}

static void tg3_write_sig_pre_reset(struct tg3_priv *tp, int kind)
{
	tg3_write_mem(tp, NIC_SRAM_FIRMWARE_MBOX, NIC_SRAM_FIRMWARE_MBOX_MAGIC1);
}

static int tg3_poll_fw(struct tg3_priv *tp)
{
	int i;
	u32 val;

	if (tg3_flag(tp, NO_FWARE_REPORTED))
		return 0;

	if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		/* Wait up to 20ms for init done. */
		for (i = 0; i < 200; i++) {
			if (tr32(VCPU_STATUS) & VCPU_STATUS_INIT_DONE)
				return 0;

			udelay(100);
		}
		return -ENODEV;
	}

	/* Wait for firmware initialization to complete. */
	for (i = 0; i < 100000; i++) {
		tg3_read_mem(tp, NIC_SRAM_FIRMWARE_MBOX, &val);
		if (val == ~NIC_SRAM_FIRMWARE_MBOX_MAGIC1)
			break;
		udelay(10);
	}

	/* Chip might not be fitted with firmware.  Some Sun onboard
	 * parts are configured like that.  So don't signal the timeout
	 * of the above loop as an error, but do report the lack of
	 * running firmware once.
	 */
	if (i >= 100000 && !tg3_flag(tp, NO_FWARE_REPORTED)) {
		tg3_flag_set(tp, NO_FWARE_REPORTED);

		printf("#   No firmware running\n");
	}

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_57765_A0) {
		/* The 57765 A0 needs a little more
		 * time to do some important work.
		 */
		mdelay(10);
	}

	return 0;
}

static void tg3_phy_fet_toggle_apd(struct tg3_priv *tp, bool enable)
{
	u32 phytest;

	if (!tg3_readphy(tp, MII_TG3_FET_TEST, &phytest)) {
		u32 phy;

		tg3_writephy(tp, MII_TG3_FET_TEST,
			     phytest | MII_TG3_FET_SHADOW_EN);
		if (!tg3_readphy(tp, MII_TG3_FET_SHDW_AUXSTAT2, &phy)) {
			if (enable)
				phy |= MII_TG3_FET_SHDW_AUXSTAT2_APD;
			else
				phy &= ~MII_TG3_FET_SHDW_AUXSTAT2_APD;
			tg3_writephy(tp, MII_TG3_FET_SHDW_AUXSTAT2, phy);
		}
		tg3_writephy(tp, MII_TG3_FET_TEST, phytest);
	}
}

static void tg3_phy_toggle_apd(struct tg3_priv *tp, bool enable)
{
	u32 reg;

	if (!tg3_flag(tp, 5705_PLUS) ||
	    (tg3_flag(tp, 5717_PLUS) &&
	     (tp->phy_flags & TG3_PHYFLG_MII_SERDES)))
		return;

	if (tp->phy_flags & TG3_PHYFLG_IS_FET) {
		tg3_phy_fet_toggle_apd(tp, enable);
		return;
	}

	reg = MII_TG3_MISC_SHDW_SCR5_LPED |
	      MII_TG3_MISC_SHDW_SCR5_DLPTLM |
	      MII_TG3_MISC_SHDW_SCR5_SDTL |
	      MII_TG3_MISC_SHDW_SCR5_C125OE;
	if (tg3_asic_rev(tp) != ASIC_REV_5784 || !enable)
		reg |= MII_TG3_MISC_SHDW_SCR5_DLLAPD;

	tg3_phy_shdw_write(tp, MII_TG3_MISC_SHDW_SCR5_SEL, reg);


	reg = MII_TG3_MISC_SHDW_APD_WKTM_84MS;
	if (enable)
		reg |= MII_TG3_MISC_SHDW_APD_ENABLE;

	tg3_phy_shdw_write(tp, MII_TG3_MISC_SHDW_APD_SEL, reg);
}

static void tg3_phy_toggle_automdix(struct tg3_priv *tp, bool enable)
{
	u32 phy;

	if (!tg3_flag(tp, 5705_PLUS) ||
	    (tp->phy_flags & TG3_PHYFLG_ANY_SERDES))
		return;

	if (tp->phy_flags & TG3_PHYFLG_IS_FET) {
		u32 ephy;

		if (!tg3_readphy(tp, MII_TG3_FET_TEST, &ephy)) {
			u32 reg = MII_TG3_FET_SHDW_MISCCTRL;

			tg3_writephy(tp, MII_TG3_FET_TEST,
				     ephy | MII_TG3_FET_SHADOW_EN);
			if (!tg3_readphy(tp, reg, &phy)) {
				if (enable)
					phy |= MII_TG3_FET_SHDW_MISCCTRL_MDIX;
				else
					phy &= ~MII_TG3_FET_SHDW_MISCCTRL_MDIX;
				tg3_writephy(tp, reg, phy);
			}
			tg3_writephy(tp, MII_TG3_FET_TEST, ephy);
		}
	} else {
		int ret;

		ret = tg3_phy_auxctl_read(tp,
					  MII_TG3_AUXCTL_SHDWSEL_MISC, &phy);
		if (!ret) {
			if (enable)
				phy |= MII_TG3_AUXCTL_MISC_FORCE_AMDIX;
			else
				phy &= ~MII_TG3_AUXCTL_MISC_FORCE_AMDIX;
			tg3_phy_auxctl_write(tp,
					     MII_TG3_AUXCTL_SHDWSEL_MISC, phy);
		}
	}
}

static void tg3_phy_set_wirespeed(struct tg3_priv *tp)
{
	int ret;
	u32 val;

	if (tp->phy_flags & TG3_PHYFLG_NO_ETH_WIRE_SPEED)
		return;

	ret = tg3_phy_auxctl_read(tp, MII_TG3_AUXCTL_SHDWSEL_MISC, &val);
	if (!ret)
		tg3_phy_auxctl_write(tp, MII_TG3_AUXCTL_SHDWSEL_MISC,
				     val | MII_TG3_AUXCTL_MISC_WIRESPD_EN);
}

static void tg3_phy_apply_otp(struct tg3_priv *tp)
{
	u32 otp, phy;

	if (!tp->phy_otp)
		return;

	otp = tp->phy_otp;

	if (tg3_phy_toggle_auxctl_smdsp(tp, true))
		return;

	phy = ((otp & TG3_OTP_AGCTGT_MASK) >> TG3_OTP_AGCTGT_SHIFT);
	phy |= MII_TG3_DSP_TAP1_AGCTGT_DFLT;
	tg3_phydsp_write(tp, MII_TG3_DSP_TAP1, phy);

	phy = ((otp & TG3_OTP_HPFFLTR_MASK) >> TG3_OTP_HPFFLTR_SHIFT) |
	      ((otp & TG3_OTP_HPFOVER_MASK) >> TG3_OTP_HPFOVER_SHIFT);
	tg3_phydsp_write(tp, MII_TG3_DSP_AADJ1CH0, phy);

	phy = ((otp & TG3_OTP_LPFDIS_MASK) >> TG3_OTP_LPFDIS_SHIFT);
	phy |= MII_TG3_DSP_AADJ1CH3_ADCCKADJ;
	tg3_phydsp_write(tp, MII_TG3_DSP_AADJ1CH3, phy);

	phy = ((otp & TG3_OTP_VDAC_MASK) >> TG3_OTP_VDAC_SHIFT);
	tg3_phydsp_write(tp, MII_TG3_DSP_EXP75, phy);

	phy = ((otp & TG3_OTP_10BTAMP_MASK) >> TG3_OTP_10BTAMP_SHIFT);
	tg3_phydsp_write(tp, MII_TG3_DSP_EXP96, phy);

	phy = ((otp & TG3_OTP_ROFF_MASK) >> TG3_OTP_ROFF_SHIFT) |
	      ((otp & TG3_OTP_RCOFF_MASK) >> TG3_OTP_RCOFF_SHIFT);
	tg3_phydsp_write(tp, MII_TG3_DSP_EXP97, phy);

	tg3_phy_toggle_auxctl_smdsp(tp, false);
}

static int tg3_wait_macro_done(struct tg3_priv *tp)
{
	int limit = 100;

	while (limit--) {
		u32 tmp32;

		if (!tg3_readphy(tp, MII_TG3_DSP_CONTROL, &tmp32)) {
			if ((tmp32 & 0x1000) == 0)
				break;
		}
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static int tg3_phy_write_and_check_testpat(struct tg3_priv *tp, int *resetp)
{
	static const u32 test_pat[4][6] = {
	{ 0x00005555, 0x00000005, 0x00002aaa, 0x0000000a, 0x00003456, 0x00000003 },
	{ 0x00002aaa, 0x0000000a, 0x00003333, 0x00000003, 0x0000789a, 0x00000005 },
	{ 0x00005a5a, 0x00000005, 0x00002a6a, 0x0000000a, 0x00001bcd, 0x00000003 },
	{ 0x00002a5a, 0x0000000a, 0x000033c3, 0x00000003, 0x00002ef1, 0x00000005 }
	};
	int chan;

	for (chan = 0; chan < 4; chan++) {
		int i;

		tg3_writephy(tp, MII_TG3_DSP_ADDRESS,
			     (chan * 0x2000) | 0x0200);
		tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0002);

		for (i = 0; i < 6; i++)
			tg3_writephy(tp, MII_TG3_DSP_RW_PORT,
				     test_pat[chan][i]);

		tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0202);
		if (tg3_wait_macro_done(tp)) {
			*resetp = 1;
			return -EBUSY;
		}

		tg3_writephy(tp, MII_TG3_DSP_ADDRESS,
			     (chan * 0x2000) | 0x0200);
		tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0082);
		if (tg3_wait_macro_done(tp)) {
			*resetp = 1;
			return -EBUSY;
		}

		tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0802);
		if (tg3_wait_macro_done(tp)) {
			*resetp = 1;
			return -EBUSY;
		}

		for (i = 0; i < 6; i += 2) {
			u32 low, high;

			if (tg3_readphy(tp, MII_TG3_DSP_RW_PORT, &low) ||
			    tg3_readphy(tp, MII_TG3_DSP_RW_PORT, &high) ||
			    tg3_wait_macro_done(tp)) {
				*resetp = 1;
				return -EBUSY;
			}
			low &= 0x7fff;
			high &= 0x000f;
			if (low != test_pat[chan][i] ||
			    high != test_pat[chan][i+1]) {
				tg3_writephy(tp, MII_TG3_DSP_ADDRESS, 0x000b);
				tg3_writephy(tp, MII_TG3_DSP_RW_PORT, 0x4001);
				tg3_writephy(tp, MII_TG3_DSP_RW_PORT, 0x4005);

				return -EBUSY;
			}
		}
	}

	return 0;
}

static int tg3_phy_reset_chanpat(struct tg3_priv *tp)
{
	int chan;

	for (chan = 0; chan < 4; chan++) {
		int i;

		tg3_writephy(tp, MII_TG3_DSP_ADDRESS,
			     (chan * 0x2000) | 0x0200);
		tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0002);
		for (i = 0; i < 6; i++)
			tg3_writephy(tp, MII_TG3_DSP_RW_PORT, 0x000);
		tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0202);
		if (tg3_wait_macro_done(tp))
			return -EBUSY;
	}

	return 0;
}

static int tg3_phy_reset_5703_4_5(struct tg3_priv *tp)
{
	u32 reg32, phy9_orig;
	int retries, do_phy_reset, err;

	retries = 10;
	do_phy_reset = 1;
	do {
		if (do_phy_reset) {
			err = tg3_bmcr_reset(tp);
			if (err)
				return err;
			do_phy_reset = 0;
		}

		/* Disable transmitter and interrupt.  */
		if (tg3_readphy(tp, MII_TG3_EXT_CTRL, &reg32))
			continue;

		reg32 |= 0x3000;
		tg3_writephy(tp, MII_TG3_EXT_CTRL, reg32);

		/* Set full-duplex, 1000 mbps.  */
		tg3_writephy(tp, MII_BMCR,
			     BMCR_FULLDPLX | BMCR_SPEED1000);

		/* Set to master mode.  */
		if (tg3_readphy(tp, MII_CTRL1000, &phy9_orig))
			continue;

		tg3_writephy(tp, MII_CTRL1000,
			     CTL1000_AS_MASTER | CTL1000_ENABLE_MASTER);

		err = tg3_phy_toggle_auxctl_smdsp(tp, true);
		if (err)
			return err;

		/* Block the PHY control access.  */
		tg3_phydsp_write(tp, 0x8005, 0x0800);

		err = tg3_phy_write_and_check_testpat(tp, &do_phy_reset);
		if (!err)
			break;
	} while (--retries);

	err = tg3_phy_reset_chanpat(tp);
	if (err)
		return err;

	tg3_phydsp_write(tp, 0x8005, 0x0000);

	tg3_writephy(tp, MII_TG3_DSP_ADDRESS, 0x8200);
	tg3_writephy(tp, MII_TG3_DSP_CONTROL, 0x0000);

	tg3_phy_toggle_auxctl_smdsp(tp, false);

	tg3_writephy(tp, MII_CTRL1000, phy9_orig);

	err = tg3_readphy(tp, MII_TG3_EXT_CTRL, &reg32);
	if (err)
		return err;

	reg32 &= ~0x3000;
	tg3_writephy(tp, MII_TG3_EXT_CTRL, reg32);

	return 0;
}

/* This will reset the tigon3 PHY if there is no valid
 * link unless the FORCE argument is non-zero.
 */
static int tg3_phy_reset(struct tg3_priv *tp)
{
	u32 val, cpmuctrl;
	int err;

	if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		val = tr32(GRC_MISC_CFG);
		tw32_f(GRC_MISC_CFG, val & ~GRC_MISC_CFG_EPHY_IDDQ);
		udelay(40);
	}
	err  = tg3_readphy(tp, MII_BMSR, &val);
	err |= tg3_readphy(tp, MII_BMSR, &val);
	if (err != 0)
		return -EBUSY;

	if (tg3_asic_rev(tp) == ASIC_REV_5703 ||
	    tg3_asic_rev(tp) == ASIC_REV_5704 ||
	    tg3_asic_rev(tp) == ASIC_REV_5705) {
		err = tg3_phy_reset_5703_4_5(tp);
		if (err)
			return err;
		goto out;
	}

	cpmuctrl = 0;
	if (tg3_asic_rev(tp) == ASIC_REV_5784 &&
	    tg3_chip_rev(tp) != CHIPREV_5784_AX) {
		cpmuctrl = tr32(TG3_CPMU_CTRL);
		if (cpmuctrl & CPMU_CTRL_GPHY_10MB_RXONLY)
			tw32(TG3_CPMU_CTRL,
			     cpmuctrl & ~CPMU_CTRL_GPHY_10MB_RXONLY);
	}

	err = tg3_bmcr_reset(tp);
	if (err)
		return err;

	if (cpmuctrl & CPMU_CTRL_GPHY_10MB_RXONLY) {
		val = MII_TG3_DSP_EXP8_AEDW | MII_TG3_DSP_EXP8_REJ2MHz;
		tg3_phydsp_write(tp, MII_TG3_DSP_EXP8, val);

		tw32(TG3_CPMU_CTRL, cpmuctrl);
	}

	if (tg3_chip_rev(tp) == CHIPREV_5784_AX ||
	    tg3_chip_rev(tp) == CHIPREV_5761_AX) {
		val = tr32(TG3_CPMU_LSPD_1000MB_CLK);
		if ((val & CPMU_LSPD_1000MB_MACCLK_MASK) ==
		    CPMU_LSPD_1000MB_MACCLK_12_5) {
			val &= ~CPMU_LSPD_1000MB_MACCLK_MASK;
			udelay(40);
			tw32_f(TG3_CPMU_LSPD_1000MB_CLK, val);
		}
	}

	if (tg3_flag(tp, 5717_PLUS) &&
	    (tp->phy_flags & TG3_PHYFLG_MII_SERDES))
		return 0;

	tg3_phy_apply_otp(tp);

	if (tp->phy_flags & TG3_PHYFLG_ENABLE_APD)
		tg3_phy_toggle_apd(tp, true);
	else
		tg3_phy_toggle_apd(tp, false);

out:
	if ((tp->phy_flags & TG3_PHYFLG_ADC_BUG) &&
	    !tg3_phy_toggle_auxctl_smdsp(tp, true)) {
		tg3_phydsp_write(tp, 0x201f, 0x2aaa);
		tg3_phydsp_write(tp, 0x000a, 0x0323);
		tg3_phy_toggle_auxctl_smdsp(tp, false);
	}

	if (tp->phy_flags & TG3_PHYFLG_5704_A0_BUG) {
		tg3_writephy(tp, MII_TG3_MISC_SHDW, 0x8d68);
		tg3_writephy(tp, MII_TG3_MISC_SHDW, 0x8d68);
	}

	if (tp->phy_flags & TG3_PHYFLG_BER_BUG) {
		if (!tg3_phy_toggle_auxctl_smdsp(tp, true)) {
			tg3_phydsp_write(tp, 0x000a, 0x310b);
			tg3_phydsp_write(tp, 0x201f, 0x9506);
			tg3_phydsp_write(tp, 0x401f, 0x14e2);
			tg3_phy_toggle_auxctl_smdsp(tp, false);
		}
	} else if (tp->phy_flags & TG3_PHYFLG_JITTER_BUG) {
		if (!tg3_phy_toggle_auxctl_smdsp(tp, true)) {
			tg3_writephy(tp, MII_TG3_DSP_ADDRESS, 0x000a);
			if (tp->phy_flags & TG3_PHYFLG_ADJUST_TRIM) {
				tg3_writephy(tp, MII_TG3_DSP_RW_PORT, 0x110b);
				tg3_writephy(tp, MII_TG3_TEST1,
					     MII_TG3_TEST1_TRIM_EN | 0x4);
			} else {
				tg3_writephy(tp, MII_TG3_DSP_RW_PORT, 0x010b);
			}

			tg3_phy_toggle_auxctl_smdsp(tp, false);
		}
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		/* adjust output voltage */
		tg3_writephy(tp, MII_TG3_FET_PTEST, 0x12);
	}

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5762_A0)
		tg3_phydsp_write(tp, 0xffb, 0x4000);

	tg3_phy_toggle_automdix(tp, true);
	tg3_phy_set_wirespeed(tp);
	return 0;
}

#define TG3_GPIO_MSG_DRVR_PRES		 0x00000001
#define TG3_GPIO_MSG_NEED_VAUX		 0x00000002
#define TG3_GPIO_MSG_MASK		 (TG3_GPIO_MSG_DRVR_PRES | \
					  TG3_GPIO_MSG_NEED_VAUX)
#define TG3_GPIO_MSG_ALL_DRVR_PRES_MASK \
	((TG3_GPIO_MSG_DRVR_PRES << 0) | \
	 (TG3_GPIO_MSG_DRVR_PRES << 4) | \
	 (TG3_GPIO_MSG_DRVR_PRES << 8) | \
	 (TG3_GPIO_MSG_DRVR_PRES << 12))

#define TG3_GPIO_MSG_ALL_NEED_VAUX_MASK \
	((TG3_GPIO_MSG_NEED_VAUX << 0) | \
	 (TG3_GPIO_MSG_NEED_VAUX << 4) | \
	 (TG3_GPIO_MSG_NEED_VAUX << 8) | \
	 (TG3_GPIO_MSG_NEED_VAUX << 12))

static inline u32 tg3_set_function_status(struct tg3_priv *tp, u32 newstat)
{
	u32 status, shift;

	status = tr32(TG3_CPMU_DRV_STATUS);

	shift = TG3_APE_GPIO_MSG_SHIFT + 4 * tp->pci_fn;
	status &= ~(TG3_GPIO_MSG_MASK << shift);
	status |= (newstat << shift);

	tw32(TG3_CPMU_DRV_STATUS, status);

	return status >> TG3_APE_GPIO_MSG_SHIFT;
}

static inline int tg3_pwrsrc_switch_to_vmain(struct tg3_priv *tp)
{
	if (!tg3_flag(tp, IS_NIC))
		return 0;

	if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
	    tg3_asic_rev(tp) == ASIC_REV_5719 ||
	    tg3_asic_rev(tp) == ASIC_REV_5720) {
		tg3_set_function_status(tp, TG3_GPIO_MSG_DRVR_PRES);

		tw32_wait_f(GRC_LOCAL_CTRL, tp->grc_local_ctrl,
			    TG3_GRC_LCLCTL_PWRSW_DELAY);

	} else {
		tw32_wait_f(GRC_LOCAL_CTRL, tp->grc_local_ctrl,
			    TG3_GRC_LCLCTL_PWRSW_DELAY);
	}

	return 0;
}

static int tg3_5700_link_polarity(struct tg3_priv *tp, u32 speed)
{
	if (tp->led_ctrl == LED_CTRL_MODE_PHY_2) {
		return 1;
	} else if ((tp->phy_id & TG3_PHY_ID_MASK) == TG3_PHY_ID_BCM5411) {
		if (speed != SPEED_10)
			return 1;
	} else if (speed == SPEED_10) {
		return 1;
	}

	return 0;
}

static void __tg3_set_one_mac_addr(struct tg3_priv *tp, u8 *mac_addr, int index)
{
	u32 addr_high, addr_low;

	addr_high = ((mac_addr[0] << 8) | mac_addr[1]);
	addr_low = ((mac_addr[2] << 24) | (mac_addr[3] << 16) |
		    (mac_addr[4] <<  8) | mac_addr[5]);

	if (index < 4) {
		tw32(MAC_ADDR_0_HIGH + (index * 8), addr_high);
		tw32(MAC_ADDR_0_LOW + (index * 8), addr_low);
	} else {
		index -= 4;
		tw32(MAC_EXTADDR_0_HIGH + (index * 8), addr_high);
		tw32(MAC_EXTADDR_0_LOW + (index * 8), addr_low);
	}
}

static void __tg3_set_mac_addr(struct tg3_priv *tp, bool skip_mac_1)
{
	u32 addr_high;
	int i;

	for (i = 0; i < 4; i++) {
		if (i == 1 && skip_mac_1)
			continue;
		__tg3_set_one_mac_addr(tp, tp->mac_address, i);
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5703 ||
	    tg3_asic_rev(tp) == ASIC_REV_5704) {
		for (i = 4; i < 16; i++)
			__tg3_set_one_mac_addr(tp, tp->mac_address, i);
	}
	addr_high = (tp->mac_address[0] + tp->mac_address[1] +
		     tp->mac_address[2] + tp->mac_address[3] +
		     tp->mac_address[4] + tp->mac_address[5]) &
		    TX_BACKOFF_SEED_MASK;
	tw32(MAC_TX_BACKOFF_SEED, addr_high);
}

static void tg3_enable_register_access(struct tg3_priv *tp)
{
	/*
	 * Make sure register accesses (indirect or otherwise) will function
	 * correctly.
	 */
	pci_write_config_dword(tp->devno,
			       TG3PCI_MISC_HOST_CTRL, tp->misc_host_ctrl);
}

static int tg3_power_up(struct tg3_priv *tp)
{
	tg3_enable_register_access(tp);

	/* Switch out of Vaux if it is a NIC */
	tg3_pwrsrc_switch_to_vmain(tp);

	return 0;
}

static int tg3_setup_phy(struct tg3_priv*, bool);

static void tg3_aux_stat_to_speed_duplex(struct tg3_priv *tp, u32 val, u16 *speed,
					 u8 *duplex)
{
	switch (val & MII_TG3_AUX_STAT_SPDMASK) {
	case MII_TG3_AUX_STAT_10HALF:
		*speed = SPEED_10;
		*duplex = DUPLEX_HALF;
		break;

	case MII_TG3_AUX_STAT_10FULL:
		*speed = SPEED_10;
		*duplex = DUPLEX_FULL;
		break;

	case MII_TG3_AUX_STAT_100HALF:
		*speed = SPEED_100;
		*duplex = DUPLEX_HALF;
		break;

	case MII_TG3_AUX_STAT_100FULL:
		*speed = SPEED_100;
		*duplex = DUPLEX_FULL;
		break;

	case MII_TG3_AUX_STAT_1000HALF:
		*speed = SPEED_1000;
		*duplex = DUPLEX_HALF;
		break;

	case MII_TG3_AUX_STAT_1000FULL:
		*speed = SPEED_1000;
		*duplex = DUPLEX_FULL;
		break;

	default:
		if (tp->phy_flags & TG3_PHYFLG_IS_FET) {
			*speed = (val & MII_TG3_AUX_STAT_100) ? SPEED_100 :
				 SPEED_10;
			*duplex = (val & MII_TG3_AUX_STAT_FULL) ? DUPLEX_FULL :
				  DUPLEX_HALF;
			break;
		}
		*speed = SPEED_UNKNOWN;
		*duplex = DUPLEX_UNKNOWN;
		break;
	}
}

static int tg3_phy_autoneg_cfg(struct tg3_priv *tp, u32 advertise)
{
	int err = 0;
	u32  new_adv;

	new_adv = ADVERTISE_CSMA | ADVERTISE_ALL;

	err = tg3_writephy(tp, MII_ADVERTISE, new_adv);
	if (err)
		goto done;

	if (!(tp->phy_flags & TG3_PHYFLG_10_100_ONLY)) {
		new_adv = ADVERTISE_CSMA | ADVERTISE_ALL | ADVERTISE_1000FULL;

		if (tg3_chip_rev_id(tp) == CHIPREV_ID_5701_A0 ||
		    tg3_chip_rev_id(tp) == CHIPREV_ID_5701_B0)
			new_adv |= CTL1000_AS_MASTER | CTL1000_ENABLE_MASTER;

		err = tg3_writephy(tp, MII_CTRL1000, new_adv);
		if (err)
			goto done;
	}

done:
	return err;
}

static void tg3_phy_copper_begin(struct tg3_priv *tp)
{
	if (tp->link_config.autoneg == AUTONEG_ENABLE ||
	    (tp->phy_flags & TG3_PHYFLG_IS_LOW_POWER)) {
		u32 adv;

		if ((tp->phy_flags & TG3_PHYFLG_IS_LOW_POWER) &&
		    !(tp->phy_flags & TG3_PHYFLG_KEEP_LINK_ON_PWRDN)) {
			adv = ADVERTISED_10baseT_Half |
			      ADVERTISED_10baseT_Full;
			if (tg3_flag(tp, WOL_SPEED_100MB))
				adv |= ADVERTISED_100baseT_Half |
				       ADVERTISED_100baseT_Full;
			if (tp->phy_flags & TG3_PHYFLG_1G_ON_VAUX_OK) {
				if (!(tp->phy_flags &
				      TG3_PHYFLG_DISABLE_1G_HD_ADV))
					adv |= ADVERTISED_1000baseT_Half;
				adv |= ADVERTISED_1000baseT_Full;
			}
		} else {
			adv = tp->link_config.advertising;
			if (tp->phy_flags & TG3_PHYFLG_10_100_ONLY)
				adv &= ~(ADVERTISED_1000baseT_Half |
					 ADVERTISED_1000baseT_Full);
		}

		tg3_phy_autoneg_cfg(tp, adv);

		if ((tp->phy_flags & TG3_PHYFLG_IS_LOW_POWER) &&
		    (tp->phy_flags & TG3_PHYFLG_KEEP_LINK_ON_PWRDN)) {
			/* Normally during power down we want to autonegotiate
			 * the lowest possible speed for WOL. However, to avoid
			 * link flap, we leave it untouched.
			 */
			return;
		}

		tg3_writephy(tp, MII_BMCR,
			     BMCR_ANENABLE | BMCR_ANRESTART);
	} else {
		int i;
		u32 bmcr, orig_bmcr;

		tp->link_config.active_speed = tp->link_config.speed;
		tp->link_config.active_duplex = tp->link_config.duplex;

		if (tg3_asic_rev(tp) == ASIC_REV_5714) {
			/* With autoneg disabled, 5715 only links up when the
			 * advertisement register has the configured speed
			 * enabled.
			 */
			tg3_writephy(tp, MII_ADVERTISE, ADVERTISE_ALL);
		}

		bmcr = 0;
		switch (tp->link_config.speed) {
		default:
		case SPEED_10:
			break;

		case SPEED_100:
			bmcr |= BMCR_SPEED100;
			break;

		case SPEED_1000:
			bmcr |= BMCR_SPEED1000;
			break;
		}

		if (tp->link_config.duplex == DUPLEX_FULL)
			bmcr |= BMCR_FULLDPLX;

		if (!tg3_readphy(tp, MII_BMCR, &orig_bmcr) &&
		    (bmcr != orig_bmcr)) {
			tg3_writephy(tp, MII_BMCR, BMCR_LOOPBACK);
			for (i = 0; i < 1500; i++) {
				u32 tmp;

				udelay(10);
				if (tg3_readphy(tp, MII_BMSR, &tmp) ||
				    tg3_readphy(tp, MII_BMSR, &tmp))
					continue;
				if (!(tmp & BMSR_LSTATUS)) {
					udelay(40);
					break;
				}
			}
			tg3_writephy(tp, MII_BMCR, bmcr);
			udelay(40);
		}
	}
}

/* TODO */
/* BEGIN: Borrowed from Linux */

/**
 * ethtool_adv_to_mii_adv_x
 * @ethadv: the ethtool advertisement settings
 *
 * A small helper function that translates ethtool advertisement
 * settings to phy autonegotiation advertisements for the
 * MII_CTRL1000 register when in 1000Base-X mode.
 */
static inline u32 ethtool_adv_to_mii_adv_x(u32 ethadv)
{
	u32 result = 0;

	if (ethadv & ADVERTISED_1000baseT_Half)
		result |= ADVERTISE_1000XHALF;
	if (ethadv & ADVERTISED_1000baseT_Full)
		result |= ADVERTISE_1000XFULL;

	return result;
}

/**
 * mii_adv_to_ethtool_adv_t
 * @adv: value of the MII_ADVERTISE register
 *
 * A small helper function that translates MII_ADVERTISE bits
 * to ethtool advertisement settings.
 */
static inline u32 mii_adv_to_ethtool_adv_t(u32 adv)
{
	u32 result = 0;

	if (adv & ADVERTISE_10HALF)
		result |= ADVERTISED_10baseT_Half;
	if (adv & ADVERTISE_10FULL)
		result |= ADVERTISED_10baseT_Full;
	if (adv & ADVERTISE_100HALF)
		result |= ADVERTISED_100baseT_Half;
	if (adv & ADVERTISE_100FULL)
		result |= ADVERTISED_100baseT_Full;

	return result;
}

/**
 * mii_ctrl1000_to_ethtool_adv_t
 * @adv: value of the MII_CTRL1000 register
 *
 * A small helper function that translates MII_CTRL1000
 * bits, when in 1000Base-T mode, to ethtool
 * advertisement settings.
 */
static inline u32 mii_ctrl1000_to_ethtool_adv_t(u32 adv)
{
	u32 result = 0;

	if (adv & ADVERTISE_1000HALF)
		result |= ADVERTISED_1000baseT_Half;
	if (adv & ADVERTISE_1000FULL)
		result |= ADVERTISED_1000baseT_Full;

	return result;
}
/**
 * mii_adv_to_ethtool_adv_x
 * @adv: value of the MII_CTRL1000 register
 *
 * A small helper function that translates MII_CTRL1000
 * bits, when in 1000Base-X mode, to ethtool
 * advertisement settings.
 */
static inline u32 mii_adv_to_ethtool_adv_x(u32 adv)
{
	u32 result = 0;

	if (adv & ADVERTISE_1000XHALF)
		result |= ADVERTISED_1000baseT_Half;
	if (adv & ADVERTISE_1000XFULL)
		result |= ADVERTISED_1000baseT_Full;
	return result;
}
/* END: Borrowed from Linux */

static int tg3_phy_pull_config(struct tg3_priv *tp)
{
	int err;
	u32 val;

	err = tg3_readphy(tp, MII_BMCR, &val);
	if (err)
		goto done;

	if (!(val & BMCR_ANENABLE)) {
		tp->link_config.autoneg = AUTONEG_DISABLE;
		tp->link_config.advertising = 0;
		tg3_flag_clear(tp, PAUSE_AUTONEG);

		err = -EIO;

		switch (val & (BMCR_SPEED1000 | BMCR_SPEED100)) {
		case 0:
			if (tp->phy_flags & TG3_PHYFLG_ANY_SERDES)
				goto done;

			tp->link_config.speed = SPEED_10;
			break;
		case BMCR_SPEED100:
			if (tp->phy_flags & TG3_PHYFLG_ANY_SERDES)
				goto done;

			tp->link_config.speed = SPEED_100;
			break;
		case BMCR_SPEED1000:
			if (!(tp->phy_flags & TG3_PHYFLG_10_100_ONLY)) {
				tp->link_config.speed = SPEED_1000;
				break;
			}
			/* Fall through */
		default:
			goto done;
		}

		if (val & BMCR_FULLDPLX)
			tp->link_config.duplex = DUPLEX_FULL;
		else
			tp->link_config.duplex = DUPLEX_HALF;

		err = 0;
		goto done;
	}

	tp->link_config.autoneg = AUTONEG_ENABLE;
	tp->link_config.advertising = ADVERTISED_Autoneg;

	if (!(tp->phy_flags & TG3_PHYFLG_ANY_SERDES)) {
		u32 adv;

		err = tg3_readphy(tp, MII_ADVERTISE, &val);
		if (err)
			goto done;

		adv = mii_adv_to_ethtool_adv_t(val & ADVERTISE_ALL);
		tp->link_config.advertising |= adv | ADVERTISED_TP;

	} else {
		tp->link_config.advertising |= ADVERTISED_FIBRE;
	}

	if (!(tp->phy_flags & TG3_PHYFLG_10_100_ONLY)) {
		u32 adv;

		if (!(tp->phy_flags & TG3_PHYFLG_ANY_SERDES)) {
			err = tg3_readphy(tp, MII_CTRL1000, &val);
			if (err)
				goto done;

			adv = mii_ctrl1000_to_ethtool_adv_t(val);
		} else {
			err = tg3_readphy(tp, MII_ADVERTISE, &val);
			if (err)
				goto done;

			adv = 0;

			val &= (ADVERTISE_1000XHALF | ADVERTISE_1000XFULL);
			adv = mii_adv_to_ethtool_adv_x(val);
		}

		tp->link_config.advertising |= adv;
	}

done:
	return err;
}

static int tg3_init_5401phy_dsp(struct tg3_priv *tp)
{
	int err;

	/* Turn off tap power management. */
	/* Set Extended packet length bit */
	err = tg3_phy_auxctl_write(tp, MII_TG3_AUXCTL_SHDWSEL_AUXCTL, 0x4c20);

	err |= tg3_phydsp_write(tp, 0x0012, 0x1804);
	err |= tg3_phydsp_write(tp, 0x0013, 0x1204);
	err |= tg3_phydsp_write(tp, 0x8006, 0x0132);
	err |= tg3_phydsp_write(tp, 0x8006, 0x0232);
	err |= tg3_phydsp_write(tp, 0x201f, 0x0a20);

	udelay(40);

	return err;
}

static bool tg3_phy_copper_an_config_ok(struct tg3_priv *tp, u32 *lcladv)
{
	u32 advmsk, tgtadv;

	/* TODO NOTE: Assuming we always want 10/100/1000 */

	tgtadv = ADVERTISE_ALL;
	advmsk = ADVERTISE_ALL;

	if (tg3_readphy(tp, MII_ADVERTISE, lcladv))
		return false;

	if ((*lcladv & advmsk) != tgtadv)
		return false;

	if (!(tp->phy_flags & TG3_PHYFLG_10_100_ONLY)) {
		u32 tg3_ctrl;

		tgtadv = ADVERTISE_ALL | ADVERTISE_1000FULL;

		if (tg3_readphy(tp, MII_CTRL1000, &tg3_ctrl))
			return false;

		if (tgtadv &&
		    (tg3_chip_rev_id(tp) == CHIPREV_ID_5701_A0 ||
		     tg3_chip_rev_id(tp) == CHIPREV_ID_5701_B0)) {
			tgtadv |= CTL1000_AS_MASTER | CTL1000_ENABLE_MASTER;
			tg3_ctrl &= (ADVERTISE_1000HALF | ADVERTISE_1000FULL |
				     CTL1000_AS_MASTER | CTL1000_ENABLE_MASTER);
		} else {
			tg3_ctrl &= (ADVERTISE_1000HALF | ADVERTISE_1000FULL);
		}

		if (tg3_ctrl != tgtadv)
			return false;
	}

	return true;
}

static bool tg3_phy_copper_fetch_rmtadv(struct tg3_priv *tp, u32 *rmtadv)
{
	if (!(tp->phy_flags & TG3_PHYFLG_10_100_ONLY)) {
		u32 val;

		if (tg3_readphy(tp, MII_STAT1000, &val))
			return false;
	}

	if (tg3_readphy(tp, MII_LPA, rmtadv))
		return false;

	return true;
}

static void tg3_clear_mac_status(struct tg3_priv *tp)
{
	tw32(MAC_EVENT, 0);

	tw32_f(MAC_STATUS,
	       MAC_STATUS_SYNC_CHANGED |
	       MAC_STATUS_CFG_CHANGED |
	       MAC_STATUS_MI_COMPLETION |
	       MAC_STATUS_LNKSTATE_CHANGED);
	udelay(40);
}

static int tg3_setup_copper_phy(struct tg3_priv *tp, bool force_reset)
{
	bool current_link_up;
	u32 bmsr, val;
	u32 lcl_adv, rmt_adv;
	u16 current_speed;
	u8 current_duplex;
	int i, err;

	tg3_clear_mac_status(tp);

	if ((tp->mi_mode & MAC_MI_MODE_AUTO_POLL) != 0) {
		tw32_f(MAC_MI_MODE,
		     (tp->mi_mode & ~MAC_MI_MODE_AUTO_POLL));
		udelay(80);
	}

	tg3_phy_auxctl_write(tp, MII_TG3_AUXCTL_SHDWSEL_PWRCTL, 0);

	/* Some third-party PHYs need to be reset on link going
	 * down.
	 */
	if (force_reset)
		tg3_phy_reset(tp);

	if ((tp->phy_id & TG3_PHY_ID_MASK) == TG3_PHY_ID_BCM5401) {
		tg3_readphy(tp, MII_BMSR, &bmsr);
		if (tg3_readphy(tp, MII_BMSR, &bmsr) ||
		    !tg3_flag(tp, INIT_COMPLETE))
			bmsr = 0;

		if (!(bmsr & BMSR_LSTATUS)) {
			err = tg3_init_5401phy_dsp(tp);
			if (err)
				return err;

			tg3_readphy(tp, MII_BMSR, &bmsr);
			for (i = 0; i < 1000; i++) {
				udelay(10);
				if (!tg3_readphy(tp, MII_BMSR, &bmsr) &&
				    (bmsr & BMSR_LSTATUS)) {
					udelay(40);
					break;
				}
			}

			if ((tp->phy_id & TG3_PHY_ID_REV_MASK) ==
			    TG3_PHY_REV_BCM5401_B0 &&
			    !(bmsr & BMSR_LSTATUS) &&
			    tp->link_config.active_speed == SPEED_1000) {
				err = tg3_phy_reset(tp);
				if (!err)
					err = tg3_init_5401phy_dsp(tp);
				if (err)
					return err;
			}
		}
	} else if (tg3_chip_rev_id(tp) == CHIPREV_ID_5701_A0 ||
		   tg3_chip_rev_id(tp) == CHIPREV_ID_5701_B0) {
		/* 5701 {A0,B0} CRC bug workaround */
		tg3_writephy(tp, 0x15, 0x0a75);
		tg3_writephy(tp, MII_TG3_MISC_SHDW, 0x8c68);
		tg3_writephy(tp, MII_TG3_MISC_SHDW, 0x8d68);
		tg3_writephy(tp, MII_TG3_MISC_SHDW, 0x8c68);
	}

	/* Clear pending interrupts... */
	tg3_readphy(tp, MII_TG3_ISTAT, &val);
	tg3_readphy(tp, MII_TG3_ISTAT, &val);

	if (tp->phy_flags & TG3_PHYFLG_USE_MI_INTERRUPT)
		tg3_writephy(tp, MII_TG3_IMASK, ~MII_TG3_INT_LINKCHG);
	else if (!(tp->phy_flags & TG3_PHYFLG_IS_FET))
		tg3_writephy(tp, MII_TG3_IMASK, ~0);

	if (tg3_asic_rev(tp) == ASIC_REV_5700 ||
	    tg3_asic_rev(tp) == ASIC_REV_5701) {
		if (tp->led_ctrl == LED_CTRL_MODE_PHY_1)
			tg3_writephy(tp, MII_TG3_EXT_CTRL,
				     MII_TG3_EXT_CTRL_LNK3_LED_MODE);
		else
			tg3_writephy(tp, MII_TG3_EXT_CTRL, 0);
	}

	current_link_up = false;
	current_speed = SPEED_UNKNOWN;
	current_duplex = DUPLEX_UNKNOWN;
	tp->phy_flags &= ~TG3_PHYFLG_MDIX_STATE;

	if (tp->phy_flags & TG3_PHYFLG_CAPACITIVE_COUPLING) {
		err = tg3_phy_auxctl_read(tp,
					  MII_TG3_AUXCTL_SHDWSEL_MISCTEST,
					  &val);
		if (!err && !(val & (1 << 10))) {
			tg3_phy_auxctl_write(tp,
					     MII_TG3_AUXCTL_SHDWSEL_MISCTEST,
					     val | (1 << 10));
			goto relink;
		}
	}

	bmsr = 0;
	for (i = 0; i < 100; i++) {
		tg3_readphy(tp, MII_BMSR, &bmsr);
		if (!tg3_readphy(tp, MII_BMSR, &bmsr) &&
		    (bmsr & BMSR_LSTATUS))
			break;
		udelay(40);
	}

	if (bmsr & BMSR_LSTATUS) {
		u32 aux_stat, bmcr;

		tg3_readphy(tp, MII_TG3_AUX_STAT, &aux_stat);
		for (i = 0; i < 2000; i++) {
			udelay(10);
			if (!tg3_readphy(tp, MII_TG3_AUX_STAT, &aux_stat) &&
			    aux_stat)
				break;
		}

		tg3_aux_stat_to_speed_duplex(tp, aux_stat,
					     &current_speed,
					     &current_duplex);

		bmcr = 0;
		for (i = 0; i < 200; i++) {
			tg3_readphy(tp, MII_BMCR, &bmcr);
			if (tg3_readphy(tp, MII_BMCR, &bmcr))
				continue;
			if (bmcr && bmcr != 0x7fff)
				break;
			udelay(10);
		}

		lcl_adv = 0;
		rmt_adv = 0;

		tp->link_config.active_speed = current_speed;
		tp->link_config.active_duplex = current_duplex;

		if (tp->link_config.autoneg == AUTONEG_ENABLE) {
			if ((bmcr & BMCR_ANENABLE) &&
			    tg3_phy_copper_an_config_ok(tp, &lcl_adv) &&
			    tg3_phy_copper_fetch_rmtadv(tp, &rmt_adv))
				current_link_up = true;

		} else {
			if (!(bmcr & BMCR_ANENABLE) &&
			    tp->link_config.speed == current_speed &&
			    tp->link_config.duplex == current_duplex) {
				current_link_up = true;
			}
		}

		if (current_link_up &&
		    tp->link_config.active_duplex == DUPLEX_FULL) {
			u32 reg, bit;

			if (tp->phy_flags & TG3_PHYFLG_IS_FET) {
				reg = MII_TG3_FET_GEN_STAT;
				bit = MII_TG3_FET_GEN_STAT_MDIXSTAT;
			} else {
				reg = MII_TG3_EXT_STAT;
				bit = MII_TG3_EXT_STAT_MDIX;
			}

			if (!tg3_readphy(tp, reg, &val) && (val & bit))
				tp->phy_flags |= TG3_PHYFLG_MDIX_STATE;
		}
	}

relink:
	if (!current_link_up || (tp->phy_flags & TG3_PHYFLG_IS_LOW_POWER)) {
		tg3_phy_copper_begin(tp);

		if (tg3_flag(tp, ROBOSWITCH)) {
			current_link_up = true;
			/* FIXME: when BCM5325 switch is used use 100 MBit/s */
			current_speed = SPEED_1000;
			current_duplex = DUPLEX_FULL;
			tp->link_config.active_speed = current_speed;
			tp->link_config.active_duplex = current_duplex;
		}

		tg3_readphy(tp, MII_BMSR, &bmsr);
		if ((!tg3_readphy(tp, MII_BMSR, &bmsr) && (bmsr & BMSR_LSTATUS)) ||
		    (tp->mac_mode & MAC_MODE_PORT_INT_LPBACK))
			current_link_up = true;
	}

	tp->mac_mode &= ~MAC_MODE_PORT_MODE_MASK;
	if (current_link_up) {
		if (tp->link_config.active_speed == SPEED_100 ||
		    tp->link_config.active_speed == SPEED_10)
			tp->mac_mode |= MAC_MODE_PORT_MODE_MII;
		else
			tp->mac_mode |= MAC_MODE_PORT_MODE_GMII;
	} else if (tp->phy_flags & TG3_PHYFLG_IS_FET) {
		tp->mac_mode |= MAC_MODE_PORT_MODE_MII;
	} else {
		tp->mac_mode |= MAC_MODE_PORT_MODE_GMII;
	}

	/* In order for the 5750 core in BCM4785 chip to work properly
	 * in RGMII mode, the Led Control Register must be set up.
	 */
	if (tg3_flag(tp, RGMII_MODE)) {
		u32 led_ctrl = tr32(MAC_LED_CTRL);
		led_ctrl &= ~(LED_CTRL_1000MBPS_ON | LED_CTRL_100MBPS_ON);

		if (tp->link_config.active_speed == SPEED_10)
			led_ctrl |= LED_CTRL_LNKLED_OVERRIDE;
		else if (tp->link_config.active_speed == SPEED_100)
			led_ctrl |= (LED_CTRL_LNKLED_OVERRIDE |
				     LED_CTRL_100MBPS_ON);
		else if (tp->link_config.active_speed == SPEED_1000)
			led_ctrl |= (LED_CTRL_LNKLED_OVERRIDE |
				     LED_CTRL_1000MBPS_ON);

		tw32(MAC_LED_CTRL, led_ctrl);
		udelay(40);
	}

	tp->mac_mode &= ~MAC_MODE_HALF_DUPLEX;
	if (tp->link_config.active_duplex == DUPLEX_HALF)
		tp->mac_mode |= MAC_MODE_HALF_DUPLEX;

	if (tg3_asic_rev(tp) == ASIC_REV_5700) {
		if (current_link_up &&
		    tg3_5700_link_polarity(tp, tp->link_config.active_speed))
			tp->mac_mode |= MAC_MODE_LINK_POLARITY;
		else
			tp->mac_mode &= ~MAC_MODE_LINK_POLARITY;
	}

	/* ??? Without this setting Netgear GA302T PHY does not
	 * ??? send/receive packets...
	 */
	if ((tp->phy_id & TG3_PHY_ID_MASK) == TG3_PHY_ID_BCM5411 &&
	    tg3_chip_rev_id(tp) == CHIPREV_ID_5700_ALTIMA) {
		tp->mi_mode |= MAC_MI_MODE_AUTO_POLL;
		tw32_f(MAC_MI_MODE, tp->mi_mode);
		udelay(80);
	}

	tw32_f(MAC_MODE, tp->mac_mode);
	udelay(40);

	if (tg3_flag(tp, USE_LINKCHG_REG)) {
		/* Polled via timer. */
		tw32_f(MAC_EVENT, 0);
	} else {
		tw32_f(MAC_EVENT, MAC_EVENT_LNKSTATE_CHANGED);
	}
	udelay(40);

	if (tg3_asic_rev(tp) == ASIC_REV_5700 &&
	    current_link_up &&
	    tp->link_config.active_speed == SPEED_1000 &&
	    (tg3_flag(tp, PCI_HIGH_SPEED))) {
		udelay(120);
		tw32_f(MAC_STATUS,
		     (MAC_STATUS_SYNC_CHANGED |
		      MAC_STATUS_CFG_CHANGED));
		udelay(40);
		tg3_write_mem(tp,
			      NIC_SRAM_FIRMWARE_MBOX,
			      NIC_SRAM_FIRMWARE_MBOX_MAGIC2);
	}

	return 0;
}

struct tg3_fiber_aneginfo {
	int state;
#define ANEG_STATE_UNKNOWN		0
#define ANEG_STATE_AN_ENABLE		1
#define ANEG_STATE_RESTART_INIT		2
#define ANEG_STATE_RESTART		3
#define ANEG_STATE_DISABLE_LINK_OK	4
#define ANEG_STATE_ABILITY_DETECT_INIT	5
#define ANEG_STATE_ABILITY_DETECT	6
#define ANEG_STATE_ACK_DETECT_INIT	7
#define ANEG_STATE_ACK_DETECT		8
#define ANEG_STATE_COMPLETE_ACK_INIT	9
#define ANEG_STATE_COMPLETE_ACK		10
#define ANEG_STATE_IDLE_DETECT_INIT	11
#define ANEG_STATE_IDLE_DETECT		12
#define ANEG_STATE_LINK_OK		13
#define ANEG_STATE_NEXT_PAGE_WAIT_INIT	14
#define ANEG_STATE_NEXT_PAGE_WAIT	15

	u32 flags;
#define MR_AN_ENABLE		0x00000001
#define MR_RESTART_AN		0x00000002
#define MR_AN_COMPLETE		0x00000004
#define MR_PAGE_RX		0x00000008
#define MR_NP_LOADED		0x00000010
#define MR_TOGGLE_TX		0x00000020
#define MR_LP_ADV_FULL_DUPLEX	0x00000040
#define MR_LP_ADV_HALF_DUPLEX	0x00000080
#define MR_LP_ADV_REMOTE_FAULT1	0x00000400
#define MR_LP_ADV_REMOTE_FAULT2	0x00000800
#define MR_LP_ADV_NEXT_PAGE	0x00001000
#define MR_TOGGLE_RX		0x00002000
#define MR_NP_RX		0x00004000

#define MR_LINK_OK		0x80000000

	unsigned long link_time, cur_time;

	u32 ability_match_cfg;
	int ability_match_count;

	char ability_match, idle_match, ack_match;

	u32 txconfig, rxconfig;
#define ANEG_CFG_NP		0x00000080
#define ANEG_CFG_ACK		0x00000040
#define ANEG_CFG_RF2		0x00000020
#define ANEG_CFG_RF1		0x00000010
#define ANEG_CFG_PS2		0x00000001
#define ANEG_CFG_PS1		0x00008000
#define ANEG_CFG_HD		0x00004000
#define ANEG_CFG_FD		0x00002000
#define ANEG_CFG_INVAL		0x00001f06

};
#define ANEG_OK		0
#define ANEG_DONE	1
#define ANEG_TIMER_ENAB	2
#define ANEG_FAILED	-1

#define ANEG_STATE_SETTLE_TIME	10000

static int tg3_fiber_aneg_smachine(struct tg3_priv *tp,
				   struct tg3_fiber_aneginfo *ap)
{
	unsigned long delta;
	u32 rx_cfg_reg;
	int ret;

	if (ap->state == ANEG_STATE_UNKNOWN) {
		ap->rxconfig = 0;
		ap->link_time = 0;
		ap->cur_time = 0;
		ap->ability_match_cfg = 0;
		ap->ability_match_count = 0;
		ap->ability_match = 0;
		ap->idle_match = 0;
		ap->ack_match = 0;
	}
	ap->cur_time++;

	if (tr32(MAC_STATUS) & MAC_STATUS_RCVD_CFG) {
		rx_cfg_reg = tr32(MAC_RX_AUTO_NEG);

		if (rx_cfg_reg != ap->ability_match_cfg) {
			ap->ability_match_cfg = rx_cfg_reg;
			ap->ability_match = 0;
			ap->ability_match_count = 0;
		} else {
			if (++ap->ability_match_count > 1) {
				ap->ability_match = 1;
				ap->ability_match_cfg = rx_cfg_reg;
			}
		}
		if (rx_cfg_reg & ANEG_CFG_ACK)
			ap->ack_match = 1;
		else
			ap->ack_match = 0;

		ap->idle_match = 0;
	} else {
		ap->idle_match = 1;
		ap->ability_match_cfg = 0;
		ap->ability_match_count = 0;
		ap->ability_match = 0;
		ap->ack_match = 0;

		rx_cfg_reg = 0;
	}

	ap->rxconfig = rx_cfg_reg;
	ret = ANEG_OK;

	switch (ap->state) {
	case ANEG_STATE_UNKNOWN:
		if (ap->flags & (MR_AN_ENABLE | MR_RESTART_AN))
			ap->state = ANEG_STATE_AN_ENABLE;

		/* fallthru */
	case ANEG_STATE_AN_ENABLE:
		ap->flags &= ~(MR_AN_COMPLETE | MR_PAGE_RX);
		if (ap->flags & MR_AN_ENABLE) {
			ap->link_time = 0;
			ap->cur_time = 0;
			ap->ability_match_cfg = 0;
			ap->ability_match_count = 0;
			ap->ability_match = 0;
			ap->idle_match = 0;
			ap->ack_match = 0;

			ap->state = ANEG_STATE_RESTART_INIT;
		} else {
			ap->state = ANEG_STATE_DISABLE_LINK_OK;
		}
		break;

	case ANEG_STATE_RESTART_INIT:
		ap->link_time = ap->cur_time;
		ap->flags &= ~(MR_NP_LOADED);
		ap->txconfig = 0;
		tw32(MAC_TX_AUTO_NEG, 0);
		tp->mac_mode |= MAC_MODE_SEND_CONFIGS;
		tw32_f(MAC_MODE, tp->mac_mode);
		udelay(40);

		ret = ANEG_TIMER_ENAB;
		ap->state = ANEG_STATE_RESTART;

	/* fallthru */
	case ANEG_STATE_RESTART:
		delta = ap->cur_time - ap->link_time;
		if (delta > ANEG_STATE_SETTLE_TIME)
			ap->state = ANEG_STATE_ABILITY_DETECT_INIT;
		else
			ret = ANEG_TIMER_ENAB;
		break;

	case ANEG_STATE_DISABLE_LINK_OK:
		ret = ANEG_DONE;
		break;

	case ANEG_STATE_ABILITY_DETECT_INIT:
		ap->flags &= ~(MR_TOGGLE_TX);
		ap->txconfig = ANEG_CFG_FD;
		tw32(MAC_TX_AUTO_NEG, ap->txconfig);
		tp->mac_mode |= MAC_MODE_SEND_CONFIGS;
		tw32_f(MAC_MODE, tp->mac_mode);
		udelay(40);

		ap->state = ANEG_STATE_ABILITY_DETECT;
		break;

	case ANEG_STATE_ABILITY_DETECT:
		if (ap->ability_match != 0 && ap->rxconfig != 0)
			ap->state = ANEG_STATE_ACK_DETECT_INIT;
		break;

	case ANEG_STATE_ACK_DETECT_INIT:
		ap->txconfig |= ANEG_CFG_ACK;
		tw32(MAC_TX_AUTO_NEG, ap->txconfig);
		tp->mac_mode |= MAC_MODE_SEND_CONFIGS;
		tw32_f(MAC_MODE, tp->mac_mode);
		udelay(40);

		ap->state = ANEG_STATE_ACK_DETECT;

		/* fallthru */
	case ANEG_STATE_ACK_DETECT:
		if (ap->ack_match != 0) {
			if ((ap->rxconfig & ~ANEG_CFG_ACK) ==
			    (ap->ability_match_cfg & ~ANEG_CFG_ACK)) {
				ap->state = ANEG_STATE_COMPLETE_ACK_INIT;
			} else {
				ap->state = ANEG_STATE_AN_ENABLE;
			}
		} else if (ap->ability_match != 0 &&
			   ap->rxconfig == 0) {
			ap->state = ANEG_STATE_AN_ENABLE;
		}
		break;

	case ANEG_STATE_COMPLETE_ACK_INIT:
		if (ap->rxconfig & ANEG_CFG_INVAL) {
			ret = ANEG_FAILED;
			break;
		}
		ap->flags &= ~(MR_LP_ADV_FULL_DUPLEX |
			       MR_LP_ADV_HALF_DUPLEX |
			       MR_LP_ADV_REMOTE_FAULT1 |
			       MR_LP_ADV_REMOTE_FAULT2 |
			       MR_LP_ADV_NEXT_PAGE |
			       MR_TOGGLE_RX |
			       MR_NP_RX);
		if (ap->rxconfig & ANEG_CFG_FD)
			ap->flags |= MR_LP_ADV_FULL_DUPLEX;
		if (ap->rxconfig & ANEG_CFG_HD)
			ap->flags |= MR_LP_ADV_HALF_DUPLEX;
		if (ap->rxconfig & ANEG_CFG_RF1)
			ap->flags |= MR_LP_ADV_REMOTE_FAULT1;
		if (ap->rxconfig & ANEG_CFG_RF2)
			ap->flags |= MR_LP_ADV_REMOTE_FAULT2;
		if (ap->rxconfig & ANEG_CFG_NP)
			ap->flags |= MR_LP_ADV_NEXT_PAGE;

		ap->link_time = ap->cur_time;

		ap->flags ^= (MR_TOGGLE_TX);
		if (ap->rxconfig & 0x0008)
			ap->flags |= MR_TOGGLE_RX;
		if (ap->rxconfig & ANEG_CFG_NP)
			ap->flags |= MR_NP_RX;
		ap->flags |= MR_PAGE_RX;

		ap->state = ANEG_STATE_COMPLETE_ACK;
		ret = ANEG_TIMER_ENAB;
		break;

	case ANEG_STATE_COMPLETE_ACK:
		if (ap->ability_match != 0 &&
		    ap->rxconfig == 0) {
			ap->state = ANEG_STATE_AN_ENABLE;
			break;
		}
		delta = ap->cur_time - ap->link_time;
		if (delta > ANEG_STATE_SETTLE_TIME) {
			if (!(ap->flags & (MR_LP_ADV_NEXT_PAGE))) {
				ap->state = ANEG_STATE_IDLE_DETECT_INIT;
			} else {
				if ((ap->txconfig & ANEG_CFG_NP) == 0 &&
				    !(ap->flags & MR_NP_RX)) {
					ap->state = ANEG_STATE_IDLE_DETECT_INIT;
				} else {
					ret = ANEG_FAILED;
				}
			}
		}
		break;

	case ANEG_STATE_IDLE_DETECT_INIT:
		ap->link_time = ap->cur_time;
		tp->mac_mode &= ~MAC_MODE_SEND_CONFIGS;
		tw32_f(MAC_MODE, tp->mac_mode);
		udelay(40);

		ap->state = ANEG_STATE_IDLE_DETECT;
		ret = ANEG_TIMER_ENAB;
		break;

	case ANEG_STATE_IDLE_DETECT:
		if (ap->ability_match != 0 &&
		    ap->rxconfig == 0) {
			ap->state = ANEG_STATE_AN_ENABLE;
			break;
		}
		delta = ap->cur_time - ap->link_time;
		if (delta > ANEG_STATE_SETTLE_TIME) {
			/* XXX another gem from the Broadcom driver :( */
			ap->state = ANEG_STATE_LINK_OK;
		}
		break;

	case ANEG_STATE_LINK_OK:
		ap->flags |= (MR_AN_COMPLETE | MR_LINK_OK);
		ret = ANEG_DONE;
		break;

	case ANEG_STATE_NEXT_PAGE_WAIT_INIT:
		/* ??? unimplemented */
		break;

	case ANEG_STATE_NEXT_PAGE_WAIT:
		/* ??? unimplemented */
		break;

	default:
		ret = ANEG_FAILED;
		break;
	}

	return ret;
}

static int fiber_autoneg(struct tg3_priv *tp, u32 *txflags, u32 *rxflags)
{
	int res = 0;
	struct tg3_fiber_aneginfo aninfo;
	int status = ANEG_FAILED;
	unsigned int tick;
	u32 tmp;

	tw32_f(MAC_TX_AUTO_NEG, 0);

	tmp = tp->mac_mode & ~MAC_MODE_PORT_MODE_MASK;
	tw32_f(MAC_MODE, tmp | MAC_MODE_PORT_MODE_GMII);
	udelay(40);

	tw32_f(MAC_MODE, tp->mac_mode | MAC_MODE_SEND_CONFIGS);
	udelay(40);

	memset(&aninfo, 0, sizeof(aninfo));
	aninfo.flags |= MR_AN_ENABLE;
	aninfo.state = ANEG_STATE_UNKNOWN;
	aninfo.cur_time = 0;
	tick = 0;
	while (++tick < 195000) {
		status = tg3_fiber_aneg_smachine(tp, &aninfo);
		if (status == ANEG_DONE || status == ANEG_FAILED)
			break;

		udelay(1);
	}

	tp->mac_mode &= ~MAC_MODE_SEND_CONFIGS;
	tw32_f(MAC_MODE, tp->mac_mode);
	udelay(40);

	*txflags = aninfo.txconfig;
	*rxflags = aninfo.flags;

	if (status == ANEG_DONE &&
	    (aninfo.flags & (MR_AN_COMPLETE | MR_LINK_OK |
			     MR_LP_ADV_FULL_DUPLEX)))
		res = 1;

	return res;
}

static void tg3_init_bcm8002(struct tg3_priv *tp)
{
	u32 mac_status = tr32(MAC_STATUS);
	int i;

	/* Reset when initting first time or we have a link. */
	if (tg3_flag(tp, INIT_COMPLETE) &&
	    !(mac_status & MAC_STATUS_PCS_SYNCED))
		return;

	/* Set PLL lock range. */
	tg3_writephy(tp, 0x16, 0x8007);

	/* SW reset */
	tg3_writephy(tp, MII_BMCR, BMCR_RESET);

	/* Wait for reset to complete. */
	/* XXX schedule_timeout() ... */
	for (i = 0; i < 500; i++)
		udelay(10);

	/* Config mode; select PMA/Ch 1 regs. */
	tg3_writephy(tp, 0x10, 0x8411);

	/* Enable auto-lock and comdet, select txclk for tx. */
	tg3_writephy(tp, 0x11, 0x0a10);

	tg3_writephy(tp, 0x18, 0x00a0);
	tg3_writephy(tp, 0x16, 0x41ff);

	/* Assert and deassert POR. */
	tg3_writephy(tp, 0x13, 0x0400);
	udelay(40);
	tg3_writephy(tp, 0x13, 0x0000);

	tg3_writephy(tp, 0x11, 0x0a50);
	udelay(40);
	tg3_writephy(tp, 0x11, 0x0a10);

	/* Wait for signal to stabilize */
	/* XXX schedule_timeout() ... */
	for (i = 0; i < 15000; i++)
		udelay(10);

	/* Deselect the channel register so we can read the PHYID
	 * later.
	 */
	tg3_writephy(tp, 0x10, 0x8011);
}

static bool tg3_setup_fiber_hw_autoneg(struct tg3_priv *tp, u32 mac_status)
{
	bool current_link_up;
	u32 sg_dig_ctrl, sg_dig_status;
	u32 serdes_cfg, expected_sg_dig_ctrl;
	int workaround, port_a;

	serdes_cfg = 0;
	expected_sg_dig_ctrl = 0;
	workaround = 0;
	port_a = 1;
	current_link_up = false;

	if (tg3_chip_rev_id(tp) != CHIPREV_ID_5704_A0 &&
	    tg3_chip_rev_id(tp) != CHIPREV_ID_5704_A1) {
		workaround = 1;
		if (tr32(TG3PCI_DUAL_MAC_CTRL) & DUAL_MAC_CTRL_ID)
			port_a = 0;

		/* preserve bits 0-11,13,14 for signal pre-emphasis */
		/* preserve bits 20-23 for voltage regulator */
		serdes_cfg = tr32(MAC_SERDES_CFG) & 0x00f06fff;
	}

	sg_dig_ctrl = tr32(SG_DIG_CTRL);

	if (tp->link_config.autoneg != AUTONEG_ENABLE) {
		if (sg_dig_ctrl & SG_DIG_USING_HW_AUTONEG) {
			if (workaround) {
				u32 val = serdes_cfg;

				if (port_a)
					val |= 0xc010000;
				else
					val |= 0x4010000;
				tw32_f(MAC_SERDES_CFG, val);
			}

			tw32_f(SG_DIG_CTRL, SG_DIG_COMMON_SETUP);
		}
		if (mac_status & MAC_STATUS_PCS_SYNCED) {
			current_link_up = true;
		}
		goto out;
	}

	/* Want auto-negotiation.  */
	expected_sg_dig_ctrl = SG_DIG_USING_HW_AUTONEG | SG_DIG_COMMON_SETUP;

	if (sg_dig_ctrl != expected_sg_dig_ctrl) {
		if ((tp->phy_flags & TG3_PHYFLG_PARALLEL_DETECT) &&
		    tp->serdes_counter &&
		    ((mac_status & (MAC_STATUS_PCS_SYNCED |
				    MAC_STATUS_RCVD_CFG)) ==
		     MAC_STATUS_PCS_SYNCED)) {
			tp->serdes_counter--;
			current_link_up = true;
			goto out;
		}
restart_autoneg:
		if (workaround)
			tw32_f(MAC_SERDES_CFG, serdes_cfg | 0xc011000);
		tw32_f(SG_DIG_CTRL, expected_sg_dig_ctrl | SG_DIG_SOFT_RESET);
		udelay(5);
		tw32_f(SG_DIG_CTRL, expected_sg_dig_ctrl);

		tp->serdes_counter = SERDES_AN_TIMEOUT_5704S;
		tp->phy_flags &= ~TG3_PHYFLG_PARALLEL_DETECT;
	} else if (mac_status & (MAC_STATUS_PCS_SYNCED |
				 MAC_STATUS_SIGNAL_DET)) {
		sg_dig_status = tr32(SG_DIG_STATUS);
		mac_status = tr32(MAC_STATUS);

		if ((sg_dig_status & SG_DIG_AUTONEG_COMPLETE) &&
		    (mac_status & MAC_STATUS_PCS_SYNCED)) {

			current_link_up = true;
			tp->serdes_counter = 0;
			tp->phy_flags &= ~TG3_PHYFLG_PARALLEL_DETECT;
		} else if (!(sg_dig_status & SG_DIG_AUTONEG_COMPLETE)) {
			if (tp->serdes_counter)
				tp->serdes_counter--;
			else {
				if (workaround) {
					u32 val = serdes_cfg;

					if (port_a)
						val |= 0xc010000;
					else
						val |= 0x4010000;

					tw32_f(MAC_SERDES_CFG, val);
				}

				tw32_f(SG_DIG_CTRL, SG_DIG_COMMON_SETUP);
				udelay(40);

				/* Link parallel detection - link is up */
				/* only if we have PCS_SYNC and not */
				/* receiving config code words */
				mac_status = tr32(MAC_STATUS);
				if ((mac_status & MAC_STATUS_PCS_SYNCED) &&
				    !(mac_status & MAC_STATUS_RCVD_CFG)) {
					current_link_up = true;
					tp->phy_flags |=
						TG3_PHYFLG_PARALLEL_DETECT;
					tp->serdes_counter =
						SERDES_PARALLEL_DET_TIMEOUT;
				} else
					goto restart_autoneg;
			}
		}
	} else {
		tp->serdes_counter = SERDES_AN_TIMEOUT_5704S;
		tp->phy_flags &= ~TG3_PHYFLG_PARALLEL_DETECT;
	}

out:
	return current_link_up;
}

static bool tg3_setup_fiber_by_hand(struct tg3_priv *tp, u32 mac_status)
{
	bool current_link_up = false;

	if (!(mac_status & MAC_STATUS_PCS_SYNCED))
		goto out;

	if (tp->link_config.autoneg == AUTONEG_ENABLE) {
		u32 txflags, rxflags;
		int i;

		if (fiber_autoneg(tp, &txflags, &rxflags)) {

			current_link_up = true;
		}
		for (i = 0; i < 30; i++) {
			udelay(20);
			tw32_f(MAC_STATUS,
			       (MAC_STATUS_SYNC_CHANGED |
				MAC_STATUS_CFG_CHANGED));
			udelay(40);
			if ((tr32(MAC_STATUS) &
			     (MAC_STATUS_SYNC_CHANGED |
			      MAC_STATUS_CFG_CHANGED)) == 0)
				break;
		}

		mac_status = tr32(MAC_STATUS);
		if (!current_link_up &&
		    (mac_status & MAC_STATUS_PCS_SYNCED) &&
		    !(mac_status & MAC_STATUS_RCVD_CFG))
			current_link_up = true;
	} else {
		/* Forcing 1000FD link up. */
		current_link_up = true;

		tw32_f(MAC_MODE, (tp->mac_mode | MAC_MODE_SEND_CONFIGS));
		udelay(40);

		tw32_f(MAC_MODE, tp->mac_mode);
		udelay(40);
	}

out:
	return current_link_up;
}

static int tg3_setup_fiber_phy(struct tg3_priv *tp, bool force_reset)
{
	u32 mac_status;
	bool current_link_up;
	int i;

	tw32_f(MAC_TX_AUTO_NEG, 0);

	tp->mac_mode &= ~(MAC_MODE_PORT_MODE_MASK | MAC_MODE_HALF_DUPLEX);
	tp->mac_mode |= MAC_MODE_PORT_MODE_TBI;
	tw32_f(MAC_MODE, tp->mac_mode);
	udelay(40);

	if (tp->phy_id == TG3_PHY_ID_BCM8002)
		tg3_init_bcm8002(tp);

	/* Enable link change event even when serdes polling.  */
	tw32_f(MAC_EVENT, MAC_EVENT_LNKSTATE_CHANGED);
	udelay(40);

	current_link_up = false;
	mac_status = tr32(MAC_STATUS);

	if (tg3_flag(tp, HW_AUTONEG))
		current_link_up = tg3_setup_fiber_hw_autoneg(tp, mac_status);
	else
		current_link_up = tg3_setup_fiber_by_hand(tp, mac_status);

	tp->napi[0].hw_status->status =
		(SD_STATUS_UPDATED |
		 (tp->napi[0].hw_status->status & ~SD_STATUS_LINK_CHG));

	for (i = 0; i < 100; i++) {
		tw32_f(MAC_STATUS, (MAC_STATUS_SYNC_CHANGED |
				    MAC_STATUS_CFG_CHANGED));
		udelay(5);
		if ((tr32(MAC_STATUS) & (MAC_STATUS_SYNC_CHANGED |
					 MAC_STATUS_CFG_CHANGED |
					 MAC_STATUS_LNKSTATE_CHANGED)) == 0)
			break;
	}

	mac_status = tr32(MAC_STATUS);
	if ((mac_status & MAC_STATUS_PCS_SYNCED) == 0) {
		current_link_up = false;
		if (tp->link_config.autoneg == AUTONEG_ENABLE &&
		    tp->serdes_counter == 0) {
			tw32_f(MAC_MODE, (tp->mac_mode |
					  MAC_MODE_SEND_CONFIGS));
			udelay(1);
			tw32_f(MAC_MODE, tp->mac_mode);
		}
	}

	if (current_link_up) {
		tp->link_config.active_speed = SPEED_1000;
		tp->link_config.active_duplex = DUPLEX_FULL;
		tw32(MAC_LED_CTRL, (tp->led_ctrl |
				    LED_CTRL_LNKLED_OVERRIDE |
				    LED_CTRL_1000MBPS_ON));
	} else {
		tp->link_config.active_speed = SPEED_UNKNOWN;
		tp->link_config.active_duplex = DUPLEX_UNKNOWN;
		tw32(MAC_LED_CTRL, (tp->led_ctrl |
				    LED_CTRL_LNKLED_OVERRIDE |
				    LED_CTRL_TRAFFIC_OVERRIDE));
	}

	return 0;
}

static int tg3_setup_fiber_mii_phy(struct tg3_priv *tp, bool force_reset)
{
	int err = 0;
	u32 bmsr, bmcr;
	u16 current_speed = SPEED_UNKNOWN;
	u8 current_duplex = DUPLEX_UNKNOWN;
	__maybe_unused bool current_link_up = false;
	u32 local_adv, remote_adv, sgsr;

	if ((tg3_asic_rev(tp) == ASIC_REV_5719 ||
	     tg3_asic_rev(tp) == ASIC_REV_5720) &&
	     !tg3_readphy(tp, SERDES_TG3_1000X_STATUS, &sgsr) &&
	     (sgsr & SERDES_TG3_SGMII_MODE)) {

		if (force_reset)
			tg3_phy_reset(tp);

		tp->mac_mode &= ~MAC_MODE_PORT_MODE_MASK;

		if (!(sgsr & SERDES_TG3_LINK_UP)) {
			tp->mac_mode |= MAC_MODE_PORT_MODE_GMII;
		} else {
			current_link_up = true;
			if (sgsr & SERDES_TG3_SPEED_1000) {
				current_speed = SPEED_1000;
				tp->mac_mode |= MAC_MODE_PORT_MODE_GMII;
			} else if (sgsr & SERDES_TG3_SPEED_100) {
				current_speed = SPEED_100;
				tp->mac_mode |= MAC_MODE_PORT_MODE_MII;
			} else {
				current_speed = SPEED_10;
				tp->mac_mode |= MAC_MODE_PORT_MODE_MII;
			}

			if (sgsr & SERDES_TG3_FULL_DUPLEX)
				current_duplex = DUPLEX_FULL;
			else
				current_duplex = DUPLEX_HALF;
		}

		tw32_f(MAC_MODE, tp->mac_mode);
		udelay(40);

		tg3_clear_mac_status(tp);

		goto fiber_setup_done;
	}

	tp->mac_mode |= MAC_MODE_PORT_MODE_GMII;
	tw32_f(MAC_MODE, tp->mac_mode);
	udelay(40);

	tg3_clear_mac_status(tp);

	if (force_reset)
		tg3_phy_reset(tp);

	err |= tg3_readphy(tp, MII_BMSR, &bmsr);
	err |= tg3_readphy(tp, MII_BMSR, &bmsr);
	if (tg3_asic_rev(tp) == ASIC_REV_5714) {
		if (tr32(MAC_TX_STATUS) & TX_STATUS_LINK_UP)
			bmsr |= BMSR_LSTATUS;
		else
			bmsr &= ~BMSR_LSTATUS;
	}

	err |= tg3_readphy(tp, MII_BMCR, &bmcr);

	if ((tp->link_config.autoneg == AUTONEG_ENABLE) && !force_reset &&
	    (tp->phy_flags & TG3_PHYFLG_PARALLEL_DETECT)) {
		/* do nothing, just check for link up at the end */
	} else if (tp->link_config.autoneg == AUTONEG_ENABLE) {
		u32 adv, newadv;

		err |= tg3_readphy(tp, MII_ADVERTISE, &adv);
		newadv = adv & ~(ADVERTISE_1000XFULL | ADVERTISE_1000XHALF |
				 ADVERTISE_1000XPAUSE |
				 ADVERTISE_1000XPSE_ASYM |
				 ADVERTISE_SLCT);

		newadv |= ethtool_adv_to_mii_adv_x(tp->link_config.advertising);

		if ((newadv != adv) || !(bmcr & BMCR_ANENABLE)) {
			tg3_writephy(tp, MII_ADVERTISE, newadv);
			bmcr |= BMCR_ANENABLE | BMCR_ANRESTART;
			tg3_writephy(tp, MII_BMCR, bmcr);

			tw32_f(MAC_EVENT, MAC_EVENT_LNKSTATE_CHANGED);
			tp->serdes_counter = SERDES_AN_TIMEOUT_5714S;
			tp->phy_flags &= ~TG3_PHYFLG_PARALLEL_DETECT;

			return err;
		}
	} else {
		u32 new_bmcr;

		bmcr &= ~BMCR_SPEED1000;
		new_bmcr = bmcr & ~(BMCR_ANENABLE | BMCR_FULLDPLX);

		if (tp->link_config.duplex == DUPLEX_FULL)
			new_bmcr |= BMCR_FULLDPLX;

		if (new_bmcr != bmcr) {
			/* BMCR_SPEED1000 is a reserved bit that needs
			 * to be set on write.
			 */
			new_bmcr |= BMCR_SPEED1000;

			tg3_writephy(tp, MII_BMCR, new_bmcr);
			bmcr = new_bmcr;
			err |= tg3_readphy(tp, MII_BMSR, &bmsr);
			err |= tg3_readphy(tp, MII_BMSR, &bmsr);
			if (tg3_asic_rev(tp) == ASIC_REV_5714) {
				if (tr32(MAC_TX_STATUS) & TX_STATUS_LINK_UP)
					bmsr |= BMSR_LSTATUS;
				else
					bmsr &= ~BMSR_LSTATUS;
			}
			tp->phy_flags &= ~TG3_PHYFLG_PARALLEL_DETECT;
		}
	}

	if (bmsr & BMSR_LSTATUS) {
		current_speed = SPEED_1000;
		current_link_up = true;
		if (bmcr & BMCR_FULLDPLX)
			current_duplex = DUPLEX_FULL;
		else
			current_duplex = DUPLEX_HALF;

		local_adv = 0;
		remote_adv = 0;

		if (bmcr & BMCR_ANENABLE) {
			u32 common;

			err |= tg3_readphy(tp, MII_ADVERTISE, &local_adv);
			err |= tg3_readphy(tp, MII_LPA, &remote_adv);
			common = local_adv & remote_adv;
			if (common & (ADVERTISE_1000XHALF |
				      ADVERTISE_1000XFULL)) {
				if (common & ADVERTISE_1000XFULL)
					current_duplex = DUPLEX_FULL;
				else
					current_duplex = DUPLEX_HALF;

			} else if (!tg3_flag(tp, 5780_CLASS)) {
				/* Link is up via parallel detect */
			} else {
				current_link_up = false;
			}
		}
	}

fiber_setup_done:
	tp->mac_mode &= ~MAC_MODE_HALF_DUPLEX;
	if (tp->link_config.active_duplex == DUPLEX_HALF)
		tp->mac_mode |= MAC_MODE_HALF_DUPLEX;

	tw32_f(MAC_MODE, tp->mac_mode);
	udelay(40);

	tw32_f(MAC_EVENT, MAC_EVENT_LNKSTATE_CHANGED);

	tp->link_config.active_speed = current_speed;
	tp->link_config.active_duplex = current_duplex;

	return err;
}

static int tg3_setup_phy(struct tg3_priv *tp, bool force_reset)
{
	u32 val;
	int err;

	if (tp->phy_flags & TG3_PHYFLG_PHY_SERDES)
		err = tg3_setup_fiber_phy(tp, force_reset);
	else if (tp->phy_flags & TG3_PHYFLG_MII_SERDES)
		err = tg3_setup_fiber_mii_phy(tp, force_reset);
	else
		err = tg3_setup_copper_phy(tp, force_reset);

	if (tg3_chip_rev(tp) == CHIPREV_5784_AX) {
		u32 scale;

		val = tr32(TG3_CPMU_CLCK_STAT) & CPMU_CLCK_STAT_MAC_CLCK_MASK;
		if (val == CPMU_CLCK_STAT_MAC_CLCK_62_5)
			scale = 65;
		else if (val == CPMU_CLCK_STAT_MAC_CLCK_6_25)
			scale = 6;
		else
			scale = 12;

		val = tr32(GRC_MISC_CFG) & ~GRC_MISC_CFG_PRESCALAR_MASK;
		val |= (scale << GRC_MISC_CFG_PRESCALAR_SHIFT);
		tw32(GRC_MISC_CFG, val);
	}

	val = (2 << TX_LENGTHS_IPG_CRS_SHIFT) |
	      (6 << TX_LENGTHS_IPG_SHIFT);
	if (tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762)
		val |= tr32(MAC_TX_LENGTHS) &
		       (TX_LENGTHS_JMB_FRM_LEN_MSK |
			TX_LENGTHS_CNT_DWN_VAL_MSK);

	if (tp->link_config.active_speed == SPEED_1000 &&
	    tp->link_config.active_duplex == DUPLEX_HALF)
		tw32(MAC_TX_LENGTHS, val |
		     (0xff << TX_LENGTHS_SLOT_TIME_SHIFT));
	else
		tw32(MAC_TX_LENGTHS, val |
		     (32 << TX_LENGTHS_SLOT_TIME_SHIFT));

	if (tg3_flag(tp, ASPM_WORKAROUND)) {
		val = tr32(PCIE_PWR_MGMT_THRESH);
		val = (val & ~PCIE_PWR_MGMT_L1_THRESH_MSK) | tp->pwrmgmt_thresh;
		tw32(PCIE_PWR_MGMT_THRESH, val);
	}

	return err;
}

/* The RX ring scheme is composed of multiple rings which post fresh buffers to
 * the chip, and one special ring the chip uses to report status back to the
 * host.
 *
 * The special ring reports the status of received packets to the host.  The
 * chip does not write into the original descriptor the RX buffer was obtained
 * from.  The chip simply takes the original descriptor as provided by the
 * host, updates the status and length field, then writes this into the next
 * status ring entry.
 *
 * Each ring the host uses to post buffers to the chip is described by a
 * TG3_BDINFO entry in the chips SRAM area.  When a packet arrives, it is first
 * placed into the on-chip ram.  When the packet's length is known, it walks
 * down the TG3_BDINFO entries to select the ring.  Each TG3_BDINFO specifies a
 * MAXLEN field and the first TG3_BDINFO which is within the range of the new
 * packet's length is chosen.
 *
 * The "separate ring for rx status" scheme may sound queer, but it makes sense
 * from a cache coherency perspective.  If only the host writes to the buffer
 * post rings, and only the chip writes to the rx status rings, then cache
 * lines never move beyond shared-modified state.  If both the host and chip
 * were to write into the same ring, cache line eviction could occur since both
 * entities want it in an exclusive state.
 */
int tg3_recv(struct eth_device *dev)
{
	struct tg3_priv *tp = dev->priv;
	struct tg3_napi *tnapi = &tp->napi[0];

	int rx_producer = tp->rx_producer;
	int rx_consumer = tp->rx_consumer;

	/* Loop the Return ring until there are no more packets */
	while (rx_consumer  != tnapi->hw_status->idx[0].rx_producer) {
		void *packet_address = (void *) (uintptr_t) (((uint64_t) tnapi->rx_rcb[rx_consumer].addr_hi << 32) +
							      (uint64_t) tnapi->rx_rcb[rx_consumer].addr_lo);
		u32 packet_length = tnapi->rx_rcb[rx_consumer].idx_len;

		if (packet_length > TG3_RX_FRAME_SIZE) {
			printf("Error: Invalid packet length.\n");
			return -EIO;
		}
		/* Process the packet */
		NetReceive(packet_address , packet_length);

		/* Update Producer Ring producer */
		tnapi->prodring.rx_std[rx_producer].addr_hi = tnapi->rx_rcb[rx_consumer].addr_hi;
		tnapi->prodring.rx_std[rx_producer].addr_lo = tnapi->rx_rcb[rx_consumer].addr_lo;
		rx_producer = (rx_producer + 1) % TG3_RX_PRODUCER_RING_SIZE;
		tw32_rx_mbox(TG3_RX_STD_PROD_IDX_REG, rx_producer);

		/* Update Return Ring consumer */
		rx_consumer = (rx_consumer + 1) % TG3_RX_RETURN_RING_SIZE;
		tw32_rx_mbox(tnapi->consmbox, rx_consumer);
	}

	tp->rx_producer = rx_producer;
	tp->rx_consumer = rx_consumer;

	return 0;
}

static inline void tg3_tx_set_bd(struct tg3_tx_buffer_desc *txbd,
				 dma_addr_t mapping, u32 len, u32 flags,
				 u32 mss, u32 vlan)
{
	txbd->addr_hi = ((u64) mapping >> 32);
	txbd->addr_lo = ((u64) mapping & 0xffffffff);
	txbd->len_flags = (len << TXD_LEN_SHIFT) | (flags & 0x0000ffff);
	txbd->vlan_tag = (mss << TXD_MSS_SHIFT) | (vlan << TXD_VLAN_TAG_SHIFT);
}

static void tg3_rx_prodring_free(struct tg3_priv *tp, struct tg3_rx_prodring_set *tpr)
{
	int i;
	for (i = 0; i < TG3_RX_BUFFER_COUNT; i++)
		free(tp->rx_buffer[i]);
}

/* Initialize rx rings for packet processing.
 *
 * The chip has been shut down and the driver detached from the networking, so
 * no interrupts or new tx packets will end up in the driver.  tp->{tx,}lock
 * are held and thus we may not sleep.
 */
static int tg3_rx_prodring_alloc(struct tg3_priv *tp, struct tg3_rx_prodring_set *tpr)
{
	int i;

	/* Initialize invariants of the rings, we only set this stuff once.
	 * This works because the card does not write into the rx buffer
	 * posting rings.
	 */
	for (i = 0; i < TG3_RX_PRODUCER_RING_SIZE; i++) {
		struct tg3_rx_buffer_desc *rxd;

		rxd = &tpr->rx_std[i];
		rxd->idx_len = TG3_RX_FRAME_SIZE << RXD_LEN_SHIFT;
		rxd->type_flags = (RXD_FLAG_END << RXD_FLAGS_SHIFT);
		rxd->opaque = (RXD_OPAQUE_RING_STD | (i << RXD_OPAQUE_INDEX_SHIFT));
	}

	/* Allocate memory for packets that we receive */
	for (i = 0; i < TG3_RX_BUFFER_COUNT; i++) {
		dma_addr_t temp_buffer_mapping;
		void *temp_buffer = dma_alloc_coherent(TG3_RX_FRAME_SIZE, &temp_buffer_mapping);

		if (temp_buffer == NULL) {
			printf("We've run out of memory D: (Rx buffer %d)\n", i);
		}
		tp->rx_buffer[i] = temp_buffer;
		tpr->rx_std[i].addr_hi = ((u64)temp_buffer_mapping >> 32);
		tpr->rx_std[i].addr_lo = ((u64)temp_buffer_mapping & 0xffffffff);
		tp->rx_producer = i;
	}

	return 0;
}

/* Free up pending packets in all rx/tx rings.
 *
 * The chip has been shut down and the driver detached from
 * the networking, so no interrupts or new tx packets will
 * end up in the driver.  tp->{tx,}lock is not held and we are not
 * in an interrupt context and thus may sleep.
 */
static void tg3_free_rings(struct tg3_priv *tp)
{
	struct tg3_napi *tnapi = &tp->napi[0];
	tg3_rx_prodring_free(tp, &tnapi->prodring);
}

/* Initialize tx/rx rings for packet processing.
 *
 * The chip has been shut down and the driver detached from
 * the networking, so no interrupts or new tx packets will
 * end up in the driver.  tp->{tx,}lock are held and thus
 * we may not sleep.
 */
static int tg3_init_rings(struct tg3_priv *tp)
{
	struct tg3_napi *tnapi = &tp->napi[0];

	tnapi->hw_status->status = 0;
	tnapi->hw_status->status_tag = 0;
	memset(tnapi->hw_status, 0, TG3_HW_STATUS_SIZE);

	tp->tx_producer = 0;
	tp->rx_producer = 0;
	tp->rx_consumer = 0;

	if (tg3_rx_prodring_alloc(tp, &tnapi->prodring)) {
		tg3_free_rings(tp);
		return -ENOMEM;
	}

	return 0;
}

static int tg3_mem_ring_acquire(struct tg3_priv *tp)
{
	struct tg3_napi *tnapi = tp->napi;

	/* Allocate Rx Producer Ring */
	tnapi->prodring.rx_std = dma_alloc_coherent(TG3_RX_STD_RING_BYTES, &tnapi->prodring.rx_std_mapping);
	memset(tnapi->prodring.rx_std, 0, TG3_RX_STD_RING_BYTES);

	/* Allocate Rx Return Ring */
	tnapi->rx_rcb = dma_alloc_coherent(TG3_RX_RCB_RING_BYTES, &tnapi->rx_rcb_mapping);
	memset(tnapi->rx_rcb, 0, TG3_RX_RCB_RING_BYTES);

	/* Allocate Tx Ring */
	tnapi->tx_ring = dma_alloc_coherent(TG3_TX_RING_BYTES, &tnapi->tx_desc_mapping);
	memset(tnapi->tx_ring, 0, TG3_TX_RING_BYTES);

	if (tnapi->tx_ring && tnapi->rx_rcb && tnapi->prodring.rx_std)
		return 0;

	return -ENOMEM;
}

static void tg3_free_consistent(struct tg3_priv *tp)
{
	struct tg3_napi *tnapi = &tp->napi[0];

	if (tnapi->hw_status) {
		free(tnapi->hw_status);
		tnapi->hw_status = NULL;
	}

	/* Descriptor rings */
	if (tnapi->prodring.rx_std) {
		free(tnapi->prodring.rx_std);
		tnapi->prodring.rx_std = NULL;
	}
	if (tnapi->rx_rcb) {
		free(tnapi->rx_rcb);
		tnapi->rx_rcb = NULL;
	}
	if (tnapi->tx_ring) {
		free(tnapi->tx_ring);
		tnapi->tx_ring = NULL;
	}

	if (tp->hw_stats) {
		free(tp->hw_stats);
		tp->hw_stats = NULL;
	}
}

/* TODO: Consider this comment.. Interrupts don't exist for us.
 * Must not be invoked with interrupt sources disabled and
 * the hardware shutdown down.  Can sleep.
 */
static int tg3_alloc_consistent(struct tg3_priv *tp)
{
	struct tg3_napi *tnapi = &tp->napi[0];

	tp->hw_stats = dma_alloc_coherent(sizeof(struct tg3_hw_stats), &tp->stats_mapping);
	if (!tp->hw_stats)
		goto err_out;

	tnapi->hw_status = dma_alloc_coherent(TG3_HW_STATUS_SIZE,
					      &tnapi->status_mapping);
	if (!tnapi->hw_status)
		goto err_out;

	if (tg3_mem_ring_acquire(tp))
		goto err_out;

	return 0;

err_out:
	tg3_free_consistent(tp);
	return -ENOMEM;
}

#define MAX_WAIT_CNT 1000

/* To stop a block, clear the enable bit and poll till it clears.
 */
static int tg3_stop_block(struct tg3_priv *tp, unsigned long ofs, u32 enable_bit, bool silent)
{
	unsigned int i;
	u32 val;

	if (tg3_flag(tp, 5705_PLUS)) {
		switch (ofs) {
		case RCVLSC_MODE:
		case DMAC_MODE:
		case MBFREE_MODE:
		case BUFMGR_MODE:
		case MEMARB_MODE:
			/* We can't enable/disable these bits of the
			 * 5705/5750, just say success.
			 */
			return 0;

		default:
			break;
		}
	}

	val = tr32(ofs);
	val &= ~enable_bit;
	tw32_f(ofs, val);

	for (i = 0; i < MAX_WAIT_CNT; i++) {
		udelay(100);
		val = tr32(ofs);
		if ((val & enable_bit) == 0)
			break;
	}

	if (i == MAX_WAIT_CNT && !silent) {
		printf("#   tg3_stop_block timed out, ofs=%lx enable_bit=%x\n",
		       ofs, enable_bit);
		return -ENODEV;
	}

	return 0;
}

static int tg3_abort_hw(struct tg3_priv *tp, bool silent)
{
	int i, err;

	tg3_disable_ints(tp);

	tp->rx_mode &= ~RX_MODE_ENABLE;
	tw32_f(MAC_RX_MODE, tp->rx_mode);
	udelay(10);

	err  = tg3_stop_block(tp, RCVBDI_MODE, RCVBDI_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, RCVLPC_MODE, RCVLPC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, RCVLSC_MODE, RCVLSC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, RCVDBDI_MODE, RCVDBDI_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, RCVDCC_MODE, RCVDCC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, RCVCC_MODE, RCVCC_MODE_ENABLE, silent);

	err |= tg3_stop_block(tp, SNDBDS_MODE, SNDBDS_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, SNDBDI_MODE, SNDBDI_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, SNDDATAI_MODE, SNDDATAI_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, RDMAC_MODE, RDMAC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, SNDDATAC_MODE, SNDDATAC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, DMAC_MODE, DMAC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, SNDBDC_MODE, SNDBDC_MODE_ENABLE, silent);

	tp->mac_mode &= ~MAC_MODE_TDE_ENABLE;
	tw32_f(MAC_MODE, tp->mac_mode);
	udelay(40);

	tp->tx_mode &= ~TX_MODE_ENABLE;
	tw32_f(MAC_TX_MODE, tp->tx_mode);

	for (i = 0; i < MAX_WAIT_CNT; i++) {
		udelay(100);
		if (!(tr32(MAC_TX_MODE) & TX_MODE_ENABLE))
			break;
	}
	if (i >= MAX_WAIT_CNT) {
		printf("#  %s timed out, TX_MODE_ENABLE will not clear MAC_TX_MODE=%08x\n",
		       __func__, tr32(MAC_TX_MODE));
		err |= -ENODEV;
	}

	err |= tg3_stop_block(tp, HOSTCC_MODE, HOSTCC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, WDMAC_MODE, WDMAC_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, MBFREE_MODE, MBFREE_MODE_ENABLE, silent);

	tw32(FTQ_RESET, 0xffffffff);
	tw32(FTQ_RESET, 0x00000000);

	err |= tg3_stop_block(tp, BUFMGR_MODE, BUFMGR_MODE_ENABLE, silent);
	err |= tg3_stop_block(tp, MEMARB_MODE, MEMARB_MODE_ENABLE, silent);

	for (i = 0; i < 1/*tp->irq_cnt*/; i++) {
		struct tg3_napi *tnapi = &tp->napi[i];
		if (tnapi->hw_status)
			memset(tnapi->hw_status, 0, TG3_HW_STATUS_SIZE);
	}

	return err;
}

/* Save PCI command register before chip reset */
static void tg3_save_pci_state(struct tg3_priv *tp)
{
	pci_read_config_word(tp->devno, PCI_COMMAND, &tp->pci_cmd);
}

/* Restore PCI state after chip reset */
static void tg3_restore_pci_state(struct tg3_priv *tp)
{
	u32 val;

	/* Re-enable indirect register accesses. */
	pci_write_config_dword(tp->devno, TG3PCI_MISC_HOST_CTRL,
			       tp->misc_host_ctrl);

	/* Set MAX PCI retry to zero. */
	val = (PCISTATE_ROM_ENABLE | PCISTATE_ROM_RETRY_ENABLE);

	pci_write_config_dword(tp->devno, TG3PCI_PCISTATE, val);

	pci_write_config_word(tp->devno, PCI_COMMAND, tp->pci_cmd);

	pci_write_config_byte(tp->devno, PCI_CACHE_LINE_SIZE, tp->pci_cacheline_sz);
	pci_write_config_byte(tp->devno, PCI_LATENCY_TIMER, tp->pci_lat_timer);

}

static void tg3_override_clk(struct tg3_priv *tp)
{
	u32 val;

	switch (tg3_asic_rev(tp)) {
	case ASIC_REV_5717:
		val = tr32(TG3_CPMU_CLCK_ORIDE_ENABLE);
		tw32(TG3_CPMU_CLCK_ORIDE_ENABLE, val |
		     TG3_CPMU_MAC_ORIDE_ENABLE);
		break;

	case ASIC_REV_5719:
	case ASIC_REV_5720:
		tw32(TG3_CPMU_CLCK_ORIDE, CPMU_CLCK_ORIDE_MAC_ORIDE_EN);
		break;

	default:
		return;
	}
}

static void tg3_restore_clk(struct tg3_priv *tp)
{
	u32 val;

	switch (tg3_asic_rev(tp)) {
	case ASIC_REV_5717:
		val = tr32(TG3_CPMU_CLCK_ORIDE_ENABLE);
		tw32(TG3_CPMU_CLCK_ORIDE_ENABLE,
		     val & ~TG3_CPMU_MAC_ORIDE_ENABLE);
		break;

	case ASIC_REV_5719:
	case ASIC_REV_5720:
		val = tr32(TG3_CPMU_CLCK_ORIDE);
		tw32(TG3_CPMU_CLCK_ORIDE, val & ~CPMU_CLCK_ORIDE_MAC_ORIDE_EN);
		break;

	default:
		return;
	}
}

static int tg3_chip_reset(struct tg3_priv *tp)
{
	u32 val;
	void (*write_op)(struct tg3_priv *, u32, u32);
	int err;

	/* GRC_MISC_CFG core clock reset will clear the memory
	 * enable bit in PCI register 4 and the MSI enable bit
	 * on some chips, so we save relevant registers here.
	 */
	tg3_save_pci_state(tp);

	if (tg3_asic_rev(tp) == ASIC_REV_5752 ||
	    tg3_flag(tp, 5755_PLUS))
		tw32(GRC_FASTBOOT_PC, 0);

	/*
	 * We must avoid the readl() that normally takes place.
	 * It locks machines, causes machine checks, and other
	 * fun things.  So, temporarily disable the 5701
	 * hardware workaround, while we do the reset.
	 */
	write_op = tp->write32;
	if (write_op == tg3_write_flush_reg32)
		tp->write32 = tg3_write32;

	tg3_flag_set(tp, CHIP_RESETTING);
	struct tg3_napi *tnapi = &tp->napi[0];
	if (tnapi->hw_status) {
		tnapi->hw_status->status = 0;
		tnapi->hw_status->status_tag = 0;
	}

	smp_mb();

	if (tg3_asic_rev(tp) == ASIC_REV_57780) {
		val = tr32(TG3_PCIE_LNKCTL) & ~TG3_PCIE_LNKCTL_L1_PLL_PD_EN;
		tw32(TG3_PCIE_LNKCTL, val | TG3_PCIE_LNKCTL_L1_PLL_PD_DIS);
	}

	/* do the reset */
	val = GRC_MISC_CFG_CORECLK_RESET;

	if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		tw32(VCPU_STATUS, tr32(VCPU_STATUS) | VCPU_STATUS_DRV_RESET);
		tw32(GRC_VCPU_EXT_CTRL,
		     tr32(GRC_VCPU_EXT_CTRL) & ~GRC_VCPU_EXT_CTRL_HALT_CPU);
	}

	/* Set the clock to the highest frequency to avoid timeouts. With link
	 * aware mode, the clock speed could be slow and bootcode does not
	 * complete within the expected time. Override the clock to allow the
	 * bootcode to finish sooner and then restore it.
	 */
	tg3_override_clk(tp);

	/* Manage gphy power for all CPMU absent PCIe devices. */
	if (tg3_flag(tp, 5705_PLUS) && !tg3_flag(tp, CPMU_PRESENT))
		val |= GRC_MISC_CFG_KEEP_GPHY_POWER;

	tw32(GRC_MISC_CFG, val);

	/* restore 5701 hardware bug workaround write method */
	tp->write32 = write_op;

	/* Unfortunately, we have to delay before the PCI read back.
	 * Some 575X chips even will not respond to a PCI cfg access
	 * when the reset command is given to the chip.
	 *
	 * How do these hardware designers expect things to work
	 * properly if the PCI write is posted for a long period
	 * of time?  It is always necessary to have some method by
	 * which a register read back can occur to push the write
	 * out which does the reset.
	 *
	 * For most tg3 variants the trick below was working.
	 * Ho hum...
	 */
	udelay(120);

	/* Flush PCI posted writes.  The normal MMIO registers
	 * are inaccessible at this time so this is the only
	 * way to make this reliably (actually, this is no longer
	 * the case, see above).  I tried to use indirect
	 * register read/write but this upset some 5701 variants.
	 */
	pci_read_config_dword(tp->devno, PCI_COMMAND, &val);

	udelay(120);

	tg3_restore_pci_state(tp);

	tg3_flag_clear(tp, CHIP_RESETTING);
	tg3_flag_clear(tp, ERROR_PROCESSED);

	val = 0;
	if (tg3_flag(tp, 5780_CLASS))
		val = tr32(MEMARB_MODE);
	tw32(MEMARB_MODE, val | MEMARB_MODE_ENABLE);

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5750_A3) {
		tw32(0x5000, 0x400);
	}

	err = tg3_poll_fw(tp);
	if (err)
		return err;

	tw32(GRC_MODE, tp->grc_mode);

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5705_A0) {
		val = tr32(0xc4);

		tw32(0xc4, val | (1 << 15));
	}

	if ((tp->nic_sram_data_cfg & NIC_SRAM_DATA_CFG_MINI_PCI) != 0 &&
	    tg3_asic_rev(tp) == ASIC_REV_5705) {
		tp->pci_clock_ctrl |= CLOCK_CTRL_CLKRUN_OENABLE;
		if (tg3_chip_rev_id(tp) == CHIPREV_ID_5705_A0)
			tp->pci_clock_ctrl |= CLOCK_CTRL_FORCE_CLKRUN;
		tw32(TG3PCI_CLOCK_CTRL, tp->pci_clock_ctrl);
	}

	if (tp->phy_flags & TG3_PHYFLG_PHY_SERDES) {
		tp->mac_mode = MAC_MODE_PORT_MODE_TBI;
		val = tp->mac_mode;
	} else if (tp->phy_flags & TG3_PHYFLG_MII_SERDES) {
		tp->mac_mode = MAC_MODE_PORT_MODE_GMII;
		val = tp->mac_mode;
	} else
		val = 0;

	tw32_f(MAC_MODE, val);
	udelay(40);

	tg3_mdio_start(tp);

	tg3_restore_clk(tp);

	return 0;
}

static int tg3_halt__(struct tg3_priv *tp, int kind, bool silent)
{
	int err;

	tg3_write_sig_pre_reset(tp, kind);

	tg3_abort_hw(tp, silent);
	err = tg3_chip_reset(tp);

	__tg3_set_mac_addr(tp, false);

	return err;
}

static void tg3_set_bdinfo(struct tg3_priv *tp, u32 bdinfo_addr, dma_addr_t mapping,
			   u32 maxlen_flags, u32 nic_addr)
{
	tg3_write_mem(tp, (bdinfo_addr + TG3_BDINFO_HOST_ADDR + TG3_64BIT_REG_HIGH),
		      ((u64) mapping >> 32));
	tg3_write_mem(tp, (bdinfo_addr + TG3_BDINFO_HOST_ADDR + TG3_64BIT_REG_LOW),
		      ((u64) mapping & 0xffffffff));
	tg3_write_mem(tp, (bdinfo_addr + TG3_BDINFO_MAXLEN_FLAGS), maxlen_flags);

	if (!tg3_flag(tp, 5705_PLUS))
	    tg3_write_mem(tp, (bdinfo_addr + TG3_BDINFO_NIC_ADDR), nic_addr);
}

static void tg3_coal_tx_init(struct tg3_priv *tp)
{
	tw32(HOSTCC_TXCOL_TICKS, 1000);
	tw32(HOSTCC_TXMAX_FRAMES, 1);
	tw32(HOSTCC_TXCOAL_MAXF_INT, 1);
}

static void tg3_coal_rx_init(struct tg3_priv *tp)
{
	tw32(HOSTCC_RXCOL_TICKS, 1000);
	tw32(HOSTCC_RXMAX_FRAMES, 1);
	tw32(HOSTCC_RXCOAL_MAXF_INT, 1);
}

static void __tg3_set_coalesce(struct tg3_priv *tp)
{
	tg3_coal_tx_init(tp);
	tg3_coal_rx_init(tp);
}

static void tg3_tx_rcbs_disable(struct tg3_priv *tp)
{
	u32 txrcb, limit;

	/* Disable all transmit rings but the first. */
	if (!tg3_flag(tp, 5705_PLUS))
		limit = NIC_SRAM_SEND_RCB + TG3_BDINFO_SIZE * 16;
	else if (tg3_flag(tp, 5717_PLUS))
		limit = NIC_SRAM_SEND_RCB + TG3_BDINFO_SIZE * 4;
	else if (tg3_flag(tp, 57765_CLASS) ||
		 tg3_asic_rev(tp) == ASIC_REV_5762)
		limit = NIC_SRAM_SEND_RCB + TG3_BDINFO_SIZE * 2;
	else
		limit = NIC_SRAM_SEND_RCB + TG3_BDINFO_SIZE;

	for (txrcb = NIC_SRAM_SEND_RCB + TG3_BDINFO_SIZE;
	     txrcb < limit; txrcb += TG3_BDINFO_SIZE)
		tg3_write_mem(tp, txrcb + TG3_BDINFO_MAXLEN_FLAGS,
			      BDINFO_FLAGS_DISABLED);
}

static void tg3_tx_rcbs_init(struct tg3_priv *tp)
{
	u32 txrcb = NIC_SRAM_SEND_RCB;

	struct tg3_napi *tnapi = &tp->napi[0];

	if (!tnapi->tx_ring)
		return;

	tg3_set_bdinfo(tp, txrcb, tnapi->tx_desc_mapping,
		       (TG3_TX_RING_SIZE << BDINFO_FLAGS_MAXLEN_SHIFT),
			NIC_SRAM_TX_BUFFER_DESC);
}

static void tg3_rx_ret_rcbs_disable(struct tg3_priv *tp)
{
	u32 rxrcb, limit;

	/* Disable all receive return rings but the first. */
	if (tg3_flag(tp, 5717_PLUS))
		limit = NIC_SRAM_RCV_RET_RCB + TG3_BDINFO_SIZE * 17;
	else if (!tg3_flag(tp, 5705_PLUS))
		limit = NIC_SRAM_RCV_RET_RCB + TG3_BDINFO_SIZE * 16;
	else if (tg3_asic_rev(tp) == ASIC_REV_5755 ||
		 tg3_asic_rev(tp) == ASIC_REV_5762 ||
		 tg3_flag(tp, 57765_CLASS))
		limit = NIC_SRAM_RCV_RET_RCB + TG3_BDINFO_SIZE * 4;
	else
		limit = NIC_SRAM_RCV_RET_RCB + TG3_BDINFO_SIZE;

	for (rxrcb = NIC_SRAM_RCV_RET_RCB + TG3_BDINFO_SIZE;
	     rxrcb < limit; rxrcb += TG3_BDINFO_SIZE)
		tg3_write_mem(tp, rxrcb + TG3_BDINFO_MAXLEN_FLAGS,
			      BDINFO_FLAGS_DISABLED);
}

static void tg3_rx_ret_rcbs_init(struct tg3_priv *tp)
{
	u32 rxrcb = NIC_SRAM_RCV_RET_RCB;

	struct tg3_napi *tnapi = &tp->napi[0];

	if (!tnapi->rx_rcb) {
		printf("#   WARNING: Skipping the only one we have! %d\n", __LINE__);
		return;
	}

	tg3_set_bdinfo(tp, rxrcb, tnapi->rx_rcb_mapping,
		       TG3_RX_RETURN_RING_SIZE <<
		       BDINFO_FLAGS_MAXLEN_SHIFT, 0);
}

static void tg3_rings_reset(struct tg3_priv *tp)
{
	int i;
	u32 stblk;
	struct tg3_napi *tnapi = &tp->napi[0];

	tg3_tx_rcbs_disable(tp);

	tg3_rx_ret_rcbs_disable(tp);

	/* Disable interrupts */
	tw32_mailbox_f(tp->napi[0].int_mbox, 1);

	tw32_mailbox(tp->napi[0].prodmbox, 0);
	tw32_rx_mbox(tp->napi[0].consmbox, 0);

	/* Make sure the NIC-based send BD rings are disabled. */
	if (!tg3_flag(tp, 5705_PLUS)) {
		u32 mbox = MAILBOX_SNDNIC_PROD_IDX_0 + TG3_64BIT_REG_LOW;
		for (i = 0; i < 16; i++)
			tw32_tx_mbox(mbox + i * 8, 0);
	}

	/* Clear status block in ram. */
	memset(tnapi->hw_status, 0, TG3_HW_STATUS_SIZE);

	/* Set status block DMA address */
	tw32(HOSTCC_STATUS_BLK_HOST_ADDR + TG3_64BIT_REG_HIGH,
	     ((u64) tnapi->status_mapping >> 32));
	tw32(HOSTCC_STATUS_BLK_HOST_ADDR + TG3_64BIT_REG_LOW,
	     ((u64) tnapi->status_mapping & 0xffffffff));

	stblk = HOSTCC_STATBLCK_RING1;

	for (i = 1, tnapi++; i < 1/*tp->irq_cnt*/; i++, tnapi++) {
		u64 mapping = (u64)tnapi->status_mapping;
		tw32(stblk + TG3_64BIT_REG_HIGH, mapping >> 32);
		tw32(stblk + TG3_64BIT_REG_LOW, mapping & 0xffffffff);
		stblk += 8;

		/* Clear status block in ram. */
		memset(tnapi->hw_status, 0, TG3_HW_STATUS_SIZE);
	}

	tg3_tx_rcbs_init(tp);
	tg3_rx_ret_rcbs_init(tp);
}

static void tg3_setup_rxbd_thresholds(struct tg3_priv *tp)
{
	u32 val, bdcache_maxcnt, host_rep_thresh, nic_rep_thresh;

	if (!tg3_flag(tp, 5750_PLUS) ||
	    tg3_flag(tp, 5780_CLASS) ||
	    tg3_asic_rev(tp) == ASIC_REV_5750 ||
	    tg3_asic_rev(tp) == ASIC_REV_5752 ||
	    tg3_flag(tp, 57765_PLUS))
		bdcache_maxcnt = TG3_SRAM_RX_STD_BDCACHE_SIZE_5700;
	else if (tg3_asic_rev(tp) == ASIC_REV_5755 ||
		 tg3_asic_rev(tp) == ASIC_REV_5787)
		bdcache_maxcnt = TG3_SRAM_RX_STD_BDCACHE_SIZE_5755;
	else
		bdcache_maxcnt = TG3_SRAM_RX_STD_BDCACHE_SIZE_5906;

	nic_rep_thresh = min(bdcache_maxcnt / 2, tp->rx_std_max_post);
	host_rep_thresh = max_t(u32, TG3_RX_BUFFER_COUNT / 8, 1);

	val = min(nic_rep_thresh, host_rep_thresh);
	tw32(RCVBDI_STD_THRESH, val);

	if (tg3_flag(tp, 57765_PLUS))
		tw32(STD_REPLENISH_LWM, bdcache_maxcnt);

}

static void tg3_set_multi(struct tg3_priv *tp, unsigned int accept_all)
{
	/* accept or reject all multicast frames */
	tw32(MAC_HASH_REG_0, accept_all ? 0xffffffff : 0);
	tw32(MAC_HASH_REG_1, accept_all ? 0xffffffff : 0);
	tw32(MAC_HASH_REG_2, accept_all ? 0xffffffff : 0);
	tw32(MAC_HASH_REG_3, accept_all ? 0xffffffff : 0);
}

static void __tg3_set_rx_mode(struct tg3_priv *tp)
{
	u32 rx_mode;

	rx_mode = tp->rx_mode & ~(RX_MODE_PROMISC | RX_MODE_KEEP_VLAN_TAG);

#if !defined(CONFIG_VLAN_8021Q) && !defined(CONFIG_VLAN_8021Q_MODULE)
	rx_mode |= RX_MODE_KEEP_VLAN_TAG;
#endif

	tg3_set_multi(tp, 1);

	if (rx_mode != tp->rx_mode) {
		tp->rx_mode = rx_mode;
		tw32_f(MAC_RX_MODE, rx_mode);
		udelay(10);
	}
}

static inline u32 tg3_lso_rd_dma_workaround_bit(struct tg3_priv *tp)
{
	if (tg3_asic_rev(tp) == ASIC_REV_5719)
		return TG3_LSO_RD_DMA_TX_LENGTH_WA_5719;
	else
		return TG3_LSO_RD_DMA_TX_LENGTH_WA_5720;
}

static int tg3_reset_hw(struct tg3_priv *tp, bool reset_phy)
{
	u32 val, rdmac_mode;
	int i, err, limit;
	struct tg3_rx_prodring_set *tpr = &tp->napi[0].prodring;

	tg3_disable_ints(tp);

	tg3_write_sig_pre_reset(tp, RESET_KIND_INIT);

	if (tg3_flag(tp, INIT_COMPLETE))
		tg3_abort_hw(tp, 1);

	if ((tp->phy_flags & TG3_PHYFLG_KEEP_LINK_ON_PWRDN) &&
	    !(tp->phy_flags & TG3_PHYFLG_USER_CONFIGURED)) {
		tg3_phy_pull_config(tp);
		tp->phy_flags |= TG3_PHYFLG_USER_CONFIGURED;
	}

	if (reset_phy)
		tg3_phy_reset(tp);

	err = tg3_chip_reset(tp);
	if (err)
		return err;

	if (tg3_chip_rev(tp) == CHIPREV_5784_AX) {
		val = tr32(TG3_CPMU_CTRL);
		val &= ~(CPMU_CTRL_LINK_AWARE_MODE | CPMU_CTRL_LINK_IDLE_MODE);
		tw32(TG3_CPMU_CTRL, val);

		val = tr32(TG3_CPMU_LSPD_10MB_CLK);
		val &= ~CPMU_LSPD_10MB_MACCLK_MASK;
		val |= CPMU_LSPD_10MB_MACCLK_6_25;
		tw32(TG3_CPMU_LSPD_10MB_CLK, val);

		val = tr32(TG3_CPMU_LNK_AWARE_PWRMD);
		val &= ~CPMU_LNK_AWARE_MACCLK_MASK;
		val |= CPMU_LNK_AWARE_MACCLK_6_25;
		tw32(TG3_CPMU_LNK_AWARE_PWRMD, val);

		val = tr32(TG3_CPMU_HST_ACC);
		val &= ~CPMU_HST_ACC_MACCLK_MASK;
		val |= CPMU_HST_ACC_MACCLK_6_25;
		tw32(TG3_CPMU_HST_ACC, val);
	}

	if (tg3_asic_rev(tp) == ASIC_REV_57780) {
		val = tr32(PCIE_PWR_MGMT_THRESH) & ~PCIE_PWR_MGMT_L1_THRESH_MSK;
		val |= PCIE_PWR_MGMT_EXT_ASPM_TMR_EN |
		       PCIE_PWR_MGMT_L1_THRESH_4MS;
		tw32(PCIE_PWR_MGMT_THRESH, val);

		val = tr32(TG3_PCIE_EIDLE_DELAY) & ~TG3_PCIE_EIDLE_DELAY_MASK;
		tw32(TG3_PCIE_EIDLE_DELAY, val | TG3_PCIE_EIDLE_DELAY_13_CLKS);

		tw32(TG3_CORR_ERR_STAT, TG3_CORR_ERR_STAT_CLEAR);

		val = tr32(TG3_PCIE_LNKCTL) & ~TG3_PCIE_LNKCTL_L1_PLL_PD_EN;
		tw32(TG3_PCIE_LNKCTL, val | TG3_PCIE_LNKCTL_L1_PLL_PD_DIS);
	}

	if (tg3_flag(tp, L1PLLPD_EN)) {
		u32 grc_mode = tr32(GRC_MODE);

		/* Access the lower 1K of PL PCIE block registers. */
		val = grc_mode & ~GRC_MODE_PCIE_PORT_MASK;
		tw32(GRC_MODE, val | GRC_MODE_PCIE_PL_SEL);

		val = tr32(TG3_PCIE_TLDLPL_PORT + TG3_PCIE_PL_LO_PHYCTL1);
		tw32(TG3_PCIE_TLDLPL_PORT + TG3_PCIE_PL_LO_PHYCTL1,
		     val | TG3_PCIE_PL_LO_PHYCTL1_L1PLLPD_EN);

		tw32(GRC_MODE, grc_mode);
	}

	if (tg3_flag(tp, 57765_CLASS)) {
		if (tg3_chip_rev_id(tp) == CHIPREV_ID_57765_A0) {
			u32 grc_mode = tr32(GRC_MODE);

			/* Access the lower 1K of PL PCIE block registers. */
			val = grc_mode & ~GRC_MODE_PCIE_PORT_MASK;
			tw32(GRC_MODE, val | GRC_MODE_PCIE_PL_SEL);

			val = tr32(TG3_PCIE_TLDLPL_PORT +
				   TG3_PCIE_PL_LO_PHYCTL5);
			tw32(TG3_PCIE_TLDLPL_PORT + TG3_PCIE_PL_LO_PHYCTL5,
			     val | TG3_PCIE_PL_LO_PHYCTL5_DIS_L2CLKREQ);

			tw32(GRC_MODE, grc_mode);
		}

		if (tg3_chip_rev(tp) != CHIPREV_57765_AX) {
			u32 grc_mode;

			/* Fix transmit hangs */
			val = tr32(TG3_CPMU_PADRNG_CTL);
			val |= TG3_CPMU_PADRNG_CTL_RDIV2;
			tw32(TG3_CPMU_PADRNG_CTL, val);

			grc_mode = tr32(GRC_MODE);

			/* Access the lower 1K of DL PCIE block registers. */
			val = grc_mode & ~GRC_MODE_PCIE_PORT_MASK;
			tw32(GRC_MODE, val | GRC_MODE_PCIE_DL_SEL);

			val = tr32(TG3_PCIE_TLDLPL_PORT +
				   TG3_PCIE_DL_LO_FTSMAX);
			val &= ~TG3_PCIE_DL_LO_FTSMAX_MSK;
			tw32(TG3_PCIE_TLDLPL_PORT + TG3_PCIE_DL_LO_FTSMAX,
			     val | TG3_PCIE_DL_LO_FTSMAX_VAL);

			tw32(GRC_MODE, grc_mode);
		}

		val = tr32(TG3_CPMU_LSPD_10MB_CLK);
		val &= ~CPMU_LSPD_10MB_MACCLK_MASK;
		val |= CPMU_LSPD_10MB_MACCLK_6_25;
		tw32(TG3_CPMU_LSPD_10MB_CLK, val);
	}

	/* This works around an issue with Athlon chipsets on
	 * B3 tigon3 silicon.  This bit has no effect on any
	 * other revision.  But do not set this on PCI Express
	 * chips and don't even touch the clocks if the CPMU is present.
	 */
	if (!tg3_flag(tp, CPMU_PRESENT)) {
		tp->pci_clock_ctrl |= CLOCK_CTRL_DELAY_PCI_GRANT;
		tw32_f(TG3PCI_CLOCK_CTRL, tp->pci_clock_ctrl);
	}

	if (tg3_chip_rev(tp) == CHIPREV_5704_BX) {
		/* Enable some hw fixes.  */
		val = tr32(TG3PCI_MSI_DATA);
		val |= (1 << 26) | (1 << 28) | (1 << 29);
		tw32(TG3PCI_MSI_DATA, val);
	}

	/* Descriptor ring init may make accesses to the
	 * NIC SRAM area to setup the TX descriptors, so we
	 * can only do this after the hardware has been
	 * successfully reset.
	 */
	err = tg3_init_rings(tp);
	if (err)
		return err;

	if (tg3_flag(tp, 57765_PLUS)) {
		val = tr32(TG3PCI_DMA_RW_CTRL) &
		      ~DMA_RWCTRL_DIS_CACHE_ALIGNMENT;
		if (tg3_chip_rev_id(tp) == CHIPREV_ID_57765_A0)
			val &= ~DMA_RWCTRL_CRDRDR_RDMA_MRRS_MSK;
		if (!tg3_flag(tp, 57765_CLASS) &&
		    tg3_asic_rev(tp) != ASIC_REV_5717 &&
		    tg3_asic_rev(tp) != ASIC_REV_5762)
			val |= DMA_RWCTRL_TAGGED_STAT_WA;
		tw32(TG3PCI_DMA_RW_CTRL, val | tp->dma_rwctrl);
	} else if (tg3_asic_rev(tp) != ASIC_REV_5784 &&
		   tg3_asic_rev(tp) != ASIC_REV_5761) {
		/* This value is determined during the probe time DMA
		 * engine test, tg3_test_dma.
		 */
		tw32(TG3PCI_DMA_RW_CTRL, tp->dma_rwctrl);
	}

	tp->grc_mode &= ~(GRC_MODE_HOST_SENDBDS |
			  GRC_MODE_4X_NIC_SEND_RINGS |
			  GRC_MODE_NO_TX_PHDR_CSUM |
			  GRC_MODE_NO_RX_PHDR_CSUM);
	tp->grc_mode |= GRC_MODE_HOST_SENDBDS;

	/* Pseudo-header checksum is done by hardware logic and not
	 * the offload processers, so make the chip do the pseudo-
	 * header checksums on receive.  For transmit it is more
	 * convenient to do the pseudo-header checksum in software
	 * as Linux does that on transmit for us in all cases.
	 */
	tp->grc_mode |= GRC_MODE_NO_TX_PHDR_CSUM;

	val = GRC_MODE_IRQ_ON_MAC_ATTN | GRC_MODE_HOST_STACKUP;

	tw32(GRC_MODE, tp->grc_mode | val);

	/* Setup the timer prescalar register.  Clock is always 66Mhz. */
	val = tr32(GRC_MISC_CFG);
	val &= ~0xff;
	val |= (65 << GRC_MISC_CFG_PRESCALAR_SHIFT);
	tw32(GRC_MISC_CFG, val);

	/* Initialize MBUF/DESC pool. */
	if (tg3_flag(tp, 5750_PLUS)) {
		/* Do nothing.  */
	} else if (tg3_asic_rev(tp) != ASIC_REV_5705) {
		tw32(BUFMGR_MB_POOL_ADDR, NIC_SRAM_MBUF_POOL_BASE);
		if (tg3_asic_rev(tp) == ASIC_REV_5704)
			tw32(BUFMGR_MB_POOL_SIZE, NIC_SRAM_MBUF_POOL_SIZE64);
		else
			tw32(BUFMGR_MB_POOL_SIZE, NIC_SRAM_MBUF_POOL_SIZE96);
		tw32(BUFMGR_DMA_DESC_POOL_ADDR, NIC_SRAM_DMA_DESC_POOL_BASE);
		tw32(BUFMGR_DMA_DESC_POOL_SIZE, NIC_SRAM_DMA_DESC_POOL_SIZE);
	}

	tw32(BUFMGR_MB_RDMA_LOW_WATER, tp->bufmgr_config.mbuf_read_dma_low_water);
	tw32(BUFMGR_MB_MACRX_LOW_WATER, tp->bufmgr_config.mbuf_mac_rx_low_water);
	tw32(BUFMGR_MB_HIGH_WATER, tp->bufmgr_config.mbuf_high_water);

	tw32(BUFMGR_DMA_LOW_WATER, tp->bufmgr_config.dma_low_water);
	tw32(BUFMGR_DMA_HIGH_WATER, tp->bufmgr_config.dma_high_water);

	val = BUFMGR_MODE_ENABLE | BUFMGR_MODE_ATTN_ENABLE;
	if (tg3_asic_rev(tp) == ASIC_REV_5719)
		val |= BUFMGR_MODE_NO_TX_UNDERRUN;
	if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762 ||
	    tg3_chip_rev_id(tp) == CHIPREV_ID_5719_A0 ||
	    tg3_chip_rev_id(tp) == CHIPREV_ID_5720_A0)
		val |= BUFMGR_MODE_MBLOW_ATTN_ENAB;
	tw32(BUFMGR_MODE, val);
	for (i = 0; i < 2000; i++) {
		if (tr32(BUFMGR_MODE) & BUFMGR_MODE_ENABLE)
			break;
		udelay(10);
	}
	if (i >= 2000) {
		printf("#  %s cannot enable BUFMGR\n", __func__);
		return -ENODEV;
	}

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5906_A1)
		tw32(ISO_PKT_TX, (tr32(ISO_PKT_TX) & ~0x3) | 0x2);

	tg3_setup_rxbd_thresholds(tp);

	/* Initialize TG3_BDINFO's at:
	 *  RCVDBDI_STD_BD:	standard eth size rx ring
	 *  RCVDBDI_JUMBO_BD:	jumbo frame rx ring
	 *  RCVDBDI_MINI_BD:	small frame rx ring (??? does not work)
	 *
	 * like so:
	 *  TG3_BDINFO_HOST_ADDR:	high/low parts of DMA address of ring
	 *  TG3_BDINFO_MAXLEN_FLAGS:	(rx max buffer size << 16) |
	 *                              ring attribute flags
	 *  TG3_BDINFO_NIC_ADDR:	location of descriptors in nic SRAM
	 *
	 * Standard receive ring @ NIC_SRAM_RX_BUFFER_DESC, 512 entries.
	 * Jumbo receive ring @ NIC_SRAM_RX_JUMBO_BUFFER_DESC, 256 entries.
	 *
	 * The size of each ring is fixed in the firmware, but the location is
	 * configurable.
	 */
	tw32(RCVDBDI_STD_BD + TG3_BDINFO_HOST_ADDR + TG3_64BIT_REG_HIGH,
	     ((u64) tpr->rx_std_mapping >> 32));
	tw32(RCVDBDI_STD_BD + TG3_BDINFO_HOST_ADDR + TG3_64BIT_REG_LOW,
	     ((u64) tpr->rx_std_mapping & 0xffffffff));
	if (!tg3_flag(tp, 5717_PLUS))
		tw32(RCVDBDI_STD_BD + TG3_BDINFO_NIC_ADDR,
		     NIC_SRAM_RX_BUFFER_DESC);

	/* Disable the mini ring */
	if (!tg3_flag(tp, 5705_PLUS))
		tw32(RCVDBDI_MINI_BD + TG3_BDINFO_MAXLEN_FLAGS,
		     BDINFO_FLAGS_DISABLED);

	/* Program the jumbo buffer descriptor ring control
	 * blocks on those devices that have them.
	 */
	/* Disable jumbo */
	tw32(RCVDBDI_JUMBO_BD + TG3_BDINFO_MAXLEN_FLAGS, BDINFO_FLAGS_DISABLED);

	val  = TG3_RX_PRODUCER_RING_SIZE << BDINFO_FLAGS_MAXLEN_SHIFT;
	val |= TG3_RX_FRAME_SIZE << 2; /* Packet size */

	tw32(RCVDBDI_STD_BD + TG3_BDINFO_MAXLEN_FLAGS, val);

	tw32_rx_mbox(TG3_RX_STD_PROD_IDX_REG, 0);

	tg3_rings_reset(tp);

	/* Initialize MAC address and backoff seed. */
	__tg3_set_mac_addr(tp, false);

	/* MTU + ethernet header + FCS + optional VLAN tag */
	tw32(MAC_RX_MTU_SIZE, TG3_RX_FRAME_SIZE);

	/* The slot time is changed by tg3_setup_phy if we
	 * run at gigabit with half duplex.
	 */
	val = (2 << TX_LENGTHS_IPG_CRS_SHIFT) |
	      (6 << TX_LENGTHS_IPG_SHIFT) |
	      (32 << TX_LENGTHS_SLOT_TIME_SHIFT);

	if (tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762)
		val |= tr32(MAC_TX_LENGTHS) &
		       (TX_LENGTHS_JMB_FRM_LEN_MSK |
			TX_LENGTHS_CNT_DWN_VAL_MSK);

	tw32(MAC_TX_LENGTHS, val);

	/* Receive rules. */
	tw32(MAC_RCV_RULE_CFG, RCV_RULE_CFG_DEFAULT_CLASS);
	tw32(RCVLPC_CONFIG, 0x0181);

	/* Calculate RDMAC_MODE setting early, we need it to determine
	 * the RCVLPC_STATE_ENABLE mask.
	 */
	rdmac_mode = (RDMAC_MODE_ENABLE | RDMAC_MODE_TGTABORT_ENAB |
		      RDMAC_MODE_MSTABORT_ENAB | RDMAC_MODE_PARITYERR_ENAB |
		      RDMAC_MODE_ADDROFLOW_ENAB | RDMAC_MODE_FIFOOFLOW_ENAB |
		      RDMAC_MODE_FIFOURUN_ENAB | RDMAC_MODE_FIFOOREAD_ENAB |
		      RDMAC_MODE_LNGREAD_ENAB);

	if (tg3_asic_rev(tp) == ASIC_REV_5717)
		rdmac_mode |= RDMAC_MODE_MULT_DMA_RD_DIS;

	if (tg3_asic_rev(tp) == ASIC_REV_5784 ||
	    tg3_asic_rev(tp) == ASIC_REV_5785 ||
	    tg3_asic_rev(tp) == ASIC_REV_57780)
		rdmac_mode |= RDMAC_MODE_BD_SBD_CRPT_ENAB |
			      RDMAC_MODE_MBUF_RBD_CRPT_ENAB |
			      RDMAC_MODE_MBUF_SBD_CRPT_ENAB;

	if (tg3_asic_rev(tp) == ASIC_REV_5705 &&
	    tg3_chip_rev_id(tp) != CHIPREV_ID_5705_A0) {
		if (!(tr32(TG3PCI_PCISTATE) & PCISTATE_BUS_SPEED_HIGH) &&
		     !tg3_flag(tp, IS_5788)) {
			rdmac_mode |= RDMAC_MODE_FIFO_LONG_BURST;
		}
	}

	if (tg3_flag(tp, 57765_PLUS) ||
	    tg3_asic_rev(tp) == ASIC_REV_5785 ||
	    tg3_asic_rev(tp) == ASIC_REV_57780)
		rdmac_mode |= RDMAC_MODE_IPV6_LSO_EN;

	if (tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762)
		rdmac_mode |= tr32(RDMAC_MODE) & RDMAC_MODE_H2BNC_VLAN_DET;

	if (tg3_asic_rev(tp) == ASIC_REV_5761 ||
	    tg3_asic_rev(tp) == ASIC_REV_5784 ||
	    tg3_asic_rev(tp) == ASIC_REV_5785 ||
	    tg3_asic_rev(tp) == ASIC_REV_57780 ||
	    tg3_flag(tp, 57765_PLUS)) {
		u32 tgtreg;

		if (tg3_asic_rev(tp) == ASIC_REV_5762)
			tgtreg = TG3_RDMA_RSRVCTRL_REG2;
		else
			tgtreg = TG3_RDMA_RSRVCTRL_REG;

		val = tr32(tgtreg);
		if (tg3_chip_rev_id(tp) == CHIPREV_ID_5719_A0 ||
		    tg3_asic_rev(tp) == ASIC_REV_5762) {
			val &= ~(TG3_RDMA_RSRVCTRL_TXMRGN_MASK |
				 TG3_RDMA_RSRVCTRL_FIFO_LWM_MASK |
				 TG3_RDMA_RSRVCTRL_FIFO_HWM_MASK);
			val |= TG3_RDMA_RSRVCTRL_TXMRGN_320B |
			       TG3_RDMA_RSRVCTRL_FIFO_LWM_1_5K |
			       TG3_RDMA_RSRVCTRL_FIFO_HWM_1_5K;
		}
		tw32(tgtreg, val | TG3_RDMA_RSRVCTRL_FIFO_OFLW_FIX);
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5719 ||
	    tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762) {
		u32 tgtreg;

		if (tg3_asic_rev(tp) == ASIC_REV_5762)
			tgtreg = TG3_LSO_RD_DMA_CRPTEN_CTRL2;
		else
			tgtreg = TG3_LSO_RD_DMA_CRPTEN_CTRL;

		val = tr32(tgtreg);
		tw32(tgtreg, val |
		     TG3_LSO_RD_DMA_CRPTEN_CTRL_BLEN_BD_4K |
		     TG3_LSO_RD_DMA_CRPTEN_CTRL_BLEN_LSO_4K);
	}

	/* Receive/send statistics. */
	if (tg3_flag(tp, 5750_PLUS)) {
		val = tr32(RCVLPC_STATS_ENABLE);
		val &= ~RCVLPC_STATSENAB_DACK_FIX;
		tw32(RCVLPC_STATS_ENABLE, val);
	} else {
		tw32(RCVLPC_STATS_ENABLE, 0xffffff);
	}
	tw32(RCVLPC_STATSCTRL, RCVLPC_STATSCTRL_ENABLE);
	tw32(SNDDATAI_STATSENAB, 0xffffff);
	tw32(SNDDATAI_STATSCTRL,
	     (SNDDATAI_SCTRL_ENABLE |
	      SNDDATAI_SCTRL_FASTUPD));

	/* Setup host coalescing engine. */
	tw32(HOSTCC_MODE, 0);
	for (i = 0; i < 2000; i++) {
		if (!(tr32(HOSTCC_MODE) & HOSTCC_MODE_ENABLE))
			break;
		udelay(10);
	}

	__tg3_set_coalesce(tp);

	if (!tg3_flag(tp, 5705_PLUS)) {
		/* Status/statistics block address.  See tg3_timer,
		 * the tg3_periodic_fetch_stats call there, and
		 * tg3_get_stats to see how this works for 5705/5750 chips.
		 */
		tw32(HOSTCC_STATS_BLK_HOST_ADDR + TG3_64BIT_REG_HIGH,
		     ((u64) tp->stats_mapping >> 32));
		tw32(HOSTCC_STATS_BLK_HOST_ADDR + TG3_64BIT_REG_LOW,
		     ((u64) tp->stats_mapping & 0xffffffff));
		tw32(HOSTCC_STATS_BLK_NIC_ADDR, NIC_SRAM_STATS_BLK);

		tw32(HOSTCC_STATUS_BLK_NIC_ADDR, NIC_SRAM_STATUS_BLK);

		/* Clear statistics and status block memory areas */
		for (i = NIC_SRAM_STATS_BLK;
		     i < NIC_SRAM_STATUS_BLK + TG3_HW_STATUS_SIZE;
		     i += sizeof(u32)) {
			tg3_write_mem(tp, i, 0);
			udelay(40);
		}
	}

	tw32(HOSTCC_MODE, HOSTCC_MODE_ENABLE | tp->coalesce_mode);

	tw32(RCVCC_MODE, RCVCC_MODE_ENABLE | RCVCC_MODE_ATTN_ENABLE);
	tw32(RCVLPC_MODE, RCVLPC_MODE_ENABLE);
	if (!tg3_flag(tp, 5705_PLUS))
		tw32(RCVLSC_MODE, RCVLSC_MODE_ENABLE | RCVLSC_MODE_ATTN_ENABLE);

	if (tp->phy_flags & TG3_PHYFLG_MII_SERDES) {
		tp->phy_flags &= ~TG3_PHYFLG_PARALLEL_DETECT;
		/* reset to prevent losing 1st rx packet intermittently */
		tw32_f(MAC_RX_MODE, RX_MODE_RESET);
		udelay(10);
	}

	tp->mac_mode |= MAC_MODE_TXSTAT_ENABLE | MAC_MODE_RXSTAT_ENABLE |
			MAC_MODE_TDE_ENABLE | MAC_MODE_RDE_ENABLE |
			MAC_MODE_FHDE_ENABLE;

	if (!tg3_flag(tp, 5705_PLUS) &&
	    !(tp->phy_flags & TG3_PHYFLG_PHY_SERDES) &&
	    tg3_asic_rev(tp) != ASIC_REV_5700)
		tp->mac_mode |= MAC_MODE_LINK_POLARITY;
	tw32_f(MAC_MODE, tp->mac_mode | MAC_MODE_RXSTAT_CLEAR | MAC_MODE_TXSTAT_CLEAR);
	udelay(40);

	/* tp->grc_local_ctrl is partially set up during tg3_get_invariants().
	 * If TG3_FLAG_IS_NIC is zero, we should read the
	 * register to preserve the GPIO settings for LOMs. The GPIOs,
	 * whether used as inputs or outputs, are set by boot code after
	 * reset.
	 */
	if (!tg3_flag(tp, IS_NIC)) {
		u32 gpio_mask;

		gpio_mask = GRC_LCLCTRL_GPIO_OE0 | GRC_LCLCTRL_GPIO_OE1 |
			    GRC_LCLCTRL_GPIO_OE2 | GRC_LCLCTRL_GPIO_OUTPUT0 |
			    GRC_LCLCTRL_GPIO_OUTPUT1 | GRC_LCLCTRL_GPIO_OUTPUT2;

		if (tg3_asic_rev(tp) == ASIC_REV_5752)
			gpio_mask |= GRC_LCLCTRL_GPIO_OE3 |
				     GRC_LCLCTRL_GPIO_OUTPUT3;

		if (tg3_asic_rev(tp) == ASIC_REV_5755)
			gpio_mask |= GRC_LCLCTRL_GPIO_UART_SEL;

		tp->grc_local_ctrl &= ~gpio_mask;
		tp->grc_local_ctrl |= tr32(GRC_LOCAL_CTRL) & gpio_mask;

		/* GPIO1 must be driven high for eeprom write protect */
		if (tg3_flag(tp, EEPROM_WRITE_PROT))
			tp->grc_local_ctrl |= (GRC_LCLCTRL_GPIO_OE1 |
					       GRC_LCLCTRL_GPIO_OUTPUT1);
	}
	tw32_f(GRC_LOCAL_CTRL, tp->grc_local_ctrl);
	udelay(100);

	if (!tg3_flag(tp, 5705_PLUS)) {
		tw32_f(DMAC_MODE, DMAC_MODE_ENABLE);
		udelay(40);
	}

	val = (WDMAC_MODE_ENABLE | WDMAC_MODE_TGTABORT_ENAB |
	       WDMAC_MODE_MSTABORT_ENAB | WDMAC_MODE_PARITYERR_ENAB |
	       WDMAC_MODE_ADDROFLOW_ENAB | WDMAC_MODE_FIFOOFLOW_ENAB |
	       WDMAC_MODE_FIFOURUN_ENAB | WDMAC_MODE_FIFOOREAD_ENAB |
	       WDMAC_MODE_LNGREAD_ENAB);

	if (tg3_asic_rev(tp) == ASIC_REV_5705 &&
	    tg3_chip_rev_id(tp) != CHIPREV_ID_5705_A0) {
		if (!(tr32(TG3PCI_PCISTATE) & PCISTATE_BUS_SPEED_HIGH)
			&& !tg3_flag(tp, IS_5788)) {
			val |= WDMAC_MODE_RX_ACCEL;
		}
	}

	/* Enable host coalescing bug fix */
	if (tg3_flag(tp, 5755_PLUS))
		val |= WDMAC_MODE_STATUS_TAG_FIX;

	if (tg3_asic_rev(tp) == ASIC_REV_5785)
		val |= WDMAC_MODE_BURST_ALL_DATA;

	tw32_f(WDMAC_MODE, val);
	udelay(40);

	tw32_f(RDMAC_MODE, rdmac_mode);
	udelay(40);

	if (tg3_asic_rev(tp) == ASIC_REV_5719 ||
	    tg3_asic_rev(tp) == ASIC_REV_5720) {
		for (i = 0; i < TG3_NUM_RDMA_CHANNELS; i++) {
			if (tr32(TG3_RDMA_LENGTH + (i << 2)) > TG3_MAX_MTU(tp))
				break;
		}
		if (i < TG3_NUM_RDMA_CHANNELS) {
			val = tr32(TG3_LSO_RD_DMA_CRPTEN_CTRL);
			val |= tg3_lso_rd_dma_workaround_bit(tp);
			tw32(TG3_LSO_RD_DMA_CRPTEN_CTRL, val);
			tg3_flag_set(tp, 5719_5720_RDMA_BUG);
		}
	}

	tw32(RCVDCC_MODE, RCVDCC_MODE_ENABLE | RCVDCC_MODE_ATTN_ENABLE);
	if (!tg3_flag(tp, 5705_PLUS))
		tw32(MBFREE_MODE, MBFREE_MODE_ENABLE);

	if (tg3_asic_rev(tp) == ASIC_REV_5761)
		tw32(SNDDATAC_MODE,
		     SNDDATAC_MODE_ENABLE | SNDDATAC_MODE_CDELAY);
	else
		tw32(SNDDATAC_MODE, SNDDATAC_MODE_ENABLE);

	tw32(SNDBDC_MODE, SNDBDC_MODE_ENABLE | SNDBDC_MODE_ATTN_ENABLE);
	tw32(RCVBDI_MODE, RCVBDI_MODE_ENABLE | RCVBDI_MODE_RCB_ATTN_ENAB);
	val = RCVDBDI_MODE_ENABLE | RCVDBDI_MODE_INV_RING_SZ;
	if (tg3_flag(tp, LRG_PROD_RING_CAP))
		val |= RCVDBDI_MODE_LRG_RING_SZ;
	tw32(RCVDBDI_MODE, val);
	tw32(SNDDATAI_MODE, SNDDATAI_MODE_ENABLE);

	val = SNDBDI_MODE_ENABLE | SNDBDI_MODE_ATTN_ENABLE;
	tw32(SNDBDI_MODE, val);
	tw32(SNDBDS_MODE, SNDBDS_MODE_ENABLE | SNDBDS_MODE_ATTN_ENABLE);

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5701_A0) {
		printf("#   Note: Not loading firmware fix.\n");
	}

	tp->tx_mode = TX_MODE_ENABLE;

	if (tg3_flag(tp, 5755_PLUS) ||
	    tg3_asic_rev(tp) == ASIC_REV_5906)
		tp->tx_mode |= TX_MODE_MBUF_LOCKUP_FIX;

	if (tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762) {
		val = TX_MODE_JMB_FRM_LEN | TX_MODE_CNT_DN_MODE;
		tp->tx_mode &= ~val;
		tp->tx_mode |= tr32(MAC_TX_MODE) & val;
	}

	tw32_f(MAC_TX_MODE, tp->tx_mode);
	udelay(100);

	tp->rx_mode = RX_MODE_ENABLE;
	if (tg3_flag(tp, 5755_PLUS))
		tp->rx_mode |= RX_MODE_IPV6_CSUM_ENABLE;

	if (tg3_asic_rev(tp) == ASIC_REV_5762)
		tp->rx_mode |= RX_MODE_IPV4_FRAG_FIX;

	tw32_f(MAC_RX_MODE, tp->rx_mode);
	udelay(10);

	tw32(MAC_LED_CTRL, tp->led_ctrl);

	tw32(MAC_MI_STAT, MAC_MI_STAT_LNKSTAT_ATTN_ENAB);
	if (tp->phy_flags & TG3_PHYFLG_PHY_SERDES) {
		tw32_f(MAC_RX_MODE, RX_MODE_RESET);
		udelay(10);
	}
	tw32_f(MAC_RX_MODE, tp->rx_mode);
	udelay(10);

	if (tp->phy_flags & TG3_PHYFLG_PHY_SERDES) {
		if ((tg3_asic_rev(tp) == ASIC_REV_5704) &&
		    !(tp->phy_flags & TG3_PHYFLG_SERDES_PREEMPHASIS)) {
			/* Set drive transmission level to 1.2V  */
			/* only if the signal pre-emphasis bit is not set  */
			val = tr32(MAC_SERDES_CFG);
			val &= 0xfffff000;
			val |= 0x880;
			tw32(MAC_SERDES_CFG, val);
		}
		if (tg3_chip_rev_id(tp) == CHIPREV_ID_5703_A1)
			tw32(MAC_SERDES_CFG, 0x616000);
	}

	/* Prevent chip from dropping frames when flow control
	 * is enabled.
	 */
	if (tg3_flag(tp, 57765_CLASS))
		val = 1;
	else
		val = 2;
	tw32_f(MAC_LOW_WMARK_MAX_RX_FRAME, val);

	if (tg3_asic_rev(tp) == ASIC_REV_5704 &&
	    (tp->phy_flags & TG3_PHYFLG_PHY_SERDES)) {
		/* Use hardware link auto-negotiation */
		tg3_flag_set(tp, HW_AUTONEG);
	}

	if ((tp->phy_flags & TG3_PHYFLG_MII_SERDES) &&
	    tg3_asic_rev(tp) == ASIC_REV_5714) {
		u32 tmp;

		tmp = tr32(SERDES_RX_CTRL);
		tw32(SERDES_RX_CTRL, tmp | SERDES_RX_SIG_DETECT);
		tp->grc_local_ctrl &= ~GRC_LCLCTRL_USE_EXT_SIG_DETECT;
		tp->grc_local_ctrl |= GRC_LCLCTRL_USE_SIG_DETECT;
		tw32(GRC_LOCAL_CTRL, tp->grc_local_ctrl);
	}


	if (tp->phy_flags & TG3_PHYFLG_IS_LOW_POWER)
		tp->phy_flags &= ~TG3_PHYFLG_IS_LOW_POWER;

	err = tg3_setup_phy(tp, false);
	if (err)
		return err;

	if (!(tp->phy_flags & TG3_PHYFLG_PHY_SERDES) &&
	    !(tp->phy_flags & TG3_PHYFLG_IS_FET)) {
		u32 tmp;

		/* Clear CRC stats. */
		if (!tg3_readphy(tp, MII_TG3_TEST1, &tmp)) {
			tg3_writephy(tp, MII_TG3_TEST1, tmp | MII_TG3_TEST1_CRC_EN);
			tg3_readphy(tp, MII_TG3_RXR_COUNTERS, &tmp);
		}
	}

	__tg3_set_rx_mode(tp);

	/* Initialize receive rules. */
	tw32(MAC_RCV_RULE_0,  0xc2000000 & RCV_RULE_DISABLE_MASK);
	tw32(MAC_RCV_VALUE_0, 0xffffffff & RCV_RULE_DISABLE_MASK);
	tw32(MAC_RCV_RULE_1,  0x86000004 & RCV_RULE_DISABLE_MASK);
	tw32(MAC_RCV_VALUE_1, 0xffffffff & RCV_RULE_DISABLE_MASK);

	if (tg3_flag(tp, 5705_PLUS) && !tg3_flag(tp, 5780_CLASS))
		limit = 8;
	else
		limit = 16;
	switch (limit) {
	case 16:
		tw32(MAC_RCV_RULE_15,  0); tw32(MAC_RCV_VALUE_15,  0);
	case 15:
		tw32(MAC_RCV_RULE_14,  0); tw32(MAC_RCV_VALUE_14,  0);
	case 14:
		tw32(MAC_RCV_RULE_13,  0); tw32(MAC_RCV_VALUE_13,  0);
	case 13:
		tw32(MAC_RCV_RULE_12,  0); tw32(MAC_RCV_VALUE_12,  0);
	case 12:
		tw32(MAC_RCV_RULE_11,  0); tw32(MAC_RCV_VALUE_11,  0);
	case 11:
		tw32(MAC_RCV_RULE_10,  0); tw32(MAC_RCV_VALUE_10,  0);
	case 10:
		tw32(MAC_RCV_RULE_9,  0); tw32(MAC_RCV_VALUE_9,  0);
	case 9:
		tw32(MAC_RCV_RULE_8,  0); tw32(MAC_RCV_VALUE_8,  0);
	case 8:
		tw32(MAC_RCV_RULE_7,  0); tw32(MAC_RCV_VALUE_7,  0);
	case 7:
		tw32(MAC_RCV_RULE_6,  0); tw32(MAC_RCV_VALUE_6,  0);
	case 6:
		tw32(MAC_RCV_RULE_5,  0); tw32(MAC_RCV_VALUE_5,  0);
	case 5:
		tw32(MAC_RCV_RULE_4,  0); tw32(MAC_RCV_VALUE_4,  0);
	default:
		break;
	}

	return 0;
}

/* Called at device open time to get the chip ready for
 * packet processing.  Invoked with tp->lock held.
 */
static int tg3_init_hw(struct tg3_priv *tp, bool reset_phy)
{
	/* Chip may have been just powered on. If so, the boot code may still
	 * be running initialization. Wait for it to finish to avoid races in
	 * accessing the hardware.
	 */
	tg3_enable_register_access(tp);
	tg3_poll_fw(tp);

	tg3_switch_clocks(tp);

	tw32(TG3PCI_MEM_WIN_BASE_ADDR, 0);

	return tg3_reset_hw(tp, reset_phy);
}

static int tg3_start(struct tg3_priv *tp, bool reset_phy, bool init)
{
	int err;

	/* The placement of this call is tied to the setup and use of Host TX
	 * descriptors.
	 */
	err = tg3_alloc_consistent(tp);
	if (err)
		goto out_ints_fini;

	err = tg3_init_hw(tp, reset_phy);
	if (err) {
		printf("# Error: line %d.\n", __LINE__);
		tg3_halt__(tp, RESET_KIND_SHUTDOWN, 1);
		tg3_free_rings(tp);
	}

	tg3_flag_set(tp, INIT_COMPLETE);

	return 0;

out_ints_fini:

	return err;
}

void tg3_halt(struct eth_device *dev)
{
	/* Once the card is initialized,
	 * we will leave it initialized. */
	return;
}

static void tg3_get_eeprom_hw_cfg(struct tg3_priv *tp)
{
	u32 val;

	tp->phy_id = TG3_PHY_ID_INVALID;
	tp->led_ctrl = LED_CTRL_MODE_PHY_1;

	/* Assume an onboard device and WOL capable by default.  */
	tg3_flag_set(tp, EEPROM_WRITE_PROT);

	if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		if (!(tr32(PCIE_TRANSACTION_CFG) & PCIE_TRANS_CFG_LOM)) {
			tg3_flag_clear(tp, EEPROM_WRITE_PROT);
			tg3_flag_set(tp, IS_NIC);
		}
		val = tr32(VCPU_CFGSHDW);
		if (val & VCPU_CFGSHDW_ASPM_DBNC)
			tg3_flag_set(tp, ASPM_WORKAROUND);
		goto done;
	}

	tg3_read_mem(tp, NIC_SRAM_DATA_SIG, &val);
	if (val == NIC_SRAM_DATA_SIG_MAGIC) {
		u32 nic_cfg, led_cfg;
		u32 cfg2 = 0, cfg4 = 0, cfg5 = 0;
		u32 nic_phy_id, ver, eeprom_phy_id;
		int eeprom_phy_serdes = 0;

		tg3_read_mem(tp, NIC_SRAM_DATA_CFG, &nic_cfg);
		tp->nic_sram_data_cfg = nic_cfg;

		tg3_read_mem(tp, NIC_SRAM_DATA_VER, &ver);
		ver >>= NIC_SRAM_DATA_VER_SHIFT;
		if (tg3_asic_rev(tp) != ASIC_REV_5700 &&
		    tg3_asic_rev(tp) != ASIC_REV_5701 &&
		    tg3_asic_rev(tp) != ASIC_REV_5703 &&
		    (ver > 0) && (ver < 0x100))
			tg3_read_mem(tp, NIC_SRAM_DATA_CFG_2, &cfg2);

		if (tg3_asic_rev(tp) == ASIC_REV_5785)
			tg3_read_mem(tp, NIC_SRAM_DATA_CFG_4, &cfg4);

		if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
		    tg3_asic_rev(tp) == ASIC_REV_5719 ||
		    tg3_asic_rev(tp) == ASIC_REV_5720)
			tg3_read_mem(tp, NIC_SRAM_DATA_CFG_5, &cfg5);

		if ((nic_cfg & NIC_SRAM_DATA_CFG_PHY_TYPE_MASK) ==
		    NIC_SRAM_DATA_CFG_PHY_TYPE_FIBER)
			eeprom_phy_serdes = 1;

		tg3_read_mem(tp, NIC_SRAM_DATA_PHY_ID, &nic_phy_id);
		if (nic_phy_id != 0) {
			u32 id1 = nic_phy_id & NIC_SRAM_DATA_PHY_ID1_MASK;
			u32 id2 = nic_phy_id & NIC_SRAM_DATA_PHY_ID2_MASK;

			eeprom_phy_id  = (id1 >> 16) << 10;
			eeprom_phy_id |= (id2 & 0xfc00) << 16;
			eeprom_phy_id |= (id2 & 0x03ff) <<  0;
		} else {
			eeprom_phy_id = 0;
		}

		tp->phy_id = eeprom_phy_id;
		if (eeprom_phy_serdes) {
			if (!tg3_flag(tp, 5705_PLUS))
				tp->phy_flags |= TG3_PHYFLG_PHY_SERDES;
			else
				tp->phy_flags |= TG3_PHYFLG_MII_SERDES;
		}

		if (tg3_flag(tp, 5750_PLUS))
			led_cfg = cfg2 & (NIC_SRAM_DATA_CFG_LED_MODE_MASK |
				    SHASTA_EXT_LED_MODE_MASK);
		else
			led_cfg = nic_cfg & NIC_SRAM_DATA_CFG_LED_MODE_MASK;

		switch (led_cfg) {
		default:
		case NIC_SRAM_DATA_CFG_LED_MODE_PHY_1:
			tp->led_ctrl = LED_CTRL_MODE_PHY_1;
			break;

		case NIC_SRAM_DATA_CFG_LED_MODE_PHY_2:
			tp->led_ctrl = LED_CTRL_MODE_PHY_2;
			break;

		case NIC_SRAM_DATA_CFG_LED_MODE_MAC:
			tp->led_ctrl = LED_CTRL_MODE_MAC;

			/* Default to PHY_1_MODE if 0 (MAC_MODE) is
			 * read on some older 5700/5701 bootcode.
			 */
			if (tg3_asic_rev(tp) == ASIC_REV_5700 ||
			    tg3_asic_rev(tp) == ASIC_REV_5701)
				tp->led_ctrl = LED_CTRL_MODE_PHY_1;

			break;

		case SHASTA_EXT_LED_SHARED:
			tp->led_ctrl = LED_CTRL_MODE_SHARED;
			if (tg3_chip_rev_id(tp) != CHIPREV_ID_5750_A0 &&
			    tg3_chip_rev_id(tp) != CHIPREV_ID_5750_A1)
				tp->led_ctrl |= (LED_CTRL_MODE_PHY_1 |
						 LED_CTRL_MODE_PHY_2);

			if (tg3_flag(tp, 5717_PLUS) ||
			    tg3_asic_rev(tp) == ASIC_REV_5762)
				tp->led_ctrl |= LED_CTRL_BLINK_RATE_OVERRIDE |
						LED_CTRL_BLINK_RATE_MASK;

			break;

		case SHASTA_EXT_LED_MAC:
			tp->led_ctrl = LED_CTRL_MODE_SHASTA_MAC;
			break;

		case SHASTA_EXT_LED_COMBO:
			tp->led_ctrl = LED_CTRL_MODE_COMBO;
			if (tg3_chip_rev_id(tp) != CHIPREV_ID_5750_A0)
				tp->led_ctrl |= (LED_CTRL_MODE_PHY_1 |
						 LED_CTRL_MODE_PHY_2);
			break;
		}

		if (tg3_chip_rev(tp) == CHIPREV_5784_AX)
			tp->led_ctrl = LED_CTRL_MODE_PHY_1;

		if (nic_cfg & NIC_SRAM_DATA_CFG_EEPROM_WP) {
			tg3_flag_set(tp, EEPROM_WRITE_PROT);
		} else {
			tg3_flag_clear(tp, EEPROM_WRITE_PROT);
			tg3_flag_set(tp, IS_NIC);
		}

		if (nic_cfg & NIC_SRAM_DATA_CFG_ASF_ENABLE) {
			printf("#  Note: ASF not implemented.\n");
		}

		if (cfg2 & (1 << 17))
			tp->phy_flags |= TG3_PHYFLG_CAPACITIVE_COUPLING;

		/* serdes signal pre-emphasis in register 0x590 set by */
		/* bootcode if bit 18 is set */
		if (cfg2 & (1 << 18))
			tp->phy_flags |= TG3_PHYFLG_SERDES_PREEMPHASIS;

		if ((tg3_flag(tp, 57765_PLUS) ||
		     (tg3_asic_rev(tp) == ASIC_REV_5784 &&
		      tg3_chip_rev(tp) != CHIPREV_5784_AX)) &&
		    (cfg2 & NIC_SRAM_DATA_CFG_2_APD_EN))
			tp->phy_flags |= TG3_PHYFLG_ENABLE_APD;

		if (cfg4 & NIC_SRAM_RGMII_INBAND_DISABLE)
			tg3_flag_set(tp, RGMII_INBAND_DISABLE);
		if (cfg4 & NIC_SRAM_RGMII_EXT_IBND_RX_EN)
			tg3_flag_set(tp, RGMII_EXT_IBND_RX_EN);
		if (cfg4 & NIC_SRAM_RGMII_EXT_IBND_TX_EN)
			tg3_flag_set(tp, RGMII_EXT_IBND_TX_EN);

		if (cfg5 & NIC_SRAM_DISABLE_1G_HALF_ADV)
			tp->phy_flags |= TG3_PHYFLG_DISABLE_1G_HD_ADV;
	}
done:
	return;
}

static int tg3_issue_otp_command(struct tg3_priv *tp, u32 cmd)
{
	int i;
	u32 val;

	tw32(OTP_CTRL, cmd | OTP_CTRL_OTP_CMD_START);
	tw32(OTP_CTRL, cmd);

	/* Wait for up to 1 ms for command to execute. */
	for (i = 0; i < 100; i++) {
		val = tr32(OTP_STATUS);
		if (val & OTP_STATUS_CMD_DONE)
			break;
		udelay(10);
	}

	return (val & OTP_STATUS_CMD_DONE) ? 0 : -EBUSY;
}

/* Read the gphy configuration from the OTP region of the chip.  The gphy
 * configuration is a 32-bit value that straddles the alignment boundary.
 * We do two 32-bit reads and then shift and merge the results.
 */
static u32 tg3_read_otp_phycfg(struct tg3_priv *tp)
{
	u32 bhalf_otp, thalf_otp;

	tw32(OTP_MODE, OTP_MODE_OTP_THRU_GRC);

	if (tg3_issue_otp_command(tp, OTP_CTRL_OTP_CMD_INIT))
		return 0;

	tw32(OTP_ADDRESS, OTP_ADDRESS_MAGIC1);

	if (tg3_issue_otp_command(tp, OTP_CTRL_OTP_CMD_READ))
		return 0;

	thalf_otp = tr32(OTP_READ_DATA);

	tw32(OTP_ADDRESS, OTP_ADDRESS_MAGIC2);

	if (tg3_issue_otp_command(tp, OTP_CTRL_OTP_CMD_READ))
		return 0;

	bhalf_otp = tr32(OTP_READ_DATA);

	return ((thalf_otp & 0x0000ffff) << 16) | (bhalf_otp >> 16);
}

static void tg3_phy_init_link_config(struct tg3_priv *tp)
{
	u32 adv = ADVERTISED_Autoneg;

	if (!(tp->phy_flags & TG3_PHYFLG_10_100_ONLY)) {
		if (!(tp->phy_flags & TG3_PHYFLG_DISABLE_1G_HD_ADV))
			adv |= ADVERTISED_1000baseT_Half;
			adv |= ADVERTISED_1000baseT_Full;
	}

	if (!(tp->phy_flags & TG3_PHYFLG_ANY_SERDES))
		adv |= ADVERTISED_100baseT_Half |
		       ADVERTISED_100baseT_Full |
		       ADVERTISED_10baseT_Half |
		       ADVERTISED_10baseT_Full |
		       ADVERTISED_TP;
	else
		adv |= ADVERTISED_FIBRE;

	tp->link_config.advertising = adv;
	tp->link_config.speed = SPEED_UNKNOWN;
	tp->link_config.duplex = DUPLEX_UNKNOWN;
	tp->link_config.autoneg = AUTONEG_ENABLE;
	tp->link_config.active_speed = SPEED_UNKNOWN;
	tp->link_config.active_duplex = DUPLEX_UNKNOWN;
}

static int tg3_phy_probe(struct tg3_priv *tp)
{
	u32 hw_phy_id_1, hw_phy_id_2;
	u32 hw_phy_id, hw_phy_id_masked;
	int err;

	if (!(tp->phy_flags & TG3_PHYFLG_ANY_SERDES) &&
	    !(tp->phy_flags & TG3_PHYFLG_10_100_ONLY))
		tp->phy_flags &= ~(TG3_PHYFLG_1G_ON_VAUX_OK |
				   TG3_PHYFLG_KEEP_LINK_ON_PWRDN);

	/* Reading the PHY ID register can conflict with ASF
	 * firmware access to the PHY hardware.
	 */
	err = 0;
	/* Now read the physical PHY_ID from the chip and verify
	 * that it is sane.  If it doesn't look good, we fall back
	 * to either the hard-coded table based PHY_ID and failing
	 * that the value found in the eeprom area.
	 */
	err |= tg3_readphy(tp, MII_PHYSID1, &hw_phy_id_1);
	err |= tg3_readphy(tp, MII_PHYSID2, &hw_phy_id_2);

	hw_phy_id  = (hw_phy_id_1 & 0xffff) << 10;
	hw_phy_id |= (hw_phy_id_2 & 0xfc00) << 16;
	hw_phy_id |= (hw_phy_id_2 & 0x03ff) <<  0;

	hw_phy_id_masked = hw_phy_id & TG3_PHY_ID_MASK;

	if (!err && TG3_KNOWN_PHY_ID(hw_phy_id_masked)) {
		tp->phy_id = hw_phy_id;
		if (hw_phy_id_masked == TG3_PHY_ID_BCM8002)
			tp->phy_flags |= TG3_PHYFLG_PHY_SERDES;
		else
			tp->phy_flags &= ~TG3_PHYFLG_PHY_SERDES;
	} else {
		if (tp->phy_id != TG3_PHY_ID_INVALID) {
			/* Do nothing, phy ID already set up in
			 * tg3_get_eeprom_hw_cfg().
			 */
		} else {
			printf("# Tigon3 subsys vendor code not implemented.\n");
			return -ENODEV;
		}
	}

	tg3_phy_init_link_config(tp);

	if (!(tp->phy_flags & TG3_PHYFLG_KEEP_LINK_ON_PWRDN) &&
	    !(tp->phy_flags & TG3_PHYFLG_ANY_SERDES)) {
		u32 bmsr, dummy;

		tg3_readphy(tp, MII_BMSR, &bmsr);
		if (!tg3_readphy(tp, MII_BMSR, &bmsr) &&
		    (bmsr & BMSR_LSTATUS))
			goto skip_phy_reset;

		err = tg3_phy_reset(tp);
		if (err)
			return err;

		tg3_phy_set_wirespeed(tp);

		if (!tg3_phy_copper_an_config_ok(tp, &dummy)) {
			tg3_phy_autoneg_cfg(tp, tp->link_config.advertising);

			tg3_writephy(tp, MII_BMCR,
				     BMCR_ANENABLE | BMCR_ANRESTART);
		}
	}

skip_phy_reset:
	if ((tp->phy_id & TG3_PHY_ID_MASK) == TG3_PHY_ID_BCM5401) {
		err = tg3_init_5401phy_dsp(tp);
		if (err)
			return err;

		err = tg3_init_5401phy_dsp(tp);
	}

	return err;
}

static void tg3_detect_asic_rev(struct tg3_priv *tp, u32 misc_ctrl_reg)
{
	tp->pci_chip_rev_id = misc_ctrl_reg >> MISC_HOST_CTRL_CHIPREV_SHIFT;
	if (tg3_asic_rev(tp) == ASIC_REV_USE_PROD_ID_REG) {
		u32 reg;

		/* All devices that use the alternate
		 * ASIC REV location have a CPMU.
		 */
		tg3_flag_set(tp, CPMU_PRESENT);

		if (tp->pci_device == TG3PCI_DEVICE_TIGON3_5717 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5717_C ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5718 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5719 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5720 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_57767 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_57764 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5762 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5725 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_5727 ||
		    tp->pci_device == TG3PCI_DEVICE_TIGON3_57787)
			reg = TG3PCI_GEN2_PRODID_ASICREV;
		else if (tp->pci_device == TG3PCI_DEVICE_TIGON3_57781 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57785 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57761 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57765 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57791 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57795 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57762 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57766 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57782 ||
			 tp->pci_device == TG3PCI_DEVICE_TIGON3_57786)
			reg = TG3PCI_GEN15_PRODID_ASICREV;
		else
			reg = TG3PCI_PRODID_ASICREV;

		pci_read_config_dword(tp->devno, reg, &tp->pci_chip_rev_id);
	}

	/* Wrong chip ID in 5752 A0. This code can be removed later
	 * as A0 is not in production.
	 */
	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5752_A0_HW)
		tp->pci_chip_rev_id = CHIPREV_ID_5752_A0;

	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5717_C0)
		tp->pci_chip_rev_id = CHIPREV_ID_5720_A0;

	if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
	    tg3_asic_rev(tp) == ASIC_REV_5719 ||
	    tg3_asic_rev(tp) == ASIC_REV_5720)
		tg3_flag_set(tp, 5717_PLUS);

	if (tg3_asic_rev(tp) == ASIC_REV_57765 ||
	    tg3_asic_rev(tp) == ASIC_REV_57766)
		tg3_flag_set(tp, 57765_CLASS);

	if (tg3_flag(tp, 57765_CLASS) || tg3_flag(tp, 5717_PLUS) ||
	     tg3_asic_rev(tp) == ASIC_REV_5762)
		tg3_flag_set(tp, 57765_PLUS);

	/* Intentionally exclude ASIC_REV_5906 */
	if (tg3_asic_rev(tp) == ASIC_REV_5755 ||
	    tg3_asic_rev(tp) == ASIC_REV_5787 ||
	    tg3_asic_rev(tp) == ASIC_REV_5784 ||
	    tg3_asic_rev(tp) == ASIC_REV_5761 ||
	    tg3_asic_rev(tp) == ASIC_REV_5785 ||
	    tg3_asic_rev(tp) == ASIC_REV_57780 ||
	    tg3_flag(tp, 57765_PLUS))
		tg3_flag_set(tp, 5755_PLUS);

	if (tg3_asic_rev(tp) == ASIC_REV_5780 ||
	    tg3_asic_rev(tp) == ASIC_REV_5714)
		tg3_flag_set(tp, 5780_CLASS);

	if (tg3_asic_rev(tp) == ASIC_REV_5750 ||
	    tg3_asic_rev(tp) == ASIC_REV_5752 ||
	    tg3_asic_rev(tp) == ASIC_REV_5906 ||
	    tg3_flag(tp, 5755_PLUS) ||
	    tg3_flag(tp, 5780_CLASS))
		tg3_flag_set(tp, 5750_PLUS);

	if (tg3_asic_rev(tp) == ASIC_REV_5705 ||
	    tg3_flag(tp, 5750_PLUS))
		tg3_flag_set(tp, 5705_PLUS);
}

static bool tg3_10_100_only_device(struct tg3_priv *tp)
{
	u32 grc_misc_cfg = tr32(GRC_MISC_CFG) & GRC_MISC_CFG_BOARD_ID_MASK;

	if ((tg3_asic_rev(tp) == ASIC_REV_5703 &&
	     (grc_misc_cfg == 0x8000 || grc_misc_cfg == 0x4000)) ||
	    (tp->phy_flags & TG3_PHYFLG_IS_FET))
		return true;
/* This will need to be changed before it will work with U-Boot.
	if (ent->driver_data & TG3_DRV_DATA_FLAG_10_100_ONLY) {
		if (tg3_asic_rev(tp) == ASIC_REV_5705) {
			if (ent->driver_data & TG3_DRV_DATA_FLAG_5705_10_100)
				return true;
		} else {
			return true;
		}
	}
*/
	return false;
}

/* TODO: I have left workarounds for boards other than
 *       the 5718 for now to speed up development. Some
 *       may need to be re-added before delivery. */
static int tg3_get_invariants(struct tg3_priv *tp/*, const struct pci_device_id *ent*/)
{
	u32 misc_ctrl_reg;
	u32 pci_state_reg, grc_misc_cfg;
	u32 val;
	u16 pci_cmd;
	int err;

	/* Force memory write invalidate off.  If we leave it on,
	 * then on 5700_BX chips we have to enable a workaround.
	 * The workaround is to set the TG3PCI_DMA_RW_CTRL boundary
	 * to match the cacheline size.  The Broadcom driver have this
	 * workaround but turns MWI off all the times so never uses
	 * it.  This seems to suggest that the workaround is insufficient.
	 */
	pci_read_config_word(tp->devno, PCI_COMMAND, &pci_cmd);
	pci_cmd &= ~PCI_COMMAND_INVALIDATE;
	pci_write_config_word(tp->devno, PCI_COMMAND, pci_cmd);

	/* Important! -- Make sure register accesses are byteswapped
	 * correctly.  Also, for those chips that require it, make
	 * sure that indirect register accesses are enabled before
	 * the first operation.
	 */
	pci_read_config_dword(tp->devno, TG3PCI_MISC_HOST_CTRL,
			      &misc_ctrl_reg);
	tp->misc_host_ctrl |= (misc_ctrl_reg &
			       MISC_HOST_CTRL_CHIPREV);
	pci_write_config_dword(tp->devno, TG3PCI_MISC_HOST_CTRL,
			       tp->misc_host_ctrl);

	tg3_detect_asic_rev(tp, misc_ctrl_reg);

	/* If we have 5702/03 A1 or A2 on certain ICH chipsets,
	 * we need to disable memory and use config. cycles
	 * only to access all registers. The 5702/03 chips
	 * can mistakenly decode the special cycles from the
	 * ICH chipsets as memory write cycles, causing corruption
	 * of register and memory space. Only certain ICH bridges
	 * will drive special cycles with non-zero data during the
	 * address phase which can fall within the 5703's address
	 * range. This is not an ICH bug as the PCI spec allows
	 * non-zero address during special cycles. However, only
	 * these ICH bridges are known to drive non-zero addresses
	 * during special cycles.
	 *
	 * Since special cycles do not cross PCI bridges, we only
	 * enable this workaround if the 5703 is on the secondary
	 * bus of these ICH bridges.
	 */
#if 0
	if ((tg3_chip_rev_id(tp) == CHIPREV_ID_5703_A1) ||
	    (tg3_chip_rev_id(tp) == CHIPREV_ID_5703_A2)) {
		static struct tg3_dev_id {
			u32	vendor;
			u32	device;
			u32	rev;
		} ich_chipsets[] = {
			{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82801AA_8,
			  PCI_ANY_ID },
			{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82801AB_8,
			  PCI_ANY_ID },
			{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82801BA_11,
			  0xa },
			{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82801BA_6,
			  PCI_ANY_ID },
			{ },
		};
		struct tg3_dev_id *pci_id = &ich_chipsets[0];
		struct pci_dev *bridge = NULL;

		while (pci_id->vendor != 0) {
			bridge = pci_get_device(pci_id->vendor, pci_id->device,
						bridge);
			if (!bridge) {
				pci_id++;
				continue;
			}
			if (pci_id->rev != PCI_ANY_ID) {
				if (bridge->revision > pci_id->rev)
					continue;
			}
			if (bridge->subordinate &&
			    (bridge->subordinate->number ==
			     tp->pdev->bus->number)) {
				tg3_flag_set(tp, ICH_WORKAROUND);
				pci_dev_put(bridge);
				break;
			}
		}
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5701) {
		static struct tg3_dev_id {
			u32	vendor;
			u32	device;
		} bridge_chipsets[] = {
			{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_PXH_0 },
			{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_PXH_1 },
			{ },
		};
		struct tg3_dev_id *pci_id = &bridge_chipsets[0];
		struct pci_dev *bridge = NULL;

		while (pci_id->vendor != 0) {
			bridge = pci_get_device(pci_id->vendor,
						pci_id->device,
						bridge);
			if (!bridge) {
				pci_id++;
				continue;
			}
			if (bridge->subordinate &&
			    (bridge->subordinate->number <=
			     tp->pdev->bus->number) &&
			    (bridge->subordinate->busn_res.end >=
			     tp->pdev->bus->number)) {
				tg3_flag_set(tp, 5701_DMA_BUG);
				pci_dev_put(bridge);
				break;
			}
		}
	}
#endif

	/* The EPB bridge inside 5714, 5715, and 5780 cannot support
	 * DMA addresses > 40-bit. This bridge may have other additional
	 * 57xx devices behind it in some 4-port NIC designs for example.
	 * Any tg3 device found behind the bridge will also need the 40-bit
	 * DMA workaround.
	 */
	if (tg3_flag(tp, 5780_CLASS)) {
		tg3_flag_set(tp, 40BIT_DMA_BUG);
	}
#if 0
	else {
		struct pci_dev *bridge = NULL;

		do {
			bridge = pci_get_device(PCI_VENDOR_ID_SERVERWORKS,
						PCI_DEVICE_ID_SERVERWORKS_EPB,
						bridge);
			if (bridge && bridge->subordinate &&
			    (bridge->subordinate->number <=
			     tp->pdev->bus->number) &&
			    (bridge->subordinate->busn_res.end >=
			     tp->pdev->bus->number)) {
				tg3_flag_set(tp, 40BIT_DMA_BUG);
				pci_dev_put(bridge);
				break;
			}
		} while (bridge);
	}
#endif

	if (tg3_asic_rev(tp) == ASIC_REV_5704 ||
	    tg3_asic_rev(tp) == ASIC_REV_5714)
		printf("tg3_find_peer() not implemented.\n");

	if (tg3_flag(tp, 5755_PLUS) || tg3_asic_rev(tp) == ASIC_REV_5906)
		tg3_flag_set(tp, SHORT_DMA_BUG);

	if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
	    tg3_asic_rev(tp) == ASIC_REV_5719 ||
	    tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762)
		tg3_flag_set(tp, LRG_PROD_RING_CAP);

	pci_read_config_dword(tp->devno, TG3PCI_PCISTATE, &pci_state_reg);

	/* If we have an AMD 762 or VIA K8T800 chipset, write
	 * reordering to the mailbox registers done by the host
	 * controller can cause major troubles.  We read back from
	 * every mailbox register write to force the writes to be
	 * posted to the chip in order.
	 */
#if 0
	if (pci_dev_present(tg3_write_reorder_chipsets))
		tg3_flag_set(tp, MBOX_WRITE_REORDER);
#endif

	pci_read_config_byte(tp->devno, PCI_CACHE_LINE_SIZE, &tp->pci_cacheline_sz);
	pci_read_config_byte(tp->devno, PCI_LATENCY_TIMER, &tp->pci_lat_timer);
	if (tg3_asic_rev(tp) == ASIC_REV_5703 && tp->pci_lat_timer < 64) {
		tp->pci_lat_timer = 64;
		pci_write_config_byte(tp->devno, PCI_LATENCY_TIMER, tp->pci_lat_timer);
	}

	if (tg3_chip_rev(tp) == CHIPREV_5700_BX) {
		/* 5700 BX chips need to have their TX producer index
		 * mailboxes written twice to workaround a bug.
		 */
		tg3_flag_set(tp, TXD_MBOX_HWBUG);
	}

	if ((pci_state_reg & PCISTATE_BUS_SPEED_HIGH) != 0)
		tg3_flag_set(tp, PCI_HIGH_SPEED);
	if ((pci_state_reg & PCISTATE_BUS_32BIT) != 0)
		tg3_flag_set(tp, PCI_32BIT);

	/* Chip-specific fixup from Broadcom driver */
	if ((tg3_chip_rev_id(tp) == CHIPREV_ID_5704_A0) &&
	    (!(pci_state_reg & PCISTATE_RETRY_SAME_DMA))) {
		pci_state_reg |= PCISTATE_RETRY_SAME_DMA;
		pci_write_config_dword(tp->devno, TG3PCI_PCISTATE, pci_state_reg);
	}

	/* Default fast path register access methods */
	tp->read32 = tg3_read32;
	tp->write32 = tg3_write32;
	tp->read32_mbox = tg3_read32;
	tp->write32_mbox = tg3_write32;
	tp->write32_tx_mbox = tg3_write32;
	tp->write32_rx_mbox = tg3_write32;

	/* Various workaround register access methods */
	if (tg3_flag(tp, PCIX_TARGET_HWBUG))
		tp->write32 = tg3_write_indirect_reg32;
	else if (tg3_asic_rev(tp) == ASIC_REV_5701) {
		/*
		 * Back to back register writes can cause problems on these
		 * chips, the workaround is to read back all reg writes
		 * except those to mailbox regs.
		 *
		 * See tg3_write_indirect_reg32().
		 */
		tp->write32 = tg3_write_flush_reg32;
	}

	if (tg3_flag(tp, TXD_MBOX_HWBUG) || tg3_flag(tp, MBOX_WRITE_REORDER)) {
		tp->write32_tx_mbox = tg3_write32_tx_mbox;
		if (tg3_flag(tp, MBOX_WRITE_REORDER))
			tp->write32_rx_mbox = tg3_write_flush_reg32;
	}

	if (tg3_flag(tp, ICH_WORKAROUND)) {
		tp->read32 = tg3_read_indirect_reg32;
		tp->write32 = tg3_write_indirect_reg32;
		tp->read32_mbox = tg3_read_indirect_mbox;
		tp->write32_mbox = tg3_write_indirect_mbox;
		tp->write32_tx_mbox = tg3_write_indirect_mbox;
		tp->write32_rx_mbox = tg3_write_indirect_mbox;

#if 0
		iounmap(tp->regs);
#endif
		tp->regs = NULL;

		pci_read_config_word(tp->devno, PCI_COMMAND, &pci_cmd);
		pci_cmd &= ~PCI_COMMAND_MEMORY;
		pci_write_config_word(tp->devno, PCI_COMMAND, pci_cmd);
	}
	if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		tp->read32_mbox = tg3_read32_mbox_5906;
		tp->write32_mbox = tg3_write32_mbox_5906;
		tp->write32_tx_mbox = tg3_write32_mbox_5906;
		tp->write32_rx_mbox = tg3_write32_mbox_5906;
	}

	if (tp->write32 == tg3_write_indirect_reg32 ||
	    tg3_asic_rev(tp) == ASIC_REV_5701)
		tg3_flag_set(tp, SRAM_USE_CONFIG);

	/* The memory arbiter has to be enabled in order for SRAM accesses
	 * to succeed.  Normally on powerup the tg3 chip firmware will make
	 * sure it is enabled, but other entities such as system netboot
	 * code might disable it.
	 */
	val = tr32(MEMARB_MODE);
	tw32(MEMARB_MODE, val | MEMARB_MODE_ENABLE);

	tp->pci_fn = PCI_FUNC(tp->devno) & 3;

	if (tg3_asic_rev(tp) == ASIC_REV_5704 || tg3_flag(tp, 5780_CLASS)) {
	} else if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
		   tg3_asic_rev(tp) == ASIC_REV_5719 ||
		   tg3_asic_rev(tp) == ASIC_REV_5720) {
		tg3_read_mem(tp, NIC_SRAM_CPMU_STATUS, &val);
		if ((val & NIC_SRAM_CPMUSTAT_SIG_MSK) != NIC_SRAM_CPMUSTAT_SIG)
			val = tr32(TG3_CPMU_STATUS);

		if (tg3_asic_rev(tp) == ASIC_REV_5717)
			tp->pci_fn = (val & TG3_CPMU_STATUS_FMSK_5717) ? 1 : 0;
		else
			tp->pci_fn = (val & TG3_CPMU_STATUS_FMSK_5719) >>
				     TG3_CPMU_STATUS_FSHFT_5719;
	}

	if (tg3_flag(tp, FLUSH_POSTED_WRITES)) {
		tp->write32_tx_mbox = tg3_write_flush_reg32;
		tp->write32_rx_mbox = tg3_write_flush_reg32;
	}

	/* Get eeprom hw config before calling tg3_set_power_state().
	 * In particular, the TG3_FLAG_IS_NIC flag must be
	 * determined before calling tg3_set_power_state() so that
	 * we know whether or not to switch out of Vaux power.
	 * When the flag is set, it means that GPIO1 is used for eeprom
	 * write protect and also implies that it is a LOM where GPIOs
	 * are not used to switch power.
	 */
	tg3_get_eeprom_hw_cfg(tp);

	/* Set up tp->grc_local_ctrl before calling
	 * tg3_pwrsrc_switch_to_vmain().  GPIO1 driven high
	 * will bring 5700's external PHY out of reset.
	 * It is also used as eeprom write protect on LOMs.
	 */
	tp->grc_local_ctrl = GRC_LCLCTRL_INT_ON_ATTN | GRC_LCLCTRL_AUTO_SEEPROM;
	if (tg3_asic_rev(tp) == ASIC_REV_5700 ||
	    tg3_flag(tp, EEPROM_WRITE_PROT))
		tp->grc_local_ctrl |= (GRC_LCLCTRL_GPIO_OE1 |
				       GRC_LCLCTRL_GPIO_OUTPUT1);
	/* Unused GPIO3 must be driven as output on 5752 because there
	 * are no pull-up resistors on unused GPIO pins.
	 */
	else if (tg3_asic_rev(tp) == ASIC_REV_5752)
		tp->grc_local_ctrl |= GRC_LCLCTRL_GPIO_OE3;

	if (tg3_asic_rev(tp) == ASIC_REV_5755 ||
	    tg3_asic_rev(tp) == ASIC_REV_57780 ||
	    tg3_flag(tp, 57765_CLASS))
		tp->grc_local_ctrl |= GRC_LCLCTRL_GPIO_UART_SEL;

	if (tp->pci_device == TG3PCI_DEVICE_TIGON3_5761 ||
	    tp->pci_device == TG3PCI_DEVICE_TIGON3_5761S) {
		/* Turn off the debug UART. */
		tp->grc_local_ctrl |= GRC_LCLCTRL_GPIO_UART_SEL;
		if (tg3_flag(tp, IS_NIC))
			/* Keep VMain power. */
			tp->grc_local_ctrl |= GRC_LCLCTRL_GPIO_OE0 |
					      GRC_LCLCTRL_GPIO_OUTPUT0;
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5762)
		tp->grc_local_ctrl |=
			tr32(GRC_LOCAL_CTRL) & GRC_LCLCTRL_GPIO_UART_SEL;

	/* Switch out of Vaux if it is a NIC */
	tg3_pwrsrc_switch_to_vmain(tp);

	if (tg3_asic_rev(tp) == ASIC_REV_5906)
		tp->phy_flags |= TG3_PHYFLG_IS_FET;

	/* A few boards don't want Ethernet@WireSpeed phy feature */
	if (tg3_asic_rev(tp) == ASIC_REV_5700 ||
	    (tg3_asic_rev(tp) == ASIC_REV_5705 &&
	     (tg3_chip_rev_id(tp) != CHIPREV_ID_5705_A0) &&
	     (tg3_chip_rev_id(tp) != CHIPREV_ID_5705_A1)) ||
	    (tp->phy_flags & TG3_PHYFLG_IS_FET) ||
	    (tp->phy_flags & TG3_PHYFLG_ANY_SERDES))
		tp->phy_flags |= TG3_PHYFLG_NO_ETH_WIRE_SPEED;

	if (tg3_chip_rev(tp) == CHIPREV_5703_AX ||
	    tg3_chip_rev(tp) == CHIPREV_5704_AX)
		tp->phy_flags |= TG3_PHYFLG_ADC_BUG;
	if (tg3_chip_rev_id(tp) == CHIPREV_ID_5704_A0)
		tp->phy_flags |= TG3_PHYFLG_5704_A0_BUG;

	if (tg3_flag(tp, 5705_PLUS) &&
	    !(tp->phy_flags & TG3_PHYFLG_IS_FET) &&
	    tg3_asic_rev(tp) != ASIC_REV_5785 &&
	    tg3_asic_rev(tp) != ASIC_REV_57780 &&
	    !tg3_flag(tp, 57765_PLUS)) {
		if (tg3_asic_rev(tp) == ASIC_REV_5755 ||
		    tg3_asic_rev(tp) == ASIC_REV_5787 ||
		    tg3_asic_rev(tp) == ASIC_REV_5784 ||
		    tg3_asic_rev(tp) == ASIC_REV_5761) {
			if (tp->pci_device != TG3PCI_DEVICE_TIGON3_5756 &&
			    tp->pci_device != TG3PCI_DEVICE_TIGON3_5722)
				tp->phy_flags |= TG3_PHYFLG_JITTER_BUG;
			if (tp->pci_device == TG3PCI_DEVICE_TIGON3_5755M)
				tp->phy_flags |= TG3_PHYFLG_ADJUST_TRIM;
		} else
			tp->phy_flags |= TG3_PHYFLG_BER_BUG;
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5784 &&
	    tg3_chip_rev(tp) != CHIPREV_5784_AX) {
		tp->phy_otp = tg3_read_otp_phycfg(tp);
		if (tp->phy_otp == 0)
			tp->phy_otp = TG3_OTP_DEFAULT;
	}

	if (tg3_flag(tp, CPMU_PRESENT))
		tp->mi_mode = MAC_MI_MODE_500KHZ_CONST;
	else
		tp->mi_mode = MAC_MI_MODE_BASE;

	tp->coalesce_mode = 0;
	if (tg3_chip_rev(tp) != CHIPREV_5700_AX &&
	    tg3_chip_rev(tp) != CHIPREV_5700_BX)
		tp->coalesce_mode |= HOSTCC_MODE_32BYTE;

	/* Set these bits to enable statistics workaround. */
	if (tg3_asic_rev(tp) == ASIC_REV_5717 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762 ||
	    tg3_chip_rev_id(tp) == CHIPREV_ID_5719_A0 ||
	    tg3_chip_rev_id(tp) == CHIPREV_ID_5720_A0) {
		tp->coalesce_mode |= HOSTCC_MODE_ATTN;
		tp->grc_mode |= GRC_MODE_IRQ_ON_FLOW_ATTN;
	}

	if (tg3_asic_rev(tp) == ASIC_REV_5785 ||
	    tg3_asic_rev(tp) == ASIC_REV_57780)	{
		printf("Error:  USE_PHYLIB not implemented.\n");
		return -ENODEV;
	}

	err = tg3_mdio_init(tp);
	if (err)
		return err;

	/* Initialize data/descriptor byte/word swapping. */
	val = tr32(GRC_MODE);
	if (tg3_asic_rev(tp) == ASIC_REV_5720 ||
	    tg3_asic_rev(tp) == ASIC_REV_5762)
		val &= (GRC_MODE_BYTE_SWAP_B2HRX_DATA |
			GRC_MODE_WORD_SWAP_B2HRX_DATA |
			GRC_MODE_B2HRX_ENABLE |
			GRC_MODE_HTX2B_ENABLE |
			GRC_MODE_HOST_STACKUP);
	else
		val &= GRC_MODE_HOST_STACKUP;

	tw32(GRC_MODE, val | tp->grc_mode);

	tg3_switch_clocks(tp);

	/* Clear this out for sanity. */
	tw32(TG3PCI_MEM_WIN_BASE_ADDR, 0);

	/* Clear TG3PCI_REG_BASE_ADDR to prevent hangs. */
	tw32(TG3PCI_REG_BASE_ADDR, 0);

	pci_read_config_dword(tp->devno, TG3PCI_PCISTATE, &pci_state_reg);
	if ((pci_state_reg & PCISTATE_CONV_PCI_MODE) == 0 &&
	    !tg3_flag(tp, PCIX_TARGET_HWBUG)) {
		if (tg3_chip_rev_id(tp) == CHIPREV_ID_5701_A0 ||
		    tg3_chip_rev_id(tp) == CHIPREV_ID_5701_B0 ||
		    tg3_chip_rev_id(tp) == CHIPREV_ID_5701_B2 ||
		    tg3_chip_rev_id(tp) == CHIPREV_ID_5701_B5) {
			void __iomem *sram_base;

			/* Write some dummy words into the SRAM status block
			 * area, see if it reads back correctly.  If the return
			 * value is bad, force enable the PCIX workaround.
			 */
			sram_base = tp->regs + NIC_SRAM_WIN_BASE + NIC_SRAM_STATS_BLK;

			writel(0x00000000, sram_base);
			writel(0x00000000, sram_base + 4);
			writel(0xffffffff, sram_base + 4);
			if (readl(sram_base) != 0x00000000)
				tg3_flag_set(tp, PCIX_TARGET_HWBUG);
		}
	}

	udelay(50);

	grc_misc_cfg = tr32(GRC_MISC_CFG);
	grc_misc_cfg &= GRC_MISC_CFG_BOARD_ID_MASK;

	if (tg3_asic_rev(tp) == ASIC_REV_5705 &&
	    (grc_misc_cfg == GRC_MISC_CFG_BOARD_ID_5788 ||
	     grc_misc_cfg == GRC_MISC_CFG_BOARD_ID_5788M))
		tg3_flag_set(tp, IS_5788);

	if (!tg3_flag(tp, IS_5788) &&
	    tg3_asic_rev(tp) != ASIC_REV_5700)
		tg3_flag_set(tp, TAGGED_STATUS);
	if (tg3_flag(tp, TAGGED_STATUS)) {
		tp->coalesce_mode |= (HOSTCC_MODE_CLRTICK_RXBD |
				      HOSTCC_MODE_CLRTICK_TXBD);

		tp->misc_host_ctrl |= MISC_HOST_CTRL_TAGGED_STATUS;
		pci_write_config_dword(tp->devno, TG3PCI_MISC_HOST_CTRL,
				       tp->misc_host_ctrl);
	}

	tp->mac_mode = 0;

	if (tg3_10_100_only_device(tp))
		tp->phy_flags |= TG3_PHYFLG_10_100_ONLY;

	err = tg3_phy_probe(tp);
	if (err) {
		printf("#   phy probe failed, err %d\n", err);
	}

	/* 5700 {AX,BX} chips have a broken status block link
	 * change bit implementation, so we must use the
	 * status register in those cases.
	 */
	if (tg3_asic_rev(tp) == ASIC_REV_5700)
		tg3_flag_set(tp, USE_LINKCHG_REG);
	else
		tg3_flag_clear(tp, USE_LINKCHG_REG);

	/* For all SERDES we poll the MAC status register. */
	if (tp->phy_flags & TG3_PHYFLG_PHY_SERDES)
		tg3_flag_set(tp, POLL_SERDES);
	else
		tg3_flag_clear(tp, POLL_SERDES);

	tp->rx_std_ring_mask = TG3_RX_PRODUCER_RING_SIZE - 1;
	tp->rx_ret_ring_mask = TG3_RX_RETURN_RING_SIZE - 1;

	tp->rx_std_max_post = tp->rx_std_ring_mask + 1;

	/* Increment the rx prod index on the rx std ring by at most
	 * 8 for these chips to workaround hw errata.
	 */
	if (tg3_asic_rev(tp) == ASIC_REV_5750 ||
	    tg3_asic_rev(tp) == ASIC_REV_5752 ||
	    tg3_asic_rev(tp) == ASIC_REV_5755)
		tp->rx_std_max_post = 8;

	if (tg3_flag(tp, ASPM_WORKAROUND))
		tp->pwrmgmt_thresh = tr32(PCIE_PWR_MGMT_THRESH) &
				     PCIE_PWR_MGMT_L1_THRESH_MSK;

	return err;
}

static int tg3_get_device_address(struct tg3_priv *tp)
{
	u32 hi, lo, mac_offset;
	int addr_ok = 0;

	mac_offset = 0x7c;
	if (tg3_asic_rev(tp) == ASIC_REV_5704 ||
	    tg3_flag(tp, 5780_CLASS)) {
		if (tr32(TG3PCI_DUAL_MAC_CTRL) & DUAL_MAC_CTRL_ID)
			mac_offset = 0xcc;
	} else if (tg3_flag(tp, 5717_PLUS)) {
		if (tp->pci_fn & 1)
			mac_offset = 0xcc;
		if (tp->pci_fn > 1)
			mac_offset += 0x18c;
	} else if (tg3_asic_rev(tp) == ASIC_REV_5906) {
		mac_offset = 0x10;
	}

	/* First try to get it from MAC address mailbox. */
	tg3_read_mem(tp, NIC_SRAM_MAC_ADDR_HIGH_MBOX, &hi);
	if ((hi >> 16) == 0x484b) {
		tp->mac_address[0] = (hi >>  8) & 0xff;
		tp->mac_address[1] = (hi >>  0) & 0xff;

		tg3_read_mem(tp, NIC_SRAM_MAC_ADDR_LOW_MBOX, &lo);
		tp->mac_address[2] = (lo >> 24) & 0xff;
		tp->mac_address[3] = (lo >> 16) & 0xff;
		tp->mac_address[4] = (lo >>  8) & 0xff;
		tp->mac_address[5] = (lo >>  0) & 0xff;

		/* Some old bootcode may report a 0 MAC address in SRAM */
		addr_ok = is_valid_ether_addr(&tp->mac_address[0]);
	}
	if (!addr_ok) {
		/* Note: NVRAM code removed. */
		/* Finally just fetch it out of the MAC control regs. */
		hi = tr32(MAC_ADDR_0_HIGH);
		lo = tr32(MAC_ADDR_0_LOW);

		tp->mac_address[5] = lo & 0xff;
		tp->mac_address[4] = (lo >> 8) & 0xff;
		tp->mac_address[3] = (lo >> 16) & 0xff;
		tp->mac_address[2] = (lo >> 24) & 0xff;
		tp->mac_address[1] = hi & 0xff;
		tp->mac_address[0] = (hi >> 8) & 0xff;
	}

	if (!is_valid_ether_addr(&tp->mac_address[0]))
		return -EINVAL;

	return 0;
}

#define BOUNDARY_SINGLE_CACHELINE	1
#define BOUNDARY_MULTI_CACHELINE	2

static u32 tg3_calc_dma_bndry(struct tg3_priv *tp, u32 val)
{
	int cacheline_size;
	u8 byte;
	int goal;

	pci_read_config_byte(tp->devno, PCI_CACHE_LINE_SIZE, &byte);
	if (byte == 0)
		cacheline_size = 1024;
	else
		cacheline_size = (int) byte * 4;

	/* On 5703 and later chips, the boundary bits have no
	 * effect.
	 */
	if (tg3_asic_rev(tp) != ASIC_REV_5700 &&
	    tg3_asic_rev(tp) != ASIC_REV_5701)
		goto out;

#if defined(CONFIG_PPC64) || defined(CONFIG_IA64) || defined(CONFIG_PARISC)
	goal = BOUNDARY_MULTI_CACHELINE;
#else
#if defined(CONFIG_SPARC64) || defined(CONFIG_ALPHA)
	goal = BOUNDARY_SINGLE_CACHELINE;
#else
	goal = 0;
#endif
#endif

	if (tg3_flag(tp, 57765_PLUS)) {
		val = goal ? 0 : DMA_RWCTRL_DIS_CACHE_ALIGNMENT;
		goto out;
	}

	if (!goal)
		goto out;

	/* PCI controllers on most RISC systems tend to disconnect
	 * when a device tries to burst across a cache-line boundary.
	 * Therefore, letting tg3 do so just wastes PCI bandwidth.
	 *
	 * Unfortunately, for PCI-E there are only limited
	 * write-side controls for this, and thus for reads
	 * we will still get the disconnects.  We'll also waste
	 * these PCI cycles for both read and write for chips
	 * other than 5700 and 5701 which do not implement the
	 * boundary bits.
	 */
	switch (cacheline_size) {
	case 16:
		if (goal == BOUNDARY_SINGLE_CACHELINE) {
			val |= (DMA_RWCTRL_READ_BNDRY_16 |
				DMA_RWCTRL_WRITE_BNDRY_16);
			break;
		}
		/* fallthrough */
	case 32:
		if (goal == BOUNDARY_SINGLE_CACHELINE) {
			val |= (DMA_RWCTRL_READ_BNDRY_32 |
				DMA_RWCTRL_WRITE_BNDRY_32);
			break;
		}
		/* fallthrough */
	case 64:
		if (goal == BOUNDARY_SINGLE_CACHELINE) {
			val |= (DMA_RWCTRL_READ_BNDRY_64 |
				DMA_RWCTRL_WRITE_BNDRY_64);
			break;
		}
		/* fallthrough */
	case 128:
		if (goal == BOUNDARY_SINGLE_CACHELINE) {
			val |= (DMA_RWCTRL_READ_BNDRY_128 |
				DMA_RWCTRL_WRITE_BNDRY_128);
			break;
		}
		/* fallthrough */
	case 256:
		val |= (DMA_RWCTRL_READ_BNDRY_256 |
			DMA_RWCTRL_WRITE_BNDRY_256);
		break;
	case 512:
		val |= (DMA_RWCTRL_READ_BNDRY_512 |
			DMA_RWCTRL_WRITE_BNDRY_512);
		break;
	case 1024:
	default:
		val |= (DMA_RWCTRL_READ_BNDRY_1024 |
			DMA_RWCTRL_WRITE_BNDRY_1024);
		break;
	}

out:
	return val;
}

static int tg3_do_test_dma(struct tg3_priv *tp, u32 *buf, dma_addr_t buf_dma, int size, bool to_device)
{
	struct tg3_internal_buffer_desc test_desc;
	u32 sram_dma_descs;
	int i, ret;

	sram_dma_descs = NIC_SRAM_DMA_DESC_POOL_BASE;

	tw32(FTQ_RCVBD_COMP_FIFO_ENQDEQ, 0);
	tw32(FTQ_RCVDATA_COMP_FIFO_ENQDEQ, 0);
	tw32(RDMAC_STATUS, 0);
	tw32(WDMAC_STATUS, 0);

	tw32(BUFMGR_MODE, 0);
	tw32(FTQ_RESET, 0);

	test_desc.addr_hi = ((u64) buf_dma) >> 32;
	test_desc.addr_lo = buf_dma & 0xffffffff;
	test_desc.nic_mbuf = 0x00002100;
	test_desc.len = size;

	/*
	 * HP ZX1 was seeing test failures for 5701 cards running at 33Mhz
	 * the *second* time the tg3 driver was getting loaded after an
	 * initial scan.
	 *
	 * Broadcom tells me:
	 *   ...the DMA engine is connected to the GRC block and a DMA
	 *   reset may affect the GRC block in some unpredictable way...
	 *   The behavior of resets to individual blocks has not been tested.
	 *
	 * Broadcom noted the GRC reset will also reset all sub-components.
	 */
	if (to_device) {
		test_desc.cqid_sqid = (13 << 8) | 2;

		tw32_f(RDMAC_MODE, RDMAC_MODE_ENABLE);
		udelay(40);
	} else {
		test_desc.cqid_sqid = (16 << 8) | 7;

		tw32_f(WDMAC_MODE, WDMAC_MODE_ENABLE);
		udelay(40);
	}
	test_desc.flags = 0x00000005;

	for (i = 0; i < (sizeof(test_desc) / sizeof(u32)); i++) {
		u32 val;

		val = *(((u32 *)&test_desc) + i);
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_BASE_ADDR,
				       sram_dma_descs + (i * sizeof(u32)));
		pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_DATA, val);
	}
	pci_write_config_dword(tp->devno, TG3PCI_MEM_WIN_BASE_ADDR, 0);

	if (to_device)
		tw32(FTQ_DMA_HIGH_READ_FIFO_ENQDEQ, sram_dma_descs);
	else
		tw32(FTQ_DMA_HIGH_WRITE_FIFO_ENQDEQ, sram_dma_descs);

	ret = -ENODEV;
	for (i = 0; i < 40; i++) {
		u32 val;

		if (to_device)
			val = tr32(FTQ_RCVBD_COMP_FIFO_ENQDEQ);
		else
			val = tr32(FTQ_RCVDATA_COMP_FIFO_ENQDEQ);
		if ((val & 0xffff) == sram_dma_descs) {
			ret = 0;
			break;
		}

		udelay(100);
	}

	return ret;
}

#define TEST_BUFFER_SIZE	0x2000

#if 0
static DEFINE_PCI_DEVICE_TABLE(tg3_dma_wait_state_chipsets) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, PCI_DEVICE_ID_APPLE_UNI_N_PCI15) },
	{ },
};
#endif

static int tg3_test_dma(struct tg3_priv *tp)
{
	dma_addr_t buf_dma;
	u32 *buf, saved_dma_rwctrl;
	int ret = 0;

	buf = dma_alloc_coherent(TEST_BUFFER_SIZE, &buf_dma);
	if (!buf) {
		ret = -ENOMEM;
		goto out_nofree;
	}

	tp->dma_rwctrl = ((0x7 << DMA_RWCTRL_PCI_WRITE_CMD_SHIFT) |
			  (0x6 << DMA_RWCTRL_PCI_READ_CMD_SHIFT));

	tp->dma_rwctrl = tg3_calc_dma_bndry(tp, tp->dma_rwctrl);

	if (tg3_flag(tp, 57765_PLUS))
		goto out;

	if (tg3_asic_rev(tp) == ASIC_REV_5705 ||
	    tg3_asic_rev(tp) == ASIC_REV_5750)
		tp->dma_rwctrl |= 0x003f0000;
	else
		tp->dma_rwctrl |= 0x003f000f;

	if (tg3_flag(tp, ONE_DMA_AT_ONCE))
		tp->dma_rwctrl |= DMA_RWCTRL_ONE_DMA;

	if (tg3_asic_rev(tp) == ASIC_REV_5703 ||
	    tg3_asic_rev(tp) == ASIC_REV_5704)
		tp->dma_rwctrl &= 0xfffffff0;

	if (tg3_asic_rev(tp) == ASIC_REV_5700 ||
	    tg3_asic_rev(tp) == ASIC_REV_5701) {
		/* Remove this if it causes problems for some boards. */
		tp->dma_rwctrl |= DMA_RWCTRL_USE_MEM_READ_MULT;

		/* On 5700/5701 chips, we need to set this bit.
		 * Otherwise the chip will issue cacheline transactions
		 * to streamable DMA memory with not all the byte
		 * enables turned on.  This is an error on several
		 * RISC PCI controllers, in particular sparc64.
		 *
		 */
		tp->dma_rwctrl |= DMA_RWCTRL_ASSERT_ALL_BE;
	}

	tw32(TG3PCI_DMA_RW_CTRL, tp->dma_rwctrl);

	if (tg3_asic_rev(tp) != ASIC_REV_5700 && tg3_asic_rev(tp) !=
		ASIC_REV_5701)
		goto out;

	/* It is best to perform DMA test with maximum write burst size
	 * to expose the 5700/5701 write DMA bug.
	 */
	saved_dma_rwctrl = tp->dma_rwctrl;
	tp->dma_rwctrl &= ~DMA_RWCTRL_WRITE_BNDRY_MASK;
	tw32(TG3PCI_DMA_RW_CTRL, tp->dma_rwctrl);

	while (1) {
		u32 *p = buf, i;

		for (i = 0; i < TEST_BUFFER_SIZE / sizeof(u32); i++)
			p[i] = i;

		/* Send the buffer to the chip. */
		ret = tg3_do_test_dma(tp, buf, buf_dma, TEST_BUFFER_SIZE, true);
		if (ret) {
			printf("#   %s: Buffer write failed. err = %d\n", __func__, ret);
			break;
		}

		/* Now read it back. */
		ret = tg3_do_test_dma(tp, buf, buf_dma, TEST_BUFFER_SIZE, false);
		if (ret) {
			printf("#   %s: Buffer read failed. err = %d\n", __func__, ret);
			break;
		}

		/* Verify it. */
		for (i = 0; i < TEST_BUFFER_SIZE / sizeof(u32); i++) {
			if (p[i] == i)
				continue;

			if ((tp->dma_rwctrl & DMA_RWCTRL_WRITE_BNDRY_MASK) !=
			    DMA_RWCTRL_WRITE_BNDRY_16) {
				tp->dma_rwctrl &= ~DMA_RWCTRL_WRITE_BNDRY_MASK;
				tp->dma_rwctrl |= DMA_RWCTRL_WRITE_BNDRY_16;
				tw32(TG3PCI_DMA_RW_CTRL, tp->dma_rwctrl);
				break;
			} else {
				printf("# %s: Buffer corrupted on read back! (%d != %d)\n",
				       __func__, p[i], i);
				ret = -ENODEV;
				goto out;
			}
		}

		if (i == (TEST_BUFFER_SIZE / sizeof(u32))) {
			/* Success. */
			ret = 0;
			break;
		}
	}
	if ((tp->dma_rwctrl & DMA_RWCTRL_WRITE_BNDRY_MASK) !=
	    DMA_RWCTRL_WRITE_BNDRY_16) {
		/* DMA test passed without adjusting DMA boundary,
		 * now look for chipsets that are known to expose the
		 * DMA bug without failing the test.
		 */
#if 0
		if (pci_dev_present(tg3_dma_wait_state_chipsets)) {
			tp->dma_rwctrl &= ~DMA_RWCTRL_WRITE_BNDRY_MASK;
			tp->dma_rwctrl |= DMA_RWCTRL_WRITE_BNDRY_16;
		} else {
			/* Safe to use the calculated DMA boundary. */
			tp->dma_rwctrl = saved_dma_rwctrl;
		}
#else
		/* Safe to use the calculated DMA boundary. */
		tp->dma_rwctrl = saved_dma_rwctrl;
#endif

		tw32(TG3PCI_DMA_RW_CTRL, tp->dma_rwctrl);
	}

out:
	free(buf);
out_nofree:
	return ret;
}

static void tg3_init_bufmgr_config(struct tg3_priv *tp)
{
	if (tg3_flag(tp, 57765_PLUS)) {
		tp->bufmgr_config.mbuf_read_dma_low_water =
			DEFAULT_MB_RDMA_LOW_WATER_5705;
		tp->bufmgr_config.mbuf_mac_rx_low_water =
			DEFAULT_MB_MACRX_LOW_WATER_57765;
		tp->bufmgr_config.mbuf_high_water =
			DEFAULT_MB_HIGH_WATER_57765;

		tp->bufmgr_config.mbuf_read_dma_low_water_jumbo =
			DEFAULT_MB_RDMA_LOW_WATER_5705;
		tp->bufmgr_config.mbuf_mac_rx_low_water_jumbo =
			DEFAULT_MB_MACRX_LOW_WATER_JUMBO_57765;
		tp->bufmgr_config.mbuf_high_water_jumbo =
			DEFAULT_MB_HIGH_WATER_JUMBO_57765;
	} else if (tg3_flag(tp, 5705_PLUS)) {
		tp->bufmgr_config.mbuf_read_dma_low_water =
			DEFAULT_MB_RDMA_LOW_WATER_5705;
		tp->bufmgr_config.mbuf_mac_rx_low_water =
			DEFAULT_MB_MACRX_LOW_WATER_5705;
		tp->bufmgr_config.mbuf_high_water =
			DEFAULT_MB_HIGH_WATER_5705;
		if (tg3_asic_rev(tp) == ASIC_REV_5906) {
			tp->bufmgr_config.mbuf_mac_rx_low_water =
				DEFAULT_MB_MACRX_LOW_WATER_5906;
			tp->bufmgr_config.mbuf_high_water =
				DEFAULT_MB_HIGH_WATER_5906;
		}

		tp->bufmgr_config.mbuf_read_dma_low_water_jumbo =
			DEFAULT_MB_RDMA_LOW_WATER_JUMBO_5780;
		tp->bufmgr_config.mbuf_mac_rx_low_water_jumbo =
			DEFAULT_MB_MACRX_LOW_WATER_JUMBO_5780;
		tp->bufmgr_config.mbuf_high_water_jumbo =
			DEFAULT_MB_HIGH_WATER_JUMBO_5780;
	} else {
		tp->bufmgr_config.mbuf_read_dma_low_water =
			DEFAULT_MB_RDMA_LOW_WATER;
		tp->bufmgr_config.mbuf_mac_rx_low_water =
			DEFAULT_MB_MACRX_LOW_WATER;
		tp->bufmgr_config.mbuf_high_water =
			DEFAULT_MB_HIGH_WATER;

		tp->bufmgr_config.mbuf_read_dma_low_water_jumbo =
			DEFAULT_MB_RDMA_LOW_WATER_JUMBO;
		tp->bufmgr_config.mbuf_mac_rx_low_water_jumbo =
			DEFAULT_MB_MACRX_LOW_WATER_JUMBO;
		tp->bufmgr_config.mbuf_high_water_jumbo =
			DEFAULT_MB_HIGH_WATER_JUMBO;
	}

	tp->bufmgr_config.dma_low_water = DEFAULT_DMA_LOW_WATER;
	tp->bufmgr_config.dma_high_water = DEFAULT_DMA_HIGH_WATER;
}

static int tg3_phy_copper_get_link(struct tg3_priv *tp)
{
	int i = 0;
	u32 val;

	if (tg3_readphy(tp, MII_BMSR, &val))
		return -EIO;

	if (val & BMSR_LSTATUS)
		return 0;

	printf("Waiting for PHY auto negotiation to complete");
	while (!(val & BMSR_ANEGCOMPLETE)) {
		if (i > PHY_ANEG_TIMEOUT) {
			printf(" TIMEOUT !\n");
			return -1;
		}

		if (ctrlc()) {
			puts(" user interrupt!\n");
			return -EINTR;
		}

		if ((i++ % 500) == 0)
			printf(".");

		udelay(1000);	/* 1 ms */

		if (tg3_readphy(tp, MII_BMSR, &val))
			return -EIO;
	}
	printf(" done\n");

	return 0;
}

static int tg3_phy_serdes_get_link(struct tg3_priv *tp)
{
	/* TODO: Probably need to look at one of SG_DIG_STATUS or
	 * MAC_MI_STAT but currently no hardware to test with.
	 */
	return 0;
}

int tg3_init(struct eth_device *dev, bd_t *bis)
{
	struct tg3_priv *tp = dev->priv;
	int err;

	if (tp->card_init_state == TG3_INIT_STATE_DONE) {
		/* Initialization has already happened. Once init'd,
		 * we leave the card init'd so we do not need to re-
		 * do this */
		if ((tp->phy_flags & TG3_PHYFLG_PHY_SERDES) ||
		    (tp->phy_flags & TG3_PHYFLG_MII_SERDES)) {
			return tg3_phy_serdes_get_link(tp);
		} else {
			return tg3_phy_copper_get_link(tp);
		}
	}

	err = tg3_power_up(tp);
	if (err)
		printf("#   Error: tgp_power_up failed [%d].\n", __LINE__);

	tg3_disable_ints(tp);
	tg3_flag_clear(tp, INIT_COMPLETE);

	err = tg3_start(tp, !(tp->phy_flags & TG3_PHYFLG_KEEP_LINK_ON_PWRDN), true);
	if (err)
		printf("#   Error: tg3_start failed [%d].\n", __LINE__);

	/* Init some TX stuff */
	tp->tx_buffer = dma_alloc_coherent(1536, &tp->tx_buffer_mapping);

	/* Init some RX stuff */
	tw32_rx_mbox(TG3_RX_STD_PROD_IDX_REG, tp->rx_producer);

	tp->card_init_state = TG3_INIT_STATE_DONE;

	return err;
}

int tg3_send(struct eth_device *dev, void *packet, int length)
{
	struct tg3_priv *tp = dev->priv;
	struct tg3_napi *tnapi = tp->napi;

	int i;
	int tx_producer = tp->tx_producer;

	if (length > 1536)
		printf("Error: Packet is more than 1536 bytes!\n");

	memcpy(tp->tx_buffer, packet, length);
	tg3_tx_set_bd(&tnapi->tx_ring[tx_producer], tp->tx_buffer_mapping,
		      length, TXD_FLAG_END | TXD_FLAG_COAL_NOW, 0, 0);

	/* Increment index and poke the card */
	tx_producer = (tx_producer + 1) % TG3_TX_RING_SIZE;
	tw32_tx_mbox(tnapi->prodmbox, tx_producer);

	/* Wait for the card to take the packet... */
	for (i = 0; i < 100000; i++) {
		if (tnapi->hw_status->idx[0].tx_consumer == tx_producer)
			break;
		udelay(10);
	}

	tp->tx_producer = tx_producer;

	return 0;
}

int tg3_write_hwaddr(struct eth_device *dev)
{
	int i;
	struct tg3_priv *tp = dev->priv;
	for (i = 0; i < 6; i++)
		tp->mac_address[i] = dev->enetaddr[i];

	__tg3_set_mac_addr(tp, false);
	return 0;
}

#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

/* Initialize cards in advance so they are closer to ready when
 * we actually want to use them */
#define PRE_INIT_MAX 8

/* This function will search for and register all Tigon3 devices
 * in the system. It will also initialize up to PRE_INIT_MAX
 * Tigon3 devices in advance so that we do not have to wait as
 * long before sending our first frame */
int tg3_register(bd_t *bis)
{
	unsigned int i, j;
	pci_dev_t devno;
	int err;
	struct eth_device *pre_init_list[PRE_INIT_MAX];
	int pre_init_list_count = 0;

	for (i = 0; (devno = pci_find_devices(tg3_supported, i)) >= 0; i++) {
		struct tg3_priv *tp;
		struct eth_device *dev;
		u32 sndmbx, rcvmbx, intmbx;
		u64 dma_mask,
		__maybe_unused persist_dma_mask;
		u32 val;

		tp = malloc(sizeof(*tp));
		if (tp == NULL)
			return -ENOMEM;

		dev = malloc(sizeof(*dev));
		if (dev == NULL) {
			free(tp);
			return -ENOMEM;
		}

		memset(dev, 0, sizeof(*dev));
		sprintf(dev->name, "Tigon3-%d", i);

		pci_read_config_dword(devno, PCI_BASE_ADDRESS_0, &val);

		dev->iobase = val;
		dev->init = tg3_init;
		dev->halt = tg3_halt;
		dev->send = tg3_send;
		dev->recv = tg3_recv;
		dev->write_hwaddr = tg3_write_hwaddr;
		dev->priv = tp;

		tp->devno = devno;
		tp->card_init_state = TG3_INIT_STATE_NONE;
		pci_read_config_word(devno, PCI_VENDOR_ID, &tp->pci_vendor);
		pci_read_config_word(devno, PCI_DEVICE_ID, &tp->pci_device);

		if (tp->pci_vendor != 0x14e4 || tp->pci_device != 0x1656)
			printf("Note: Driver untested with this card.\n");

		eth_register(dev);

		/* Try to enable I/O accesses and bus-mastering */
		val = PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
		pci_write_config_dword(tp->devno, PCI_COMMAND, val);

		/* Make sure it worked */
		pci_read_config_dword(tp->devno, PCI_COMMAND, &val);
		if (!(val & PCI_COMMAND_MEMORY)) {
			printf("#   Can't enable I/O memory\n");
			continue;
		}
		if (!(val & PCI_COMMAND_MASTER)) {
			printf("#   Can't enable bus-mastering\n");
			continue;
		}

		tp->rx_mode = TG3_DEF_RX_MODE;
		tp->tx_mode = TG3_DEF_TX_MODE;

		/* The word/byte swap controls here control register access byte
		 * swapping.  DMA data byte swapping is controlled in the GRC_MODE
		 * setting below.
		 */
		tp->misc_host_ctrl = MISC_HOST_CTRL_MASK_PCI_INT |
		    MISC_HOST_CTRL_WORD_SWAP | MISC_HOST_CTRL_INDIR_ACCESS |
		    MISC_HOST_CTRL_PCISTATE_RW;

		/* The NONFRM (non-frame) byte/word swap controls take effect on
		 * descriptor entries, anything which isn't packet data.
		 *
		 * The StrongARM chips on the board (one for tx, one for rx) are
		 * running in big-endian mode.
		 */
		tp->grc_mode = (GRC_MODE_WSWAP_DATA | GRC_MODE_BSWAP_DATA |
			GRC_MODE_WSWAP_NONFRM_DATA);

#ifdef __BIG_ENDIAN
		tp->grc_mode |= GRC_MODE_BSWAP_NONFRM_DATA;
#endif

		tp->regs = pci_map_bar(tp->devno, PCI_BASE_ADDRESS_0, PCI_REGION_MEM);

		err = tg3_get_invariants(tp);
		if (err) {
			printf("#    Problem fetching invariants of chip, aborting\n");
			continue;
		}

		/* The EPB bridge inside 5714, 5715, and 5780 and any
		 * device behind the EPB cannot support DMA addresses > 40-bit.
		 * On 64-bit systems with IOMMU, use 40-bit dma_mask.
		 * On 64-bit systems without IOMMU, use 64-bit dma_mask and
		 * do DMA address check in tg3_start_xmit().
		 */
		if (tg3_flag(tp, IS_5788)) {
			persist_dma_mask = dma_mask = DMA_BIT_MASK(32);
		} else if (tg3_flag(tp, 40BIT_DMA_BUG)) {
			persist_dma_mask = dma_mask = DMA_BIT_MASK(40);
#ifdef CONFIG_HIGHMEM
			dma_mask = DMA_BIT_MASK(64);
#endif
		} else {
			persist_dma_mask = dma_mask = DMA_BIT_MASK(64);
		}

#if 0
		/* Configure DMA attributes. */
		if (dma_mask > DMA_BIT_MASK(32)) {
			err = pci_set_dma_mask(pdev, dma_mask);
			if (!err) {
				features |= NETIF_F_HIGHDMA;
				err = pci_set_consistent_dma_mask(pdev,
								  persist_dma_mask);
				if (err < 0) {
					dev_err(&pdev->dev, "Unable to obtain 64 bit "
						"DMA for consistent allocations\n");
					goto err_out_apeunmap;
				}
			}
		}
		if (err || dma_mask == DMA_BIT_MASK(32)) {
			err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
			if (err) {
				dev_err(&pdev->dev,
					"No usable DMA configuration, aborting\n");
				goto err_out_apeunmap;
			}
		}
#endif

		tg3_init_bufmgr_config(tp);

		if (tg3_chip_rev_id(tp) == CHIPREV_ID_5705_A1 &&
		    !(tr32(TG3PCI_PCISTATE) & PCISTATE_BUS_SPEED_HIGH)) {
			tg3_flag_set(tp, MAX_RXPEND_64);
		}

		err = tg3_get_device_address(tp);
		if (err) {
			printf("#   Could not obtain valid ethernet address, aborting\n");
			continue;
		}

		/*
		 * Reset chip in case UNDI or EFI driver did not shutdown
		 * DMA self test will enable WDMAC and we'll see (spurious)
		 * pending DMA on the PCI bus at that point.
		 */
		if ((tr32(HOSTCC_MODE) & HOSTCC_MODE_ENABLE) ||
		    (tr32(WDMAC_MODE) & WDMAC_MODE_ENABLE)) {
			tw32(MEMARB_MODE, MEMARB_MODE_ENABLE);
			tg3_halt__(tp, RESET_KIND_SHUTDOWN, 1);
		}

		err = tg3_test_dma(tp);
		if (err) {
			printf("#   DMA engine test failed, aborting\n");
			continue;
		}

		intmbx = MAILBOX_INTERRUPT_0 + TG3_64BIT_REG_LOW;
		rcvmbx = MAILBOX_RCVRET_CON_IDX_0 + TG3_64BIT_REG_LOW;
		sndmbx = MAILBOX_SNDHOST_PROD_IDX_0 + TG3_64BIT_REG_LOW;

		struct tg3_napi *tnapi = &tp->napi[0];

		tnapi->tp = tp;
		tnapi->int_mbox = intmbx;
		tnapi->consmbox = rcvmbx;
		tnapi->prodmbox = sndmbx;

		for (j = 0; j < 6; j++)
			dev->enetaddr[j] = tp->mac_address[j];

		if (pre_init_list_count < PRE_INIT_MAX)
			pre_init_list[pre_init_list_count++] = dev;
	}

	/* Cards can take a little time to link up fully. To speed things
	 * up, we will init the cards now, so that we don't have to wait
	 * as long when calling tg3_init before sending a frame */
	for (i = 0; i < pre_init_list_count; i++) {
		struct eth_device *dev = pre_init_list[i];
		struct tg3_priv *tp = dev->priv;
		tp->card_init_state = TG3_INIT_STATE_PRE;
		tg3_init(dev, NULL);
	}

	return i;
}
