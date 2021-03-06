/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the phytec PCM-052 SoM.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/imx-regs.h>

#define CONFIG_VF610

#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SYS_THUMB_BUILD

#define CONFIG_SKIP_LOWLEVEL_INIT

/* Enable passing of ATAGs */
#define CONFIG_CMDLINE_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 2 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F

#define CONFIG_FSL_LPUART
#define LPUART_BASE			UART1_BASE

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_UART_PORT		(1)
#define CONFIG_BAUDRATE			115200

#undef CONFIG_CMD_IMLS

/* NAND support */
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS
#define CONFIG_SYS_NAND_ONFI_DETECTION

#ifdef CONFIG_CMD_NAND
#define CONFIG_USE_ARCH_MEMCPY
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		NFC_BASE_ADDR

#define CONFIG_JFFS2_NAND

/* UBI */
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_RBTREE
#define CONFIG_LZO

/* Dynamic MTD partition support */
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE
#define MTDIDS_DEFAULT			"nand0=NAND,nor0=qspi0-a,nor1=qspi0-b"
#define MTDPARTS_DEFAULT		"mtdparts=NAND:256k(spare)"\
					",384k(bootloader)"\
					",128k(env1)"\
					",128k(env2)"\
					",3840k(kernel)"\
					",-(rootfs)"\
					",qspi0-a:-(jffs2),qspio0-b:-(jffs2)"
#endif

#define CONFIG_MMC
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_ESDHC_NUM	1

/*#define CONFIG_ESDHC_DETECT_USE_EXTERN_IRQ1*/
#define CONFIG_SYS_FSL_ERRATUM_ESDHC135
#define CONFIG_SYS_FSL_ERRATUM_ESDHC111
#define CONFIG_SYS_FSL_ERRATUM_ESDHC_A001

#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_FEC_MXC_PHYADDR          0
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL

/* QSPI Configs*/
#define CONFIG_FSL_QSPI

#ifdef CONFIG_FSL_QSPI
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define FSL_QSPI_FLASH_SIZE		(1 << 24)
#define FSL_QSPI_FLASH_NUM		2
#define CONFIG_SYS_FSL_QSPI_LE
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC_I2C3
#define CONFIG_SYS_I2C_MXC

/* RTC (actually an RV-4162 but M41T62-compatible) */
#define CONFIG_CMD_DATE
#define CONFIG_RTC_M41T62
#define CONFIG_SYS_I2C_RTC_ADDR 0x68
#define CONFIG_SYS_RTC_BUS_NUM 2

/* EEPROM (24FC256) */
#define CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_EEPROM_ADDR 0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 2
#define CONFIG_SYS_I2C_EEPROM_BUS 2

#define CONFIG_BOOTDELAY		3

#define CONFIG_LOADADDR			0x82000000

/* We boot from the gfxRAM area of the OCRAM. */
#define CONFIG_SYS_TEXT_BASE		0x3f408000
#define CONFIG_BOARD_SIZE_LIMIT		524288

#define CONFIG_BOOTCOMMAND              "run bootcmd_sd"
#define CONFIG_EXTRA_ENV_SETTINGS                                       \
	"bootfile=uImage\0"                             \
	"bootargs_base=setenv bootargs rw mem=256M "                    \
		"console=ttymxc1,115200n8\0"            \
	"bootargs_sd=setenv bootargs ${bootargs} "                      \
		"root=/dev/mmcblk0p2 rootwait\0"        \
	"bootargs_net=setenv bootargs ${bootargs} root=/dev/nfs ip=dhcp " \
		"nfsroot=${serverip}:${nfs_root},v3,tcp\0"              \
	"bootargs_nand=setenv bootargs ${bootargs} "                    \
		"root=/dev/mtdblock2 rootfstype=jffs2\0"                \
	"bootargs_mtd=setenv bootargs ${bootargs} ${mtdparts}\0"        \
	"bootcmd_sd=run bootargs_base bootargs_sd bootargs_mtd; mmc rescan; " \
		"fatload mmc 0:1 ${loadaddr} ${bootfile}; bootm ${loadaddr}\0" \
	"bootcmd_net=run bootargs_base bootargs_net bootargs_mtd; "     \
		"tftpboot ${loadaddr} ${tftploc}${bootfile}; bootm\0"   \
	"bootcmd_nand='run bootargs_base bootargs_nand bootargs_mtd; "  \
		"nand read ${loadaddr} 0x000E0000 0x3C0000; "           \
		"bootm ${loadaddr}\0"                                   \
	"tftploc=/path/to/tftp/directory/\0"                            \
	"nfs_root=/path/to/nfs/root\0"                                  \
	"mtdparts=" MTDPARTS_DEFAULT "\0"                               \
	"update_kernel_from_sd=mw.b $(loadaddr) 0xff 0x3C0000; "        \
		"mmc rescan; fatload mmc 0:2 ${loadaddr} ${bootfile}; " \
		"nand erase 0xE0000 0x3C0000; "                         \
		"nand write.i ${loadaddr} 0xE0000 0x3C0000\0"           \
	"update_rootfs_from_tftp=mw.b ${loadaddr} 0xff 0x8F20000; "     \
		"tftp ${loadaddr} ${tftp}${filesys}; "                  \
		"nand erase 0x4A0000 0x8F20000; "                       \
		"nand write.i ${loadaddr} 0x4A0000 0x8F20000\0"         \
	"filesys=rootfs.jffs2\0"

/* miscellaneous commands */
#define CONFIG_CMD_ELF

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
#define CONFIG_SYS_HUSH_PARSER		/* use "hush" command parser */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE		\
			(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16	/* max number of command args */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80010000
#define CONFIG_SYS_MEMTEST_END		0x87C00000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/*
 * Stack sizes
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE		(128 * 1024)	/* regular stack */

/* Physical memory map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			(0x80000000)
#define PHYS_SDRAM_SIZE			(256 * 1024 * 1024)

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_OFFSET		(12 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif

#ifdef CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_SECT_SIZE		(128 * 1024)
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_OFFSET		0x80000
#define CONFIG_ENV_SIZE_REDUND		(8 * 1024)
#define CONFIG_ENV_OFFSET_REDUND	0xA0000
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#endif
