/*
 * am335x_genepi.h
 *
 * Copyright (C) 2017, Eaton
 * 
 * based on am335x_evm.h, which is:
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CONFIG_AM335X_GENEPI_H
#define __CONFIG_AM335X_GENEPI_H

#include <configs/ti_am335x_common.h>

#define CONFIG_SPI_FLASH_ISSI

/* Signature */
#ifdef VALIDATE_SIGNATURE
#define VALIDATE_SIGNATURE_KERNEL_ARG ""
#else
#define VALIDATE_SIGNATURE_KERNEL_ARG "signed=no "
#endif		

/* Nandflash timing definition */
/* CONFIG 7 is at 0 because it is configured by the calling function*/
#define M_NAND_GPMC_CONFIG1	0x00000800
#define M_NAND_GPMC_CONFIG2	0x00050500
#define M_NAND_GPMC_CONFIG3	0x00050401
#define M_NAND_GPMC_CONFIG4	0x04000600
#define M_NAND_GPMC_CONFIG5	0x00070909
#define M_NAND_GPMC_CONFIG6	0x84000000
#define M_NAND_GPMC_CONFIG7	0x00000F48

/* polarity 1 = active low; 0 is active high */
#define GPMC_WAIT1PIN_POLARITY 1
#define GPMC_WAIT0PIN_POLARITY 0
#define GPMC_WRITE_PROTECT_POLARITY 0
#define GPMC_LIMITED_ADDRESS 0
#define GPMC_NAND_FORCE_POSTED_WRITE 0

#ifndef CONFIG_SPL_BUILD
#define CONFIG_NAND
#define CONFIG_NAND_OMAP_GPMC
# define CONFIG_TIMESTAMP
# define CONFIG_LZO
#endif

#define GPMC_CS0_FLASH				MTD_DEV_TYPE_NAND
#define CONFIG_SYS_NAND_BASE		0x08000000
#define CONFIG_SYS_NAND_SIZE		(256*1024*1024)
#define CONFIG_SYS_MAX_NAND_DEVICE  1

/* set max size to 128 MiB */
#define CONFIG_SYS_BOOTM_LEN		(16 << 19)

#define MACH_TYPE_AM335X_GENEPI MACH_TYPE_AM335XEVM /*TODO set our own value*/
#define CONFIG_MACH_TYPE		MACH_TYPE_AM335X_GENEPI

/* Clock Defines */
#define V_OSCK				25000000  /* Clock output from T2 */
#define V_SCLK				(V_OSCK)

#define MTDIDS_DEFAULT			"nand0=s34ml02g2"
#define MTDPARTS_DEFAULT		"mtdparts=s34ml02g2:"\
	"128k(private-store)," \
	"131072k(system)," \
	"-(data)"
/*
 * We setup defaults based on constraints from the Linux kernel, which should
 * also be safe elsewhere.  We have the default load at 32MB into DDR (for
 * the kernel), FDT above 112MB (the maximum location for the end of the
 * kernel), and the ramdisk 512KB above that (allowing for hopefully never
 * seen large trees).  We say all of this must be within the first 128MB
 * as that will normally be within the kernel lowmem and thus visible via
 * bootm_size and we only run on platforms with 128MB or more of memory.
 */
#define GENEPI_LINUX_BOOT_ENV \
	"loadaddr=0x82000000\0" \
	"kernel_addr_r=0x82000000\0" \
	"fdtaddr=0x87000000\0" \
	"fdt_addr_r=0x87000000\0" \
	"bootm_size=0x8000000\0" \
	"zimage_magic_number_addr=0x82000024\0"

/* Custom script for NOR */
#define CONFIG_SYS_LDSCRIPT		"board/eaton/am335x_genepi/u-boot.lds"

/* Always 128 KiB env size */
#define CONFIG_ENV_SIZE			(128 << 10)

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define BOOTENV_DEV_LEGACY_MMC(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel #instance "=" \
	"setenv mmcdev " #instance"; "\
	"setenv bootpart " #instance":2 ; "\
	"run mmcboot\0"
#define BOOTENV_DEV_NAME_LEGACY_MMC(devtypeu, devtypel, instance) \
	#devtypel #instance " "

#define BOOTENV_DEV_NAND(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel "=" \
	"run nandboot\0"

#define BOOTENV_DEV_NAME_NAND(devtypeu, devtypel, instance) \
	#devtypel #instance " "

/*
 * These are the devices the boot will try to boot from, in order. To add devices, 
 * add a func(...) line here
 */
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) 

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
	"if test ${boot_source} = \"SPI\"; then " \
		"run nandboot; " \
	"else "\
		"run mmcboot; " \
	"fi;"

#ifndef CONFIG_SPL_BUILD
#define CONFIG_EXTRA_ENV_SETTINGS \
	GENEPI_LINUX_BOOT_ENV \
	"mtdids=" MTDIDS_DEFAULT "\0" \
	"mtdparts=" MTDPARTS_DEFAULT "\0" \
	"boot_fdt=try\0" \
	"bootpart=0:2\0" \
	"bootdir=/boot\0" \
	"bootfile=fitImage\0" \
	"console=ttyS4,115200n8 earlyprintk\0" \
	"optargs=panic=10 nohz=off\0" \
	"mmcdev=0\0" \
	"mmcboot_dtb=am335x-io2200-mmc.dtb\0" \
	"mmcroot=/dev/mmcblk0p2 ro\0" \
	"mmcrootfstype=ext4\0" \
	"nfsopts=nolock\0" \
	"bootcounter=0\0" \
	"bootcounterlimit=4\0" \
	"boot_type=sep\0" \
	"force_rescue=0\0" \
	"power_fail=0\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${mmcroot} " \
		"rootfstype=${mmcrootfstype} " \
		"boot_type=${boot_type} " \
		"\0" \
	"nandargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"boot_type=${boot_type} " \
		"boot_cause=${boot_cause} " \
		"power_fail=${power_fail} "\
		VALIDATE_SIGNATURE_KERNEL_ARG \
		"\0" \
	"loadimage=load mmc ${bootpart} ${loadaddr} ${bootdir}/${bootfile}\0" \
	"mmcloados=run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"bootm ${loadaddr}#conf@${mmcboot_dtb};" \
		"fi;\0" \
	"ubifs=  ubi part system; " \
			"ubifsmount ubi0:system; " \
		"\0" \
	"nandboot2= " \
		"if ubifsload $loadaddr /boot/${boot_type}/fitImage-initramfs-io2200-io2200.bin; then " \
			"run nandargs; " \
			"bootm $loadaddr; " \
		"fi; " \
		"\0" \
	"nandboot= " \
		"if test ${force_rescue} = 1; then " \
			"echo Booting in rescue mode (button); " \
			"setenv boot_type ses; " \
			"save_boot_data ${bootcounterlimit} ${resetflag}; " \
		"elif test ${bootcounter} -ge ${bootcounterlimit}; then " \
			"echo Booting in rescue mode (counter=${bootcounter}); " \
			"setenv boot_type ses; " \
			"save_boot_data ${bootcounterlimit} ${resetflag}; " \
		"else " \
			"echo Booting in primary mode (counter=${bootcounter}); " \
			"setenv boot_type sep; " \
			"save_boot_data ${bootcounter} ${resetflag}; " \
		"fi; " \
		"run ubifs; " \
		"run nandboot2; " \
		"setenv boot_type ses; " \
		"echo Booting in rescue mode (primary failed);" \
		"save_boot_data ${bootcounterlimit} ${resetflag}; " \
		"run nandboot2; " \
		"\0" \
	"mmcboot= mmc dev ${mmcdev}; " \
			"if mmc rescan; then " \
				"echo SD/MMC found on device ${mmcdev};" \
				"if run loadimage; then " \
					"run mmcloados;" \
				"else " \
					"echo failed to load fitImage;" \
				"fi;" \
			"fi;\0"
#endif

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_COM1		0x44e09000	/* Base GENEPI has UART0 */
#define CONFIG_SYS_NS16550_COM2		0x48022000	/* UART1 */
#define CONFIG_SYS_NS16550_COM3		0x48024000	/* UART2 */
#define CONFIG_SYS_NS16550_COM4		0x481a6000	/* UART3 */
#define CONFIG_SYS_NS16550_COM5		0x481a8000	/* UART4 */
#define CONFIG_SYS_NS16550_COM6		0x481aa000	/* UART5 */

#define CONFIG_CMD_EEPROM
#define CONFIG_ENV_EEPROM_IS_ON_I2C
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50	/* Main EEPROM */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	2

/* PMIC support */
#define CONFIG_POWER_TPS65217
#define CONFIG_POWER_TPS65910

/* SPL */
#ifndef CONFIG_NOR_BOOT
/* Bootcount using the RTC block */
/*#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_AM33XX
#define CONFIG_SYS_BOOTCOUNT_BE*/

/* USB gadget RNDIS */
#undef CONFIG_SPL_LDSCRIPT
#define CONFIG_SPL_LDSCRIPT		"arch/arm/mach-omap2/am33xx/u-boot-spl.lds"
#endif

#ifdef CONFIG_NAND

// UBI
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_RBTREE
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_LZO
#define CONFIG_MTD_UBI_WL_THRESHOLD 4096	/* From kernel config */
#define CONFIG_MTD_UBI_BEB_LIMIT 20			/* From kernel config */


/* NAND: device related configs */
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_PAGE_COUNT	(CONFIG_SYS_NAND_BLOCK_SIZE / \
					 CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_PAGE_SIZE	2048
#define CONFIG_SYS_NAND_OOBSIZE		64
#define CONFIG_SYS_NAND_BLOCK_SIZE	(128*1024)
/* NAND: driver related configs */
#define CONFIG_NAND_OMAP_GPMC
#define CONFIG_NAND_OMAP_GPMC_PREFETCH
#define CONFIG_NAND_OMAP_ELM
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	NAND_LARGE_BADBLOCK_POS
#define CONFIG_SYS_NAND_ECCPOS		{ 2, 3, 4, 5, 6, 7, 8, 9, \
					 10, 11, 12, 13, 14, 15, 16, 17, \
					 18, 19, 20, 21, 22, 23, 24, 25, \
					 26, 27, 28, 29, 30, 31, 32, 33, \
					 34, 35, 36, 37, 38, 39, 40, 41, \
					 42, 43, 44, 45, 46, 47, 48, 49, \
					 50, 51, 52, 53, 54, 55, 56, 57, }

#define CONFIG_SYS_NAND_ECCSIZE		512
#define CONFIG_SYS_NAND_ECCBYTES	14
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_NAND_OMAP_ECCSCHEME	OMAP_ECC_BCH8_CODE_HW
/*#define MTDIDS_DEFAULT			"nand0=nand.0"*/
/*#define MTDPARTS_DEFAULT		"mtdparts=nand.0:" \
					"128k(NAND.SPL)," \
					"128k(NAND.SPL.backup1)," \
					"128k(NAND.SPL.backup2)," \
					"128k(NAND.SPL.backup3)," \
					"256k(NAND.u-boot-spl-os)," \
					"1m(NAND.u-boot)," \
					"128k(NAND.u-boot-env)," \
					"128k(NAND.u-boot-env.backup1)," \
					"8m(NAND.kernel)," \
					"-(NAND.file-system)"*/
#define CONFIG_SYS_NAND_U_BOOT_OFFS	0x000c0000
/* NAND: SPL related configs */
#ifdef CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_NAND_AM33XX_BCH
#endif
#ifdef CONFIG_SPL_OS_BOOT
#define CONFIG_CMD_SPL_NAND_OFS	0x00080000 /* os parameters */
#define CONFIG_SYS_NAND_SPL_KERNEL_OFFS	0x00200000 /* kernel offset */
#define CONFIG_CMD_SPL_WRITE_SIZE	0x2000
#endif
#endif /* !CONFIG_NAND */

/*
 * For NOR boot, we must set this to the start of where NOR is mapped
 * in memory.
 */
#ifdef CONFIG_NOR_BOOT
#define CONFIG_SYS_TEXT_BASE		0x08000000
#define CONFIG_SYS_UBOOT_BASE		0x08000000
#endif

#ifdef CONFIG_USB
/*
 * USB configuration.  We enable MUSB support, both for host and for
 * gadget.  We set USB0 as peripheral and USB1 as host, based on the
 * board schematic and physical port wired to each.  Then for host we
 * add mass storage support and for gadget we add both RNDIS ethernet
 * and DFU.
 */
#define CONFIG_USB_MUSB_DSPS
#define CONFIG_USB_MUSB_PIO_ONLY
#define CONFIG_USB_MUSB_DISABLE_BULK_COMBINE_SPLIT
#define CONFIG_AM335X_USB0
#define CONFIG_AM335X_USB0_MODE	MUSB_PERIPHERAL
#define CONFIG_AM335X_USB1
#define CONFIG_AM335X_USB1_MODE MUSB_HOST

#ifndef CONFIG_SPL_USBETH_SUPPORT
/* Fastboot */
#define CONFIG_USB_FUNCTION_FASTBOOT
#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_FASTBOOT_BUF_ADDR	CONFIG_SYS_LOAD_ADDR
#define CONFIG_FASTBOOT_BUF_SIZE	0x07000000

#define CONFIG_FASTBOOT_FLASH_MMC_DEV   1
#endif

#ifdef CONFIG_USB_MUSB_GADGET
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_RNDIS
#define CONFIG_USBNET_HOST_ADDR	"de:ad:be:af:00:00"
#endif /* CONFIG_USB_MUSB_GADGET */
#endif /* CONFIG_USB */

/*
 * Disable MMC DM for SPL build and can be re-enabled after adding
 * DM support in SPL
 */
#ifdef CONFIG_SPL_BUILD
#undef CONFIG_DM_MMC
#undef CONFIG_TIMER
#undef CONFIG_DM_USB
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_USBETH_SUPPORT)
/* Remove other SPL modes. */
#define CONFIG_ENV_IS_NOWHERE
#undef CONFIG_ENV_IS_IN_NAND
/* disable host part of MUSB in SPL */
/* disable EFI partitions and partition UUID support */
#endif

/* USB Device Firmware Update support */
#ifndef CONFIG_SPL_BUILD
#define DFUARGS \
	"dfu_alt_info_emmc=rawemmc raw 0 3751936\0" \
	DFU_ALT_INFO_MMC \
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_NAND
#endif

/*
 * Default to using SPI for environment, etc.
 * 0x000000 - 0x020000 : SPL (128KiB)
 * 0x020000 - 0x0A0000 : U-Boot (512KiB)
 * 0x0A0000 - 0x0BFFFF : First copy of U-Boot Environment (128KiB)
 * 0x0C0000 - 0x0DFFFF : Second copy of U-Boot Environment (128KiB)
 * 0x0E0000 - 0x442000 : Linux Kernel
 * 0x442000 - 0x800000 : Userland
 */
#if defined(CONFIG_SPI_BOOT)
/* SPL related */
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

/*#define CONFIG_ENV_IS_IN_SPI_FLASH*/
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SECT_SIZE		(4 << 10) /* 4 KB sectors */
#define CONFIG_ENV_OFFSET		(768 << 10) /* 768 KiB in */
#define CONFIG_ENV_OFFSET_REDUND	(896 << 10) /* 896 KiB in */
/*#define MTDIDS_DEFAULT			"nor0=m25p80-flash.0"*/
/*#define MTDPARTS_DEFAULT		"mtdparts=m25p80-flash.0:128k(SPL)," \
					"512k(u-boot),128k(u-boot-env1)," \
					"128k(u-boot-env2),3464k(kernel)," \
					"-(rootfs)"*/
#define CONFIG_SYS_NAND_U_BOOT_OFFS	 0x000c0000
#elif defined(CONFIG_EMMC_BOOT)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		1
#define CONFIG_SYS_MMC_ENV_PART		2
#define CONFIG_ENV_OFFSET		0x0
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_SYS_MMC_MAX_DEVICE	2
#elif defined(CONFIG_NOR_BOOT)
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_SECT_SIZE		(128 << 10)	/* 128 KiB */
#define CONFIG_ENV_OFFSET		(512 << 10)	/* 512 KiB */
#define CONFIG_ENV_OFFSET_REDUND	(768 << 10)	/* 768 KiB */
#define MTDIDS_DEFAULT			"nor0=physmap-flash.0"
#define MTDPARTS_DEFAULT		"mtdparts=physmap-flash.0:" \
					"512k(u-boot)," \
					"128k(u-boot-env1)," \
					"128k(u-boot-env2)," \
					"4m(kernel),-(rootfs)"
#elif defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_ENV_OFFSET		0x001c0000
#define CONFIG_ENV_OFFSET_REDUND	0x001e0000
#define CONFIG_SYS_ENV_SECT_SIZE	CONFIG_SYS_NAND_BLOCK_SIZE
#elif !defined(CONFIG_ENV_IS_NOWHERE)
/* Not NAND, SPI, NOR or eMMC env, so put ENV in a file on FAT */
#define CONFIG_ENV_IS_IN_FAT
#define FAT_ENV_INTERFACE		"mmc"
#define FAT_ENV_DEVICE_AND_PART		"0:1"
#define FAT_ENV_FILE			"uboot.env"
#endif

/* SPI flash. */
#define CONFIG_SF_DEFAULT_SPEED		24000000

/* Network. */
#define CONFIG_PHY_GIGE
#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC
/* Enable Atheros phy driver */
#define CONFIG_PHY_ATHEROS

/*
 * NOR Size = 16 MiB
 * Number of Sectors/Blocks = 128
 * Sector Size = 128 KiB
 * Word length = 16 bits
 * Default layout:
 * 0x000000 - 0x07FFFF : U-Boot (512 KiB)
 * 0x080000 - 0x09FFFF : First copy of U-Boot Environment (128 KiB)
 * 0x0A0000 - 0x0BFFFF : Second copy of U-Boot Environment (128 KiB)
 * 0x0C0000 - 0x4BFFFF : Linux Kernel (4 MiB)
 * 0x4C0000 - 0xFFFFFF : Userland (11 MiB + 256 KiB)
 */
#if defined(CONFIG_NOR)
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE
#define CONFIG_SYS_FLASH_PROTECTION
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_FLASH_CFI_MTD
#define CONFIG_SYS_MAX_FLASH_SECT	128
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_FLASH_BASE		(0x08000000)
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#define CONFIG_SYS_FLASH_SIZE		0x01000000
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE
#endif  /* NOR support */

#ifdef CONFIG_DRIVER_TI_CPSW
#define CONFIG_CLOCK_SYNTHESIZER
#define CLK_SYNTHESIZER_I2C_ADDR 0x65
#endif

#endif	/* ! __CONFIG_AM335X_GENEPI_H */
