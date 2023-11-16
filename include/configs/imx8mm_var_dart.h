/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018 NXP
 * Copyright 2018-2020 Variscite Ltd.
 */

#ifndef __IMX8MM_VAR_DART_H
#define __IMX8MM_VAR_DART_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define CONFIG_SYS_BOOTM_LEN		(64 * SZ_1M)
#define CONFIG_SPL_MAX_SIZE		(148 * 1024)
#define CONFIG_SYS_MONITOR_LEN		SZ_512K
#define CONFIG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_STACK		0x920000
#define CONFIG_SPL_BSS_START_ADDR	0x910000
#define CONFIG_SPL_BSS_MAX_SIZE		SZ_8K	/* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START	0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE	SZ_512K	/* 512 KB */

/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CONFIG_MALLOC_F_ADDR		0x930000
/* For RAW image gives a error info not panic */
#define CONFIG_SPL_ABORT_ON_RAW_IMAGE

#define CONFIG_POWER_BD71837
#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_REMAKE_ELF

#ifndef KERNEL_EXTRA_ARGS
#define KERNEL_EXTRA_ARGS ""
#endif

/* ENET Config */
#if defined(CONFIG_FEC_MXC)
#define CONFIG_ETHPRIME                 "FEC"
#define PHY_ANEG_TIMEOUT 20000

#define CONFIG_FEC_XCV_TYPE             RGMII
#endif

#ifdef CONFIG_DISTRO_DEFAULTS
#define BOOT_TARGET_DEVICES(func) \
       func(USB, usb, 0) \
       func(MMC, mmc, 1) \
       func(MMC, mmc, 2)

#include <config_distro_bootcmd.h>
/* redefine BOOTENV_EFI_SET_FDTFILE_FALLBACK to use Variscite function to load fdt */
#undef BOOTENV_EFI_SET_FDTFILE_FALLBACK
#define BOOTENV_EFI_SET_FDTFILE_FALLBACK \
	"setenv efi_dtb_prefixes; " \
	"run loadfdt; "
#else
#define BOOTENV
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x43800000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev=2\0"\
	"sd_dev=1\0" \

#if !defined(SMP_OFFICIAL_VERSION) || SMP_OFFICIAL_VERSION != 0
#define SMP_EXTRA_ENV_SETTINGS \
	"smp_official=1\0" \
	"try_boot_mmc1=no\0" \
	"try_boot_mmc2=no\0" \
	"bootdelay=-2\0" \

#else
#define SMP_EXTRA_ENV_SETTINGS \
	"smp_official=0\0" \
	"try_boot_mmc1=yes\0" \
	"try_boot_mmc2=yes\0" \
	"bootdelay=3\0" \
	"forcerescue_delay=1\0" \
	"erase_mmc2=ask confirm_erase_mmc2 'Enter FORMAT to erase the EMMC:' 6; if env exists confirm_erase_mmc2 && test ${confirm_erase_mmc2} = FORMAT ; then mmc dev 2; mmc erase 0 20480; sleep 5; reset ; fi\0" \
	"boot_cmd_3=setenv force_rescue 0; setenv bootcounter 0; run emmcboot\0" \
	"boot_cmd_4=setenv force_rescue 1; run emmcboot\0" \
	"boot_cmd_5=run erase_mmc2; bootmenu -1\0" \
	"bootmenu_0=Boot (default)=boot\0" \
	"bootmenu_1=Boot from SD Card=run mmcboot\0" \
	"bootmenu_2=Boot from EMMC=run emmcboot\0" \
	"bootmenu_3=Boot from EMMC (primary)=run boot_cmd_3\0" \
	"bootmenu_4=Boot from EMMC (rescue)=run boot_cmd_4\0" \
	"bootmenu_5=Erase EMMC=run boot_cmd_5\0" \
	"bootmenu_6=Memory test=mtest\0" \
	"bootmenu_7=Reboot=reset\0" \
	"bootmenu_default=0\0" \
	"bootmenu_show=1\0" \
	"bootmenu_delay=10\0" \

#endif

/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	SMP_EXTRA_ENV_SETTINGS \
    "bootdir=/boot\0" \
	"console=ttymxc0,115200\0" \
	"script=boot.scr\0" \
    \  
	"scriptaddr="__stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
    \
	"kernel_addr_r="__stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
    \
	"bsp_script=boot.scr\0" \
	"image=Image.gz\0" \
    \
	"img_addr="__stringify(CONFIG_IMAGE_LOAD_ADDR)"\0" \
    \
	"fdt_addr="__stringify(CONFIG_FDT_LOAD_ADDR)"\0" \
	"fdt_addr_r="__stringify(CONFIG_FDT_LOAD_ADDR)"\0" \
    \
	"fdt_high=0xffffffffffffffff\0" \
	"boot_fdt=try\0" \
	"boot_fit=no\0" \
	"fdt_file=imx8mm-var-dart-da3050.dtb\0" \
	"ip_dyn=yes\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcblk=1\0" \
	"mmcpart=1\0" \
	"mmcautodetect=no\0" \
	"mmcroot=/dev/mmcblk1p1\0" \
	"mmcrootfstype=ext4\0" \
	"emmcdev=2\0" \
	"part_store=2\0" \
	"part_kexec=5\0" \
	"part_system=7\0" \
	"m4_addr=0x7e0000\0" \
	"m4_bin=hello_world.bin\0" \
	"use_m4=no\0" \
	"optargs=panic=10 nohz=off\0" \
	"bootcounter=0\0" \
	"bootcounterlimit=4\0" \
	"boot_type=sep\0" \
	"force_rescue=0\0" \
	"power_fail=0\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"earlycon=ec_imx6q,0x30860000,115200 " \
		"${optargs} " \
		"${kernelargs} " \
		"${cma_size} " \
		"root=${mmcroot} rootwait rw " \
		"rootfstype=${mmcrootfstype} " \
		"boot_type=${boot_type} " \
		KERNEL_EXTRA_ARGS \
		"\0" \
	"emmcargs=setenv bootargs console=${console} " \
		"earlycon=ec_imx6q,0x30860000,115200 " \
		"${optargs} " \
		"${kernelargs} " \
		"${cma_size} " \
		"boot_type=${boot_type} " \
		"boot_cause=${boot_cause} " \
		"power_fail=${power_fail} " \
		"force_rescue=${force_rescue} " \
		KERNEL_EXTRA_ARGS \
		"\0" \
	"loadm4bin=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootdir}/${m4_bin}; " \
		"cp.b ${loadaddr} ${m4_addr} ${filesize}; " \
		"echo Init rsc_table region memory; " \
		"mw.b 400ff000 0 10\0" \
	"runm4bin=" \
		"if test ${m4_addr} = 0x7e0000; then " \
			"echo Booting M4 from TCM; " \
		"else " \
			"echo Booting M4 from DRAM; " \
			"dcache flush; " \
		"fi; " \
		"bootaux ${m4_addr};\0" \
	"ramsize_check="\
		"if test $sdram_size -gt 2048; then " \
			"setenv cma_size cma=320M@-2048M; " \
		"fi;\0" \
	"mmcboot=echo Booting from MMC${mmcdev} ...; " \
		"run ramsize_check; " \
		"mmc dev ${mmcdev}; " \
		"if test -e mmc ${mmcdev} ${bootdir}/${image}; then " \
			"if load mmc ${mmcdev}:${mmcpart} ${img_addr} ${bootdir}/${image}; then " \
				"unzip ${img_addr} ${loadaddr}; " \
				"if load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${bootdir}/${fdt_file}; then " \
					"run mmcargs; " \
					"booti ${loadaddr} - ${fdt_addr}; " \
				"else " \
					"echo Failed to load ${bootdir}/${fdt_file}; " \
				"fi; " \
			"else " \
				" echo Failed to load ${bootdir}/${image}; " \
			"fi; " \
		"elif test -e mmc ${mmcdev} ${bootdir}/fitImage; then " \
			"if load mmc ${mmcdev}:${mmcpart} ${img_addr} ${bootdir}/fitImage; then " \
				"run mmcargs; " \
	                        "setenv loadaddr ${img_addr}; " \
				"bootm ${img_addr}; " \
			"else " \
				"echo Failed to load ${bootdir}/fitImage; " \
			"fi; " \
		"else " \
			"echo Failed to load image from MMC${mmcdev} (no kernel found); " \
		"fi;\0" \
	"emmcboot2= " \
		"if test -e mmc ${emmcdev}:${part_system} /boot/${boot_type}/kernel.bin; then " \
			"load mmc ${emmcdev}:${part_system} ${img_addr} /boot/${boot_type}/kernel.bin; " \
			"run emmcargs; " \
	                "setenv loadaddr ${img_addr}; "	      \
			"bootm ${img_addr}; " \
		"else " \
			"echo No kernel found in /boot/${boot_type}; " \
		"fi; " \
		"\0" \
	"emmcboot=echo Booting from MMC${emmcdev} ...; " \
		"run ramsize_check; " \
		"mmc dev ${emmcdev}; " \
		"if test ${force_rescue} != 0; then " \
			"save_boot_data ${bootcounterlimit}; " \
			"echo Booting in rescue mode (button); " \
			"setenv boot_type ses; " \
			"setenv force_rescue 1; " \
		"elif test ${bootcounter} -ge ${bootcounterlimit}; then " \
			"save_boot_data ${bootcounterlimit}; " \
			"echo Booting in rescue mode (counter=${bootcounter}); " \
			"setenv boot_type ses; " \
			"if test ${bootcounter} -eq 128; then " \
				"setenv force_rescue 4; " \
			"elif test ${bootcounter} -eq 129; then " \
				"setenv force_rescue 5; " \
			"else " \
				"setenv force_rescue 2; " \
			"fi; " \
		"elif test -e mmc ${emmcdev}:${part_system} /boot/diag/kernel.bin ; then " \
			"save_boot_data ${bootcounter}; " \
			"echo Booting in diagnostics mode; " \
			"setenv boot_type diag; " \
			"run emmcboot2; " \
		"else " \
			"save_boot_data ${bootcounter}; " \
			"echo Booting in primary mode (counter=${bootcounter}); " \
			"setenv boot_type sep; " \
		"fi; " \
		"if test -e mmc ${emmcdev}:${part_kexec} /boot/kernel.bin; then " \
			"load mmc ${emmcdev}:${part_kexec} ${img_addr} /boot/kernel.bin; " \
			"run emmcargs; " \
                        "setenv loadaddr ${img_addr}; " \
			"bootm ${img_addr}; " \
		"fi; " \
		"run emmcboot2; " \
		"echo Booting in rescue mode (${boot_type} failed);" \
		"setenv boot_type ses; " \
		"setenv force_rescue 3; " \
		"save_boot_data ${bootcounterlimit}; " \
		"run emmcboot2; " \
		"echo Failed to start the rescue; " \
		"sleep 5; " \
		"reset; " \
		"\0" \

#define CONFIG_BOOTCOMMAND \
	"if mmc dev ${bootmmcdev} && mmc rescan; then " \
		"if test ${bootmmcdev} = ${mmcdev}; then " \
			"if mmc dev ${emmcdev} && test ${try_boot_mmc2} = yes && test -e mmc ${emmcdev}:${part_store} /try-boot-mmc2; then " \
				"if test ${bootcounter} -lt ${bootcounterlimit}; then " \
					"check_force_rescue; " \
				"fi; " \
				"run emmcboot; " \
			"fi; " \
			"run mmcboot; " \
			"echo Failed to start kernel from MMC1; " \
		"elif test ${bootmmcdev} = ${emmcdev}; then " \
			"if test ${try_boot_mmc1} = yes && test -e mmc ${emmcdev}:${part_store} /try-boot-mmc1 && mmc dev 1; then " \
				"run mmcboot; " \
			"fi; " \
			"if test ${bootcounter} -lt ${bootcounterlimit}; then " \
				"check_force_rescue; " \
			"fi; " \
			"run emmcboot; " \
			"echo Failed to start kernel from MMC2; " \
		"else " \
			"echo Unknown boot device; " \
		"fi; " \
	"else " \
		"echo Failed to MMC rescan; " \
	"fi;"

/* Link Definitions */
#define CONFIG_LOADADDR             0x40600000
#define CONFIG_SYS_LOAD_ADDR        CONFIG_LOADADDR
#define CONFIG_IMAGE_LOAD_ADDR      0x46000000
#define CONFIG_FDT_LOAD_ADDR        0x48000000

#define CONFIG_SYS_INIT_RAM_ADDR        0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE        0x200000
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_MMC_ENV_DEV		1   /* USDHC2 */

/* DDR configs */
#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#define DEFAULT_SDRAM_SIZE		(512 * SZ_1M) /* 512MB Minimum DDR4, see get_dram_size */
#define VAR_EEPROM_DRAM_START          (CONFIG_SYS_MEMTEST_START + \
					(DEFAULT_SDRAM_SIZE >> 1))

#define CONFIG_MXC_UART_BASE		UART1_BASE_ADDR

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_IMX_BOOTAUX

/* USDHC */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* USB configs */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_USBD_HS
#endif

#define CONFIG_USB_GADGET_VBUS_DRAW 2

#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2

/* Carrier board EEPROM */
#define CARRIER_EEPROM_BUS_SOM		0x02
#define CARRIER_EEPROM_BUS_DART		0x01
#define CARRIER_EEPROM_ADDR		0x54

#if defined(CONFIG_ANDROID_SUPPORT)
#include "imx8mm_var_dart_android.h"
#endif

#endif
