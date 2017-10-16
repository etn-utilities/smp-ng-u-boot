/*
 * based on board.c from original Genepi code
 *
 * Copyright (C) 2017, Eaton.
 *
 * which in turn was based on board.c from the TI am335x board, which is:
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <errno.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <phy.h>
#include <malloc.h>
#include <spi.h>
#include <spi_flash.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include <environment.h>
#include "board.h"
#include "configs/am335x_genepi.h"

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SKIP_LOWLEVEL_INIT

/*  Genepi DDR 400 MHz Configuration */

static const struct ddr_data ddr3_genepi400MHz_data = {
	.datardsratio0 = IS43TR16640B15GBLI_RD_DQS_400MHZ,
	.datawdsratio0 = IS43TR16640B15GBLI_WR_DQS_400MHZ,
	.datafwsratio0 = IS43TR16640B15GBLI_PHY_FIFO_WE_400MHZ,
	.datawrsratio0 = IS43TR16640B15GBLI_PHY_WR_DATA_400MHZ,
};

static const struct cmd_control ddr3_genepi400MHz_cmd_ctrl_data = {
	.cmd0csratio = IS43TR16640B15GBLI_RATIO_400MHZ,
	.cmd0iclkout = IS43TR16640B15GBLI_INVERT_CLKOUT_400MHZ,

	.cmd1csratio = IS43TR16640B15GBLI_RATIO_400MHZ,
	.cmd1iclkout = IS43TR16640B15GBLI_INVERT_CLKOUT_400MHZ,

	.cmd2csratio = IS43TR16640B15GBLI_RATIO_400MHZ,
	.cmd2iclkout = IS43TR16640B15GBLI_INVERT_CLKOUT_400MHZ,
};

static struct emif_regs ddr3_genepi400MHz_emif_reg_data = {
	.sdram_config = IS43TR16640B15GBLI_EMIF_SDCFG_400MHZ,
	.ref_ctrl = IS43TR16640B15GBLI_EMIF_SDREF_400MHZ,
	.sdram_tim1 = IS43TR16640B15GBLI_EMIF_TIM1_400MHZ,
	.sdram_tim2 = IS43TR16640B15GBLI_EMIF_TIM2_400MHZ,
	.sdram_tim3 = IS43TR16640B15GBLI_EMIF_TIM3_400MHZ,
	.zq_config = IS43TR16640B15GBLI_ZQ_CFG_400MHZ,
	.emif_ddr_phy_ctlr_1 = IS43TR16640B15GBLI_EMIF_READ_LATENCY_400MHZ,
};


/*  Genepi DDR 303 MHz Configuration */
static const struct ddr_data ddr3_genepi303MHz_data = {
	.datardsratio0 = IS43TR16sssr15GBLI_RD_DQS_303MHZ,
	.datawdsratio0 = IS43TR16sssr15GBLI_WR_DQS_303MHZ,
	.datafwsratio0 = IS43TR16sssr15GBLI_PHY_FIFO_WE_303MHZ,
	.datawrsratio0 = IS43TR16sssr15GBLI_PHY_WR_DATA_303MHZ,
};

static const struct cmd_control ddr3_genepi303MHz_cmd_ctrl_data = {
	.cmd0csratio = IS43TR16sssr15GBLI_RATIO_303MHZ,
	.cmd0iclkout = IS43TR16sssr15GBLI_INVERT_CLKOUT_303MHZ,

	.cmd1csratio = IS43TR16sssr15GBLI_RATIO_303MHZ,
	.cmd1iclkout = IS43TR16sssr15GBLI_INVERT_CLKOUT_303MHZ,

	.cmd2csratio = IS43TR16sssr15GBLI_RATIO_303MHZ,
	.cmd2iclkout = IS43TR16sssr15GBLI_INVERT_CLKOUT_303MHZ,
};

static struct emif_regs ddr3_genepi303MHz_emif_reg_data = {
	.sdram_config = IS43TR16sssr15GBLI_EMIF_SDCFG_303MHZ,
	.ref_ctrl = IS43TR16sssr15GBLI_EMIF_SDREF_303MHZ,
	.sdram_tim1 = IS43TR16sssr15GBLI_EMIF_TIM1_303MHZ,
	.sdram_tim2 = IS43TR16sssr15GBLI_EMIF_TIM2_303MHZ,
	.sdram_tim3 = IS43TR16sssr15GBLI_EMIF_TIM3_303MHZ,
	.zq_config = IS43TR16sssr15GBLI_ZQ_CFG_303MHZ,
	.emif_ddr_phy_ctlr_1 = IS43TR16sssr15GBLI_EMIF_READ_LATENCY_303MHZ,
};

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
/* Not implemented*/
	return 0;
}
#endif

#define OSC	(V_OSCK/1000000)

const struct dpll_params dpll_ddr_genepi_400MHz = {
		400, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_genepi_303MHz = {
		303, OSC-1, 1, -1, -1, -1, -1};

const struct dpll_params *get_dpll_ddr_params(void)
{	
#ifdef RAM_SPEED_303
	printf("Configure DDR DLL @ 303MHz\n");
	return &dpll_ddr_genepi_303MHz;
#else	
	printf("Configure DDR DLL @ 400MHz\n");
	return &dpll_ddr_genepi_400MHz;
#endif
}

void set_uart_mux_conf(void)
{
#if CONFIG_CONS_INDEX == 1
	enable_uart0_pin_mux();
#elif CONFIG_CONS_INDEX == 2
	enable_uart1_pin_mux();
#elif CONFIG_CONS_INDEX == 3
	enable_uart2_pin_mux();
#elif CONFIG_CONS_INDEX == 4
	enable_uart3_pin_mux();
#elif CONFIG_CONS_INDEX == 5
	enable_uart4_pin_mux();
#elif CONFIG_CONS_INDEX == 6
	enable_uart5_pin_mux();
#endif
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}


const struct ctrl_ioregs ioregs_bonelt = {
	.cm0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm2ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
};

/* Genepi */
const struct ctrl_ioregs ioregs_genepi400MHz = {
	.cm0ioctl		= IS43TR16640B15GBLI_IOCTRL_VALUE_400MHZ,
	.cm1ioctl		= IS43TR16640B15GBLI_IOCTRL_VALUE_400MHZ,
	.cm2ioctl		= IS43TR16640B15GBLI_IOCTRL_VALUE_400MHZ,
	.dt0ioctl		= IS43TR16640B15GBLI_IOCTRL_VALUE_400MHZ,
	.dt1ioctl		= IS43TR16640B15GBLI_IOCTRL_VALUE_400MHZ,
};

const struct ctrl_ioregs ioregs_genepi303MHz = {
	.cm0ioctl		= IS43TR16sssr15GBLI_IOCTRL_VALUE_303MHZ,
	.cm1ioctl		= IS43TR16sssr15GBLI_IOCTRL_VALUE_303MHZ,
	.cm2ioctl		= IS43TR16sssr15GBLI_IOCTRL_VALUE_303MHZ,
	.dt0ioctl		= IS43TR16sssr15GBLI_IOCTRL_VALUE_303MHZ,
	.dt1ioctl		= IS43TR16sssr15GBLI_IOCTRL_VALUE_303MHZ,
};

void sdram_init(void)
{
  (void) ddr3_genepi400MHz_emif_reg_data;
  (void) ddr3_genepi303MHz_emif_reg_data;  
#ifdef RAM_SPEED_303
   printf("Configure DDR timing @ 303MHz\n");
   config_ddr(303, &ioregs_genepi303MHz,
			   &ddr3_genepi303MHz_data,
			   &ddr3_genepi303MHz_cmd_ctrl_data,
			   &ddr3_genepi303MHz_emif_reg_data, 0);	
#else
   printf("Configure DDR timing @ 400MHz\n");
   config_ddr(400, &ioregs_genepi400MHz,
			   &ddr3_genepi400MHz_data,
			   &ddr3_genepi400MHz_cmd_ctrl_data,
			   &ddr3_genepi400MHz_emif_reg_data, 0);	
#endif
}
#endif

/*
 * USB threshold setting.
 */
void usb_threshold_mngt(void)
{
	unsigned int gpio;
	gpio_lookup_name("GPIO0_20", NULL, NULL, &gpio);
	gpio_request(gpio, "cmd_gpio");
	gpio_direction_output(gpio, 0);
	gpio_lookup_name("GPIO3_20", NULL, NULL, &gpio);
	gpio_request(gpio, "cmd_gpio");
	gpio_direction_output(gpio, 0);
}

/* Timing definitions for nanflash */
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
static const u32 gpmc_nandflash[GPMC_MAX_REG] = {
	M_NAND_EATON_GPMC_CONFIG1,
	M_NAND_EATON_GPMC_CONFIG2,
	M_NAND_EATON_GPMC_CONFIG3,
	M_NAND_EATON_GPMC_CONFIG4,
	M_NAND_EATON_GPMC_CONFIG5,
	M_NAND_EATON_GPMC_CONFIG6,
	M_NAND_EATON_GPMC_CONFIG7,
};
#endif

void genepi_mcu_board_init(unsigned short mpuSpeed)
{
  dpll_mpu_opp100.m = mpuSpeed;  
  do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
  struct gpmc *gpmc_cfg;
#endif
  
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
  gpmc_init();
  /*above gpmc_init program default uboot timing*/
  /*below enable_gpmc_cs_config function overwrite it with value from gpmc_nandflash*/
  gpmc_cfg = (struct gpmc *)GPMC_BASE; 
  enable_gpmc_cs_config(gpmc_nandflash, &gpmc_cfg->cs[0],
		CONFIG_SYS_NAND_BASE, GPMC_SIZE_16M);
#endif

	usb_threshold_mngt();
    
	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
#define BOARD_NAME "Genepi"

int board_late_init(void)
{
	char safe_string[HDR_NAME_LEN + 1];

	/* Now set variables based on the header. */
	strncpy(safe_string, BOARD_NAME, sizeof(BOARD_NAME));
	safe_string[sizeof(BOARD_NAME)] = 0;
	setenv("board_name", safe_string);
	setenv("bootdelay", "2");

	return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 0,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 1,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};
#endif

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
#if ((defined(CONFIG_SPL_ETH_SUPPORT) || defined(CONFIG_SPL_USBETH_SUPPORT)) \
		&& defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET)) && \
	 !defined(CONFIG_SPL_BUILD))


// initialize 88E1510 Marvell Phy
int board_phy_config(struct phy_device *phydev)
{
  //disable CLK125 output as it is not used (EMI purpose)
  phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0002);
  phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x444E);

  //set LED configuration
  phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0003);
  phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x1012);

  //return to page 0
  phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0000);


  if (phydev->drv->config)
    phydev->drv->config(phydev);

  return 0;
}


static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

int board_eth_init(bd_t *bis)
{
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");
		eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	/*if (read_eeprom(&header) < 0)
		puts("Could not get board ID.\n");*/

	// Keep Starter Kit Config (RGMI) evm
	writel((RGMII_MODE_ENABLE | RGMII_INT_DELAY), &cdev->miisel);
	cpsw_slaves[0].phy_if = cpsw_slaves[1].phy_if = PHY_INTERFACE_MODE_RGMII;

	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif

	
#endif
#if defined(CONFIG_USB_ETHER) && \
	(!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
	if (is_valid_ether_addr(mac_addr))
		eth_setenv_enetaddr("usbnet_devaddr", mac_addr);

	rv = usb_eth_initialize(bis);
	if (rv < 0)
		printf("Error %d registering USB_ETHER\n", rv);
	else
		n += rv;
#endif
	return n;

	
	return 0;
}	 

#endif







