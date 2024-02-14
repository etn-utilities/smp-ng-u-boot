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
#include <linux/mtd/rawnand.h>
#include <linux/mtd/omap_gpmc.h>
#include "configs/am335x_genepi.h"
#include <linux/mtd/mtd.h>
#include <jffs2/jffs2.h>
#include <nand.h>
#include <command.h>
#include <linux/delay.h>
#include <asm/omap_common.h>

#include <bootdata/bootdata.h>

DECLARE_GLOBAL_DATA_PTR;
static u32 boot_device;
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#define BOOT_COUNTER_LIMIT		BOOT_COUNT_LIMIT
#define FORCE_BOOT_RESCUE_DELAY 5000
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
	gpio_request(GPIO_TO_PIN(1, 24), "gpmc_ad8");
	gpio_direction_output(GPIO_TO_PIN(1, 24), 1);
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

void genepi_mcu_board_init(unsigned short mpuSpeed)
{
  dpll_mpu_opp100.m = mpuSpeed;  
  do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

static void genepi_set_delay_reset_low(void)
{
	int ret;
	unsigned int gpio0_23 = GPIO_TO_PIN(0,23);

	ret = gpio_request(gpio0_23, "gpmc_ad9");

	if (ret)
		printf("genepi_set_delay_reset_low: requesting pin %u (GPIO0_23) failed\n", gpio0_23);

	gpio_direction_output(gpio0_23, 0);
}

/**
 *  Returns 1 if GPIO0_26 value is 1 (high), returns 0 otherwise
 **/
static int genepi_read_front_button(void)
{
	int ret, value;
	unsigned int gpio0_27 = GPIO_TO_PIN(0,27);

	ret = gpio_request(gpio0_27, "gpmc_ad11");
	if (ret && ret != -EBUSY) {
		printf("genepi_read_front_button: requesting pin %u (GPIO0_27) failed\n", gpio0_27);
		return 0;
	}

	gpio_direction_input(gpio0_27);
	value = gpio_get_value(gpio0_27);

	gpio_free(gpio0_27);

	return value;
}

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
#ifndef CONFIG_SPL_BUILD
int board_init(void)
{
	// Write PRM_RSTTIME register to set reset duration to maximum (in number of clock cycles)
	writel(0x00001fff, PRM_RSTTIME);

#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
	printf("Watchdog Hardware: enabled\n");
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif

	genepi_set_delay_reset_low();
/*
	usb_threshold_mngt();
  */  
	return 0;
}

int save_boot_source(void)
{
	boot_device = gd->arch.omap_boot_device;
	switch ( boot_device ){
		case BOOT_DEVICE_SPI:
			printf("Booting from: SPI\n");
			env_set("boot_source", "SPI");
		break;
		case BOOT_DEVICE_MMC1:
			printf("Booting from: MMC1\n");
			env_set("boot_source", "MMC1");
		break;
		case BOOT_DEVICE_MMC2:
			printf("Booting from: MMC2\n");
			env_set("boot_source", "MMC2");
		break;
		case BOOT_DEVICE_NAND:
			printf("Booting from: NAND\n");
			env_set("boot_source", "NAND");
		break;
		case BOOT_DEVICE_NAND_I2C:
			printf("Booting from: NAND I2C\n");
			env_set("boot_source", "I2C");
		break;
		default:
			printf("Booting from: unknown: %08x\ngd: %p\n", boot_device, gd);
			env_set("boot_source", "0");
		break;
	}

	return 0;
}

int write_nand_boot_data(struct eaton_boot_data_struct *boot_data)
{
	struct mtd_info *mtd;
	nand_erase_options_t opts;
	int dev = 0;
	mtd = get_nand_dev_by_index(0);
	loff_t offset, size;
	loff_t maxsize = sizeof(struct eaton_boot_data_struct);
	int rc = 0;

	if (mtd_arg_off	("private-store", &dev, &offset, &size, &maxsize, MTD_DEV_TYPE_NAND, get_nand_dev_by_index(dev)->size)){
		printf("error: mtd_arg_off\n");;
	}

	mtd = get_nand_dev_by_index(0);
	memset(&opts, 0, sizeof(opts));
	opts.offset = offset;
	opts.length = size;
	opts.jffs2 = 0;
	opts.quiet = 0;
	opts.spread = 0;

	rc = nand_erase_opts(mtd, &opts);
	if (rc < 0) {
		printf("error: erasing nand\n");
		return rc;
	}

	size = sizeof(struct eaton_boot_data_struct);
	rc = nand_write(get_nand_dev_by_index(0), offset, (size_t *)&size, (uint8_t *)boot_data);
	if (rc < 0)
	{
		printf("error: nand_write\n");
		return rc;
	}

	return rc;
}

int read_nand_boot_data(struct eaton_boot_data_struct *boot_data)
{
	int dev = 0;
	loff_t maxsize, offset;
	size_t size;
	int rc = 0;

	if (mtd_arg_off	("private-store", &dev, &offset, (loff_t *)&size, &maxsize, MTD_DEV_TYPE_NAND, get_nand_dev_by_index(dev)->size)){
		printf("error: mtd_arg_off\n");
		return -1;
	}

	size = sizeof(struct eaton_boot_data_struct);
	rc = nand_read(get_nand_dev_by_index(dev), offset, &size, (uint8_t *)boot_data);
	if (rc < 0){
		printf("error: failed reading from nand\n");
		/* We don't return an error code at this point */
		/* Returning an error here would brick the boot */
	}

	return 0;
}

/**
 *  Returns 1 if GPIO0_26 value is 1 (high), return 0 otherwise
 **/
static int genepi_read_pflatch(void)
{
	int ret, value;
	unsigned int gpio0_26 = GPIO_TO_PIN(0,26);

	ret = gpio_request(gpio0_26, "gpmc_ad10");
	if (ret && ret != -EBUSY) {
		printf("genepi_read_pflatch: requesting pin %u (GPIO0_26) failed\n", gpio0_26);
		return 0;
	}

	gpio_direction_input(gpio0_26);
	value = gpio_get_value(gpio0_26);

	gpio_free(gpio0_26);

	return value;
}

/**
 *  Returns 0 on success, otherwise -1
 * */
static int genepi_clear_pflatch(void)
{
	int ret;
	unsigned int gpio2_22 = GPIO_TO_PIN(2,22);

	ret = gpio_request(gpio2_22, "lcd_vsync");
	if (ret && ret != -EBUSY) {
		printf("genepi_clear_pflatch: requesting pin %u (GPIO2_22) failed\n", gpio2_22);
		return -1;
	}

	gpio_direction_output(gpio2_22, 1);
	mdelay(100);

	gpio_set_value(gpio2_22, 0);
	gpio_free(gpio2_22);

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
#define BOARD_NAME "Genepi"
int board_late_init(void)
{
	char safe_string[HDR_NAME_LEN + 1];
	struct eaton_boot_data_struct boot_data;
	unsigned long boot_count;
	char boot_count_str[5];
	char reset_flag_str[5];
	u32 timer_start, timer_delay, timer_delay_print;
	int front_button_value = 1;
	int rc = 0;
	u32 boot_cause;
	char boot_cause_str[10];
	int read_pflatch_count = 0;

	/* Now set variables based on the header. */
	strncpy(safe_string, BOARD_NAME, sizeof(BOARD_NAME));
	safe_string[sizeof(BOARD_NAME)] = 0;
	env_set("board_name", safe_string);
	
	/* Reading boot cause from PRM_RSTST register */
	boot_cause = readl(PRM_RSTST);
	writel(boot_cause, PRM_RSTST);
	sprintf(boot_cause_str, "%08x", boot_cause);
	env_set("boot_cause", boot_cause_str);

	save_boot_source();

	rc = read_nand_boot_data(&boot_data);
	if (rc < 0) {
		printf("error reading from nand boot\n");
		return -1;
	}
	sprintf(reset_flag_str, "%d", boot_data.factory_reset);
	env_set("resetflag", reset_flag_str);

	boot_count = 255 - boot_data.boot_count;

	if (genepi_read_pflatch())
	{
		env_set("power_fail", "1");
		while (read_pflatch_count < 3 && genepi_read_pflatch()) {
			if (genepi_clear_pflatch() < 0) {
				printf("error while clearing the pflatch\n");
				return -1;
			}

			read_pflatch_count++;
		}
	}
	else
	{
		if (boot_count < BOOT_COUNTER_LIMIT)
		{
			boot_count++;
		}
	}

	sprintf(boot_count_str, "%d", boot_count);
	env_set("bootcounter", boot_count_str);
	sprintf(boot_count_str, "%d", BOOT_COUNTER_LIMIT);
	env_set("bootcounterlimit", boot_count_str);

	if (boot_device == BOOT_DEVICE_SPI && boot_count < BOOT_COUNTER_LIMIT) {
		printf("Press the front panel button to force rescue mode in %d seconds", FORCE_BOOT_RESCUE_DELAY / 1000);
		timer_start = get_timer(0);
		timer_delay_print = 0;
		while ((timer_delay = get_timer(timer_start)) < FORCE_BOOT_RESCUE_DELAY) {
			/*
			 * The GPIO is active low and we want to stop looping as soon as it is activated.
			 */
			front_button_value = genepi_read_front_button();
			if(front_button_value == 0)
				break;

			if (timer_delay >= timer_delay_print) {
				timer_delay_print += 1000;
				printf(".");
			}
		}
		printf("\n");
	}

	env_set("force_rescue", front_button_value == 1 ? "0" : "1");

	return 0;
}
#endif
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
	int n = 0;
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
		eth_env_set_enetaddr("ethaddr", mac_addr);
	}
	
#endif
#if defined(CONFIG_USB_ETHER) && \
	(!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
	if (is_valid_ether_addr(mac_addr))
		eth_env_set_enetaddr("usbnet_devaddr", mac_addr);

	rv = usb_eth_initialize(bis);
	if (rv < 0)
		printf("Error %d registering USB_ETHER\n", rv);
	else
		n += rv;
#endif
	return n;
}	 

#endif

#ifndef CONFIG_SPL_BUILD
int do_save_boot_data(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct eaton_boot_data_struct boot_data;
	unsigned long boot_count;
	unsigned long reset_flag;	
	int rc = 0;

	if ( argc != 3){
		return -1;
	}	

	boot_count = simple_strtoul(argv[1], NULL, 10);
	reset_flag= simple_strtoul(argv[2], NULL, 10);

	read_nand_boot_data(&boot_data);

	boot_data.boot_count = boot_count > 255 ? 0 : 255 - (u8)boot_count;
	boot_data.factory_reset = reset_flag > 255 ? 255 : (u8)reset_flag;

	rc = write_nand_boot_data(&boot_data);
	if (rc < 0) {
		printf("error writing to nand\n");
		return -1;
	}

	return rc;
}

int board_early_init_f(void)
{
	return 0;
}

U_BOOT_CMD(save_boot_data, 3, 0, do_save_boot_data,
			"save_boot_data - Save the bootstruct to nand memory. \n",
			"save_boot_data boot_count boot_limit reset_flag\n The maximum value is 0xff or 255");

#endif
