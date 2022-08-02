/*
 * Copyright 2018 NXP
 * Copyright 2018-2020 Variscite Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <env.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include <dm.h>
#include <command.h>
#include <mmc.h>
#include <blk.h>
#include <fs.h>

#include "../common/extcon-ptn5150.h"
#include "../common/imx8_eeprom.h"
#include "imx8mm_var_dart.h"

#include <bootdata/bootdata.h>

DECLARE_GLOBAL_DATA_PTR;
#define BOOT_COUNTER_LIMIT		BOOT_COUNT_LIMIT
#define FORCE_BOOT_RESCUE_DELAY 5000

#define BOOTDATA_IFACE		"mmc"
#define BOOTDATA_PART		"2:2"
#define BOOTDATA_FILENAME 	"/bootdata.bin"

extern int var_setup_mac(struct var_eeprom *eeprom);

#define GPIO_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1 | PAD_CTL_PUE | PAD_CTL_PE)

#ifdef CONFIG_SPL_BUILD
#define ID_GPIO 	IMX_GPIO_NR(2, 11)

static iomux_v3_cfg_t const id_pads[] = {
	IMX8MM_PAD_SD1_STROBE_GPIO2_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

int get_board_id(void)
{
	static int board_id = UNKNOWN_BOARD;
	board_id = DART_MX8M_MINI;

	if (board_id != UNKNOWN_BOARD)
		return board_id;

	imx_iomux_v3_setup_multiple_pads(id_pads, ARRAY_SIZE(id_pads));
	gpio_request(ID_GPIO, "board_id");
	gpio_direction_input(ID_GPIO);

	board_id = gpio_get_value(ID_GPIO) ? DART_MX8M_MINI : VAR_SOM_MX8M_MINI;

	return board_id;
}
#else
int get_board_id(void)
{
	static int board_id = UNKNOWN_BOARD;
	board_id = DART_MX8M_MINI;

	if (board_id != UNKNOWN_BOARD)
		return board_id;

	if (of_machine_is_compatible("variscite,imx8mm-var-som"))
		board_id = VAR_SOM_MX8M_MINI;
	else if (of_machine_is_compatible("variscite,imx8mm-var-dart"))
		board_id = DART_MX8M_MINI;
	else
		board_id = UNKNOWN_BOARD;

	return board_id;
}

#endif

int var_get_som_rev(struct var_eeprom *ep)
{
	switch (ep->somrev) {
	case 0:
		return SOM_REV_10;
	case 1:
		return SOM_REV_11;
	case 2:
		return SOM_REV_12;
	case 3:
		return SOM_REV_13;
	default:
		return UNKNOWN_REV;
	}
}

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

#if 0
static iomux_v3_cfg_t const uart1_pads[] = {
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#else
static iomux_v3_cfg_t const uart1_pads[] = {
	IMX8MM_PAD_SAI2_RXC_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_SAI2_RXFS_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif


static iomux_v3_cfg_t const uart4_pads[] = {
	IMX8MM_PAD_UART4_RXD_UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART4_TXD_UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

extern struct mxc_uart *mxc_base;

int board_early_init_f(void)
{
	int id;
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	id = get_board_id();

	if (id == DART_MX8M_MINI) {
		init_uart_clk(0);
		imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	}else if (id == VAR_SOM_MX8M_MINI) {
		init_uart_clk(3);
		mxc_base = (struct mxc_uart *)UART4_BASE_ADDR;
		imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
	}

	return 0;
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}
#endif

#ifdef CONFIG_CI_UDC

#ifdef CONFIG_EXTCON_PTN5150
static struct extcon_ptn5150 usb_ptn5150;
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, true);

#if (!defined(CONFIG_SPL_BUILD) && defined(CONFIG_EXTCON_PTN5150))
	if (index == 0) {
		/* Verify port is in proper mode */
		int phy_mode = extcon_ptn5150_phy_mode(&usb_ptn5150);

		//Only verify phy_mode if ptn5150 is initialized
		if (phy_mode >= 0 && phy_mode != init)
			return -ENODEV;
	}
#endif

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);

	return 0;
}

#ifdef CONFIG_EXTCON_PTN5150
int board_ehci_usb_phy_mode(struct udevice *dev)
{
	int usb_phy_mode = extcon_ptn5150_phy_mode(&usb_ptn5150);

	/* Default to host mode if not connected */
	if (usb_phy_mode < 0)
		usb_phy_mode = USB_INIT_HOST;

	return usb_phy_mode;
}
#endif
#endif

int board_init(void)
{
#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

#define DART_CARRIER_DETECT_GPIO IMX_GPIO_NR(3, 14)

static iomux_v3_cfg_t const dart_carrier_detect_pads[] = {
	IMX8MM_PAD_NAND_DQS_GPIO3_IO14 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static int var_detect_dart_carrier_rev(void)
{
	static int dart_carrier_rev = DART_CARRIER_REV_UNDEF;

	imx_iomux_v3_setup_multiple_pads(dart_carrier_detect_pads,
				ARRAY_SIZE(dart_carrier_detect_pads));

	gpio_request(DART_CARRIER_DETECT_GPIO, "dart_carrier_detect");
	gpio_direction_input(DART_CARRIER_DETECT_GPIO);

	if (gpio_get_value(DART_CARRIER_DETECT_GPIO))
		dart_carrier_rev = DART_CARRIER_REV_1;
	else
		dart_carrier_rev = DART_CARRIER_REV_2;

	return dart_carrier_rev;
}

static int bootdata_read(struct eaton_boot_data_struct *boot_data)
{
	loff_t len_read = 0;
	int ret;

	if (fs_set_blk_dev(BOOTDATA_IFACE, BOOTDATA_PART, FS_TYPE_EXT))
		return 1;

	ret = fs_read(BOOTDATA_FILENAME, (ulong)boot_data, 0, sizeof(struct eaton_boot_data_struct), &len_read);
	if (ret < 0)
		return 2;

	return 0;
}

static int bootdata_write(const struct eaton_boot_data_struct *boot_data)
{
	loff_t len_written = 0;
	int ret;

	if (fs_set_blk_dev(BOOTDATA_IFACE, BOOTDATA_PART, FS_TYPE_EXT))
		return 1;

	ret = fs_write(BOOTDATA_FILENAME, (ulong)boot_data, 0, sizeof(struct eaton_boot_data_struct), &len_written);
	if (ret < 0)
		return 2;

	return 0;
}

#define SDRAM_SIZE_STR_LEN 5
int board_late_init(void)
{
	int som_rev;
	char sdram_size_str[SDRAM_SIZE_STR_LEN];
	int id = get_board_id();
	struct var_eeprom *ep = VAR_EEPROM_DATA;
	struct var_carrier_eeprom carrier_eeprom;
	char carrier_rev[CARRIER_REV_LEN] = {0};

#ifdef CONFIG_EXTCON_PTN5150
	extcon_ptn5150_setup(&usb_ptn5150);
#endif

	struct eaton_boot_data_struct boot_data = {0};
	char boot_count_str[5];
	int rc = 0;

#ifdef CONFIG_FEC_MXC
	var_setup_mac(ep);
#endif
	var_eeprom_print_prod_info(ep);

	som_rev = var_get_som_rev(ep);

	snprintf(sdram_size_str, SDRAM_SIZE_STR_LEN, "%d", (int) (gd->ram_size / 1024 / 1024));
	env_set("sdram_size", sdram_size_str);

	if (id == VAR_SOM_MX8M_MINI) {
		env_set("board_name", "VAR-SOM-MX8M-MINI");
		env_set("console", "ttymxc3,115200");
		switch (som_rev) {
		case SOM_REV_10:
			env_set("som_rev", "som_rev10");
			break;
		case SOM_REV_11:
			env_set("som_rev", "som_rev11");
			break;
		case SOM_REV_12:
			env_set("som_rev", "som_rev12");
			break;
		case SOM_REV_13:
			env_set("som_rev", "som_rev13");
			break;
		}
		var_carrier_eeprom_read(CARRIER_EEPROM_BUS_SOM, CARRIER_EEPROM_ADDR, &carrier_eeprom);
		var_carrier_eeprom_get_revision(&carrier_eeprom, carrier_rev, sizeof(carrier_rev));
		env_set("carrier_rev", carrier_rev);
	}
	else if (id == DART_MX8M_MINI) {
		env_set("board_name", "DART-MX8M-MINI");

		var_carrier_eeprom_read(CARRIER_EEPROM_BUS_DART, CARRIER_EEPROM_ADDR, &carrier_eeprom);
		var_carrier_eeprom_get_revision(&carrier_eeprom, carrier_rev, sizeof(carrier_rev));
		env_set("carrier_rev", carrier_rev);
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	env_set_ulong("bootmmcdev", mmc_get_env_dev());

	rc = bootdata_read(&boot_data);
	if (rc != 0)
	{
		printf("Error reading bootdata.bin\n");
		memset(&boot_data, 0, sizeof(boot_data));
	}

	//TODO Power fail wi
	// if (genepi_read_pflatch())
	// {
	// 	env_set("power_fail", "1");
	// 	while (read_pflatch_count < 3 && genepi_read_pflatch()) {
	// 		if (genepi_clear_pflatch() < 0) {
	// 			printf("error while clearing the pflatch\n");
	// 			return -1;
	// 		}

	// 		read_pflatch_count++;
	// 	}
	// }
	// else
	// {
			boot_data.boot_count++;
	// }

	sprintf(boot_count_str, "%d", boot_data.boot_count);
	env_set("bootcounter", boot_count_str);
	sprintf(boot_count_str, "%d", BOOT_COUNTER_LIMIT);
	env_set("bootcounterlimit", boot_count_str);

	return 0;
}

#ifndef CONFIG_SPL_BUILD

/**
 *  Returns 1 if GPIO1_0 value is 1 (high), returns 0 otherwise
 **/
static int smp_read_front_button(void)
{
	int ret;
	int val = 0;
	struct udevice *dev = NULL;
	
	for (ret = uclass_first_device(UCLASS_GPIO, &dev);
	     dev;
	     ret = uclass_next_device(&dev)) {
		const char *bank_name;
		int num_bits;
		int banklen;
		bank_name = gpio_get_bank_info(dev, &num_bits);
		banklen = bank_name ? strlen(bank_name) : 0;
		if(!strncasecmp("GPIO1_", bank_name, banklen)) {
			struct dm_gpio_ops *ops = gpio_get_ops(dev);
			val = ops->get_value(dev, 0);
			break;
		}
	}

	return val;
}


int do_check_force_rescue(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int front_button_value = 1;
	char *delay_str = NULL;
	int delay = FORCE_BOOT_RESCUE_DELAY;

	if (argc >= 2)
		delay_str = argv[1];

	if (!delay_str)
		delay_str = env_get("forcerescue_delay");

	if (delay_str)
		delay = (int)simple_strtol(delay_str, NULL, 10);

	if (delay > 0)
		printf("Press the front panel button to force rescue mode in %d seconds", delay);

	unsigned long timer_start = get_timer(0);
	unsigned long timer_delay_print = 0;
	unsigned long timer_delay = 0;
	do
	{
		/*
			* The GPIO is active low and we want to stop looping as soon as it is activated.
		*/
		front_button_value = smp_read_front_button();
		if (front_button_value == 0)
			break;

		if (timer_delay >= timer_delay_print)
		{
			timer_delay_print += 1000;
			if (delay > 0)
				printf(".");
		}
	}
	while ((timer_delay = get_timer(timer_start)) < (delay * 1000));

	if (delay > 0)
		printf("\n");

	return env_set("force_rescue", front_button_value != 0 ? "0" : "1");
}

U_BOOT_CMD(check_force_rescue, 2, 0, do_check_force_rescue,
			"Check if the force-rescue button is pressed before it timeout.\n",
			"timeout\n The timeout in seconds.");


int do_save_boot_data(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	struct eaton_boot_data_struct boot_data = {0};
	int rc = -1;

	if (argc < 2)
	{
		printf("Insufficient input arguments\n");
		return -1;
	}

	rc = bootdata_read(&boot_data);
	if (rc != 0)
	{
		printf("Error reading bootdata.bin\n");
		memset(&boot_data, 0, sizeof(boot_data));
	}

	boot_data.boot_count = (u8)simple_strtoul(argv[1], NULL, 10);

	rc = bootdata_write(&boot_data);
	if (rc != 0)
	{
		printf("Error writing bootdata.bin\n");
		memset(&boot_data, 0, sizeof(boot_data));
	}

	return rc;
}

U_BOOT_CMD(save_boot_data, 2, 0, do_save_boot_data,
			"Save the bootstruct to emmc memory.",
			"counter\n The maximum value is 0xff or 255");
#endif
