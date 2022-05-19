/*
 * Copyright 2018 NXP
 * Copyright 2018-2020 Variscite Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
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

#include "../common/imx8_eeprom.h"
#include "imx8mm_var_dart.h"

#include <bootdata/bootdata.h>

DECLARE_GLOBAL_DATA_PTR;
static u32 g_boot_device = SMP_BOOT_UNKNOWN;
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

int set_boot_device(void)
{
	g_boot_device = smp_get_current_mmc_device();

	switch (g_boot_device)
	{
		case 1:
			printf("Booting from: MMC1 - SD CARD \n");
			env_set("boot_source", "MMC1");
			g_boot_device = SMP_BOOT_SD_CARD;
		break;
		case 2:
			printf("Booting from: MMC2 - INTERNAL EMMC\n");
			env_set("boot_source", "MMC2");
			g_boot_device = SMP_BOOT_INTERNAL_EMMC;
		break;
		default:
			printf("Booting from: unknown: %08x\ngd: %p\n", g_boot_device, gd);
			env_set("boot_source", "0");
			g_boot_device = SMP_BOOT_UNKNOWN;
		break;
	}

	return 0;
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

static iomux_v3_cfg_t const uart1_pads[] = {
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

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
int board_usb_init(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, true);

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);

	return 0;
}
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
	char carrier_rev[16] = {0};
	
	struct eaton_boot_data_struct boot_data = {0};
	unsigned long boot_count;
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

		int carrier_rev = var_detect_dart_carrier_rev();

		env_set("board_name", "DART-MX8M-MINI");

		if (carrier_rev == DART_CARRIER_REV_2)
			env_set("carrier_rev", "dt8m-2.x");
		else
			env_set("carrier_rev", "legacy");
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	set_boot_device();

	if(g_boot_device == SMP_BOOT_INTERNAL_EMMC)
	{
		rc = bootdata_read(&boot_data);		
		if (rc != 0)
		{
			printf("Error reading bootdata.bin\n");
			memset(&boot_data, 0, sizeof(boot_data));
		}

		boot_count = boot_data.boot_count;

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
				boot_count++;
		// }

		sprintf(boot_count_str, "%d", boot_count);
		env_set("bootcounter", boot_count_str);
		sprintf(boot_count_str, "%d", BOOT_COUNTER_LIMIT);
		env_set("bootcounterlimit", boot_count_str);


	//TODO Force rescue wi 
	// if (g_boot_device == BOOT_DEVICE_SPI && boot_count < BOOT_COUNTER_LIMIT) {
	// 	printf("Press the front panel button to force rescue mode in %d seconds", FORCE_BOOT_RESCUE_DELAY / 1000);
	// 	timer_start = get_timer(0);
	// 	timer_delay_print = 0;
	// 	while ((timer_delay = get_timer(timer_start)) < FORCE_BOOT_RESCUE_DELAY) {
	// 		/*
	// 		 * The GPIO is active low and we want to stop looping as soon as it is activated.
	// 		 */
	// 		front_button_value = genepi_read_front_button();
	// 		if(front_button_value == 0)
	// 			break;

	// 		if (timer_delay >= timer_delay_print) {
	// 			timer_delay_print += 1000;
	// 			printf(".");
	// 		}
	// 	}
	// 	printf("\n");
	// }

	// env_set("force_rescue", front_button_value == 1 ? "0" : "1");

	}

	return 0;
}

#ifndef CONFIG_SPL_BUILD
int do_save_boot_data(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct eaton_boot_data_struct boot_data = {0};
	int rc = -1;
	
	if (argc < 2)
	{
		printf("Insufficient input arguments\n");
		return -1;
	}

	if (g_boot_device == SMP_BOOT_INTERNAL_EMMC)
	{
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
	}

	return rc;
}

U_BOOT_CMD(save_boot_data, 3, 0, do_save_boot_data,
			"save_boot_data - Save the bootstruct to emmc memory. \n",
			"save_boot_data boot_count boot_limit reset_flag\n The maximum value is 0xff or 255");
#endif
