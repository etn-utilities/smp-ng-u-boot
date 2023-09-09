/*
 * Copyright 2018 NXP
 * Copyright 2018-2020 Variscite Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <linux/delay.h>
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
#include <inttypes.h>

#include "../common/extcon-ptn5150.h"
#include "../common/imx8_eeprom.h"
#include "imx8mm_var_dart.h"

#include <asm/mach-imx/eaton-smp.h>

#ifndef CONFIG_SPL_BUILD
#include <bootdata/bootdata.h>
#endif

DECLARE_GLOBAL_DATA_PTR;
#define BOOT_COUNTER_LIMIT		BOOT_COUNT_LIMIT
#define FORCE_BOOT_RESCUE_DELAY 5

#define LATCH_GPIO_BANK				4
#define LATCH_GPIO_OFFSET			1
#define POWER_FAIL_GPIO_BANK		1
#define POWER_FAIL_GPIO_OFFSET		11
#define FRONT_BUTTON_GPIO_BANK		1
#define FRONT_BUTTON_GPIO_OFFSET	0

#define BOOTDATA_IFACE		"mmc"
#define BOOTDATA_PART		"2:2"
#define BOOTDATA_FILENAME 	"/bootdata.bin"

extern int var_setup_mac(struct var_eeprom *eeprom);

#ifndef CONFIG_SPL_BUILD
static int smp_read_gpio_value(int bank_number, int offset);
static int smp_set_gpio_value(int bank_number, int offset, int value);
#endif

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

#ifndef CONFIG_SPL_BUILD
static int bootdata_read(struct eaton_boot_data_struct *boot_data)
{
	if (boot_data == NULL)
		return -1;

	if (fs_set_blk_dev(BOOTDATA_IFACE, BOOTDATA_PART, FS_TYPE_EXT))
		return -2;

	loff_t len_read = 0;
	int ret = fs_read(BOOTDATA_FILENAME, (ulong)boot_data, 0, sizeof(struct eaton_boot_data_struct), &len_read);
	if (ret < 0)
		return -3;

	return 0;
}

static int bootdata_write(const struct eaton_boot_data_struct *boot_data)
{
	if (boot_data == NULL)
		return -1;

	if (fs_set_blk_dev(BOOTDATA_IFACE, BOOTDATA_PART, FS_TYPE_EXT))
		return -2;

	loff_t len_written = 0;
	int ret = fs_write(BOOTDATA_FILENAME, (ulong)boot_data, 0, sizeof(struct eaton_boot_data_struct), &len_written);
	if (ret < 0)
		return -3;

	return sizeof(eaton_boot_data_struct);
}

static int bootdata_compare_write(const struct eaton_boot_data_struct *boot_data)
{
	if (boot_data == NULL)
		return -1;

	struct eaton_boot_data_struct r = {0};
	if (bootdata_read(&r) >= 0 && memcmp(&r, boot_data, sizeof(struct eaton_boot_data_struct)) == 0)
		return 0;

	return bootdata_write(boot_data);
}

extern void hab_late_init(bool fuse_device, bool close_device, bool check_boot, bool field_return, bool reset_after, uint8_t extra_cmds);
extern bool imx_hab_is_required(void);
extern int hab_late_init(struct eaton_boot_data_struct *boot_data);
extern bool imx_hab_is_enabled(void);

int board_early_check_serial_console(void)
{	
	if (mmc_get_env_dev() == 1) // 1: SD Card
	{
		printf("Serial console is needed for SD card\n");
		return 0;
	}

	struct eaton_boot_data_struct boot_data = {0};
	bootdata_read(&boot_data);
	if (boot_data.serial_console == 255 || (boot_data.serial_console == 0 && imx_hab_is_enabled() && (boot_data.ignore_dev_board != 0 || !smp_is_dev_board(false))))
	{
		printf("Disabling the serial console\n");

		gd->flags |= GD_FLG_SILENT | GD_FLG_DISABLE_CONSOLE;
		env_set("silent", "1");
	}
	else if (boot_data.serial_console >= 128)
	{
		printf("=================================================\n");
		printf("Serial console would be disabled from this point\n");
		printf("=================================================\n");
	}
	else
	{
		printf("Serial console is enabled\n");

		char buf[64];
		snprintf(buf, sizeof(buf), "serial_console=%d", boot_data.serial_console);
		buf[sizeof(buf) - 1] = 0;
		env_set("board_init_env_arg", buf);
		return 0;
	}

	printf("Serial console is disabled\n");
	env_set("console", "ttynull");
	env_set("earlyconsole_arg", "");
	env_set("board_init_env_arg", "quiet silent loglevel=0 serial_console=-1 systemd.show_status=false");

	return 0;
}

int do_check_bootdata_serial_console(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	if (mmc_get_env_dev() == 1) // SD card
		return 0;

	struct eaton_boot_data_struct boot_data = {0};
	bootdata_read(&boot_data);

	if (boot_data.serial_console >= 128 || (boot_data.serial_console == 0 && imx_hab_is_enabled() && (boot_data.ignore_dev_board != 0 || !smp_is_dev_board(false))))
	{
		printf("=================================================\n");
		printf("Disabling the serial console for good\n");
		printf("=================================================\n");
		gd->flags |= GD_FLG_SILENT | GD_FLG_DISABLE_CONSOLE;
		env_set("silent", "1");
		return 0;
	}	

	return 0;
}

U_BOOT_CMD(check_bootdata_serial_console, 1, 0, do_check_bootdata_serial_console,
			"Check bootdata serial console value and set environment values.\n",
			"");

#endif


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


#ifdef CONFIG_FEC_MXC
	var_setup_mac(ep);
#endif
	var_eeprom_print_prod_info(ep);

	printf("Serial ID: 0x%" PRIX64 " %s\n", smp_board_serial(), smp_is_dev_board(true) ? "(dev)" : "");

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

	u32 mmc_boot_dev = mmc_get_env_dev();
	env_set_ulong("bootmmcdev", mmc_boot_dev);

// The following code was included into the SPL increasing the size unintentionally
#ifndef CONFIG_SPL_BUILD
	struct eaton_boot_data_struct boot_data = {0};
	char buf[64];
	int rc = 0;
	u32 boot_cause;
	int read_pflatch_count = 0;

	rc = bootdata_read(&boot_data);
	if (rc < 0)
	{
		printf("Error reading bootdata.bin\n");
		memset(&boot_data, 0, sizeof(boot_data));
	}

	env_set("ignore_dev_board", boot_data.ignore_dev_board ? "yes" : "no");
	if (boot_data.ignore_dev_board)
		printf("This dev board will behave like it was not (ignore_dev_board)\n");

	unsigned char boot_count = boot_data.boot_count;
	if (smp_read_gpio_value(POWER_FAIL_GPIO_BANK, POWER_FAIL_GPIO_OFFSET))
	{
		printf("Power fail detected\n");
		env_set("power_fail", "1");
		while (read_pflatch_count < 3 && smp_read_gpio_value(POWER_FAIL_GPIO_BANK, POWER_FAIL_GPIO_OFFSET))
		{
			if (smp_set_gpio_value(LATCH_GPIO_BANK, LATCH_GPIO_OFFSET, 1))
			{
				return -1;
			}
			udelay(1000);
			read_pflatch_count++;
		}
	}
	else if (boot_count < BOOT_COUNTER_LIMIT)
	{
		boot_count++;
	}

	env_set_ulong("bootcounter", boot_count);
	env_set_ulong("bootcounterlimit", BOOT_COUNTER_LIMIT);
	
	boot_cause = get_imx_reset_cause();
	sprintf(buf, "%09x", boot_cause); //we only want the 9 LSBs - bits 10-31 are reserved
	env_set("boot_cause", buf);

	if (mmc_boot_dev != 1 && imx_hab_is_enabled() && (boot_data.ignore_dev_board != 0 || !smp_is_dev_board(false)))
	{
		// Disable the boot menu
		env_set("bootdelay", "-2");
		env_set("bootmenu_show", "0");

		env_set("try_boot_mmc1", "no");
		env_set("try_boot_mmc1", "no");
	}

	// JTAG + HAB lock
	bool bReset = false;

	// HAB
    uint8_t prod_close_sequence = 0;
	bool fuse_device  = false;
	bool close_device = false;
	bool check_boot  = false;
	bool field_return = false;
	bool reset_after = false;

	if (mmc_boot_dev == 2)
	{
		fuse_device         = boot_data.fuse_device_flag  ? true : false;
		close_device        = boot_data.close_device_flag ? true : false;
		check_boot          = boot_data.check_boot_flag   ? true : false;
		field_return        = boot_data.field_return_flag ? true : false;
		prod_close_sequence = boot_data.prod_close_sequence;
		
		// Sanity check.
		if (prod_close_sequence > 2) 
		{
			prod_close_sequence = 2;
		}

		if (fuse_device || close_device || check_boot || field_return || prod_close_sequence) 
		{
			uint8_t seq = prod_close_sequence;

			if (seq)
				seq--;

			boot_data.fuse_device_flag = 0;
			boot_data.close_device_flag = 0;
			boot_data.check_boot_flag = 0;
			boot_data.field_return_flag = 0;
			boot_data.prod_close_sequence = seq;
			rc = bootdata_write(&boot_data);
		}

		if (prod_close_sequence == 2) 
		{
			fuse_device = true;
			close_device = false;
			field_return = false;
			check_boot = false;
			reset_after = true;
		}	
		else if (prod_close_sequence == 1) 
		{
			fuse_device = false;
			close_device = true;
			field_return = false;
			check_boot = false;
			reset_after = false;
		}	
	}

#if defined(SMP_OFFICIAL_VERSION) && SMP_OFFICIAL_VERSION != 0
    hab_late_init(fuse_device, close_device, check_boot, field_return, reset_after, 0);
#else
	hab_late_init(false, false, false, false, false, 0);
#endif

	if (hab_late_init(&boot_data) > 0)
		bReset = true;

	rc = bootdata_compare_write(&boot_data);
	if (rc < 0)
	{
		printf("Error writing bootdata.bin\n");
	}
	else if (bReset)
	{	
		do_reset(NULL, 0, 0, NULL);
	}
#endif	

	return 0;
}

#ifndef CONFIG_SPL_BUILD

/**
 *  Returns 1 if the GPIO value is 1 (high), returns 0 otherwise
 **/
static int smp_read_gpio_value(int bank_number, int offset)
{
	int ret;
	int val = 0;
	struct udevice *dev = NULL;
	char buf[80];
	
	for (ret = uclass_first_device(UCLASS_GPIO, &dev);
	     dev;
	     ret = uclass_next_device(&dev)) {
		const char *bank_name;
		int num_bits;
		int banklen;
		bank_name = gpio_get_bank_info(dev, &num_bits);
		banklen = bank_name ? strlen(bank_name) : 0;
		snprintf(buf, sizeof(buf), "GPIO%d_", bank_number);
		if(!strncasecmp(buf, bank_name, banklen)) {
			struct dm_gpio_ops *ops = gpio_get_ops(dev);
			val = ops->get_value(dev, offset);
			break;
		}
	}

	return val;
}

/**
 *  Changes value of the GPIO, returns -1 in case of error otherwise returns 0;
 **/
static int smp_set_gpio_value(int bank_number, int offset, int value)
{
	int ret;
	struct udevice *dev = NULL;
	char buf[80];

	for (ret = uclass_first_device(UCLASS_GPIO, &dev);
	     dev;
	     ret = uclass_next_device(&dev)) {
		const char *bank_name;
		int num_bits;
		int banklen;
		bank_name = gpio_get_bank_info(dev, &num_bits);
		banklen = bank_name ? strlen(bank_name) : 0;
		snprintf(buf, sizeof(buf), "GPIO%d_", bank_number);
		if(!strncasecmp(buf, bank_name, banklen)) {
			struct dm_gpio_ops *ops = gpio_get_ops(dev);
			if(ops->direction_output(dev, offset, value) || ops->set_value(dev, offset, value))
			{
				return -1;
			}
			break;
		}
	}

	return 0;
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
		front_button_value = smp_read_gpio_value(FRONT_BUTTON_GPIO_BANK, FRONT_BUTTON_GPIO_OFFSET);
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
	if (argc < 2)
	{
		printf("Insufficient input arguments\n");
		return CMD_RET_USAGE;
	}

	int rc = bootdata_read(&boot_data);
	if (rc < 0)
	{
		printf("Error reading bootdata.bin\n");
		memset(&boot_data, 0, sizeof(boot_data));
	}

	unsigned char boot_count = (u8)simple_strtoul(argv[1], NULL, 10);
	if (boot_count != boot_data.boot_count)
	{
		boot_data.boot_count = boot_count;
		rc = bootdata_write(&boot_data);
		if (rc < 0)
		{
			printf("Error writing bootdata.bin\n");
			return CMD_RET_FAILURE;
		}
		printf("Boot counter saved: %d\n", boot_count);
	}
	else
	{
		printf("Boot counter is already at %d\n", boot_count);
	}
	
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(save_boot_data, 2, 0, do_save_boot_data,
			"Save the bootstruct to emmc memory.",
			"counter\n The maximum value is 0xff or 255");
#endif
