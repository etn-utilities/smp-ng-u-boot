/*
 * mux.c for am335x_genepi board, based on original genepi BSP by Eaton
 *
 * Copyright (C) 2017, Eaton
 *
 * which was based on the mux.c for am335x, which is
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <i2c.h>
#include <spl.h>
#include "board.h"

static struct module_pin_mux gpio_usb1_drvvbus_mux[] = {
	{OFFSET(usb1_drvvbus), (MODE(7) | PULLUDEN)},	/* GPIO3_13 */
	{-1},
};

static struct module_pin_mux gpio_usb_threshold_mux[] = {
	{OFFSET(xdma_event_intr1), (MODE(7) | PULLUDDIS)},	/* GPIO0_20 */
	{OFFSET(mcasp0_axr1), (MODE(7) | PULLUDDIS)},	/* GPIO3_20 */	
	{-1},
};

static struct module_pin_mux gpio_phy_it_mux[] = {
	{OFFSET(mcasp0_ahclkx), (MODE(7) | PULLUP_EN | RXACTIVE)},	/* GPIO3_21 */
	{-1},
};

static struct module_pin_mux uart0_pin_mux[] = {
	{OFFSET(uart0_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART0_RXD */
	{OFFSET(uart0_txd), (MODE(0) | PULLUDEN)},		/* UART0_TXD */
	{-1},
};

static struct module_pin_mux uart1_pin_mux[] = {
	{OFFSET(uart1_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART1_RXD */
	{OFFSET(uart1_txd), (MODE(0) | PULLUDEN)},		/* UART1_TXD */
	{-1},
};

static struct module_pin_mux uart2_pin_mux[] = {
	{OFFSET(spi0_sclk), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART2_RXD */
	{OFFSET(spi0_d0), (MODE(1) | PULLUDEN)},		/* UART2_TXD */
	{-1},
};

static struct module_pin_mux uart3_pin_mux[] = {
	{OFFSET(spi0_cs1), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART3_RXD */
	{OFFSET(ecap0_in_pwm0_out), (MODE(1) | PULLUDEN)},	/* UART3_TXD */
	{-1},
};

static struct module_pin_mux uart4_pin_mux[] = {
	{OFFSET(uart0_ctsn), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART4_RXD */
	{OFFSET(uart0_rtsn), (MODE(1) | PULLUDEN)},		/* UART4_TXD */
	{-1},
};

static struct module_pin_mux uart5_pin_mux[] = {
	{OFFSET(lcd_data9), (MODE(4) | PULLUP_EN | RXACTIVE)},	/* UART5_RXD */
	{OFFSET(lcd_data8), (MODE(4) | PULLUDEN)},		/* UART5_TXD */
	{-1},
};

static struct module_pin_mux mmc0_pin_mux[] = {
	{OFFSET(mmc0_dat3), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT3 */
	{OFFSET(mmc0_dat2), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT2 */
	{OFFSET(mmc0_dat1), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT1 */
	{OFFSET(mmc0_dat0), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT0 */
	{OFFSET(mmc0_clk), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_CLK */
	{OFFSET(mmc0_cmd), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_CMD */
	{OFFSET(mcasp0_aclkr), (MODE(4) | RXACTIVE)},		/* MMC0_WP */
	{OFFSET(spi0_cs1), (MODE(5) | RXACTIVE | PULLUP_EN)},	/* MMC0_CD */
	{-1},
};

static struct module_pin_mux i2c0_pin_mux[] = {
	{OFFSET(i2c0_sda), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
	{OFFSET(i2c0_scl), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux spi0_pin_mux[] = {
	{OFFSET(spi0_sclk), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_SCLK */
	{OFFSET(spi0_d0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_D0 */
	{OFFSET(spi0_d1), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_D1 */
	{OFFSET(spi0_cs0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_CS0 */
	{-1},
};

static struct module_pin_mux rgmii1_pin_mux[] = {
	{OFFSET(mii1_txen), MODE(2)},			/* RGMII1_TCTL */
	{OFFSET(mii1_rxdv), MODE(2) | RXACTIVE},	/* RGMII1_RCTL */
	{OFFSET(mii1_txd3), MODE(2)},			/* RGMII1_TD3 */
	{OFFSET(mii1_txd2), MODE(2)},			/* RGMII1_TD2 */
	{OFFSET(mii1_txd1), MODE(2)},			/* RGMII1_TD1 */
	{OFFSET(mii1_txd0), MODE(2)},			/* RGMII1_TD0 */
	{OFFSET(mii1_txclk), MODE(2)},			/* RGMII1_TCLK */
	{OFFSET(mii1_rxclk), MODE(2) | RXACTIVE},	/* RGMII1_RCLK */
	{OFFSET(mii1_rxd3), MODE(2) | RXACTIVE},	/* RGMII1_RD3 */
	{OFFSET(mii1_rxd2), MODE(2) | RXACTIVE},	/* RGMII1_RD2 */
	{OFFSET(mii1_rxd1), MODE(2) | RXACTIVE},	/* RGMII1_RD1 */
	{OFFSET(mii1_rxd0), MODE(2) | RXACTIVE},	/* RGMII1_RD0 */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN},/* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},	/* MDIO_CLK */
	{-1},
};

static struct module_pin_mux nand_pin_mux[] = {
	{OFFSET(gpmc_ad0),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD0  */
	{OFFSET(gpmc_ad1),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD1  */
	{OFFSET(gpmc_ad2),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD2  */
	{OFFSET(gpmc_ad3),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD3  */
	{OFFSET(gpmc_ad4),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD4  */
	{OFFSET(gpmc_ad5),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD5  */
	{OFFSET(gpmc_ad6),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD6  */
	{OFFSET(gpmc_ad7),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD7  */
#ifdef CONFIG_SYS_NAND_BUSWIDTH_16BIT
	{OFFSET(gpmc_ad8),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD8  */
	{OFFSET(gpmc_ad9),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD9  */
	{OFFSET(gpmc_ad10),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD10 */
	{OFFSET(gpmc_ad11),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD11 */
	{OFFSET(gpmc_ad12),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD12 */
	{OFFSET(gpmc_ad13),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD13 */
	{OFFSET(gpmc_ad14),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD14 */
	{OFFSET(gpmc_ad15),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD15 */
#endif
	{OFFSET(gpmc_wait0),	(MODE(0) | PULLUP_EN | RXACTIVE)}, /* nWAIT */
	{OFFSET(gpmc_wpn),	(MODE(7) | PULLUP_EN)},		   /* nWP */
	{OFFSET(gpmc_csn0),	(MODE(0) | PULLUP_EN)},		   /* nCS */
	{OFFSET(gpmc_wen),	(MODE(0) | PULLDOWN_EN)},	   /* WEN */
	{OFFSET(gpmc_oen_ren),	(MODE(0) | PULLDOWN_EN)},	   /* OE */
	{OFFSET(gpmc_advn_ale),	(MODE(0) | PULLDOWN_EN)},	   /* ADV_ALE */
	{OFFSET(gpmc_be0n_cle),	(MODE(0) | PULLDOWN_EN)},	   /* BE_CLE */
	{-1},
};
#if defined(CONFIG_NOR)
static struct module_pin_mux bone_norcape_pin_mux[] = {
	{OFFSET(gpmc_a0), MODE(0) | PULLUDDIS},			/* NOR_A0 */
	{OFFSET(gpmc_a1), MODE(0) | PULLUDDIS},			/* NOR_A1 */
	{OFFSET(gpmc_a2), MODE(0) | PULLUDDIS},			/* NOR_A2 */
	{OFFSET(gpmc_a3), MODE(0) | PULLUDDIS},			/* NOR_A3 */
	{OFFSET(gpmc_a4), MODE(0) | PULLUDDIS},			/* NOR_A4 */
	{OFFSET(gpmc_a5), MODE(0) | PULLUDDIS},			/* NOR_A5 */
	{OFFSET(gpmc_a6), MODE(0) | PULLUDDIS},			/* NOR_A6 */
	{OFFSET(gpmc_a7), MODE(0) | PULLUDDIS},			/* NOR_A7 */
	{OFFSET(gpmc_ad0), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD0 */
	{OFFSET(gpmc_ad1), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD1 */
	{OFFSET(gpmc_ad2), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD2 */
	{OFFSET(gpmc_ad3), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD3 */
	{OFFSET(gpmc_ad4), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD4 */
	{OFFSET(gpmc_ad5), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD5 */
	{OFFSET(gpmc_ad6), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD6 */
	{OFFSET(gpmc_ad7), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD7 */
	{OFFSET(gpmc_ad8), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD8 */
	{OFFSET(gpmc_ad9), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD9 */
	{OFFSET(gpmc_ad10), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD10 */
	{OFFSET(gpmc_ad11), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD11 */
	{OFFSET(gpmc_ad12), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD12 */
	{OFFSET(gpmc_ad13), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD13 */
	{OFFSET(gpmc_ad14), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD14 */
	{OFFSET(gpmc_ad15), MODE(0) | PULLUDDIS | RXACTIVE},	/* NOR_AD15 */
	{OFFSET(gpmc_csn0), MODE(0) | PULLUDEN | PULLUP_EN},     /* CE */
	{OFFSET(gpmc_advn_ale), MODE(0) | PULLUDEN | PULLDOWN_EN}, /* ALE */
	{OFFSET(gpmc_oen_ren), MODE(0) | PULLUDEN | PULLDOWN_EN},/* OEn_REN */
	{OFFSET(gpmc_be0n_cle), MODE(0) | PULLUDEN | PULLDOWN_EN},/* unused */
	{OFFSET(gpmc_wen), MODE(0) | PULLUDEN | PULLDOWN_EN},    /* WEN */
	{OFFSET(gpmc_wait0), MODE(0) | PULLUDEN | PULLUP_EN | RXACTIVE},/*WAIT*/
	{-1},
};
#endif

static struct module_pin_mux ethernet_hard_reset_pin_mux[] = {
	{OFFSET(gpmc_ad8),	(MODE(7) | PULLUP_EN | RXACTIVE | PULLUDEN)}, /* gpio1_24  */
	{-1},
};

static struct module_pin_mux delay_reset_pin_mux[] = {
	{OFFSET(gpmc_ad9), (MODE(7) | PULLUDDIS)}, /* DELAY_RESET */
	{-1},
};

static struct module_pin_mux pf_latch_pin_mux[] = {
	{OFFSET(lcd_vsync), (MODE(7) | PULLUDDIS)},				/* GPIO2_22 */
	{OFFSET(gpmc_ad10), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* PF-Latch GPIO */
	{-1},
};

#if defined(CONFIG_NOR_BOOT)
void enable_norboot_pin_mux(void)
{
	configure_module_pin_mux(bone_norcape_pin_mux);
}
#endif

void enable_uart0_pin_mux(void)
{
	configure_module_pin_mux(uart0_pin_mux);
}

void enable_uart1_pin_mux(void)
{
	configure_module_pin_mux(uart1_pin_mux);
}

void enable_uart2_pin_mux(void)
{
	configure_module_pin_mux(uart2_pin_mux);
}

void enable_uart3_pin_mux(void)
{
	configure_module_pin_mux(uart3_pin_mux);
}

void enable_uart4_pin_mux(void)
{
	configure_module_pin_mux(uart4_pin_mux);
}

void enable_uart5_pin_mux(void)
{
	configure_module_pin_mux(uart5_pin_mux);
}

void enable_i2c0_pin_mux(void)
{
	configure_module_pin_mux(i2c0_pin_mux);
}

void enable_ethernet_hard_reset_pin_mux(void)
{
	configure_module_pin_mux(ethernet_hard_reset_pin_mux);
}

void enable_usb_threshold_pin_mux(void)
{
	configure_module_pin_mux(gpio_usb_threshold_mux);
}
u32 spl_boot_device(void);
void enable_board_pin_mux()
{
	// Ethernet Config
	configure_module_pin_mux(rgmii1_pin_mux); // From starter kit
	
	// Config SDCard

	switch (spl_boot_device()) 
	{
	  case BOOT_DEVICE_MMC1:
	  case BOOT_DEVICE_MMC2:
	  case BOOT_DEVICE_MMC2_2:
            configure_module_pin_mux(mmc0_pin_mux);
	  break;
	}
	
	// Config NOR (boot from SPI)
	configure_module_pin_mux(spi0_pin_mux);
	
	// CONFIG_NAND ?
	configure_module_pin_mux(nand_pin_mux);	

	// I2C bus
	enable_i2c0_pin_mux();
	
	// GPIO USB_host
	//	configure_module_pin_mux(gpio_usb1_drvvbus_mux);
	// For warning compilation
	(void) gpio_usb1_drvvbus_mux;
	// GPIO USB_threshold
	enable_usb_threshold_pin_mux();

	// gpio phy interrupt
	configure_module_pin_mux(gpio_phy_it_mux);

	configure_module_pin_mux(delay_reset_pin_mux);
	configure_module_pin_mux(pf_latch_pin_mux);

	enable_ethernet_hard_reset_pin_mux();
}

