/*
 * board.h, based on board.h from the original Genepi BSP
 *
 * Copyright (C) 2017, Eaton
 *
 * which in turn was based on board.h from ../../ti/am335x, which is:
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _BOARD_H_
#define _BOARD_H_

#define HDR_NAME_LEN		8

/*DDR3 timing definition*/
/* IS43TR16640B15GBLI 400 MHz*/
/*based on -15H memory speed grade as recommended by TI*/
#define IS43TR16640B15GBLI_EMIF_READ_LATENCY_400MHZ 0x00100007
#define IS43TR16640B15GBLI_EMIF_TIM1_400MHZ 		0x0AAAE4DB
#define IS43TR16640B15GBLI_EMIF_TIM2_400MHZ 		0x286B7FDA
#define IS43TR16640B15GBLI_EMIF_TIM3_400MHZ 		0x501F867F
#define IS43TR16640B15GBLI_EMIF_SDCFG_400MHZ 		0x61C052B2
#define IS43TR16640B15GBLI_EMIF_SDREF_400MHZ 		0x00000C30
#define IS43TR16640B15GBLI_ZQ_CFG_400MHZ 		0x50074BE4
#define IS43TR16640B15GBLI_DLL_LOCK_DIFF_400MHZ 	0x1
#define IS43TR16640B15GBLI_RATIO_400MHZ 		0x80
#define IS43TR16640B15GBLI_INVERT_CLKOUT_400MHZ 	0x0
#define IS43TR16640B15GBLI_RD_DQS_400MHZ 		0x35
#define IS43TR16640B15GBLI_WR_DQS_400MHZ 		0x3C
#define IS43TR16640B15GBLI_PHY_FIFO_WE_400MHZ 		0x9D
#define IS43TR16640B15GBLI_PHY_WR_DATA_400MHZ 		0x76
#define IS43TR16640B15GBLI_IOCTRL_VALUE_400MHZ 		0x18B

/* IS43TR16640B15GBLI 303 MHz*/
/* IS43TR16128C15GBLI 303 MHz*/
/* based on -15H memory speed grade as recommended by TI */
/* sss	: memory size	*/
/* r	: die rev	*/
#define IS43TR16sssr15GBLI_EMIF_READ_LATENCY_303MHZ 	0x00100007
#define IS43TR16sssr15GBLI_EMIF_TIM1_303MHZ 		0x0888A39B
#define IS43TR16sssr15GBLI_EMIF_TIM2_303MHZ 		0x26517FDA
#define IS43TR16sssr15GBLI_EMIF_TIM3_303MHZ 		0x501F84EF
#define IS43TR16sssr15GBLI_EMIF_SDCFG_303MHZ 		0x61C052B2
#define IS43TR16sssr15GBLI_EMIF_SDREF_303MHZ 		0x93B
#define IS43TR16sssr15GBLI_ZQ_CFG_303MHZ 		0x50074BE4
#define IS43TR16sssr15GBLI_DLL_LOCK_DIFF_303MHZ 	0x1
#define IS43TR16sssr15GBLI_RATIO_303MHZ 		0x80
#define IS43TR16sssr15GBLI_INVERT_CLKOUT_303MHZ 	0x0
#define IS43TR16sssr15GBLI_RD_DQS_303MHZ 		0x37
#define IS43TR16sssr15GBLI_WR_DQS_303MHZ 		0x3A
#define IS43TR16sssr15GBLI_PHY_FIFO_WE_303MHZ 		0x9C
#define IS43TR16sssr15GBLI_PHY_WR_DATA_303MHZ 		0x75
#define IS43TR16sssr15GBLI_IOCTRL_VALUE_303MHZ 		0x18B

/*
 * We have three pin mux functions that must exist.  We must be able to enable
 * uart0, for initial output and i2c0 to read the main EEPROM.  We then have a
 * main pinmux function that can be overridden to enable all other pinmux that
 * is required on the board.
 */
void enable_uart0_pin_mux(void);
void enable_uart1_pin_mux(void);
void enable_uart2_pin_mux(void);
void enable_uart3_pin_mux(void);
void enable_uart4_pin_mux(void);
void enable_uart5_pin_mux(void);
void enable_i2c0_pin_mux(void);
void enable_board_pin_mux(void);

int read_reset_flag(void);

enum SysBoot
{
  SPI_MMC0_UART0_EMAC1 = 0x16,  // 10110
  SPI_MMC0_USB0_UART0  = 0x18,  // 11000 
  SPI_MMC0_EMAC1_UART0 = 0x19   // 11001
};

void genepi_mcu_board_init(unsigned short mpuSpeed);

#endif
