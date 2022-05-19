int var_get_som_rev(struct var_eeprom *ep);
int get_board_id(void);

typedef struct eaton_boot_data_struct* smp_boot_data_ptr;

enum {
	DART_MX8M_MINI,
	VAR_SOM_MX8M_MINI,
	UNKNOWN_BOARD,
};

enum {
	SOM_REV_10,
	SOM_REV_11,
	SOM_REV_12,
	SOM_REV_13,
	UNKNOWN_REV
};

enum {
	DART_CARRIER_REV_1,
	DART_CARRIER_REV_2,
	DART_CARRIER_REV_UNDEF,
};

enum
{
	SMP_EMMC_SWITCH_PART_2_0,
	SMP_EMMC_WRITE,
	SMP_EMMC_READ,
};

enum
{
	SMP_BOOT_SD_CARD,
	SMP_BOOT_INTERNAL_EMMC,
	SMP_BOOT_UNKNOWN,
};