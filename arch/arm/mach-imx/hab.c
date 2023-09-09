// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc.
 */

//#define LOG_DEBUG

#include <common.h>
#include <command.h>
#include <config.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fuse.h>
#include <mapmem.h>
#include <image.h>
#include <ctype.h>
#include <asm/io.h>
#include <imximage.h>
#include <asm/global_data.h>
#include <asm/system.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/hab.h>
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/iomux-v3.h>
#include <linux/arm-smccc.h>
#include <asm/mach-imx/eaton-smp.h>
#include <fs.h>

#ifndef CONFIG_SPL_BUILD
#include <bootdata/bootdata.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define NORMAL_STATS_LEVEL 3
#define DEBUG_STATS_LEVEL 6

#define BOOTROM_BASE_ADDR  0x007e0000   // TCM start address
#define BOOTROM_SIZE       0x40000      // TCM size
#define DRAM_ADDRESS       0x40480000
#define DRAM_SIZE          0x200000  // Assuming uboot does not go bigger than 2MB

#define OCOTP_HW_OCOTP_SW_STICKY 0x30350050
#define OCOTP_HW_OCOTP_SW_STICKY_FIELD_LOCK (1 << 2)

#define ALIGN_SIZE		0x1000
#define MX6DQ_PU_IROM_MMU_EN_VAR	0x009024a8
#define MX6DLS_PU_IROM_MMU_EN_VAR	0x00901dd0
#define MX6SL_PU_IROM_MMU_EN_VAR	0x00901c60
#define HAB_ENABLED_BIT \
	(is_soc_type(MXC_SOC_MX7ULP) ? 0x80000000 :	\
	 ((is_soc_type(MXC_SOC_MX7) || is_soc_type(MXC_SOC_IMX8M)) ? 0x2000000 : 0x2))

#ifdef CONFIG_MX7ULP
#define HAB_M4_PERSISTENT_START	((soc_rev() >= CHIP_REV_2_0) ? 0x20008040 : \
				  0x20008180)
#define HAB_M4_PERSISTENT_BYTES		0xB80
#endif


#define OCOTP_CTRL						0x30350000
#define BM_OCOTP_CTRL_RELOAD_SHADOWS 	0x400

#ifndef CONFIG_SPL_BUILD
static int force_fuse_refresh(void)
{
	/* Get the base address of the OCOTP CTRL register */
	void __iomem *ocotp_base = (void __iomem *)OCOTP_CTRL;

	/* Set the RELOAD_SHADOWS bit */
	writel(BM_OCOTP_CTRL_RELOAD_SHADOWS, ocotp_base);

	/* Wait for the operation to complete or timeout */	
	while (!(readl(ocotp_base) & BM_OCOTP_CTRL_RELOAD_SHADOWS))
		;

	return 0;
}
#endif

static int ivt_header_error(const char *err_str, struct ivt_header *ivt_hdr)
{
#ifndef CONFIG_SPL_BUILD		
	printf("%s magic=0x%x length=0x%02x version=0x%x\n", err_str,
	       ivt_hdr->magic, ivt_hdr->length, ivt_hdr->version);
#endif
	return 1;
}

static int verify_ivt_header(struct ivt_header *ivt_hdr)
{
	int result = 0;

	if (ivt_hdr->magic != IVT_HEADER_MAGIC)
		result = ivt_header_error("bad magic", ivt_hdr);

	if (be16_to_cpu(ivt_hdr->length) != IVT_TOTAL_LENGTH)
		result = ivt_header_error("bad length", ivt_hdr);

	if ((ivt_hdr->version & HAB_MAJ_MASK) != HAB_MAJ_VER)
		result = ivt_header_error("bad version", ivt_hdr);

	return result;
}

#ifdef CONFIG_ARM64
#define FSL_SIP_HAB		0xC2000007
#define FSL_SIP_HAB_AUTHENTICATE	0x00
#define FSL_SIP_HAB_ENTRY		0x01
#define FSL_SIP_HAB_EXIT		0x02
#define FSL_SIP_HAB_REPORT_EVENT	0x03
#define FSL_SIP_HAB_REPORT_STATUS	0x04
#define FSL_SIP_HAB_FAILSAFE		0x05
#define FSL_SIP_HAB_CHECK_TARGET	0x06
static volatile gd_t *gd_save;
#endif

static inline void save_gd(void)
{
#ifdef CONFIG_ARM64
	gd_save = gd;
#endif
}

static inline void restore_gd(void)
{
#ifdef CONFIG_ARM64
	/*
	 * Make will already error that reserving x18 is not supported at the
	 * time of writing, clang: error: unknown argument: '-ffixed-x18'
	 */
	__asm__ volatile("mov x18, %0\n" : : "r" (gd_save));
#endif
}

enum hab_status hab_rvt_report_event(enum hab_status status, u32 index,
				     u8 *event, size_t *bytes)
{
	enum hab_status ret;
	hab_rvt_report_event_t *hab_rvt_report_event_func;
	struct arm_smccc_res res __maybe_unused;

	hab_rvt_report_event_func =  (hab_rvt_report_event_t *)HAB_RVT_REPORT_EVENT;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_REPORT_EVENT, (unsigned long)index,
			      (unsigned long)event, (unsigned long)bytes, 0, 0, 0, &res);
		return (enum hab_status)res.a0;
	}
#endif

	save_gd();
	ret = hab_rvt_report_event_func(status, index, event, bytes);
	restore_gd();

	return ret;

}

enum hab_status hab_rvt_report_status(enum hab_config *config, enum hab_state *state)
{
	enum hab_status ret;
	hab_rvt_report_status_t *hab_rvt_report_status_func;
	struct arm_smccc_res res __maybe_unused;

	hab_rvt_report_status_func = (hab_rvt_report_status_t *)HAB_RVT_REPORT_STATUS;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_REPORT_STATUS, (unsigned long)config,
			      (unsigned long)state, 0, 0, 0, 0, &res);
		return (enum hab_status)res.a0;
	}
#endif

	save_gd();
	ret = hab_rvt_report_status_func(config, state);
	restore_gd();

	return ret;
}

enum hab_status hab_rvt_entry(void)
{
	enum hab_status ret;
	hab_rvt_entry_t *hab_rvt_entry_func;
	struct arm_smccc_res res __maybe_unused;

	hab_rvt_entry_func = (hab_rvt_entry_t *)HAB_RVT_ENTRY;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_ENTRY, 0, 0, 0, 0, 0, 0, &res);
		return (enum hab_status)res.a0;
	}
#endif

	save_gd();
	ret = hab_rvt_entry_func();
	restore_gd();

	return ret;
}

enum hab_status hab_rvt_exit(void)
{
	enum hab_status ret;
	hab_rvt_exit_t *hab_rvt_exit_func;
	struct arm_smccc_res res __maybe_unused;

	hab_rvt_exit_func =  (hab_rvt_exit_t *)HAB_RVT_EXIT;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_EXIT, 0, 0, 0, 0, 0, 0, &res);
		return (enum hab_status)res.a0;
	}
#endif

	save_gd();
	ret = hab_rvt_exit_func();
	restore_gd();

	return ret;
}

void hab_rvt_failsafe(void)
{
	hab_rvt_failsafe_t *hab_rvt_failsafe_func;

	hab_rvt_failsafe_func = (hab_rvt_failsafe_t *)HAB_RVT_FAILSAFE;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_FAILSAFE, 0, 0, 0, 0, 0, 0, NULL);
		return;
	}
#endif

	save_gd();
	hab_rvt_failsafe_func();
	restore_gd();
}

enum hab_status hab_rvt_check_target(enum hab_target type, const void *start,
					       size_t bytes)
{
	enum hab_status ret;
	hab_rvt_check_target_t *hab_rvt_check_target_func;
	struct arm_smccc_res res __maybe_unused;

	hab_rvt_check_target_func =  (hab_rvt_check_target_t *)HAB_RVT_CHECK_TARGET;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_CHECK_TARGET, (unsigned long)type,
			      (unsigned long)start, (unsigned long)bytes, 0, 0, 0, &res);
		return (enum hab_status)res.a0;
	}
#endif

	save_gd();
	ret = hab_rvt_check_target_func(type, start, bytes);
	restore_gd();

	return ret;
}

void *hab_rvt_authenticate_image(uint8_t cid, ptrdiff_t ivt_offset,
				 void **start, size_t *bytes, hab_loader_callback_f_t loader)
{
	void *ret;
	hab_rvt_authenticate_image_t *hab_rvt_authenticate_image_func;
	struct arm_smccc_res res __maybe_unused;

	hab_rvt_authenticate_image_func = (hab_rvt_authenticate_image_t *)HAB_RVT_AUTHENTICATE_IMAGE;
#if defined(CONFIG_ARM64)
	if (current_el() != 3) {
		/* call sip */
		arm_smccc_smc(FSL_SIP_HAB, FSL_SIP_HAB_AUTHENTICATE, (unsigned long)ivt_offset,
			      (unsigned long)start, (unsigned long)bytes, 0, 0, 0, &res);
		return (void *)res.a0;
	}
#endif

	save_gd();
	ret = hab_rvt_authenticate_image_func(cid, ivt_offset, start, bytes, loader);
	restore_gd();

	return ret;
}

#if !defined(CONFIG_SPL_BUILD)

#define MAX_RECORD_BYTES     (8*1024) /* 4 kbytes */

struct record {
	uint8_t  tag;						/* Tag */
	uint8_t  len[2];					/* Length */
	uint8_t  par;						/* Version */
	uint8_t  contents[MAX_RECORD_BYTES];/* Record Data */
	bool	 any_rec_flag;
};

static char *rsn_str[] = {
    "Any reason",
    "Engine failure",
    "Invalid address: access denied",
    "Invalid assertion",
    "Function called out of sequence",
    "Invalid certificate",
    "Invalid command: command malformed",
    "Invalid CSF",
    "Invalid DCD",
    "Invalid index: access denied",
    "Invalid IVT",
    "Invalid key",
    "Failed callback function",
    "Invalid signature",
    "Invalid data size",
    "Memory failure",
    "Expired poll count",
    "Exhausted storage region",
    "Unsupported algorithm",
    "Unsupported command",
    "Unsupported engine",
    "Unsupported configuration item",
    "Unsupported key type/parameters",
    "Unsupported protocol",
    "Unsuitable state",    
    "Invalid",        
     NULL
};

static char *sts_str[] = {
    "Any",
    "FAILURE",
    "WARNING",
    "SUCCESS",
    "INVALID",
    NULL
};

static char *eng_str[] = {
   "General",
   "Security Controller",
   "Run-time Integrity Checker",
   "Crypto Accelerator",
   "Central Security Unit",
   "Secure Clock",
   "Data Co-Processor",
   "CAAM",
   "Secure Non-Volatile Storage",
   "Fuse controller",
   "DTCP co-processor",
   "Protected ROM area",
   "HDCP Coprocessor",
   "RTL Simulation Engine",
   "Software Engine",
   "Invalid Engine",
};

static char *ctx_str[] = {
    "Any Context",
    "hab_fab_test()",
    "hab_rvt.entry()",
    "hab_rvt.check_target()",
    "hab_rvt.authenticate_image()",
    "hab_rvt.run_dcd()",
    "hab_rvt.run_csf()",
    "Executing CSF/DCD Command",
    "Authenticated Data Block",
    "hab_rvt.assert()",
    "hab_rvt.exit()",
    "Invalid",
    NULL
};

typedef struct {
    enum hab_state state;
    char *str;
    bool secure;
    bool trusted;
    bool fail;
} hab_state_info_t;


static hab_state_info_t hab_state_info[] = {
//  HAB State            Description         Secure  Trusted  Fail    
    { HAB_STATE_INITIAL,   "Initialising",     false,  false,   false },
    { HAB_STATE_CHECK,     "Check",            false,  false,   false },
    { HAB_STATE_NONSECURE, "Non-secure",       false,  false,   false },
    { HAB_STATE_TRUSTED,   "Trusted",          true,   true,    false },
    { HAB_STATE_SECURE,    "Secure",           true,   false,   false },
    { HAB_STATE_FAIL_SOFT, "Soft fail",        false,  false,   true  },
    { HAB_STATE_FAIL_HARD, "Hard fail",        false,  false,   true  },
    { HAB_STATE_NONE,      "No security",      false,  false,   true  }
};

typedef struct {
    enum hab_config config;
    char *str;
    bool field_return;
    bool secure;
} hab_config_info_t;


static hab_config_info_t hab_config_info[] = {
//  HAB Config            Description         
    { HAB_CFG_RETURN,       "Field Return", true,  false},
    { HAB_CFG_OPEN,         "Non-secure",   false, false},
    { HAB_CFG_CLOSED,       "Secure",       false, true}    
};

static uint8_t hab_statuses[5] = {
	HAB_STS_ANY,
	HAB_FAILURE,
	HAB_WARNING,
	HAB_SUCCESS
};

static uint8_t hab_reasons[26] = {
	HAB_RSN_ANY,
	HAB_ENG_FAIL,
	HAB_INV_ADDRESS,
	HAB_INV_ASSERTION,
	HAB_INV_CALL,
	HAB_INV_CERTIFICATE,
	HAB_INV_COMMAND,
	HAB_INV_CSF,
	HAB_INV_DCD,
	HAB_INV_INDEX,
	HAB_INV_IVT,
	HAB_INV_KEY,
	HAB_INV_RETURN,
	HAB_INV_SIGNATURE,
	HAB_INV_SIZE,
	HAB_MEM_FAIL,
	HAB_OVR_COUNT,
	HAB_OVR_STORAGE,
	HAB_UNS_ALGORITHM,
	HAB_UNS_COMMAND,
	HAB_UNS_ENGINE,
	HAB_UNS_ITEM,
	HAB_UNS_KEY,
	HAB_UNS_PROTOCOL,
	HAB_UNS_STATE
};

static uint8_t hab_contexts[12] = {
	HAB_CTX_ANY,
	HAB_CTX_FAB,
	HAB_CTX_ENTRY,
	HAB_CTX_TARGET,
	HAB_CTX_AUTHENTICATE,
	HAB_CTX_DCD,
	HAB_CTX_CSF,
	HAB_CTX_COMMAND,
	HAB_CTX_AUT_DAT,
	HAB_CTX_ASSERT,
	HAB_CTX_EXIT
};

static uint8_t hab_engines[16] = {
	HAB_ENG_ANY,
	HAB_ENG_SCC,
	HAB_ENG_RTIC,
	HAB_ENG_SAHARA,
	HAB_ENG_CSU,
	HAB_ENG_SRTC,
	HAB_ENG_DCP,
	HAB_ENG_CAAM,
	HAB_ENG_SNVS,
	HAB_ENG_OCOTP,
	HAB_ENG_DTCP,
	HAB_ENG_ROM,
	HAB_ENG_HDCP,
	HAB_ENG_RTL,
	HAB_ENG_SW
};


typedef struct {
    u32 bank;
    u32 word;
    u32 value;
} fuse_prog_t;

static fuse_prog_t s_fuses[] = {
   #include <srk_fuse_cmds.inc>
};

enum hab_boot_check_results {
	HAB_BOOT_CHECK_RESULT_NO_RESULT,    // No such command has been done in the last boot 
    HAB_BOOT_CHECK_RESULT_VALID,        // Boot signature is valid (for SPL)
	HAB_BOOT_CHECK_RESULT_INVALID       // Boot signature NOT valid (for SPL)
};

enum hab_fuse_prog_results {
	HAB_FUSE_PROG_RESULT_NO_RESULT,     // No such command has been done in the last boot
	HAB_FUSE_PROG_RESULT_SUCCESS,       // Fuse command succeeded.
    HAB_FUSE_PROG_RESULT_FAIL           // Fuse command failed.
};

enum hab_device_close_results {
    HAB_DEVICE_CLOSE_RESULT_NO_RESULT,  // No such command has been done in the last boot
	HAB_DEVICE_CLOSE_RESULT_SUCCESS,    // Close command succeeded
	HAB_DEVICE_CLOSE_RESULT_FAIL        // Device close command failed.
};

enum hab_device_field_return_results {
    HAB_DEVICE_FIELD_RETURN_RESULT_NO_RESULT,  // No such command has been done in the last boot
	HAB_DEVICE_FIELD_RETURN_RESULT_SUCCESS,    // Field return command succeeded
	HAB_DEVICE_FIELD_RETURN_RESULT_FAIL        // Field return command failed.
};

static fuse_prog_t const imx_srk_revoke_fuse = {
	.bank  = 9,
	.word  = 3
};

static fuse_prog_t const imx_field_return_fuse = {
	.bank = 8,
	.word = 3,
	.value = 1
};

#define BOOTARG_HAB_DEVICE_CLOSED                   "hab_device_closed"                   // 1 if hab device has been closed/secured, 0 otherwise
#define BOOTARG_HAB_FUSE_PROGRAMMED                 "hab_fuse_programmed"                 // 1 if fuse hasn't been programmed yet, 0 if programmed
#define BOOTARG_HAB_FUSE_CONTENT_CORRECT            "hab_fuse_content_correct"            // 1 if fuse has been programmed with correct content, 0 otherwise
#define BOOTARG_HAB_UBOOT_SIG_VALID                 "hab_uboot_sig_valid"                 // 1 if signature valid
#define BOOTARG_HAB_DEVICE_FIELD_RETURN             "hab_device_field_return"             // 1 if device in field return mode, 0 otherwise
#define BOOTARG_HAB_DEVICE_FIELD_RETURN_LOCKED      "hab_device_field_return_locked"      // 1 if device field return fuse is locked, 0 otherwise
#define BOOTARG_IS_DEV_BOARD                        "is_dev_board"                        // 1 if this system is a dev board.
#define BOOTARG_DEVICE_REVOCATION_BITS              "hab_device_revocation_bits"          // First 4 bits of the SRK_REVOKE register

static enum hab_boot_check_results s_hab_boot_check_cmd_result = HAB_BOOT_CHECK_RESULT_NO_RESULT;
static enum hab_fuse_prog_results s_hab_fuse_prog_cmd_result = HAB_FUSE_PROG_RESULT_NO_RESULT;
static enum hab_device_close_results s_hab_device_close_cmd_result = HAB_DEVICE_CLOSE_RESULT_NO_RESULT;
static enum hab_device_field_return_results s_hab_device_field_return_cmd_result = HAB_DEVICE_FIELD_RETURN_RESULT_NO_RESULT;
static uint8_t s_extra_cmds;

#define EXTRA_CMD_VALID         	     	(1 << 7) // Other bits are to be taken into consideration if set 
#define EXTRA_CMD_DEVICE_FUSE_FORCE 	    (1 << 0) // Device fuse state to force if EXTRA_CMD_VALID
#define EXTRA_CMD_DEVICE_CLOSE_FORCE 	    (1 << 1) // Device close state to force if EXTRA_CMD_VALID
#define EXTRA_CMD_FIELD_RETURN_FORCE        (1 << 2) // Field return state to force if EXTRA_CMD_VALID
#define EXTRA_CMD_SIG_VALID_FORCE           (1 << 3) // Boot signature state to force if EXTRA_CMD_VALID
#define EXTRA_CMD_DEV_BOARD_FORCE           (1 << 4) // Devlopment device state to force if EXTRA_CMD_VALID
#define EXTRA_CMD_SRK_REV_FORCE             (1 << 5) // Set revoke keys to 1010/0101 if EXTRA_CMD_VALID


#define SRK_REVOKE_BITS_MASK 0xf
#define NUM_SRKS 4 

void log_print(bool log, const char *format, ...) 
{
    va_list args;
    va_start(args, format);
    char line[512];
    vsnprintf(line, sizeof(line), format, args);
	line[sizeof(line) - 1] = 0;

    if (log) {
        smp_log_add(SMP_LOG_PRIORITY_INFORMATIONAL, line, SMP_LOG_PAGE_SECURITY, SMP_LOG_CODE_DEFAULT);
    }

    printf("%s\n", line);

    va_end(args);
}

static bool hab_fuse_programmed(void)
{
	bool programmed = false;
	u32 i, value;

    for (i = 0 ; i < ARRAY_SIZE(s_fuses) ; i++) {
		value = 0;
		fuse_read(s_fuses[i].bank, s_fuses[i].word, &value);
		if (value != 0) {
			programmed = true;
			break;
		}
	}
	return programmed;
}

// Return true if HAB programmed with the fuses values we expect.
static bool hab_fuse_content_correct(void)
{
	bool correct = true;
	u32 i, value;

    for (i = 0 ; i < ARRAY_SIZE(s_fuses) ; i++) 
	{
		value = 0;
		fuse_read(s_fuses[i].bank, s_fuses[i].word, &value);
		if (value != s_fuses[i].value) 
		{
			printf("HAB fuse %d,%d value 0x%08X, expected 0x%08X\n", s_fuses[i].bank, s_fuses[i].word, value, s_fuses[i].value);
			correct = false;
		}
	}

	return correct;
}

static bool hab_field_return_lock(void)
{
	bool locked;
    locked = (*((uint32_t *)OCOTP_HW_OCOTP_SW_STICKY) & OCOTP_HW_OCOTP_SW_STICKY_FIELD_LOCK) ? true : false;	
	return locked;
}

static bool hab_is_dev_board(bool bIgnoreEnv)
{
	return smp_is_dev_board(bIgnoreEnv);
}

static uint32_t hab_get_revoke_bits(void) 
{
	uint32_t bits;
    fuse_read(imx_srk_revoke_fuse.bank, imx_srk_revoke_fuse.word, &bits);
	bits &= SRK_REVOKE_BITS_MASK;	
	return bits;
}

static bool hab_sig_valid(uint32_t *count)
{
	uint32_t event_count = 0;
    uint8_t event_data[128]; /* Event data buffer */
    size_t bytes = sizeof(event_data); /* Event size in bytes */  	
	bool valid;

    *count = 0;
    while (hab_rvt_report_event(HAB_STS_ANY, event_count, event_data,
                    &bytes) == HAB_SUCCESS) {     
        event_count++;
        bytes = sizeof(event_data);
    }   
    *count = event_count;
	valid = (event_count == 0);	
	return valid;
}


void get_security_state_str(bool device_closed, bool field_return, bool boot_sig_valid, int dev_board, char* buffer, size_t buffer_size) 
{
    if (!field_return && device_closed && !dev_board) 
	{
		snprintf(buffer, buffer_size, "Secured");
    }
	else
	{
		const char *security_state = field_return ? "Field Return" : (device_closed ? "Secured" : "Insecured");
		const char *bootloader_signature = boot_sig_valid ? "Valid" : "Invalid";
		const char *os_verification = dev_board ? "Disabled" : "Enabled";
		
		snprintf(buffer, buffer_size, "%s, Bootloader Signature: %s, OS Verification: %s",
					security_state,
					bootloader_signature != NULL ? bootloader_signature : "",
					os_verification != NULL ? os_verification : ""
		);
	}
	buffer[buffer_size - 1] = 0;	
}

static void srk_revoke_to_str(uint32_t srk_revoke_register, char *buffer, size_t buffer_size) 
{
	uint32_t i;
    size_t offset = 0;
	bool first = true;

	snprintf(buffer, buffer_size, "None");	

    for (i = 0; i < NUM_SRKS ; i++) {
        if (srk_revoke_register & (1 << i)) {
            snprintf(buffer + offset, buffer_size - offset, "%sSRK%d", first ? "":",", i + 1);
			first = false;
            offset += strlen(buffer + offset); 
        }
    }
    buffer[buffer_size - 1] = 0;
}


static void hab_fuses_to_str(char *hab_fuses_to_program_str, uint32_t hab_fuses_to_program_str_size, char *hab_fuses_programmed_str, uint32_t hab_fuses_programmed_str_size)
{
    uint32_t value, len, remain1, remain2, chunk, i;
	char buf[32];

    remain1 = hab_fuses_to_program_str_size - 1;
	remain2 = hab_fuses_programmed_str_size - 1;

    for (i = 0 ; i < ARRAY_SIZE(s_fuses) ; i++) {
        value = s_fuses[i].value;		
		snprintf(buf, sizeof(buf), "0x%08X%s", value, (i == ARRAY_SIZE(s_fuses) - 1) ? "": ", ");
		strncpy(hab_fuses_to_program_str, buf, remain1);
		len = strlen(buf);
		chunk = min(remain1, len);
		hab_fuses_to_program_str += chunk;
		remain1 -= chunk;
		hab_fuses_to_program_str[0] = 0;

		fuse_read(s_fuses[i].bank, s_fuses[i].word, &value);		
		snprintf(buf, sizeof(buf), "0x%08X%s", value, (i == ARRAY_SIZE(s_fuses) - 1) ? "": ", ");
		strncpy(hab_fuses_programmed_str, buf, remain2);
		len = strlen(buf);
		chunk = min(remain2, len);
		hab_fuses_programmed_str += chunk;	
		remain2 -= chunk;
		hab_fuses_programmed_str[0] = 0;
	}
}


static inline uint32_t get_idx(uint8_t *list, uint8_t tgt, uint32_t size)
{
	uint32_t idx = 0;
	uint8_t element;
	while (idx < size) {
		element = list[idx];
		if (element == tgt)
			return idx;
		++idx;
	}
	return idx;
}



static char *event_record_to_str(char *str, uint32_t str_size, uint8_t *event_data, size_t bytes)
{

    struct record *rec = (struct record *)event_data;

    snprintf(str, str_size, "STATUS=0x%x (%s) REASON=0x%x (%s) CONTEXT=0x%x (%s) ENGINE=0x%x (%s)",  
                                                 rec->contents[0], sts_str[get_idx(hab_statuses, rec->contents[0], ARRAY_SIZE(hab_statuses))],
												 rec->contents[1], rsn_str[get_idx(hab_reasons,  rec->contents[1], ARRAY_SIZE(hab_reasons))],
                                                 rec->contents[2], ctx_str[get_idx(hab_contexts, rec->contents[2], ARRAY_SIZE(hab_contexts))],
                                                 rec->contents[3], eng_str[get_idx(hab_engines,  rec->contents[3], ARRAY_SIZE(hab_engines))]
                                                 );
    str[str_size - 1] = 0;
    return str;
}

static char *hab_config_to_str(enum hab_config config, bool *field_return, bool *secure)
{
    char *str = "Unknown";
    uint32_t idx;
    *field_return = false;
    *secure = false;
    for (idx = 0 ; idx < ARRAY_SIZE(hab_config_info); idx++) {
        if (config == hab_config_info[idx].config) {
            str = hab_config_info[idx].str;
            *field_return = hab_config_info[idx].field_return;
            *secure = hab_config_info[idx].secure;
            break;
        }
    }
    return str;
}


static char *hab_state_to_str(enum hab_state state, bool *secure, bool *trusted, bool *fail)
{
    uint32_t idx;
    char *str     = "Unknown";
    *secure  = false;    
    *trusted = false;
    *fail    = true;
    for (idx = 0 ; idx < ARRAY_SIZE(hab_state_info) ; idx++) {
        if (state == hab_state_info[idx].state) { 
            str      = hab_state_info[idx].str;
            *secure  = hab_state_info[idx].secure;
            *trusted = hab_state_info[idx].trusted;
            *fail    = hab_state_info[idx].fail;
            break;    
        }
    }
    return str;
}



static void show_hab_event(bool log, uint32_t event_number, uint8_t *event_data, size_t bytes)
{
    uint32_t i;
    char str[256];
    char line[512];

    if (!(event_data && bytes > 0))
        return;

    event_record_to_str(str, sizeof(str), event_data, bytes);
    snprintf(line, sizeof(line), "HAB EVENT[%d] %s", event_number, str);
    line[sizeof(line) - 1] = 0;      

    if (log) {
         smp_log_add(SMP_LOG_PRIORITY_INFORMATIONAL, line, SMP_LOG_PAGE_SECURITY, SMP_LOG_CODE_DEFAULT);
    }
    else {
        printf("%s\n", line);        
        for (i = 0; i < bytes; i++) {
            if (i == 0)
                printf("\t0x%02x", event_data[i]);
            else if ((i % 8) == 0)
                printf("\n\t0x%02x", event_data[i]);
            else
                printf(" 0x%02x", event_data[i]);
        }
        printf("\n\n");
    }
}

static int create_stats_and_bootargs(void)
{
    enum hab_config config = 0;
    enum hab_state state = 0;  
    enum hab_status status;    
    uint64_t uid = 0;
	uint32_t event_count = 0;
	bool     is_dev_board;	
    char *config_str;
    bool  config_field_return;
    bool  config_secure;
    char *state_str;
    bool  state_secure;
    bool  state_trusted;
	bool  device_closed;
    bool  state_fail;
	bool  field_return_lock;
	bool  fuse_programmed;
	bool  fuse_content_correct;
	bool  sig_valid;
    char  version_str[128];
	char  fuses_to_program_str[128];
	char  fuses_programmed_str[128];
	char  srk_revoke_str[128];
	uint32_t srk_revoke_bits;
	char  security_state_str[128];

	device_closed         = imx_hab_is_enabled();
	fuse_programmed       = hab_fuse_programmed();
	fuse_content_correct  = hab_fuse_content_correct();
	field_return_lock     = hab_field_return_lock();
	hab_fuses_to_str(fuses_to_program_str, sizeof(fuses_to_program_str), fuses_programmed_str, sizeof(fuses_programmed_str));
    status     = hab_rvt_report_status(&config, &state);
    config_str = hab_config_to_str(config, &config_field_return, &config_secure);

    state_str  = hab_state_to_str(state, &state_secure, &state_trusted, &state_fail);

 	sig_valid    = hab_sig_valid(&event_count);
    uid          = smp_board_serial();
	is_dev_board = hab_is_dev_board(false);

	struct hab_hdr *hdr = (struct hab_hdr *)HAB_RVT_BASE;

    version_str[0] = 0;
	if (hdr->tag == HAB_TAG_RVT) {
	    snprintf(version_str, sizeof(version_str), "%d.%d", hdr->par >> 4, hdr->par & 0xf);
        version_str[sizeof(version_str) - 1] = 0;
	}

    srk_revoke_bits = hab_get_revoke_bits();
	srk_revoke_to_str(srk_revoke_bits, srk_revoke_str, sizeof(srk_revoke_str));

	get_security_state_str(device_closed, config_field_return, sig_valid, is_dev_board, security_state_str, sizeof(security_state_str));
		
    smp_stats_begin("hab", "-------- High Assurance Boot (HAB) -------", NORMAL_STATS_LEVEL);
    // Important stats
    smp_stats_add_str("Version", DEBUG_STATS_LEVEL, "hab_version", version_str);
	smp_stats_add_str("Security state", NORMAL_STATS_LEVEL, "security_state", security_state_str);

	// Mostly debugging stats
	smp_stats_add_hex64("UID", DEBUG_STATS_LEVEL, "uid", uid);
	smp_stats_add_bool("Development board", DEBUG_STATS_LEVEL, "is_dev_board", hab_is_dev_board(true));

    smp_stats_add_str("State", DEBUG_STATS_LEVEL, "hab_state", state_str);
    smp_stats_add_uint32_t("State value", DEBUG_STATS_LEVEL, "hab_state_value", state);
    smp_stats_add_bool("State secure", DEBUG_STATS_LEVEL, "hab_state_secure", state_secure);
    smp_stats_add_bool("State trusted", DEBUG_STATS_LEVEL, "hab_state_trusted", state_trusted);
    smp_stats_add_bool("State fail", DEBUG_STATS_LEVEL, "hab_state_fail", state_fail);
	smp_stats_add_bool("Bootloader signature valid", DEBUG_STATS_LEVEL, "hab_uboot_signature_valid", sig_valid);    

    smp_stats_add_bool("Device closed", DEBUG_STATS_LEVEL, "hab_device_closed", device_closed);
    smp_stats_add_bool("Device field return fuse locked", DEBUG_STATS_LEVEL, "hab_device_field_return_locked", field_return_lock);
    smp_stats_add_bool("Device field return", DEBUG_STATS_LEVEL, "hab_device_field_return", config_field_return);
	smp_stats_add_str("Device keys revoked", DEBUG_STATS_LEVEL, "keys_revoked", srk_revoke_str);
    smp_stats_add_bool("Fuse programmed", DEBUG_STATS_LEVEL, "hab_fuse_programmed", fuse_programmed);
    smp_stats_add_bool("Fuse content correct", DEBUG_STATS_LEVEL, "hab_fuse_content_correct", fuse_content_correct);
    smp_stats_add_str("Fuse content to program", DEBUG_STATS_LEVEL, "hab_fuses_to_program", fuses_to_program_str);
    smp_stats_add_str("Fuse content programmed", DEBUG_STATS_LEVEL, "hab_fuses_programmed", fuses_programmed_str);    
    smp_stats_add_uint32_t("Event count", DEBUG_STATS_LEVEL, "hab_event_count", event_count);
    smp_stats_add_str("Config", DEBUG_STATS_LEVEL, "hab_config", config_str);
    smp_stats_add_uint32_t("Config value", DEBUG_STATS_LEVEL, "hab_config_value", config);
    smp_stats_add_bool("Config secure", DEBUG_STATS_LEVEL, "hab_config_secure", config_secure);    
    smp_stats_end();

    env_set_ulong(BOOTARG_HAB_DEVICE_CLOSED, device_closed ? 1 : 0);
	env_set_ulong(BOOTARG_HAB_DEVICE_FIELD_RETURN, config_field_return ? 1 : 0);
	env_set_ulong(BOOTARG_HAB_DEVICE_FIELD_RETURN_LOCKED, field_return_lock ? 1 : 0);
    env_set_ulong(BOOTARG_HAB_FUSE_PROGRAMMED, fuse_programmed ? 1 : 0);
    env_set_ulong(BOOTARG_HAB_FUSE_CONTENT_CORRECT, fuse_content_correct ? 1 : 0);
    env_set_ulong(BOOTARG_HAB_UBOOT_SIG_VALID, sig_valid ? 1 : 0);
    env_set_ulong(BOOTARG_IS_DEV_BOARD, is_dev_board ? 1 : 0);
	env_set_ulong(BOOTARG_DEVICE_REVOCATION_BITS, srk_revoke_bits);
	env_set_ulong(BOOTARG_JTAG_LOCKED, is_jtag_locked ? 1 : 0);

	char buf[256];
	snprintf(buf, sizeof(buf), "Secure state: %s", security_state_str);
	buf[sizeof(buf) - 1] = 0;
    smp_log_add(SMP_LOG_PRIORITY_INFORMATIONAL, buf, SMP_LOG_PAGE_SECURITY, SMP_LOG_CODE_DEFAULT);

    return 0;
}


static int get_hab_status(bool log)
{
    char *str;
    bool secure, trusted, fail, field_return;
    uint32_t index = 0; /* Loop index */
    uint8_t event_data[128]; /* Event data buffer */
    size_t bytes = sizeof(event_data); /* Event size in bytes */
    enum hab_config config = 0;
    enum hab_state state = 0;
    enum hab_status status;
    char line[512];

    /* Check HAB status */
    status = hab_rvt_report_status(&config, &state);
    str = hab_config_to_str(config, &field_return, &secure);
    snprintf(line, sizeof(line), "HAB CONFIG: %s (0x%02x)", str, config);
    line[sizeof(line) - 1] = 0;    
    if (!log) {
        printf("%s\n", line);
    }          
    str = hab_state_to_str(state, &secure, &trusted, &fail);
    snprintf(line, sizeof(line), "HAB STATE: %s (SECURE=%s TRUSTED=%s FAIL=%s) (0x%02x)", str, secure ? "y":"n", trusted ? "y":"n", fail ? "y":"n", state);     
    line[sizeof(line) - 1] = 0;

    if (!log) {    
        printf("%s\n", line);        
    }
    if (status != HAB_SUCCESS) {    
        /* Display HAB events */
        while (hab_rvt_report_event(HAB_STS_ANY, index, event_data,
                    &bytes) == HAB_SUCCESS) {                     
            show_hab_event(log, index + 1, event_data, bytes);
            bytes = sizeof(event_data);             
            index++;
        }
    }
    return 0;
}

#ifdef CONFIG_MX7ULP

static int get_record_len(struct record *rec)
{
	return (size_t)((rec->len[0] << 8) + (rec->len[1]));
}

static int get_hab_status_m4(void)
{
	unsigned int index = 0;
	uint8_t event_data[128];
	size_t record_len, offset = 0;
	enum hab_config config = 0;
	enum hab_state state = 0;

	if (imx_hab_is_enabled())
		puts("\nSecure boot enabled\n");
	else
		puts("\nSecure boot disabled\n");

	/*
	 * HAB in both A7 and M4 gather the security state
	 * and configuration of the chip from
	 * shared SNVS module
	 */
	hab_rvt_report_status(&config, &state);
	printf("\nHAB Configuration: 0x%02x, HAB State: 0x%02x\n",
	       config, state);

	struct record *rec = (struct record *)(HAB_M4_PERSISTENT_START);

	record_len = get_record_len(rec);

	/* Check if HAB persistent memory is valid */
	if (rec->tag != HAB_TAG_EVT_DEF ||
	    record_len != sizeof(struct evt_def) ||
	    (rec->par & HAB_MAJ_MASK) != HAB_MAJ_VER) {
		puts("\nERROR: Invalid HAB persistent memory\n");
		return 1;
	}

	/* Parse events in HAB M4 persistent memory region */
	while (offset < HAB_M4_PERSISTENT_BYTES) {
		rec = (struct record *)(HAB_M4_PERSISTENT_START + offset);

		record_len = get_record_len(rec);

		if (rec->tag == HAB_TAG_EVT) {
			memcpy(&event_data, rec, record_len);
			puts("\n");
			printf("--------- HAB Event %d -----------------\n",
			       index + 1);
			puts("event data:\n");
			display_event(event_data, record_len);
			puts("\n");
			index++;
		}

		offset += record_len;

		/* Ensure all records start on a word boundary */
		if ((offset % 4) != 0)
			offset =  offset + (4 - (offset % 4));
	}

	if (!index)
		puts("No HAB Events Found!\n\n");

	return 0;
}
#endif

static int do_hab_status(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
#ifdef CONFIG_MX7ULP
	if ((argc > 2)) {
		cmd_usage(cmdtp);
		return 1;
	}

	if (strcmp("m4", argv[1]) == 0)
		get_hab_status_m4();
	else
		get_hab_status();
#else
	if ((argc != 1)) {
		cmd_usage(cmdtp);
		return 1;
	}

	get_hab_status(false);
#endif

	return 0;
}

static ulong get_image_ivt_offset(ulong img_addr)
{
	const void *buf;

	buf = map_sysmem(img_addr, 0);
	switch (genimg_get_format(buf)) {
#if CONFIG_IS_ENABLED(LEGACY_IMAGE_FORMAT)
	case IMAGE_FORMAT_LEGACY:
		return (image_get_image_size((image_header_t *)img_addr)
			+ 0x1000 - 1)  & ~(0x1000 - 1);
#endif
#if CONFIG_IS_ENABLED(FIT)
	case IMAGE_FORMAT_FIT:
		return (fit_get_size(buf) + 0x1000 - 1)  & ~(0x1000 - 1);
#endif
	default:
		return 0;
	}
}

static int do_authenticate_image(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	ulong	addr, length, ivt_offset;
	int	rcode = 0;

	if (argc < 3)
		return CMD_RET_USAGE;

	addr = hextoul(argv[1], NULL);
	length = hextoul(argv[2], NULL);
	if (argc == 3)
		ivt_offset = get_image_ivt_offset(addr);
	else
		ivt_offset = hextoul(argv[3], NULL);

	rcode = imx_hab_authenticate_image(addr, length, ivt_offset, true);
	if (rcode == 0)
		rcode = CMD_RET_SUCCESS;
	else
		rcode = CMD_RET_FAILURE;

	return rcode;
}

static int do_hab_failsafe(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	if (argc != 1) {
		cmd_usage(cmdtp);
		return CMD_RET_FAILURE;
	}

	hab_rvt_failsafe();

	return 0;
}

extern int confirm_yesno(void);

static int do_hab_fuse(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = CMD_RET_FAILURE;
	uint32_t i;
	bool do_prog = false;
	bool do_log = false;
	const char *warn = 
		"\n\nTHIS COMMAND WILL PROGRAM THE HIGH ASSURANCE BOOT SRK HASH E-FUSES.\n"
		"THOSE ARE ONE-TIME PROGRAMMABLE FUSES. ONCE YOU WRITE THEM, YOU CAN'T GO BACK.\n"
		"\n"
		"DO YOU REALLY WANT TO PROGRAM THE FUSES? <y/n> ";

	if ((argc > 3)) {
		cmd_usage(cmdtp);
		goto done;
	}

	for (i = 1 ; i < argc ; i++) {
		if (!strcmp("-y", argv[i])) {
			do_prog = true;
		}
		else if (strcmp("-log", argv[i]) == 0) {
			do_log = true;
		}	
		else {
			cmd_usage(cmdtp);
			goto done;
		}	
	}	

	if (imx_hab_is_enabled()) 
	{
		log_print(do_log, "HAB_FUSE: INFO: Device is already closed.");
		ret = CMD_RET_SUCCESS;
		goto done;
	}

	if (hab_fuse_programmed()) 
	{
		if (hab_fuse_content_correct()) 
		{
			log_print(do_log, "HAB_FUSE: INFO: Fuses already programmed with correct hash. Use hab_close to secure the device permanently.");
			ret = CMD_RET_SUCCESS;
		}
		else 
		{
			log_print(do_log, "HAB_FUSE: ERROR: Fuses already programmed with an unexpected hash. Aborted.");
		}			
		goto done;				
	}

    if (hab_is_dev_board(false)) 
	{
		log_print(do_log, "HAB_FUSE: WARNING: This bootloader runs on a development board. Skipping.");
		ret = CMD_RET_SUCCESS;
		goto done;			
	}

    if (!do_prog) 
	{
		char  hab_fuses_to_program[128];
		char  hab_fuses_programmed[128];
		hab_fuses_to_str(hab_fuses_to_program, sizeof(hab_fuses_to_program), hab_fuses_programmed, sizeof(hab_fuses_programmed));		
		printf("\nFUSES VALUES TO PROGRAM: {%s}\n\n", hab_fuses_to_program);
		puts(warn);
		if (confirm_yesno()) 
		{
			do_prog = true;
		} 
		else 
		{
			puts("Fuse programming aborted");
			goto done;
		}
	}

	if (do_prog) 
	{
#if defined(SMP_OFFICIAL_VERSION) && SMP_OFFICIAL_VERSION != 0
		for (i = 0 ; i < ARRAY_SIZE(s_fuses) ; i++) 
		{
			fuse_prog(s_fuses[i].bank, s_fuses[i].word, s_fuses[i].value);
		}
		if (force_fuse_refresh() != 0) 
		{
			log_print(do_log, "HAB_FUSE: WARNING: Cannot refresh fuse registers!");	
		}
		else if (!hab_fuse_content_correct()) 
		{
			log_print(do_log, "HAB_FUSE: ERROR: Invalid fused keys!");	
		}
		else
		{
			log_print(do_log, "HAB_FUSE: SUCCESS: Fuse keys programmed with success.");
			ret = CMD_RET_SUCCESS;
		}
#else
		log_print(do_log, "HAB_FUSE: SUCCESS: Nothing done (not official build)");
		ret = CMD_RET_SUCCESS;
#endif
	}
	
done:		
    log_print(do_log, "HAB_FUSE: %s: Operation %s", (ret == CMD_RET_SUCCESS) ? "SUCCESS" : "ERROR", (ret == CMD_RET_SUCCESS) ? "succeeded" : "failed");
	return ret;
}

static int do_hab_close(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = CMD_RET_FAILURE;
	bool do_prog = false;
	bool do_log  = false;
	bool sig_valid = false;
	uint32_t i;

	const char *warn = 
		"\n\nAFTER THE DEVICE SUCCESSFULLY BOOTS A SIGNED IMAGE WITHOUT GENERATING ANY HAB EVENTS,\n"
		"IT IS SAFE TO CLOSE, OR SECURE THE DEVICE.\n"
		"\n"
		"THIS IS THE LAST STEP IN THE PROCESS. ONCE THE FUSE IS BLOWN, THE CHIP DOES NOT LOAD\n"
		"AN IMAGE THAT HAS NOT BEEN SIGNED CORRECTLY.\n"
		"\n"
		"IMPORTANT NOTES:\n"
		"THIS IS AGAIN A ONE-TIME PROGRAMMABLE E-FUSE. ONCE YOU WRITE IT, YOU CAN'T GO BACK.\n"
		"IF ANYTHING IN THE PREVIOUS STEPS WASN'T DONE CORRECTLY, THE CHIP WILL NOT BOOT\n"
		"AFTER WRITING THIS BIT.\n"
		"\n"
		"DO YOU REALLY WANT TO CLOSE THE DEVICE? <y/n> ";


	if ((argc > 3)) {
		cmd_usage(cmdtp);
		goto error;
	}
	for (i = 1 ; i < argc ; i++) {
		if (!strcmp("-y", argv[i])) {
			do_prog = true;
		}
		else if (!strcmp("-log", argv[i])) {
			do_log = true;
		}	
		else {
			cmd_usage(cmdtp);
			goto error;
		}	
	}
	if (imx_hab_is_enabled()) {
		log_print(do_log, "HAB_CLOSE: INFO: Device is already closed.");
		ret = CMD_RET_SUCCESS;
		goto error;
	}	
	if (!hab_fuse_programmed()) {
		log_print(do_log, "HAB_CLOSE: ERROR: You need to burn the SRK hash e-fuses first.  Do the hab_fuse command before this one.");
		goto error;		
	}		
	if (!hab_fuse_content_correct()) {
		log_print(do_log, "HAB_CLOSE: ERROR: Fuses already programmed with unexpected hash. Aborted.");
		ret = 0;
		goto error;		
	}

    if (!do_prog) {
		puts(warn);
		if (confirm_yesno()) {
			do_prog = true;
		} else {
			puts("Device close aborted");
			goto error;
		}
	}

    uint32_t count;
    sig_valid = hab_sig_valid(&count);
	if (!sig_valid) {
		log_print(do_log, "HAB_CLOSE: ERROR: This bootloader fails signature check. Cannot close device.");
		goto error;			
	}	

    if (hab_is_dev_board(false)) 
	{
		ret = CMD_RET_SUCCESS;
		log_print(do_log, "HAB_CLOSE: WARNING: This bootloader runs on a development board. Skipping.");
		goto error;			
	}

    if (do_prog) 
	{
#if defined(SMP_OFFICIAL_VERSION) && SMP_OFFICIAL_VERSION != 0
	    struct imx_sec_config_fuse_t *fuse =
		(struct imx_sec_config_fuse_t *)&imx_sec_config_fuse;
        fuse_prog(fuse->bank, fuse->word, HAB_ENABLED_BIT);

		if (force_fuse_refresh() != 0) {
			log_print(do_log, "HAB_CLOSE: WARNING: Cannot refresh fuse registers!");	
		}

		if (!imx_hab_is_enabled()) {
			log_print(do_log, "HAB_CLOSE: ERROR: Fuse programming error. Device not closed.");
			goto error;
		}
#else
		log_print(do_log, "HAB_CLOSE: Nothing done (not official build)");
#endif
	}
	ret = CMD_RET_SUCCESS;
error:		
    log_print(do_log, "HAB_CLOSE: %s: Operation %s.", (ret == CMD_RET_SUCCESS) ? "SUCCESS":"ERROR",(ret == CMD_RET_SUCCESS) ? "succeeded": "failed");
	return ret;
}

static int do_hab_field_return(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = CMD_RET_FAILURE;
	uint32_t  i;
	bool do_prog = false;
	bool do_log = false;

	const char *warn = 
		"\n\nFIELD RETURN UNLOCK FUSE MUST BE SET IN THE UBOOT CSF FILE FIRST.\n"
		"\n"
		"IF FUSE IS NOT UNLOCKED THIS OPERATION WILL FAIL.\n"
		"\n"
		"DO YOU REALLY WANT TO SET THE FIELD RETURN FUSE? <y/n> ";

	if ((argc > 3)) {
		cmd_usage(cmdtp);
		goto error;
	}
	for (i = 1 ; i < argc ; i++) {
		if (!strcmp("-y", argv[i])) {
			do_prog = true;
		}
		else if (!strcmp("-log", argv[i])) {
			do_log = true;
		}	
		else {
			cmd_usage(cmdtp);
			goto error;
		}	
	}	

	if (!imx_hab_is_enabled()) {
		log_print(do_log, "HAB_FIELD_RETURN: INFO: Device is not closed. Aborted.");
		goto error;
	}	

    if (!do_prog) {
		puts(warn);
		if (confirm_yesno()) {
			do_prog = true;
		} else {
			puts("HAB_FIELD_RETURN: INFO: Device field return aborted");
			goto error;
		}
	}

	if (hab_is_dev_board(false)) 
	{
		ret = CMD_RET_SUCCESS;
		log_print(do_log, "HAB_FIELD_RETURN: WARNING: This bootloader runs on a development board. Skipping.");
		goto error;			
	}	

    if (do_prog)
	{
#if defined(SMP_OFFICIAL_VERSION) && SMP_OFFICIAL_VERSION != 0
		// This command will fail if the unlock command hasn't been done in the CSF for the specific UID
		// So this is not a security hole.
		// A field return implies:
		// Modify the CSF in the [Unlock] section, and add commands to unlock the FIELD_RETURN fuse with the UID (see doc)
		// Compile and sign the UBOOT.
		// Upload this UBOOT binary to the device to return.
		// Issue a bootdata_SetFieldReturnFlag() (likely from platformd), or use the hab_field_return command from the UBOOT command line.
		// This will burn the FIELD return fuse if the device is the correct UID.
		// Device will be opened again.
        if (fuse_prog(imx_field_return_fuse.bank, imx_field_return_fuse.word, imx_field_return_fuse.value) < 0) {
			log_print(do_log, "HAB_FIELD_RETURN: ERROR: Fuse programming failed. Aborted.");
            goto error;
		}
		if (force_fuse_refresh() != 0) {
			log_print(do_log, "HAB_FIELD_RETURN: WARNING: Cannot refresh fuse registers!");	
		}
#else
		log_print(do_log, "HAB_FIELD_RETURN: Nothing done (not official build)");
#endif
	}
	ret = CMD_RET_SUCCESS;
error:		
    log_print(do_log, "HAB_FIELD_RETURN: %s: Operation %s.", (ret == CMD_RET_SUCCESS) ? "SUCCESS":"ERROR",(ret == CMD_RET_SUCCESS) ? "succeeded": "failed");
	return ret;
}

extern int _fs_read(const char *filename, ulong addr, loff_t offset, loff_t len, int do_lmb_check, loff_t *actread);

static int do_hab_is_locked(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (!imx_hab_is_enabled()) 
	{
		printf("Secure boot is disabled\n");
		return CMD_RET_FAILURE;
	}

	printf("Secure boot enabled\n");
	return CMD_RET_SUCCESS;
}

static int do_hab_is_dev(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = CMD_RET_FAILURE;

	if (!hab_is_dev_board(false))
	{
		printf("This is not a dev board\n");
		return CMD_RET_FAILURE;
	}

	printf("This is a dev board\n");

	return ret;
}

static int do_hab_is_required(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (!imx_hab_is_required()) 
	{
		printf("Secure boot is not required\n");
		return CMD_RET_FAILURE;
	}

	printf("Secure boot is required\n");
	return CMD_RET_SUCCESS;
}

static int do_hab_refresh_fuse(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (force_fuse_refresh() != 0) {
		printf("Failed to refresh fuse registers\n");
		return CMD_RET_FAILURE;
	}

	printf("Fuse registers refreshed\n");
	return CMD_RET_SUCCESS;
}

// "[-log] <interface> <dev[:part]> <filename>\n"
static int do_hab_check_file(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = CMD_RET_FAILURE;
	uint32_t size, addr, count;
	imx_header_v3_t *imx_hdr;
	uint8_t event_data[128];
	size_t bytes = sizeof(event_data);
	loff_t len_read = 0;

	bool arg_log = false;
	const char *arg_interface = env_get("hab_check_file_interface");
	const char *arg_devpart = env_get("hab_check_file_dev_part");
	const char *arg_filename = env_get("hab_check_file_file");

	if (argc < 1)
		return CMD_RET_USAGE;

	int iArg = 1;
	if (iArg < argc && strcmp("-log", argv[iArg]) == 0) 
	{
		arg_log = true;
		iArg++;
	}
	
	if (iArg < argc)
		arg_interface = argv[iArg++];
	if (iArg < argc)
		arg_devpart = argv[iArg++];
	if (iArg < argc)
		arg_filename = argv[iArg++];

	if (iArg < argc)
		return CMD_RET_USAGE;

	if (!arg_interface) {
		puts("** No interface defined **\n");
		return 1;
	}
	if (!arg_devpart) {
		puts("** No dev:part defined **\n");
		return 1;
	}
	if (!arg_filename) {
		puts("** No file defined **\n");
		return 1;
	}

	if (fs_set_blk_dev(arg_interface, arg_devpart, FS_TYPE_ANY)) {
		log_err("Can't set block device\n");
		return 1;
	}

	len_read = 0;
	ret = _fs_read(arg_filename, DRAM_ADDRESS, 0, 0, 1, &len_read);
	if (ret < 0) {
		log_err("Failed to load '%s' (%d)\n", arg_filename, ret);
		return 1;
	}

	if (len_read < sizeof(imx_header_v3_t))
	{
		log_print(arg_log, "hab_check_file: ERROR: %s has an invalid size (%d).", arg_filename, len_read);
		goto error;
	}

	imx_hdr = (imx_header_v3_t *)DRAM_ADDRESS;
	if (verify_ivt_header((struct ivt_header *)imx_hdr))
	{
		log_print(arg_log, "hab_check_file: ERROR: %s has an invalid IVT.", arg_filename);
		goto error;
	}
	addr = imx_hdr->boot_data.start + 0x400;
	size = imx_hdr->boot_data.size - 0x400;

	if ((addr < BOOTROM_BASE_ADDR) || (addr >= BOOTROM_BASE_ADDR + BOOTROM_SIZE))
	{
		log_print(arg_log, "hab_check_file: ERROR: %s is invalid.", arg_filename);
		goto error;
	}
	memcpy((char *)addr, (char *)DRAM_ADDRESS, size);
	if (imx_hab_authenticate_image(addr, size, 0, true))
	{
		log_print(arg_log, "hab_check_file: ERROR: %s fails signature check. Aborted.", arg_filename);
		goto error;
	}
	count = 0;
	while (hab_rvt_report_event(HAB_STS_ANY, count, event_data, &bytes) == HAB_SUCCESS)
	{
		count++;
		bytes = sizeof(event_data);
	}
	if (count)
	{
		log_print(arg_log, "hab_check_file: INFO: %s fails signature check.", arg_filename);
		goto error;
	}
	log_print(arg_log, "hab_check_file: INFO: %s is genuine.", arg_filename);
	ret = CMD_RET_SUCCESS;
error:
	return ret;
}

static int do_hab_version(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct hab_hdr *hdr = (struct hab_hdr *)HAB_RVT_BASE;

	if (hdr->tag != HAB_TAG_RVT) {
		printf("Unexpected header tag: %x\n", hdr->tag);
		return CMD_RET_FAILURE;
	}

	printf("HAB version: %d.%d\n", hdr->par >> 4, hdr->par & 0xf);

	return 0;
}

static int do_authenticate_image_or_failover(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = CMD_RET_FAILURE;

	if (argc != 4) {
		ret = CMD_RET_USAGE;
		goto error;
	}
	if (!imx_hab_is_enabled()) {
		printf("error: secure boot disabled\n");
		goto error;
	}
	if (do_authenticate_image(NULL, flag, argc, argv) != CMD_RET_SUCCESS) {
		fprintf(stderr, "authentication fail -> %s %s %s %s\n",
			argv[0], argv[1], argv[2], argv[3]);
		do_hab_failsafe(0, 0, 1, NULL);
	};
	ret = CMD_RET_SUCCESS;
error:
	return ret;
}

#ifdef CONFIG_MX7ULP
U_BOOT_CMD(
		hab_status, CONFIG_SYS_MAXARGS, 2, do_hab_status,
		"display HAB status and events",
		"hab_status - A7 HAB event and status\n"
		"hab_status m4 - M4 HAB event and status"
	  );
#else
U_BOOT_CMD(
		hab_status, CONFIG_SYS_MAXARGS, 1, do_hab_status,
		"display HAB status",
		""
	  );
#endif

U_BOOT_CMD(
		hab_auth_img, 4, 0, do_authenticate_image,
		"authenticate image via HAB",
		"addr length ivt_offset\n"
		"addr - image hex address\n"
		"length - image hex length\n"
		"ivt_offset - hex offset of IVT in the image"
	  );

U_BOOT_CMD(
		hab_failsafe, CONFIG_SYS_MAXARGS, 1, do_hab_failsafe,
		"run BootROM failsafe routine",
		""
	  );

U_BOOT_CMD(
		hab_auth_img_or_fail, 4, 0,
		do_authenticate_image_or_failover,
		"authenticate image via HAB on failure drop to USB BootROM mode",
		"addr length ivt_offset\n"
		"addr - image hex address\n"
		"length - image hex length\n"
		"ivt_offset - hex offset of IVT in the image"
	  );

U_BOOT_CMD(
		hab_version, 1, 0, do_hab_version,
		"print HAB major/minor version",
		""
	  );

U_BOOT_CMD(
		hab_fuse, 3, 0, do_hab_fuse,
		"fuse hash in OTP",
		"hab_fuse [-y]"
	  );

U_BOOT_CMD(
		hab_close, 3, 0, do_hab_close,
		"close the device",
		"hab_close [-y]"
	  );

U_BOOT_CMD(
		hab_field_return, 3, 0, do_hab_field_return,
		"Set the field return fuse in the device",
		"hab_field_return [-y]"
	  );	  

U_BOOT_CMD(
		hab_check_file, 6, 0, do_hab_check_file,
		"validate a file signature (HAB)",
		"[-log] <interface> <dev[:part]> <filename>\n"
		"    - Validate a file 'filename' from partition 'part' on device\n"
		"       type 'interface' instance 'dev'.\n"
	);
	
U_BOOT_CMD(
		hab_is_locked, 1, 0, do_hab_is_locked,
		"Test if the device is locked",
		"hab_is_locked"
	  );

U_BOOT_CMD(
		hab_is_dev, 1, 0, do_hab_is_dev,
		"Test if the device is a dev board",
		"hab_is_dev"
	  );

U_BOOT_CMD(
		hab_is_required, 1, 0, do_hab_is_required,
		"Test if HAB authentication is required",
		"hab_is_required"
	  );

U_BOOT_CMD(
		hab_refresh_fuse, 1, 0, do_hab_refresh_fuse,
		"refresh fuse shadow resiters",
		"hab_refresh_fuse"
	  );


#endif /* !defined(CONFIG_SPL_BUILD) */

/* Get CSF Header length */
static int get_hab_hdr_len(struct hab_hdr *hdr)
{
	return (size_t)((hdr->len[0] << 8) + (hdr->len[1]));
}

/* Check whether addr lies between start and
 * end and is within the length of the image
 */
static int chk_bounds(u8 *addr, size_t bytes, u8 *start, u8 *end)
{
	size_t csf_size = (size_t)((end + 1) - addr);

	return (addr && (addr >= start) && (addr <= end) &&
		(csf_size >= bytes));
}

/* Get Length of each command in CSF */
static int get_csf_cmd_hdr_len(u8 *csf_hdr)
{
	if (*csf_hdr == HAB_CMD_HDR)
		return sizeof(struct hab_hdr);

	return get_hab_hdr_len((struct hab_hdr *)csf_hdr);
}

/* Check if CSF is valid */
static bool csf_is_valid(struct ivt *ivt, ulong start_addr, size_t bytes)
{
	u8 *start = (u8 *)start_addr;
	u8 *csf_hdr;
	u8 *end;

	size_t csf_hdr_len;
	size_t cmd_hdr_len;
	size_t offset = 0;

	if (bytes != 0)
		end = start + bytes - 1;
	else
		end = start;

	/* Verify if CSF pointer content is zero */
	if (!ivt->csf) {
#ifndef CONFIG_SPL_BUILD			
		puts("Error: CSF pointer is NULL\n");
#endif		
		return false;
	}

	csf_hdr = (u8 *)(ulong)ivt->csf;

	/* Verify if CSF Header exist */
	if (*csf_hdr != HAB_CMD_HDR) {
#ifndef CONFIG_SPL_BUILD			
		puts("Error: CSF header command not found\n");
#endif		
		return false;
	}

	csf_hdr_len = get_hab_hdr_len((struct hab_hdr *)csf_hdr);

	/* Check if the CSF lies within the image bounds */
	if (!chk_bounds(csf_hdr, csf_hdr_len, start, end)) {
#ifndef CONFIG_SPL_BUILD			
		puts("Error: CSF lies outside the image bounds\n");
#endif		
		return false;
	}

	do {
		struct hab_hdr *cmd;

		cmd = (struct hab_hdr *)&csf_hdr[offset];

		switch (cmd->tag) {
		case (HAB_CMD_WRT_DAT):
#ifndef CONFIG_SPL_BUILD			
			puts("Error: Deprecated write command found\n");
#endif			
			return false;
		case (HAB_CMD_CHK_DAT):
#ifndef CONFIG_SPL_BUILD			
			puts("Error: Deprecated check command found\n");
#endif			
			return false;
		case (HAB_CMD_SET):
			if (cmd->par == HAB_PAR_MID) {
#ifndef CONFIG_SPL_BUILD					
				puts("Error: Deprecated Set MID command found\n");
#endif				
				return false;
			}
		default:
			break;
		}

		cmd_hdr_len = get_csf_cmd_hdr_len(&csf_hdr[offset]);
		if (!cmd_hdr_len) {
#ifndef CONFIG_SPL_BUILD				
			puts("Error: Invalid command length\n");
#endif			
			return false;
		}
		offset += cmd_hdr_len;

	} while (offset < csf_hdr_len);

	return true;
}


/*
 * Validate IVT structure of the image being authenticated
 */
static int validate_ivt(struct ivt *ivt_initial)
{
	struct ivt_header *ivt_hdr = &ivt_initial->hdr;

	if ((ulong)ivt_initial & 0x3) {
#ifndef CONFIG_SPL_BUILD			
		puts("Error: Image's start address is not 4 byte aligned\n");
#endif		
		return 0;
	}

	/* Check IVT fields before allowing authentication */
	if ((!verify_ivt_header(ivt_hdr)) && \
	    (ivt_initial->entry != 0x0) && \
	    (ivt_initial->reserved1 == 0x0) && \
	    (ivt_initial->self == \
		   (uint32_t)((ulong)ivt_initial & 0xffffffff)) && \
	    (ivt_initial->csf != 0x0) && \
	    (ivt_initial->reserved2 == 0x0)) {
		/* Report boot failure if DCD pointer is found in IVT */
		if (ivt_initial->dcd != 0x0) {
#ifndef CONFIG_SPL_BUILD				
			puts("Error: DCD pointer must be 0\n");
#endif			
		}
		else
			return 1;
	}

#ifndef CONFIG_SPL_BUILD	
	puts("Error: Invalid IVT structure\n");
	debug("\nAllowed IVT structure:\n");
	debug("IVT HDR       = 0x4X2000D1\n");
	debug("IVT ENTRY     = 0xXXXXXXXX\n");
	debug("IVT RSV1      = 0x0\n");
	debug("IVT DCD       = 0x0\n");		/* Recommended */
	debug("IVT BOOT_DATA = 0xXXXXXXXX\n");	/* Commonly 0x0 */
	debug("IVT SELF      = 0xXXXXXXXX\n");	/* = ddr_start + ivt_offset */
	debug("IVT CSF       = 0xXXXXXXXX\n");
	debug("IVT RSV2      = 0x0\n");
#endif
	/* Invalid IVT structure */
	return 0;
}

bool imx_hab_is_enabled(void)
{
	bool enabled = false;
	struct imx_sec_config_fuse_t *fuse =
		(struct imx_sec_config_fuse_t *)&imx_sec_config_fuse;
	uint32_t reg;
	int ret;

	ret = fuse_read(fuse->bank, fuse->word, &reg);
	if (ret) {
#ifndef CONFIG_SPL_BUILD			
		puts("\nSecure boot fuse read error\n");
#endif		
        goto end;
	}
   
	enabled =  (reg & HAB_ENABLED_BIT) == HAB_ENABLED_BIT;
end:
    return enabled;

}

int imx_hab_authenticate_image(uint32_t ddr_start, uint32_t image_size, uint32_t ivt_offset, bool force_required)
{
	ulong load_addr = 0;
	size_t bytes;
	ulong ivt_addr = 0;
	int result = 1;
	ulong start;
	struct ivt *ivt;
	enum hab_status status;

#ifndef CONFIG_SPL_BUILD
		if (!imx_hab_is_enabled())
			puts("hab fuse not enabled\n");
		printf("\nAuthenticate image from DDR location 0x%x...\n", ddr_start);
#endif	

	hab_caam_clock_enable(1);

	/* Calculate IVT address header */
	ivt_addr = (ulong) (ddr_start + ivt_offset);
	ivt = (struct ivt *)ivt_addr;

	/* Verify IVT header bugging out on error */
	if (!validate_ivt(ivt))
		goto hab_authentication_exit;

	start = ddr_start;
	bytes = image_size;

	/* Verify CSF */
	if (!csf_is_valid(ivt, start, bytes))
		goto hab_authentication_exit;

	if (hab_rvt_entry() != HAB_SUCCESS) {	
#ifndef CONFIG_SPL_BUILD		
		puts("hab entry function fail\n");
#endif
		goto hab_exit_failure_print_status;
	}

	status = hab_rvt_check_target(HAB_TGT_MEMORY, (void *)(ulong)ddr_start, bytes);
	if (status != HAB_SUCCESS) {
#ifndef CONFIG_SPL_BUILD		
		printf("HAB check target 0x%08x-0x%08lx fail\n", ddr_start, ddr_start + (ulong)bytes);
#endif		
		goto hab_exit_failure_print_status;
	}

#ifdef DEBUG
	printf("\nivt_offset = 0x%x, ivt addr = 0x%lx\n", ivt_offset, ivt_addr);
	printf("ivt entry = 0x%08x, dcd = 0x%08x, csf = 0x%08x\n", ivt->entry,
	       ivt->dcd, ivt->csf);
	puts("Dumping IVT\n");
	print_buffer(ivt_addr, (void *)(ivt_addr), 4, 0x8, 0);

	puts("Dumping CSF Header\n");
	print_buffer(ivt->csf, (void *)(ivt->csf), 4, 0x10, 0);

#if  !defined(CONFIG_SPL_BUILD)
	get_hab_status(false);
#endif

	puts("\nCalling authenticate_image in ROM\n");
	printf("\tivt_offset = 0x%x\n", ivt_offset);
	printf("\tstart = 0x%08lx\n", start);
	printf("\tbytes = 0x%x\n", bytes);
#endif

#ifndef CONFIG_ARM64
	/*
	 * If the MMU is enabled, we have to notify the ROM
	 * code, or it won't flush the caches when needed.
	 * This is done, by setting the "pu_irom_mmu_enabled"
	 * word to 1. You can find its address by looking in
	 * the ROM map. This is critical for
	 * authenticate_image(). If MMU is enabled, without
	 * setting this bit, authentication will fail and may
	 * crash.
	 */
	/* Check MMU enabled */
	if (is_soc_type(MXC_SOC_MX6) && get_cr() & CR_M) {
		if (is_mx6dq()) {
			/*
			 * This won't work on Rev 1.0.0 of
			 * i.MX6Q/D, since their ROM doesn't
			 * do cache flushes. don't think any
			 * exist, so we ignore them.
			 */
			if (!is_mx6dqp())
				writel(1, MX6DQ_PU_IROM_MMU_EN_VAR);
		} else if (is_mx6sdl()) {
			writel(1, MX6DLS_PU_IROM_MMU_EN_VAR);
		} else if (is_mx6sl()) {
			writel(1, MX6SL_PU_IROM_MMU_EN_VAR);
		}
	}
#endif

	load_addr = (ulong)hab_rvt_authenticate_image(
			HAB_CID_UBOOT,
			ivt_offset, (void **)&start,
			(size_t *)&bytes, NULL);
	if (hab_rvt_exit() != HAB_SUCCESS) {
#ifndef CONFIG_SPL_BUILD
		puts("hab exit function fail\n");
#endif		
		load_addr = 0;
	}

hab_exit_failure_print_status:
#if !defined(CONFIG_SPL_BUILD)
	get_hab_status(false);
#endif

hab_authentication_exit:

	if (load_addr != 0 || (!force_required && !imx_hab_is_required()))
		result = 0;

	return result;
}

int authenticate_image(u32 ddr_start, u32 raw_image_size)
{
	u32 ivt_offset;
	size_t bytes;

	ivt_offset = (raw_image_size + ALIGN_SIZE - 1) &
					~(ALIGN_SIZE - 1);
	bytes = ivt_offset + IVT_SIZE + CSF_PAD_SIZE;

	return imx_hab_authenticate_image(ddr_start, bytes, ivt_offset, false);
}

void hab_late_init(bool fuse_device, bool close_device, bool check_boot, bool field_return, bool reset_after, uint8_t extra_cmds)
{
#ifndef CONFIG_SPL_BUILD	

int hab_late_init(struct eaton_boot_data_struct *boot_data)
{	
    // Keep this code here as the following run_command() can generate events when testing for boot validity	
    get_hab_status(true);

	if (boot_data == NULL)
		return -1;

	if (boot_data->debug_prog_fuses == 0)
	{
		if (hab_fuse_programmed())
		{
			boot_data->fuse_device = hab_fuse_content_correct() ? BOOT_DATA_ACTION_SUCCESS : BOOT_DATA_ACTION_INVALID;
		}

		if (imx_hab_is_enabled())
		{
			boot_data->close_device = BOOT_DATA_ACTION_SUCCESS;
		}

		enum hab_config config = 0;
		enum hab_state state = 0;  
		enum hab_status status = hab_rvt_report_status(&config, &state);
		bool config_secure;
		bool config_field_return;
		hab_config_to_str(config, &config_field_return, &config_secure);
		if (config_field_return)
		{
			boot_data->field_return = BOOT_DATA_ACTION_SUCCESS;
		}
	}

#if defined(SMP_OFFICIAL_VERSION) && SMP_OFFICIAL_VERSION != 0
	if (boot_data->fuse_device == BOOT_DATA_ACTION_EXEC)
	{		
		if (run_command("hab_fuse -y -log", 0) != CMD_RET_SUCCESS)
		{
			boot_data->fuse_device = BOOT_DATA_ACTION_FAILED;
			if (boot_data->close_device)
				boot_data->close_device = BOOT_DATA_ACTION_CANCELED;
			if (boot_data->field_return)
				boot_data->field_return = BOOT_DATA_ACTION_CANCELED;
			if (boot_data->check_boot)
				boot_data->check_boot = BOOT_DATA_ACTION_CANCELED;

			printf("Error fusing the device\n");
		}
		else
		{
			boot_data->fuse_device = BOOT_DATA_ACTION_SUCCESS;
			return 1; // reset
		}
	}
	else if (boot_data->close_device == BOOT_DATA_ACTION_EXEC)
	{		
		if (run_command("hab_close -y -log", 0) != CMD_RET_SUCCESS)
		{
			boot_data->close_device = BOOT_DATA_ACTION_FAILED;
			if (boot_data->field_return)
				boot_data->field_return = BOOT_DATA_ACTION_CANCELED;
			if (boot_data->check_boot)
				boot_data->check_boot = BOOT_DATA_ACTION_CANCELED;
			printf("Error closing the device\n");
		}
		else
		{
			boot_data->close_device = BOOT_DATA_ACTION_SUCCESS;
			return 1; // reset
		}
	}
	else if (boot_data->field_return == BOOT_DATA_ACTION_EXEC)
	{		
		if (run_command("hab_field_return -y -log", 0) != CMD_RET_SUCCESS)
		{
			boot_data->field_return = BOOT_DATA_ACTION_FAILED;
			if (boot_data->check_boot)
				boot_data->check_boot = BOOT_DATA_ACTION_CANCELED;
			printf("Error fusing the field_return\n");
		}
		else
		{
			boot_data->field_return = BOOT_DATA_ACTION_SUCCESS;
			return 1; // reset
		}
	}
#endif

    create_stats_and_bootargs();
	smp_stats_flush();

    // Keep this last as this may generate events
	if (boot_data->check_boot == BOOT_DATA_ACTION_EXEC)
	{		
		if (run_command("hab_check_file -log", 0) != CMD_RET_SUCCESS)
		{
			boot_data->check_boot = BOOT_DATA_ACTION_FAILED;
		}
		else
		{
			boot_data->check_boot = BOOT_DATA_ACTION_SUCCESS;
		}
	}

    smp_log_flush();	

	return 0;
}
#endif

bool imx_hab_is_required(void)
{
	if (!imx_hab_is_enabled())
		return false;

#ifndef CONFIG_SPL_BUILD
	if (hab_is_dev_board(false))
		return false;
#endif

	return true;
}
