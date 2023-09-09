#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <fs.h>
#include <env.h>
#include <common.h>
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/eaton-smp.h>
#include <asm/mach-imx/hab.h>
#include <command.h>

#define FILEDATA_IFACE			  "mmc"
#define FILEDATA_PART			  "2:2"
#define SMP_LOG_SYSLOG_IDENTIFIER "uboot_hab"
#define SMP_LOG_FILENAME		  "/uboot_logs.json"
#define SMP_STATS_FILENAME		  "/uboot_stats.json"

#define MAX_READBACK_SIZE 4096
#define MAX_LOG_SIZE	  4096
#define MAX_STATS_SIZE	  4096

static char readback_buffer[MAX_READBACK_SIZE];
static char log_buffer[MAX_LOG_SIZE];
static int log_index = 0;

static char stats_buffer[MAX_STATS_SIZE];
static int stats_index = 0;

//
// Add here the list of development boards UIDs
//
uint64_t dev_board_uid_list[] = {
	0x232529AAC2A3784A, // Mario wrong keys
	0x1415B209DAB6FAD9, // DB ptorotype 2GB RAM	
	//0x102821AAC2A3784A, // DB
	0x222729AAC2A3784A, // 11000037 - AB
	0x2B1919AAC654D589, // 11000087 - AB
	0x232629AAC2A3784A, // 11000035 - FO first locked SOM

	0
};

uint64_t smp_board_serial(void)
{
	struct ocotp_regs *ocotp = (struct ocotp_regs *)OCOTP_BASE_ADDR;
	struct fuse_bank *bank = &ocotp->bank[0];
	struct fuse_bank0_regs *fuse = (struct fuse_bank0_regs *)bank->fuse_regs;

	uint64_t serial = ((uint64_t)fuse->uid_high << 32) | (uint64_t)fuse->uid_low;

	return serial;
}

bool smp_is_dev_board(bool bIgnoreEnv)
{
	uint64_t uid = smp_board_serial();

	for (uint32_t i = 0; dev_board_uid_list[i] != 0; i++)
	{
		if (uid == dev_board_uid_list[i])
		{
			if (!bIgnoreEnv && env_get_yesno("ignore_dev_board") == 1)
				return false;

			return true;
		}
	}

	return false;
}

static int smp_stats_append(const char *line)
{
	uint32_t len;
	if (stats_index < MAX_STATS_SIZE)
	{
		strncpy(&stats_buffer[stats_index], line, MAX_STATS_SIZE - stats_index);
		stats_buffer[MAX_STATS_SIZE - 1] = 0;
		len = strlen(&stats_buffer[stats_index]);
		stats_index += len;
		return 0;
	}
	return 1;
}

int smp_stats_begin(char *group, char *descr, uint32_t level)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"group\":\"%s\",\"descr\":\"%s\",\"level\":%u,\"stats\":[", group, descr, level);
	line[sizeof(line) - 1] = '\0';
    return (smp_stats_append(line));
}

int smp_stats_add_uint64_t(const char *descr, uint32_t level, const char *field, uint64_t value)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"name\":\"%s\",\"value\":%llu,\"level\":%u,\"descr\":\"%s\"},", field, value, level, descr);
	line[sizeof(line) - 1] = '\0';
    smp_stats_append(line);
}

int smp_stats_add_uint32_t(const char *descr, uint32_t level, const char *field, uint32_t value)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"name\":\"%s\",\"value\":%u,\"level\":%u,\"descr\":\"%s\"},", field, value, level, descr);
	line[sizeof(line) - 1] = '\0';
    return (smp_stats_append(line));
}

int smp_stats_add_str(const char *descr, uint32_t level, const char *field, const char *value)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"name\":\"%s\",\"value\":\"%s\",\"level\":%u,\"descr\":\"%s\"},", field, value, level, descr);
	line[sizeof(line) - 1] = '\0';
    return (smp_stats_append(line));
}

int smp_stats_add_bool(const char *descr, uint32_t level, const char *field, bool value)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"name\":\"%s\",\"value\":%s,\"level\":%u,\"descr\":\"%s\"},", field, value ? "true" : "false", level, descr);
	line[sizeof(line) - 1] = '\0';     
    return (smp_stats_append(line));
}

int smp_stats_add_hex32(const char *descr, uint32_t level, const char *field, uint32_t value)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"name\":\"%s\",\"value\":\"0x%08X\",\"level\":%u,\"descr\":\"%s\"},", field, value, level, descr);
	line[sizeof(line) - 1] = '\0';
    return (smp_stats_append(line));
}

int smp_stats_add_hex64(const char *descr, uint32_t level, const char *field, uint64_t value)
{
	char line[256];
	snprintf(line, sizeof(line), "{\"name\":\"%s\",\"value\":\"0x%16" PRIX64 "\",\"level\":%u,\"descr\":\"%s\"},", field, value, level, descr);
	line[sizeof(line) - 1] = '\0';
    return (smp_stats_append(line));
}

int smp_stats_end(void)
{
	// Remove last ','
	if (stats_index)
	{
		if (stats_buffer[stats_index - 1] == ',')
			stats_index--;
	}
	return (smp_stats_append("]}\n"));
}

int smp_file_read(const char *filename, char *buffer, uint32_t buffer_size, uint32_t *len_read)
{
	char command[128];

	*len_read = 0;
	snprintf(command, sizeof(command), "ext4load %s %s 0x%x %s %u", FILEDATA_IFACE, FILEDATA_PART, buffer, filename, buffer_size);
	command[sizeof(command) - 1] = 0;
	if (run_command(command, 0))
	{
		return 1;
	}
	*len_read = env_get_hex("filesize", 0);
	return 0;
}

int smp_stats_flush(void)
{
	uint32_t size;
	uint32_t len_written = 0;
	loff_t len_read = 0;
	int ret;

	stats_index = 0;

	size = strlen(stats_buffer);

	if (fs_set_blk_dev(FILEDATA_IFACE, FILEDATA_PART, FS_TYPE_EXT) != 0)
		return 1;

	ret = fs_read(SMP_STATS_FILENAME, (uint32_t)readback_buffer, 0, sizeof(readback_buffer), &len_read);
	if (ret >= 0)
	{
		if ((len_read == size) && !strncmp(readback_buffer, stats_buffer, len_read))
		{
			return 0;
		}
	}

	if (fs_set_blk_dev(FILEDATA_IFACE, FILEDATA_PART, FS_TYPE_EXT) != 0)
		return 1;

	ret = fs_write(SMP_STATS_FILENAME, (uint32_t)stats_buffer, 0, size, &len_written);
	if (ret < 0)
		return 2;
}

int smp_log_add(uint32_t priority, const char *log_entry, const char *log_page, const char *log_code)
{
	uint32_t len;
	int result = 0;
	if (log_index < MAX_LOG_SIZE)
	{
		snprintf(&log_buffer[log_index], MAX_LOG_SIZE - log_index, "{"
																   "\"MESSAGE\":\"%s\","
																   "\"PRIORITY\":\"%u\","
																   "\"SYSLOG_IDENTIFIER\":\"%s\","
																   "\"SMP_LOG_PAGE_NAME\":\"%s\","
																   "\"SMP_LOG_CODE\":\"%s\""
																   "}\n",
				 log_entry,
				 priority,
				 SMP_LOG_SYSLOG_IDENTIFIER,
				 log_page,
				 log_code);
		log_buffer[MAX_LOG_SIZE - 1] = 0;
		len = strlen(&log_buffer[log_index]);
		log_index += len;
	}
	else
	{
		log_buffer[MAX_LOG_SIZE - 1] = 0;
		result = 1;
	}
	return result;
}

int smp_log_flush(void)
{
	uint32_t size;
	loff_t len_read = 0;
	loff_t len_written = 0;
	int ret;
	int result = 1;

	if (fs_set_blk_dev(FILEDATA_IFACE, FILEDATA_PART, FS_TYPE_EXT) != 0)
	{
		goto end;
	}

	size = strlen(log_buffer);
	ret = fs_read(SMP_LOG_FILENAME, (uint32_t)readback_buffer, 0, sizeof(readback_buffer), &len_read);
	if (ret >= 0)
	{
		if ((len_read == size) && strncmp(readback_buffer, log_buffer, len_read) == 0)
		{
			result = 0;
			goto end;
		}
	}

	if (fs_set_blk_dev(FILEDATA_IFACE, FILEDATA_PART, FS_TYPE_EXT) != 0)
	{
		goto end;
	}

	ret = fs_write(SMP_LOG_FILENAME, (uint32_t)log_buffer, 0, size, &len_written);
	if (ret < 0)
	{
		result = 2;
		goto end;
	}
	result = 0;
end:
	return result;
}
