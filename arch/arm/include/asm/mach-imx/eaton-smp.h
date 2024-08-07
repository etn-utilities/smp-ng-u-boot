#ifndef __SMP_H
#define __SMP_H

#include <stdint.h>
#include <stdbool.h>

enum
{
	SMP_LOG_PRIORITY_EMERGENCY,
	SMP_LOG_PRIORITY_ALERT,
	SMP_LOG_PRIORITY_CRITICAL,
	SMP_LOG_PRIORITY_ERROR,
	SMP_LOG_PRIORITY_WARNING,
	SMP_LOG_PRIORITY_NOTICE,
	SMP_LOG_PRIORITY_INFORMATIONAL
};

#define SMP_LOG_PAGE_RESET	  "Reset"
#define SMP_LOG_PAGE_SECURITY "Security"
#define SMP_LOG_PAGE_STARTUP  "Startup"
#define SMP_LOG_PAGE_SYSTEM	  "System"
#define SMP_LOG_PAGE_TIME	  "Time"

#define SMP_LOG_CODE_DEFAULT "uboot"

int smp_stats_begin(char *group, char *descr, uint32_t level);
int smp_stats_add_uint64_t(const char *descr, uint32_t level, const char *field, uint64_t value);
int smp_stats_add_uint32_t(const char *descr, uint32_t level, const char *field, uint32_t value);
int smp_stats_add_str(const char *descr, uint32_t level, const char *field, const char *value);
int smp_stats_add_bool(const char *descr, uint32_t level, const char *field, bool value);
int smp_stats_add_hex32(const char *descr, uint32_t level, const char *field, uint32_t value);
int smp_stats_add_hex64(const char *descr, uint32_t level, const char *field, uint64_t value);
int smp_stats_end(void);
int smp_stats_flush(void);

int smp_log_add(uint32_t priority, const char *log_entry, const char *log_page, const char *log_code);
int smp_log_flush(void);

int smp_file_read(const char *filename, char *buffer, uint32_t buffer_size, uint32_t *len_read);
uint64_t smp_board_serial(void);
bool smp_is_dev_board(bool bIgnoreEnv);
const uint64_t* smp_get_dev_board_ids();

#endif /* _SMP_H */
