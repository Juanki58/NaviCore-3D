#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void safe_log_init(void);
void safe_log_flush_pending(void);
void safe_log(const char *message);
void safe_logf(const char *fmt, ...);

bool safe_log_pending_count(uint8_t *count_out);
