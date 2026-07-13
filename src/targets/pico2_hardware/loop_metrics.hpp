#pragma once

#include <stdint.h>

void loop_metrics_init(void);
void loop_metrics_on_loop_complete(uint64_t loop_time_us);
void loop_metrics_report_due(void);
uint32_t loop_metrics_max_loop_time_us(void);
