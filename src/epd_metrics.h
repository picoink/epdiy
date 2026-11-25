#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// The sole update owner resets/reads these counters while no scan is active.
// ISR work only updates existing LCD counters; aggregation happens on the owner.
typedef struct {
    uint32_t cycle_us;
    uint32_t scan_us;
    uint32_t lut_us;
    uint32_t feed_wait_us;
    uint32_t start_us, frame_wait_us, pace_us, feed0_us, feed1_us;
    uint32_t phase_avg_us;
    uint32_t phase_min_us;
    uint32_t phase_max_us;
    uint32_t eof_gap_us;
    uint32_t eof_work_us;
    uint32_t phases;
    uint32_t completed;
    uint32_t faults;
    uint32_t fault_phase;
    uint32_t fault_line;
    uint32_t fault_thread;

    uint32_t pixel_clock_hz;
} EpdMetrics;
void epd_metrics_reset(void);
EpdMetrics epd_metrics_get(void);
void epd_metrics_add(const EpdMetrics* sample);
#ifdef __cplusplus
}
#endif
