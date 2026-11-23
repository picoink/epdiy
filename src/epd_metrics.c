#include "epd_metrics.h"
#include <string.h>
static EpdMetrics metrics;
void epd_metrics_reset(void) { memset(&metrics, 0, sizeof(metrics)); }
EpdMetrics epd_metrics_get(void) { return metrics; }
void epd_metrics_add(const EpdMetrics* s) {
    if (!s->phases) return;
    unsigned previous = metrics.phases;
    metrics.cycle_us += s->cycle_us; metrics.scan_us += s->scan_us;
    metrics.lut_us += s->lut_us; metrics.feed_wait_us += s->feed_wait_us;
    metrics.phase_avg_us = (metrics.phase_avg_us * previous + s->phase_avg_us * s->phases) / (previous + s->phases);
    if (!previous || s->phase_min_us < metrics.phase_min_us) metrics.phase_min_us = s->phase_min_us;
    if (s->phase_max_us > metrics.phase_max_us) metrics.phase_max_us = s->phase_max_us;
    if (s->eof_gap_us > metrics.eof_gap_us) metrics.eof_gap_us = s->eof_gap_us;
    if (s->eof_work_us > metrics.eof_work_us) metrics.eof_work_us = s->eof_work_us;
    metrics.phases += s->phases; metrics.completed += s->completed;
    if (s->faults) { metrics.fault_phase=s->fault_phase; metrics.fault_line=s->fault_line; metrics.fault_thread=s->fault_thread; }
    metrics.faults += s->faults; metrics.pixel_clock_hz = s->pixel_clock_hz;
}
