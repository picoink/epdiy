#pragma once
#include <stddef.h>

// S31's native RGB transaction has no dummy phase: each active clock consumes
// payload. Keep the legacy LCD workaround only where that phase is enabled.
static inline int epd_lcd_dummy_bytes(int bus_width) {
#if defined(CONFIG_IDF_TARGET_ESP32S31)
    (void)bus_width;
    return 0;
#else
    return bus_width / 8;
#endif
}

static inline size_t epd_lcd_row_offset(size_t row, int line_bytes, int dummy_bytes) {
    return row * (size_t)(line_bytes + dummy_bytes) + (size_t)(dummy_bytes % 2);
}
