#pragma once

#include "../output_common/render_context.h"
#ifndef EPD_LCD_FULL_PHASE
#define EPD_LCD_FULL_PHASE 0
#endif
bool epd_lcd_has_phase_buffer(void);

// Two immutable phases plus one spare ring slot, including S31 IRQ-core skew.
// Full-phase publication removes the scan's dependence on feeder scheduling.
static inline int epd_lcd_phase_queue_capacity(int lines) {
    int capacity = (lines + NUM_RENDER_THREADS - 1) / NUM_RENDER_THREADS + 1;
    // Direct DMA consumes each complete queue before scanning. Its two wire
    // images replace the second queued phase rather than expanding the budget.
    if(EPD_LCD_FULL_PHASE && epd_lcd_has_phase_buffer())return capacity;
#if defined(CONFIG_IDF_TARGET_ESP32S31)
    capacity += (lines + 15) / 16;
#endif
    // Prepare the complete next phase while DMA consumes the current one.
    // A scan still starts only after both feeders publish every row.
    return 2 * (capacity - 1) + 1;
}

/**
 * Lighten / darken picels using the LCD driving method.
 */
void epd_push_pixels_lcd(RenderContext_t* ctx, short time, int color);

/**
 * Do a full update cycle with a configured context.
 */
void lcd_do_update(RenderContext_t* ctx);

/**
 * Worker thread for output calculation.
 * In LCD mode, both threads do the same thing.
 */
void lcd_calculate_frame(RenderContext_t* ctx, int thread_id);
