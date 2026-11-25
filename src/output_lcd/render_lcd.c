#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "../output_common/render_method.h"

#ifdef RENDER_METHOD_LCD

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/cache.h>

#include "../epd_internals.h"
#include "../output_common/line_queue.h"
#include "../output_common/lut.h"
#include "../output_common/spatial_wipe.h"
#include "../output_common/render_context.h"
#include "epd_board.h"
#include "epdiy.h"
#include "lcd_driver.h"
#include "render_lcd.h"
#include "../epd_metrics.h"

// declare vector optimized line mask application.
void epd_apply_line_mask_VE(uint8_t* line, const uint8_t* mask, int mask_len);

static void IRAM_ATTR record_underflow(RenderContext_t* ctx,int reason,int thread) {
    if(!ctx->fault_reason) {
        ctx->fault_phase=ctx->scan_frame+1;
        ctx->fault_line=ctx->lines_consumed;
        ctx->fault_thread=thread;
        ctx->fault_ready=ctx->lines_consumed<ctx->lines_total ?
            __atomic_load_n(&ctx->line_ready[ctx->lines_consumed],__ATOMIC_ACQUIRE)
                >= ctx->scan_frame+1 : -1;
        for(int i=0;i<NUM_RENDER_THREADS;++i) {
            ctx->fault_write[i]=atomic_load_explicit(&ctx->line_queues[i].current,memory_order_acquire);
            ctx->fault_read[i]=atomic_load_explicit(&ctx->line_queues[i].last,memory_order_acquire);
        }
        ctx->fault_reason=reason;
    }
    ctx->error|=EPD_DRAW_EMPTY_LINE_QUEUE;
}

__attribute__((optimize("O3"))) static bool IRAM_ATTR
retrieve_line_isr(RenderContext_t* ctx, uint8_t* buf) {
    if (ctx->lines_consumed >= ctx->lines_total) {
        return false;
    }
    int thread = ctx->line_threads[ctx->lines_consumed];
    BaseType_t awoken = pdFALSE;

    // Queue commit is the authoritative release/acquire publication. A producer
    // can be preempted AFTER lq_commit but BEFORE publishing line_ready. That
    // separate flag is useful for prefill, but cannot reject a readable queue.
    if (thread < 0 || thread >= NUM_RENDER_THREADS) {
        record_underflow(ctx,1,thread);
        memset(buf, 0x00, ctx->display_width / 4);
        ctx->lines_consumed += 1;
        return awoken;
    }

    LineQueue_t* lq = &ctx->line_queues[thread];

    if (lq_read(lq, buf) != 0) {
        record_underflow(ctx,2,thread);
        memset(buf, 0x00, ctx->display_width / 4);
    } else if (xPortInIsrContext()
               && atomic_load_explicit(&ctx->feed_waiting[thread], memory_order_acquire)
               && atomic_exchange_explicit(&ctx->feed_waiting[thread], false, memory_order_acq_rel)) {
        // 任务侧启动仅取走首个 bounce；由首个 EOF 统一唤醒，避免逐行切换。
        xSemaphoreGiveFromISR(ctx->feed_space[thread], &awoken);
    }

    if (ctx->lines_consumed >= ctx->display_height) {
        memset(buf, 0x00, ctx->display_width / 4);
    }
    ctx->lines_consumed += 1;
    return awoken;
}

/// start the next frame in the current update cycle
static void IRAM_ATTR handle_lcd_frame_done(RenderContext_t* ctx) {
    epd_lcd_frame_done_cb(NULL, NULL);
    epd_lcd_line_source_cb(NULL, NULL);

    // 扫描先于供数结束时置错，禁止残留行进入下一相位
    if (ctx->lines_consumed < ctx->lines_total) {
        record_underflow(ctx,3,-1);
    }

    BaseType_t task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(ctx->frame_done, &task_awoken);

    portYIELD_FROM_ISR();
}

// 各相沿同一时间轴调度，单次迟到不推迟后续目标；资源未就绪时仍等待。
#ifndef EPD_PHASE_HZ
#define EPD_PHASE_HZ 85
#endif
static const int s_prefill_per_thread = 125;
static StaticSemaphore_t s_scan_lock_storage;
static SemaphoreHandle_t s_scan_lock;

// Serialize only task-side start/stop. Never hold an interrupt-disabled lock
// while starting the LCD or waiting for a feeder on the other core.
static void scan_lock(void) {
    if (xSemaphoreTake(s_scan_lock, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE("epd_lcd", "scan start/stop did not quiesce");
        abort();
    }
}

static void cancel_scan(RenderContext_t* ctx) {
    atomic_store_explicit(&ctx->feed_cancelled, true, memory_order_release);
    for (int i = 0; i < NUM_RENDER_THREADS; ++i) {
        xSemaphoreGive(ctx->feed_space[i]);
    }
    scan_lock();
    // 停机并等待旧 ISR 退出后，才可清理回调及其上下文。
    epd_lcd_abort_frame();
    epd_lcd_line_source_cb(NULL, NULL);
    epd_lcd_frame_done_cb(NULL, NULL);
    xSemaphoreGive(s_scan_lock);
}

static bool start_scan(RenderContext_t* ctx) {
    scan_lock();
    if (atomic_load_explicit(&ctx->feed_cancelled, memory_order_acquire)) {
        xSemaphoreGive(s_scan_lock);
        return false;
    }
    epd_lcd_line_source_cb((line_cb_func_t)&retrieve_line_isr, ctx);
    epd_lcd_start_frame();
    xSemaphoreGive(s_scan_lock);
    return true;
}

static int last_phase_average, last_phase_minimum, last_phase_maximum;
void epd_lcd_get_timing(int* average,int* minimum,int* maximum) {
    *average=__atomic_load_n(&last_phase_average,__ATOMIC_RELAXED);
    *minimum=__atomic_load_n(&last_phase_minimum,__ATOMIC_RELAXED);
    *maximum=__atomic_load_n(&last_phase_maximum,__ATOMIC_RELAXED);
}

static void pace_phase(int64_t deadline_us) {
    for (;;) {
        if (esp_timer_get_time() >= deadline_us) {
            break;
        }
        vTaskDelay(0);
    }
}

static void notify_feeders(RenderContext_t* ctx) {
    // 两份通知发布后才恢复本核调度，避免一方先填满队列。
    vTaskSuspendAll();
    xTaskNotifyGive(ctx->feed_tasks[!xPortGetCoreID()]);
    xTaskNotifyGive(ctx->feed_tasks[xPortGetCoreID()]);
    xTaskResumeAll();
}

static bool wait_feeders(RenderContext_t* ctx, unsigned* active) {
    for (int i = 0; i < NUM_RENDER_THREADS; ++i) {
        if (!(*active & (1u << i))) continue;
        if (xSemaphoreTake(ctx->feed_done_smphr[i], pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGE("epd_lcd", "feed %d timeout @frame %d", i, ctx->current_frame);
            ctx->error |= EPD_DRAW_EMPTY_LINE_QUEUE;
            return false;
        }
        *active &= ~(1u << i);
    }
    return true;
}

static bool repeats_previous_phase(const RenderContext_t* ctx, int phase) {
#if EPD_LCD_FULL_PHASE
    // A complete immutable phase remains in the queue after consumption. Exact
    // waveform equality permits replaying those bytes, including all masks and
    // cropping. Spatial wipes vary by phase even when the source LUT is equal.
    if (phase > 0 && !ctx->spatial_luts) {
        const EpdWaveformPhases* phases = ctx->waveform->mode_data[ctx->waveform_index]
            ->range_data[ctx->waveform_range];
        return phases && phases->luts && phase < phases->phases &&
            !memcmp(phases->luts + 64 * (phase - 1), phases->luts + 64 * phase, 64);
    }
#endif
    return false;
}

void lcd_do_update(RenderContext_t* ctx) {
    if (ctx->cycle_frames<=0) return;
    if (s_scan_lock == NULL) {
        s_scan_lock = xSemaphoreCreateMutexStatic(&s_scan_lock_storage);
        assert(s_scan_lock != NULL);
    }
    const int64_t cycle_start=esp_timer_get_time();
    uint32_t lut_us=0, feed_wait_us=0, start_us=0, frame_wait_us=0, pace_us=0;
    memset(ctx->feed_time_us,0,sizeof(ctx->feed_time_us));
    int64_t stage_at=esp_timer_get_time();
    int64_t phase_origin=0;
    int64_t previous_phase_start=0, period_total=0;
    int period_min=INT32_MAX, period_max=0, periods=0;
    int completed_frames=0;
    int64_t previous_spv_start=0,spv_total=0;
    int spv_periods=0;
    int64_t scan_total=0;
    int scan_min=INT32_MAX,scan_max=0,eof_gap_max=0,eof_work_max=0;
    unsigned active=0;
    bool replay=false;
    ctx->fault_reason=0;
    ctx->fault_phase=ctx->fault_line=ctx->fault_thread=0;
    ctx->scan_frame=0;
    memset(ctx->line_ready,0,ctx->lines_total);
    for (int row=0;row<ctx->lines_total;++row) {
        ctx->line_threads[row]=(uint8_t)(row%NUM_RENDER_THREADS);
#if defined(CONFIG_IDF_TARGET_ESP32S31)
        // The DMA/LCD IRQ consumes part of one core's time. Give the other
        // core six extra rows per 32 when preparing alongside the prior scan.
        // Hardware timings include the IRQ cost: 13:19 balances both feeders.
        if (row%32>=26) ctx->line_threads[row]=1-epd_lcd_get_irq_core();
#endif
    }
    atomic_store_explicit(&ctx->feed_cancelled,false,memory_order_release);
    epd_set_mode(1);
    ESP_LOGD("epd_lcd", "update %d frames, lines=%d (target=%dHz)", ctx->cycle_frames,
             ctx->lines_total, EPD_PHASE_HZ);

    prepare_context_for_next_frame(ctx);
    lut_us+=esp_timer_get_time()-stage_at;
    active=(1u<<NUM_RENDER_THREADS)-1;
    notify_feeders(ctx);
    for (int k=0;k<ctx->cycle_frames;++k) {
        stage_at=esp_timer_get_time();
        for (int i=0;!replay && i<NUM_RENDER_THREADS;++i) {
            if (xSemaphoreTake(ctx->prefill_done,pdMS_TO_TICKS(2000))!=pdTRUE) {
                ESP_LOGE("epd_lcd","prefill timeout @frame %d",k+1);
                ctx->error|=EPD_DRAW_EMPTY_LINE_QUEUE;
                goto failed;
            }
        }
        if (ctx->error) goto failed;
        feed_wait_us+=esp_timer_get_time()-stage_at;
        if (k==0) phase_origin=esp_timer_get_time();
        const int64_t deadline=phase_origin+
            ((int64_t)k*1000000+EPD_PHASE_HZ-1)/EPD_PHASE_HZ;
        stage_at=esp_timer_get_time();
        pace_phase(deadline);
        pace_us+=esp_timer_get_time()-stage_at;
        const int64_t phase_start=esp_timer_get_time();
        if (previous_phase_start) {
            const int period=(int)(phase_start-previous_phase_start);
            if (period<period_min) period_min=period;
            if (period>period_max) period_max=period;
            period_total+=period; ++periods;
        }
        previous_phase_start=phase_start;
        ctx->scan_frame=k;
        ctx->lines_consumed=0;
        int phase_start_row[NUM_RENDER_THREADS];
        for(int i=0;i<NUM_RENDER_THREADS;++i)
            phase_start_row[i]=atomic_load_explicit(&ctx->line_queues[i].last,memory_order_acquire);
        epd_lcd_frame_done_cb((frame_done_func_t)handle_lcd_frame_done,ctx);
        stage_at=esp_timer_get_time();
        if (!start_scan(ctx)) goto failed;
        start_us+=esp_timer_get_time()-stage_at;

        // 供数结束后，ISR 仅持有队列数据；尾部扫描时可复用 LUT 准备下一相。
        stage_at=esp_timer_get_time();
        if (!wait_feeders(ctx,&active)) goto failed;
        feed_wait_us+=esp_timer_get_time()-stage_at;
        if (ctx->error) goto failed;
        const bool replay_next=k+1<ctx->cycle_frames && repeats_previous_phase(ctx,k+1);
        if (k+1<ctx->cycle_frames) {
            ctx->current_frame=k+1;
            stage_at=esp_timer_get_time();
            prepare_context_for_next_frame(ctx);
            lut_us+=esp_timer_get_time()-stage_at;
            if(!replay_next) {
                active=(1u<<NUM_RENDER_THREADS)-1;
                notify_feeders(ctx);
            }
        }

        stage_at=esp_timer_get_time();
        if (xSemaphoreTake(ctx->frame_done,pdMS_TO_TICKS(2000))!=pdTRUE) {
            ESP_LOGE("epd_lcd","LCD frame %d/%d timeout err=0x%x consumed=%d/%d",
                k+1,ctx->cycle_frames,ctx->error,ctx->lines_consumed,ctx->lines_total);
            ctx->error|=EPD_DRAW_EMPTY_LINE_QUEUE;
            goto failed;
        }
        if (ctx->error) goto failed;

        if(replay_next) {
            // No producer was started. Only rewind after the prior ISR has
            // completed; the immutable rows then become the next full prefill.
            for(int i=0;i<NUM_RENDER_THREADS;++i)
                atomic_store_explicit(&ctx->line_queues[i].last,phase_start_row[i],memory_order_release);
            for(int row=0;row<ctx->lines_total;++row)
                __atomic_store_n(&ctx->line_ready[row],k+2,__ATOMIC_RELEASE);
        }
        replay=replay_next;

        frame_wait_us+=esp_timer_get_time()-stage_at;
        ++completed_frames;
        const int64_t spv_start=epd_lcd_get_spv_start_us();
        if (previous_spv_start) {
            spv_total+=spv_start-previous_spv_start;
            ++spv_periods;
        }
        previous_spv_start=spv_start;
        const int scan_us=epd_lcd_get_scan_time_us();
        scan_total+=scan_us;
        if (scan_us<scan_min) scan_min=scan_us;
        if (scan_us>scan_max) scan_max=scan_us;
        int eof_gap=0,eof_work=0;
        epd_lcd_get_stream_stats(&eof_gap,&eof_work);
        if (eof_gap>eof_gap_max) eof_gap_max=eof_gap;
        if (eof_work>eof_work_max) eof_work_max=eof_work;
    }
    goto finished;

failed:
    cancel_scan(ctx);
    // 下一相可能已在尾部扫描时启动；全部确认退出后才能复用队列和 LUT。
    if (!wait_feeders(ctx,&active)) {
        ESP_LOGE("epd_lcd","feed failed to stop; retaining live buffers");
        abort();
    }
    xSemaphoreTake(ctx->frame_done,0);
    if(ctx->fault_reason) {
        int eof_gap=0,eof_work=0;
        epd_lcd_get_stream_stats(&eof_gap,&eof_work);
        ESP_LOGE("epd_lcd","stream fault reason=%d phase=%d/%d line=%d/%d owner=%d ready=%d q0=%d/%d q1=%d/%d prefill=%d eof_gap=%dus eof_work=%dus",
            ctx->fault_reason,ctx->fault_phase,ctx->cycle_frames,ctx->fault_line,ctx->lines_total,
            ctx->fault_thread,ctx->fault_ready,ctx->fault_write[0],ctx->fault_read[0],
            ctx->fault_write[1],ctx->fault_read[1],EPD_LCD_FULL_PHASE ? ctx->lines_total : s_prefill_per_thread*NUM_RENDER_THREADS,eof_gap,eof_work);
    }
    for (int i=0;i<NUM_RENDER_THREADS;++i) lq_reset(&ctx->line_queues[i]);

finished:
    while (xSemaphoreTake(ctx->prefill_done,0)==pdTRUE) {}
    for (int i=0;i<NUM_RENDER_THREADS;++i) {
        atomic_store_explicit(&ctx->feed_waiting[i],false,memory_order_release);
        xSemaphoreTake(ctx->feed_space[i],0);
    }
    epd_lcd_line_source_cb(NULL, NULL);
    epd_lcd_frame_done_cb(NULL, NULL);

    EpdMetrics sample={0};
    sample.cycle_us=esp_timer_get_time()-cycle_start; sample.scan_us=scan_total;
    sample.start_us=start_us; sample.frame_wait_us=frame_wait_us; sample.pace_us=pace_us;
    sample.feed0_us=ctx->feed_time_us[0];sample.feed1_us=ctx->feed_time_us[1];
    sample.lut_us=lut_us; sample.feed_wait_us=feed_wait_us;
    sample.phases=ctx->cycle_frames; sample.completed=completed_frames;
    sample.fault_phase=ctx->fault_phase;
    sample.fault_line=ctx->fault_line;
    sample.fault_thread=ctx->fault_thread;
    sample.faults=ctx->error ? 1 : 0; sample.pixel_clock_hz=epd_lcd_get_pixel_clock_hz();
    sample.phase_avg_us=periods ? period_total/periods : 0;
    sample.phase_min_us=periods ? period_min : 0; sample.phase_max_us=period_max;
    sample.eof_gap_us=eof_gap_max; sample.eof_work_us=eof_work_max;
    epd_metrics_add(&sample);
    epd_set_mode(0);
    if (spv_periods>0) {
        const int millihz=(int)(1000000000LL*spv_periods/spv_total);
        ESP_LOGI("EpdTiming","SPV=%d.%03dHz period_us=%d samples=%d completed=%d",
            millihz/1000,millihz%1000,(int)(spv_total/spv_periods),spv_periods,completed_frames);
    }
    if (periods) {
        __atomic_store_n(&last_phase_average,(int)(period_total/periods),__ATOMIC_RELAXED);
        __atomic_store_n(&last_phase_minimum,period_min,__ATOMIC_RELAXED);
        __atomic_store_n(&last_phase_maximum,period_max,__ATOMIC_RELAXED);
        ESP_LOGI("EpdTiming","phase cadence us: avg=%d min=%d max=%d frames=%d completed=%d cycle=%dms "
        "scan_us=%d/%d/%d eof_gap_max=%d eof_work_max=%d",
        (int)(period_total/periods),period_min,period_max,ctx->cycle_frames,
        completed_frames,(int)((esp_timer_get_time()-cycle_start)/1000),
        completed_frames ? (int)(scan_total/completed_frames) : 0,completed_frames ? scan_min : 0,scan_max,
        eof_gap_max,eof_work_max);
    }
}

__attribute__((optimize("O3"))) static bool IRAM_ATTR
push_pixels_isr(RenderContext_t* ctx, uint8_t* buf) {
    // Output no-op outside of drawn area
    if (ctx->lines_consumed < ctx->area.y) {
        memset(buf, 0, ctx->display_width / 4);
    } else if (ctx->lines_consumed >= ctx->area.y + ctx->area.height) {
        memset(buf, 0, ctx->display_width / 4);
    } else {
        memcpy(buf, ctx->static_line_buffer, ctx->display_width / 4);
    }
    ctx->lines_consumed += 1;
    return pdFALSE;
}

/**
 * Populate the line mask for use in epd_push_pixels.
 */
static void push_pixels_populate_line(RenderContext_t* ctx, int color) {
    // Select fill pattern by draw color
    int fill_byte = 0;
    switch (color) {
        case 0:
            fill_byte = DARK_BYTE;
            break;
        case 1:
            fill_byte = CLEAR_BYTE;
            break;
        default:
            fill_byte = 0x00;
    }

    // Compute a line mask based on the drawn area
    uint8_t* dirtyness = malloc(ctx->display_width / 2);
    assert(dirtyness != NULL);

    memset(dirtyness, 0, ctx->display_width / 2);

    for (int i = 0; i < ctx->display_width; i++) {
        if ((i >= ctx->area.x) && (i < ctx->area.x + ctx->area.width)) {
            dirtyness[i / 2] |= i % 2 ? 0xF0 : 0x0F;
        }
    }
    epd_populate_line_mask(ctx->line_mask, dirtyness, ctx->display_width / 4);

    // mask the line pattern with the populated mask
    memset(ctx->static_line_buffer, fill_byte, ctx->display_width / 4);
    epd_apply_line_mask(ctx->static_line_buffer, ctx->line_mask, ctx->display_width / 4);

    free(dirtyness);
}

void epd_push_pixels_lcd(RenderContext_t* ctx, short time, int color) {
    xSemaphoreTake(ctx->frame_done, 0);
    ctx->fault_reason = 0;
    ctx->current_frame = 0;
    ctx->scan_frame = 0;
    ctx->lines_total = ctx->display_height;
    ctx->lines_consumed = 0;
    assert(ctx->static_line_buffer != NULL);

    push_pixels_populate_line(ctx, color);
    epd_lcd_frame_done_cb((frame_done_func_t)handle_lcd_frame_done, ctx);
    epd_lcd_line_source_cb((line_cb_func_t)&push_pixels_isr, ctx);

    epd_set_mode(1);
    epd_lcd_start_frame();
    if (xSemaphoreTake(ctx->frame_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ctx->error |= EPD_DRAW_EMPTY_LINE_QUEUE;
        epd_lcd_abort_frame();
        epd_lcd_line_source_cb(NULL, NULL);
        epd_lcd_frame_done_cb(NULL, NULL);
        ESP_LOGE("epd_lcd", "clear scan timed out");
    }
    epd_set_mode(0);

    if (ctx->error) {
        int gap = 0, work = 0;
        epd_lcd_get_stream_stats(&gap, &work);
        ESP_LOGE("epd_lcd", "clear scan failed reason=%d line=%d/%d eof_gap=%dus eof_work=%dus",
                 ctx->fault_reason, ctx->lines_consumed, ctx->lines_total, gap, work);
        // Clear failures precede the waveform cycle; still expose them through
        // the same metrics instead of reporting zero faults after a failed boot.
        EpdMetrics sample = {0};
        sample.phases = sample.faults = sample.fault_phase = 1;
        sample.fault_line = ctx->lines_consumed;
        sample.fault_thread = (uint32_t)-1;
        sample.scan_us = epd_lcd_get_scan_time_us();
        sample.eof_gap_us = gap;
        sample.eof_work_us = work;
        sample.pixel_clock_hz = epd_lcd_get_pixel_clock_hz();
        epd_metrics_add(&sample);
    }
}

#define int_min(a, b) (((a) < (b)) ? (a) : (b))

static uint8_t* IRAM_ATTR wait_line_buffer(RenderContext_t* ctx,LineQueue_t* queue,int thread) {
    int64_t wait_start=0;
    for (;;) {
        if (atomic_load_explicit(&ctx->feed_cancelled,memory_order_acquire)
            || (ctx->error & EPD_DRAW_EMPTY_LINE_QUEUE)) {
            atomic_store_explicit(&ctx->feed_waiting[thread],false,memory_order_release);
            return NULL;
        }
        uint8_t* buffer=lq_current(queue);
        if (buffer!=NULL) return buffer;
        const int64_t now=esp_timer_get_time();
        if (!wait_start) wait_start=now;
        if (now-wait_start>=1000000) {
            ctx->error|=EPD_DRAW_EMPTY_LINE_QUEUE;
            return NULL;
        }
        // 先发布等待状态再复查，避免消费发生在检查与阻塞之间而丢失唤醒。
        atomic_store_explicit(&ctx->feed_waiting[thread],true,memory_order_release);
        buffer=lq_current(queue);
        if (buffer!=NULL) {
            atomic_store_explicit(&ctx->feed_waiting[thread],false,memory_order_release);
            return buffer;
        }
        if (!atomic_load_explicit(&ctx->feed_cancelled,memory_order_acquire)) {
            const int remaining_ms=(int)((1000000-(now-wait_start)+999)/1000);
            xSemaphoreTake(ctx->feed_space[thread],pdMS_TO_TICKS(remaining_ms)+1);
        }
        atomic_store_explicit(&ctx->feed_waiting[thread],false,memory_order_release);
    }
}

// A full-phase queue publishes only after every owned row is immutable. Radio
// interrupts may delay preparation but cannot starve a running panel scan.
__attribute__((optimize("O3"))) void IRAM_ATTR
lcd_calculate_frame(RenderContext_t* ctx, int thread_id) {
    assert(ctx->lut_lookup_func != NULL);

    LineQueue_t* lq = &ctx->line_queues[thread_id];
    int l = 0;

    // 另一生产者可能已启动并报告欠载；不得再次启动同一帧
    if (ctx->error) {
        return;
    }

    EpdRect area = ctx->area;
    int min_y, max_y, bytes_per_line, _ppB;
    const uint8_t* ptr_start;
    get_buffer_params(ctx, &bytes_per_line, &ptr_start, &min_y, &max_y, &_ppB);

    assert(area.width == ctx->display_width && area.x == 0 && !ctx->error);

#if EPD_LCD_FULL_PHASE
    const int prefill_rows = ctx->lines_total;
    assert(lq->size >= epd_lcd_phase_queue_capacity(ctx->lines_total));
#else
    const int prefill_rows = int_min(s_prefill_per_thread,lq->size-1);
#endif
    bool prefill_sent=false;
    int prefill_last=-1;
    for (int row=0,count=0;row<ctx->lines_total && count<prefill_rows;++row)
        if (ctx->line_threads[row]==thread_id) { prefill_last=row; ++count; }

    // Fixed ownership keeps current and upcoming phase rows in FIFO order.
    for (l=0; l<ctx->lines_total; ++l) {
        if (ctx->line_threads[l]!=thread_id) continue;
        if (atomic_load_explicit(&ctx->feed_cancelled, memory_order_acquire)) return;

        if (l < min_y || l >= max_y
            || (ctx->drawn_lines != NULL && !ctx->drawn_lines[l - area.y])) {
            uint8_t* buf=wait_line_buffer(ctx,lq,thread_id);
            if (buf==NULL) return;
            memset(buf, 0x00, lq->element_size);
            lq_commit(lq);
            __atomic_store_n(&ctx->line_ready[l],ctx->current_frame+1,__ATOMIC_RELEASE);
            if (!prefill_sent && l>=prefill_last) {
                prefill_sent=true;
                xSemaphoreGive(ctx->prefill_done);
            }
            continue;
        }

        const uint8_t* ptr = ptr_start + bytes_per_line * (l - min_y);
#if !CONFIG_IDF_TARGET_ESP32S31
        Cache_Start_DCache_Preload((uint32_t)ptr, ctx->display_width, 0);
#endif
        uint32_t* lp = (uint32_t*)ptr;

        uint8_t* buf=wait_line_buffer(ctx,lq,thread_id);
        if (buf==NULL) return;

        if(ctx->spatial_luts) epd_spatial_line((const uint8_t*)lp,buf,ctx->display_width,
            ctx->spatial_luts,ctx->spatial_bands,ctx->spatial_by_y,l);
        else ctx->lut_lookup_func(lp, buf, ctx->conversion_lut, ctx->display_width);
        if (ctx->line_mask_active) {
            epd_apply_line_mask_VE(buf, ctx->line_mask, ctx->display_width / 4);
        }
        lq_commit(lq);
        __atomic_store_n(&ctx->line_ready[l],ctx->current_frame+1,__ATOMIC_RELEASE);
        if (!prefill_sent && l>=prefill_last) {
            prefill_sent=true;
            xSemaphoreGive(ctx->prefill_done);
        }
    }
    if (!prefill_sent) xSemaphoreGive(ctx->prefill_done);
}

#endif
