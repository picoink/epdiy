#include <assert.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "line_queue.h"
#include "render_method.h"

// Product adapters may account these boot-critical buffers in their ledger.
__attribute__((weak)) void* epd_allocate_line_queue(size_t bytes) {
    return heap_caps_aligned_alloc(16,bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
}
__attribute__((weak)) void epd_release_line_queue(void* pointer) {heap_caps_free(pointer);}

static inline int ceil_div(int x, int y) {
    return x / y + (x % y != 0);
}

/// Initialize the line queue and allocate memory.
LineQueue_t lq_init(int queue_len, int element_size) {
    LineQueue_t queue;
    queue.element_size = element_size;
    queue.size = queue_len;
    queue.storage = NULL;
    atomic_init(&queue.current, 0);
    atomic_init(&queue.last, 0);

    int elem_buf_size = ceil_div(element_size, 16) * 16;

    queue.bufs = NULL;

    size_t line_q_bytes = 0;
#if EPD_LINE_QUEUE_PSRAM
    // The LCD ISR copies rows into its own internal DMA buffer. These queues
    // are CPU sources, never DMA sources. The integrator must gate every flash
    // cache-disable operation until the complete display scan has finished.
    queue.storage=epd_allocate_line_queue((size_t)elem_buf_size*queue.size);
    assert(queue.storage != NULL);
#else
    queue.bufs = calloc(queue.size, sizeof(uint8_t*));
    assert(queue.bufs != NULL);
#endif
    for (int i = 0; i < queue.size; i++) {
#if !EPD_LINE_QUEUE_PSRAM
        queue.bufs[i] = heap_caps_aligned_alloc(16, elem_buf_size, MALLOC_CAP_INTERNAL);
        assert(queue.bufs[i] != NULL);
#endif
        line_q_bytes += (size_t)elem_buf_size;
    }
    ESP_LOGD("epd_dram", "line_q elems=%d×%d=%u B", queue.size, elem_buf_size,
             (unsigned)line_q_bytes);

    return queue;
}

static inline uint8_t* IRAM_ATTR line_at(LineQueue_t* queue, int index) {
    // Contiguous queues need no per-row pointer table in DMA-capable SRAM.
    // The arithmetic also avoids an extra memory load in the scan ISR.
    return queue->storage ? queue->storage + ((queue->element_size + 15) & ~(size_t)15) * index
                          : queue->bufs[index];
}

/// Deinitialize the line queue and free memory.
void lq_free(LineQueue_t* queue) {
    if(queue->storage)epd_release_line_queue(queue->storage);
    else for (int i = 0; i < queue->size; i++) {
        heap_caps_free(queue->bufs[i]);
    }

    free(queue->bufs);
}

uint8_t* IRAM_ATTR lq_current(LineQueue_t* queue) {
    int current = atomic_load_explicit(&queue->current, memory_order_relaxed);
    int last = atomic_load_explicit(&queue->last, memory_order_acquire);

    if ((current + 1) % queue->size == last) {
        return NULL;
    }
    return line_at(queue,current);
}

void IRAM_ATTR lq_commit(LineQueue_t* queue) {
    int current = atomic_load_explicit(&queue->current, memory_order_relaxed);
    int next = (current + 1) % queue->size;
    atomic_store_explicit(&queue->current, next, memory_order_release);
}

int IRAM_ATTR lq_read(LineQueue_t* queue, uint8_t* dst) {
    int current = atomic_load_explicit(&queue->current, memory_order_acquire);
    int last = atomic_load_explicit(&queue->last, memory_order_relaxed);

    if (current == last) {
        return -1;
    }

    memcpy(dst, line_at(queue,last), queue->element_size);

    int next = (last + 1) % queue->size;
    atomic_store_explicit(&queue->last, next, memory_order_release);
    return 0;
}

void IRAM_ATTR lq_reset(LineQueue_t* queue) {
    atomic_store_explicit(&queue->current, 0, memory_order_relaxed);
    atomic_store_explicit(&queue->last, 0, memory_order_relaxed);
}
