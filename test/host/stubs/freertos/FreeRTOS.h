#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>
#include <stdatomic.h>
typedef int BaseType_t;
typedef struct { pthread_mutex_t lock; pthread_cond_t changed; unsigned count,max; } StaticSemaphore_t;
typedef StaticSemaphore_t* SemaphoreHandle_t;
typedef struct HostTask* TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)
#define portYIELD_FROM_ISR() ((void)0)
#define portMAX_DELAY UINT32_MAX
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t*);
SemaphoreHandle_t xSemaphoreCreateCounting(unsigned,unsigned);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
int xSemaphoreTake(SemaphoreHandle_t,unsigned);
void xSemaphoreGive(SemaphoreHandle_t);
void xSemaphoreGiveFromISR(SemaphoreHandle_t,BaseType_t*);
void vSemaphoreDelete(SemaphoreHandle_t);
void vTaskDelay(unsigned);
int xPortGetCoreID(void);
int xPortInIsrContext(void);
void vTaskSuspendAll(void);
void xTaskResumeAll(void);
void xTaskNotifyGive(TaskHandle_t);
