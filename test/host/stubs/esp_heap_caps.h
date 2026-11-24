#pragma once
#include <stdlib.h>
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_SPIRAM 2
#define MALLOC_CAP_8BIT 4
static inline void* heap_caps_aligned_alloc(size_t a,size_t n,unsigned caps) { void* p=0; return posix_memalign(&p,a,n) ? 0 : p; }
#define heap_caps_free free
