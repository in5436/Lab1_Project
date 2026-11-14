#include "memory_probe.h"
#include "stm32c0xx.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define STACK_PATTERN 0xDEADBEEFU
#define STACK_HEADROOM_BYTES 128U

extern uint8_t _estack;
extern uint8_t _sstack;
extern uint8_t _end;
extern uint8_t _Min_Heap_Size;
extern void *_sbrk(ptrdiff_t incr);

static uintptr_t s_paint_end = 0U;

void memory_probe_init(void)
{
    const uintptr_t bottom = (uintptr_t)&_sstack;
    const uintptr_t top = (uintptr_t)&_estack;
    const uint32_t primask = __get_PRIMASK();
    uintptr_t limit;
    uintptr_t p;

    __disable_irq();

    p = (uintptr_t)__get_MSP();

    limit = p > STACK_HEADROOM_BYTES
        ? ((p - STACK_HEADROOM_BYTES) & ~(uintptr_t)3U)
        : 0U;

    if (limit > top) {
        limit = top;
    }

    if ((bottom < top) && (limit > bottom)) {
        for (p = bottom; p < limit; p += 4U) {
            *(volatile uint32_t *)p = STACK_PATTERN;
        }

        s_paint_end = limit;
    }

    __set_PRIMASK(primask);
}

void memory_probe_print(void)
{
    const uintptr_t bottom = (uintptr_t)&_sstack;
    const uintptr_t top = (uintptr_t)&_estack;
    const uintptr_t heap_base = (uintptr_t)&_end;
    const uintptr_t heap_break = (uintptr_t)_sbrk(0);

    const uint32_t primask = __get_PRIMASK();

    uintptr_t p = bottom;
    const char *state;

    __disable_irq();

    if (s_paint_end > bottom && s_paint_end <= top) {
        while (p < s_paint_end &&
               *(volatile const uint32_t *)p == STACK_PATTERN) {
            p += 4U;
        }
    }

    __set_PRIMASK(primask);

    if (s_paint_end <= bottom || s_paint_end > top) {
        printf("[MEM] stack_paint=UNAVAILABLE\r\n");
    } else {
        state = (p == bottom) ? "AT_LIMIT" :
                (p == s_paint_end) ? "BELOW_FLOOR" :
                "MEASURED";

        printf("[MEM] stack_reserved=%lu depth_est=%lu spare=%lu state=%s\r\n",
               (unsigned long)(top - bottom),
               (unsigned long)(top - p),
               (unsigned long)(p - bottom),
               state);
    }

    if (heap_break >= heap_base && heap_break <= bottom) {
        printf("[MEM] heap_brk_used=%lu heap_room=%lu heap_min_reserve=%lu\r\n",
               (unsigned long)(heap_break - heap_base),
               (unsigned long)(bottom - heap_break),
               (unsigned long)(uintptr_t)&_Min_Heap_Size);
    } else {
        printf("[MEM] heap_state=INVALID\r\n");
    }
}