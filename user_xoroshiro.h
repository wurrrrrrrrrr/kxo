#pragma once

#include <stdint.h>
#include <stdlib.h>

struct state_array {
    uint64_t array[2];
};

uint64_t xoro_next(struct state_array *obj);
void xoro_jump(struct state_array *obj);
void xoro_init(struct state_array *obj);
