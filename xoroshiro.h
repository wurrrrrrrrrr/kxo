#pragma once

#include <linux/slab.h>

struct state_array {
    u64 array[2];
};

u64 xoro_next(struct state_array *obj);
void xoro_jump(struct state_array *obj);
void xoro_init(struct state_array *obj);
