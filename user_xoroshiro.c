#include "user_xoroshiro.h"
#include <stdint.h>

static inline uint64_t rotl(const uint64_t x, int k)
{
    return (x << k) | (x >> (64 - k));
}

static inline void seed(struct state_array *obj, uint64_t s0, uint64_t s1)
{
    obj->array[0] = s0;
    obj->array[1] = s1;
}

uint64_t xoro_next(struct state_array *obj)
{
    const uint64_t s0 = obj->array[0];
    uint64_t s1 = obj->array[1];
    const uint64_t result = rotl(s0 + s1, 24) + s0;

    s1 ^= s0;
    obj->array[0] = rotl(s0, 24) ^ s1 ^ (s1 << 16);
    obj->array[1] = rotl(s1, 37);

    return result;
}

void xoro_jump(struct state_array *obj)
{
    static const uint64_t JUMP[] = {0xdf900294d8f554a5ULL,
                                    0x170865df4b3201fcULL};

    uint64_t s0 = 0;
    uint64_t s1 = 0;
    for (int i = 0; i < (int) (sizeof(JUMP) / sizeof(*JUMP)); i++) {
        for (int b = 0; b < 64; b++) {
            if (JUMP[i] & ((uint64_t) 1 << b)) {
                s0 ^= obj->array[0];
                s1 ^= obj->array[1];
            }
            xoro_next(obj);
        }
    }

    obj->array[0] = s0;
    obj->array[1] = s1;
}

void xoro_init(struct state_array *obj)
{
    seed(obj, 314159265ULL, 1618033989ULL);
}
