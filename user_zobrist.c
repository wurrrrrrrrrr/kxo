#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "user_zobrist.h"

u64 zobrist_table[N_GRIDS][2];

#define HASH(key) ((key) % HASH_TABLE_SIZE)

static zobrist_entry_t *hash_table[HASH_TABLE_SIZE] = {0};

/* 64-bit hash function（簡化版 wyhash stateless） */
static inline u64 wyhash64_stateless(u64 *seed)
{
    *seed += 0x60bee2bee120fc15ULL;
    __uint128_t tmp;
    tmp = (__uint128_t) (*seed) * 0xa3b195354a39b70dULL;
    u64 m1 = (u64) (tmp >> 64) ^ (u64) tmp;
    tmp = (__uint128_t) m1 * 0x1b03738712fad5c9ULL;
    u64 m2 = (u64) (tmp >> 64) ^ (u64) tmp;
    return m2;
}

static u64 wyhash64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    u64 seed = (u64) (ts.tv_nsec ^ ts.tv_sec);
    return wyhash64_stateless(&seed);
}

void zobrist_init(void)
{
    for (int i = 0; i < N_GRIDS; i++) {
        zobrist_table[i][0] = wyhash64();
        zobrist_table[i][1] = wyhash64();
    }
    // hash_table[] 已全為 NULL，不需要 malloc 初始化
}

zobrist_entry_t *zobrist_get(u64 key)
{
    u64 hash_key = HASH(key);
    zobrist_entry_t *entry = hash_table[hash_key];
    while (entry) {
        if (entry->key == key)
            return entry;
        entry = entry->next;
    }
    return NULL;
}

void zobrist_put(u64 key, int score, int move)
{
    u64 hash_key = HASH(key);
    zobrist_entry_t *entry = malloc(sizeof(zobrist_entry_t));
    if (!entry) {
        fprintf(stderr, "zobrist_put: malloc failed\n");
        return;
    }
    entry->key = key;
    entry->score = score;
    entry->move = move;
    entry->next = hash_table[hash_key];
    hash_table[hash_key] = entry;
}

void zobrist_clear(void)
{
    for (int i = 0; i < HASH_TABLE_SIZE; i++) {
        zobrist_entry_t *entry = hash_table[i];
        while (entry) {
            zobrist_entry_t *next = entry->next;
            free(entry);
            entry = next;
        }
        hash_table[i] = NULL;
    }
}
