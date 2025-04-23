#pragma once

#include <stdint.h>
#include "user_game.h"

#define HASH_TABLE_SIZE 100003

typedef uint64_t u64;

typedef struct zobrist_entry {
    u64 key;
    int score;
    int move;
    struct zobrist_entry *next;  // ✅ 使用者空間使用 pointer linked list
} zobrist_entry_t;

extern u64 zobrist_table[N_GRIDS][2];

void zobrist_init(void);
zobrist_entry_t *zobrist_get(u64 key);
void zobrist_put(u64 key, int score, int move);
void zobrist_clear(void);
