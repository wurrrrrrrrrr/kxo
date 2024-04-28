#pragma once

#include "xoroshiro.h"

#define ITERATIONS 100000
#define EXPLORATION_FACTOR fixed_sqrt(1U << (FIXED_SCALE_BITS + 1))

struct mcts_info {
    struct state_array xoro_obj;
    int nr_active_nodes;
};

unsigned long count_active_nodes(void);
int mcts(char *table, char player);
void mcts_init(void);