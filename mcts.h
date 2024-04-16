#pragma once

#define ITERATIONS 100000
#define EXPLORATION_FACTOR fixed_sqrt(1U << (FIXED_SCALE_BITS + 1))

int mcts(char *table, char player);