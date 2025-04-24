TARGET = kxo
kxo-objs = main.o game.o xoroshiro.o mcts.o negamax.o zobrist.o
obj-m := $(TARGET).o

ccflags-y := -std=gnu99 -Wno-declaration-after-statement
CFLAGS := -std=gnu99 -Wno-declaration-after-statement
CFLAGS += -I. -MMD
TRAIN = train
RL_CFLAGS := $(CFLAGS) -D USE_RL

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

GIT_HOOKS := .git/hooks/applied
all: kmod xo-user rl

kmod: $(GIT_HOOKS) main.c
	$(MAKE) -C $(KDIR) M=$(PWD) modules

USER_SRC := xo-user.c user_mcts.c user_negamax.c user_zobrist.c user_xoroshiro.c user_game.c reinforcement_learning.c
USER_BIN := xo-user


xo-user: $(USER_SRC)
	$(CC) $(CFLAGS) -o $(USER_BIN) $(USER_SRC)

rl: $(USER_SRC)
	$(CC) $(RL_CFLAGS) -o $@ $^

$(TRAIN): $(TRAIN).c reinforcement_learning.c user_game.c
	$(CC) $(CFLAGS) -o $@ $^

$(GIT_HOOKS):
	@scripts/install-git-hooks
	@echo


clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(RM) xo-user rl train *.o *.d
