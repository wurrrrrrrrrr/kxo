TARGET = kmldrv
kmldrv-objs = simrupt.o game.o wyhash.o xoroshiro.o mcts.o negamax.o zobrist.o
obj-m := $(TARGET).o

ccflags-y := -std=gnu99 -Wno-declaration-after-statement
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

GIT_HOOKS := .git/hooks/applied
all: kmod kmldrv-user

kmod: $(GIT_HOOKS) simrupt.c
	$(MAKE) -C $(KDIR) M=$(PWD) modules

kmldrv-user: kmldrv-user.c
	$(CC) $(ccflags-y) -o $@ $<

$(GIT_HOOKS):
	@scripts/install-git-hooks
	@echo


clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(RM) kmldrv-user