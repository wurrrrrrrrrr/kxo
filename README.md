# KMLdrv: A device driver that performs tic-tac-toe game between kernel threads
## Introduction
`kmldrv` is a simple Linux kernel module which includes the concept of deferred
work and kernel concepts as following:
  - circular buffer
  - mutex lock
  - irq
  - softirq
  - tasklet
  - workqueue
  - kernel thread

It can use different machine-learning algorithm and perform tic-tac-toe games between kernel threads. The demo video are in the link below:
 - [Kernel space tic-tac-toe game](https://www.youtube.com/watch?v=Y_xdLrDVGzk)

## Installation & Usage
You can download the source code via the following command
```
$ git clone https://github.com/vax-r/KMLdrv.git
```
After the source code is downloaded, go into the directory and do as the following
```
$ make
make -C /lib/modules/6.5.0-28-generic/build M=/home/vax-r/linux2024/KMLdrv modules
make[1]: Entering directory '/usr/src/linux-headers-6.5.0-28-generic'
warning: the compiler differs from the one used to build the kernel
  The kernel was built by: x86_64-linux-gnu-gcc-12 (Ubuntu 12.3.0-1ubuntu1~22.04) 12.3.0
  You are using:           gcc-12 (Ubuntu 12.3.0-1ubuntu1~22.04) 12.3.0
  CC [M]  /home/vax-r/linux2024/KMLdrv/simrupt.o
  CC [M]  /home/vax-r/linux2024/KMLdrv/game.o
  CC [M]  /home/vax-r/linux2024/KMLdrv/wyhash.o
  CC [M]  /home/vax-r/linux2024/KMLdrv/xoroshiro.o
  CC [M]  /home/vax-r/linux2024/KMLdrv/mcts.o
  CC [M]  /home/vax-r/linux2024/KMLdrv/negamax.o
  CC [M]  /home/vax-r/linux2024/KMLdrv/zobrist.o
  LD [M]  /home/vax-r/linux2024/KMLdrv/kmldrv.o
  MODPOST /home/vax-r/linux2024/KMLdrv/Module.symvers
  CC [M]  /home/vax-r/linux2024/KMLdrv/kmldrv.mod.o
  LD [M]  /home/vax-r/linux2024/KMLdrv/kmldrv.ko
  BTF [M] /home/vax-r/linux2024/KMLdrv/kmldrv.ko
make[1]: Leaving directory '/usr/src/linux-headers-6.5.0-28-generic'
cc -std=gnu99 -Wno-declaration-after-statement -o kmldrv-user kmldrv-user.c
```
Make sure the kernel object file is compiled correctly, then you can insert the kernel module
```
$ sudo insmod kmldrv.ko
```
Now you can enjoy the tic-tac-toe games performed between kernel threads through the following command
```
$ sudo ./kmldrv-user
```
Enjoy the show !

## Features
### User space tool `kmldrv-user`
`kmldrv` provide a interface for userspace program to interact with it, for example you can use the userspace tool `kmldrv-user`. It has the following ability
- Display the status of `kmldrv`, to show whether its loaded or not
- `Ctrl + P` : Stop/Resume the displaying of chess board
- `Ctrl + Q` : Stop the tic-tac-toe games in kernel space
### Machine Learning Algorithms
Currently `kmldrv` supports two machine learning algorithms
- Monte-Carlo Tree Search
- Negamax AI Algorithm
### PRNG support
Currently `kmldrv` utilize two different PRNG (Pseudo-Random Number Generator) to generate random number
- `xoroshift`
- `wyhash`

## License

`simrupt` is released under the MIT license. Use of this source code is governed
by a MIT-style license that can be found in the LICENSE file.
