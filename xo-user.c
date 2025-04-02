#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include "game.h"

#define XO_STATUS_FILE "/sys/module/kxo/initstate"
#define XO_DEVICE_FILE "/dev/kxo"
#define XO_DEVICE_ATTR_FILE "/sys/class/kxo/kxo/kxo_state"

#define FLAG_DISPLAY (1 << 0)
#define FLAG_RESUME (1 << 1)
#define FLAG_END (1 << 2)

typedef struct {
    unsigned long long moves[17];
} kxo_move_seq_t;

#define MY_IOCTL_GET_DATA _IOR('m', 1, kxo_move_seq_t)
static bool status_check(void)
{
    FILE *fp = fopen(XO_STATUS_FILE, "r");
    if (!fp) {
        printf("kxo status : not loaded\n");
        return false;
    }

    char read_buf[20];
    fgets(read_buf, 20, fp);
    read_buf[strcspn(read_buf, "\n")] = 0;
    if (strcmp("live", read_buf)) {
        printf("kxo status : %s\n", read_buf);
        fclose(fp);
        return false;
    }
    fclose(fp);
    return true;
}

static struct termios orig_termios;

static void raw_mode_disable(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

static void raw_mode_enable(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(raw_mode_disable);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void print_moves(const unsigned long long move_seq[17])
{
    const char *labels[16] = {"A1", "B1", "C1", "D1", "A2", "B2", "C2", "D2",
                              "A3", "B3", "C3", "D3", "A4", "B4", "C4", "D4"};
    unsigned long long step_move = move_seq[16];
    uint8_t count = 15;
    uint8_t moves[16];
    bool first = true;
    for (uint8_t i = 0; i < 16; i++) {
        moves[i] = (step_move >> (i * 4)) & 0xF;
        if ((first & moves[i]) == 0) {
            first = false;
            count = i - 1;
        }
    }
    for (int g = 0; g < 16; g++) {
        unsigned long long game = move_seq[g];
        if (game == 0)
            continue;
        printf("Game %d: Moves: ", g);
        first = true;

        for (int i = moves[count] - 1; i >= 0; i--) {
            uint8_t move = (game >> (i * 4)) & 0xF;

            if (!first)
                printf(" -> ");

            printf("%s", labels[move]);
            first = false;
        }
        count = count - 1;
        printf("\n");
    }
}


static bool read_attr, end_attr;

static void listen_keyboard_handler(int device_fd)
{
    kxo_move_seq_t buffer;
    int attr_fd = open(XO_DEVICE_ATTR_FILE, O_RDWR);
    char input;

    if (read(STDIN_FILENO, &input, 1) == 1) {
        char buf[1];
        switch (input) {
        case 16: /* Ctrl-P */
            read(attr_fd, buf, 1);
            buf[0] = (buf[0] ^= FLAG_DISPLAY) ? (buf[0] & ~FLAG_DISPLAY)
                                              : (buf[0] ^ FLAG_DISPLAY);
            read_attr ^= 1;
            write(attr_fd, buf, 1);
            if (!read_attr)
                printf("Stopping to display the chess board...\n");
            break;
        case 17: /* Ctrl-Q */
            read(attr_fd, buf, 1);
            buf[0] = buf[0] ^ FLAG_END;
            read_attr = false;
            end_attr = true;
            write(attr_fd, buf, 1);
            printf("Stopping the kernel space tic-tac-toe game...\n");
            if (ioctl(device_fd, MY_IOCTL_GET_DATA, &buffer) < 0) {
                perror("ioctl");
            }
            print_moves(buffer.moves);
            break;
        }
    }
    close(attr_fd);
}

int main(int argc, char *argv[])
{
    if (!status_check())
        exit(1);

    raw_mode_enable();
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    char display_buf[DRAWBUFFER_SIZE];

    fd_set readset;
    int device_fd = open(XO_DEVICE_FILE, O_RDONLY);
    int max_fd = device_fd > STDIN_FILENO ? device_fd : STDIN_FILENO;
    read_attr = true;
    end_attr = false;

    while (!end_attr) {
        FD_ZERO(&readset);
        FD_SET(STDIN_FILENO, &readset);
        FD_SET(device_fd, &readset);

        int result = select(max_fd + 1, &readset, NULL, NULL, NULL);
        if (result < 0) {
            printf("Error with select system call\n");
            exit(1);
        }

        if (FD_ISSET(STDIN_FILENO, &readset)) {
            FD_CLR(STDIN_FILENO, &readset);
            listen_keyboard_handler(device_fd);
        } else if (read_attr && FD_ISSET(device_fd, &readset)) {
            FD_CLR(device_fd, &readset);
            printf("\033[H\033[J"); /* ASCII escape code to clear the screen */
            read(device_fd, display_buf, DRAWBUFFER_SIZE);
            printf("%s", display_buf);
        }
    }

    raw_mode_disable();
    fcntl(STDIN_FILENO, F_SETFL, flags);

    close(device_fd);

    return 0;
}
