#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "game.h"

#define KMLDRV_STATUS_FILE "/sys/module/kmldrv/initstate"
#define KMLDRV_DEVICE_FILE "/dev/kmldrv"
#define KMLDRV_DEVICE_ATTR_FILE "/sys/class/kmldrv/kmldrv/kmldrv_state"

bool kmldrv_status_check(void)
{
    FILE *fp = fopen(KMLDRV_STATUS_FILE, "r");
    if (!fp) {
        printf("kmldrv status : not loaded\n");
        return false;
    }

    char read_buf[20];
    fgets(read_buf, 20, fp);
    read_buf[strcspn(read_buf, "\n")] = 0;
    if (!strcmp("live", read_buf))
        printf("kmldrv status : live\n");
    else {
        printf("kmldrv status : %s\n", read_buf);
        fclose(fp);
        return false;
    }
    fclose(fp);
    return true;
}

static struct termios orig_termios;

static void disableRawMode(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

static void enableRawMode(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

int main(int argc, char *argv[])
{
    int c;

    while ((c = getopt(argc, argv, "d:s:ch")) != -1) {
        switch (c) {
        case 'h':
            printf(
                "kmldrv-user : A userspace tool which supports interactions "
                "with kmldrv from user-level\n");
            printf("Usage:\n\n");
            printf("\t./kmldrv-user [arguments]\n\n");
            printf("Arguments:\n\n");
            printf("\t--start - start a tic-tac-toe game\n");
            printf("\t--release - release kmldrv\n\n");
            printf("Control Options:\n\n");
            printf("\t Ctrl + S - Start a tic-tac-toe game\n");
            printf("\t Ctrl + P - Pause to show the game\n");
            printf("\t Ctrl + C - Continue to show the game\n");
            printf("\t Ctrl + R - Restart a tic-tac-toe game\n");
            return 0;
        default:
            printf("Invalid arguments\n");
            break;
        }
    }

    if (!kmldrv_status_check())
        exit(1);

    enableRawMode();
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    FILE *device_ptr = fopen(KMLDRV_DEVICE_FILE, "r");
    char display_buf[DRAWBUFFER_SIZE];

    int attr_fd = open(KMLDRV_DEVICE_ATTR_FILE, O_WRONLY);
    char input;
    while (fgets(display_buf, DRAWBUFFER_SIZE, device_ptr)) {
        printf("%s", display_buf);

        if (read(STDIN_FILENO, &input, 1) == 1) {
            switch (input) {
            case 16:
                char buf[20];
                read(attr_fd, buf, 5);
                buf[0] = (buf[0] - '0') ? '0' : '1';
                write(attr_fd, buf, 5);
                break;
            case 17:
                break;
            }
        }
    }

    disableRawMode();
    fcntl(STDIN_FILENO, F_SETFL, flags);

    fclose(device_ptr);
    close(attr_fd);

    return 0;
}