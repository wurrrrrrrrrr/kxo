#include <fcntl.h>
#include <getopt.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "list.h"
#include "user_game.h"
#include "user_mcts.h"
#include "user_negamax.h"

#ifdef USE_RL
#include "reinforcement_learning.h"

rl_agent_t agent;
unsigned int state_num;
#endif


#define XO_STATUS_FILE "/sys/module/kxo/initstate"
#define XO_DEVICE_FILE "/dev/kxo"
#define XO_DEVICE_ATTR_FILE "/sys/class/kxo/kxo/kxo_state"

#define FLAG_DISPLAY (1 << 0)
#define FLAG_RESUME (1 << 1)
#define FLAG_END (1 << 2)

typedef struct {
    unsigned long long moves[17];
} kxo_move_seq_t;

static char table[N_GRIDS];
static unsigned long long user_move_seq[17];
static unsigned long long user_tmp_move;
static char user_tmp_move_step;
static unsigned long long user_move_step;
static unsigned int user_move_index;
static int user_head = -1;
char win;

time_t current_time;
struct tm *time_info;

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
    raw.c_iflag &= ~IXON;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}
static char draw_buffer[DRAWBUFFER_SIZE];
/* Draw the board into draw_buffer */
static int draw_board(const unsigned int *btable)
{
    int i = 0, k = 0;
    draw_buffer[i++] = '\n';
    draw_buffer[i++] = '\n';

    for (int row = 0; row < BOARD_SIZE; row++) {
        for (int col = 0; col < BOARD_SIZE; col++) {
            int shift = ((N_GRIDS - 1 - k) * 2);
            unsigned int bits = (*btable >> shift) & 0x3;
            char cell;

            switch (bits) {
            case 0:
                cell = 'O';
                break;
            case 1:
                cell = 'X';
                break;
            case 2:
                cell = ' ';
                break;
            default:
                cell = '?';
                break;
            }

            draw_buffer[i++] = cell;
            if (col < BOARD_SIZE - 1)
                draw_buffer[i++] = '|';

            k++;
        }

        draw_buffer[i++] = '\n';

        if (row < BOARD_SIZE) {
            for (int j = 0; j < (BOARD_SIZE << 1) - 1; j++) {
                draw_buffer[i++] = '-';
            }
            draw_buffer[i++] = '\n';
        }
    }

    return 0;
}


static int user_draw_board(char *table)
{
    int i = 0, k = 0;
    draw_buffer[i++] = '\n';
    draw_buffer[i++] = '\n';


    while (i < DRAWBUFFER_SIZE) {
        for (int j = 0; j < (BOARD_SIZE << 1) - 1 && k < N_GRIDS; j++) {
            draw_buffer[i++] = j & 1 ? '|' : table[k++];
        }
        draw_buffer[i++] = '\n';
        for (int j = 0; j < (BOARD_SIZE << 1) - 1; j++) {
            draw_buffer[i++] = '-';
        }
        draw_buffer[i++] = '\n';
    }


    return 0;
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
        if (first && (moves[i] == 0)) {
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


static bool read_attr, end_attr, user_mode, switch_counting, user_display,
    user_flag;

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
        case 18: /* Ctrl-R */
            user_mode = true;
            break;
        }
    }
    close(attr_fd);
}


/* user space scheduler */


struct task {
    jmp_buf env;
    struct list_head list;
};


static LIST_HEAD(tasklist);
static void (**tasks)(void);
static int ntasks;
static jmp_buf sched;
static struct task *cur_task;


static void task_add(struct task *task)
{
    list_add_tail(&task->list, &tasklist);
}

static void task_switch()
{
    if (tasklist.next == &tasklist) {
        return;
    }

    struct list_head *entry = tasklist.next;
    struct task *t =
        (struct task *) ((char *) entry - offsetof(struct task, list));

    list_del(&t->list);
    cur_task = t;
    longjmp(t->env, 1);
}

void schedule(void)
{
    int i = 0;
    setjmp(sched);
    while (ntasks-- > 0) {
        tasks[i++]();
    }

    task_switch();
}

/* A task yields control n times */

void task0(void)
{
    struct task *task = malloc(sizeof(struct task));
    INIT_LIST_HEAD(&task->list);
    if (setjmp(task->env) == 0) {
        task_add(task);
        longjmp(sched, 1);
    }

    task = cur_task;

    while (user_mode) {
        time(&current_time);
        time_info = localtime(&current_time);
        if (setjmp(task->env) == 0) {
            int move;
#ifdef USE_RL
            move = play_rl(table, &agent);
#else
            move = mcts(table, 'O');
#endif
            if (move != -1) {
                table[move] = 'O';
            }
            user_tmp_move = (user_tmp_move << 4) | (move & 0xF);
            user_tmp_move_step = user_tmp_move_step + 1;
            if (user_display) {
                user_draw_board(table);
                printf(
                    "\033[H\033[J"); /* ASCII escape code to clear the screen */
                printf("%s\n", draw_buffer);
                printf("\rCurrent time: %04d-%02d-%02d %02d:%02d:%02d \n",
                       time_info->tm_year + 1900, time_info->tm_mon + 1,
                       time_info->tm_mday, time_info->tm_hour,
                       time_info->tm_min, time_info->tm_sec);
            }
            task_add(task);
            task_switch();
        }
        win = check_win(table);
        if (win != ' ') {
            memset(table, ' ', N_GRIDS);
            user_move_seq[user_move_index] = user_tmp_move;
            user_tmp_move = 0;
            if ((user_move_index + 1) == 16)
                user_flag = true;

            user_move_index = (user_move_index + 1) & 0xF;
            user_move_step = (user_move_step << 4) | user_tmp_move_step;
            user_tmp_move_step = 0;
            if (user_flag)
                user_head = (user_head + 1) & 0xF;
        }
        task = cur_task;
    }

    free(task);
    longjmp(sched, 1);
}

void task1(void)
{
    struct task *task = malloc(sizeof(struct task));
    INIT_LIST_HEAD(&task->list);

    if (setjmp(task->env) == 0) {
        task_add(task);
        longjmp(sched, 1);
    }

    task = cur_task;

    while (user_mode) {
        time(&current_time);
        time_info = localtime(&current_time);
        if (setjmp(task->env) == 0) {
            int move;
            move = mcts(table, 'X');

            if (move != -1) {
                table[move] = 'X';
            }
            user_tmp_move = (user_tmp_move << 4) | (move & 0xF);
            user_tmp_move_step = user_tmp_move_step + 1;

            if (user_display) {
                user_draw_board(table);
                printf(
                    "\033[H\033[J"); /* ASCII escape code to clear the screen */
                printf("%s\n", draw_buffer);
                printf("\rCurrent time: %04d-%02d-%02d %02d:%02d:%02d \n",
                       time_info->tm_year + 1900, time_info->tm_mon + 1,
                       time_info->tm_mday, time_info->tm_hour,
                       time_info->tm_min, time_info->tm_sec);
            }
            task_add(task);
            task_switch();
        }
        win = check_win(table);
        if (win != ' ') {
            memset(table, ' ', N_GRIDS);
            user_move_seq[user_move_index] = user_tmp_move;
            user_tmp_move = 0;
            if ((user_move_index + 1) == 16)
                user_flag = true;

            user_move_index = (user_move_index + 1) & 0xF;
            user_move_step = (user_move_step << 4) | user_tmp_move_step;
            user_tmp_move_step = 0;
            if (user_flag)
                user_head = (user_head + 1) & 0xF;
        }
        task = cur_task;
    }

    free(task);
    longjmp(sched, 1);
}

void task2(void)
{
    struct task *task = malloc(sizeof(struct task));
    INIT_LIST_HEAD(&task->list);

    if (setjmp(task->env) == 0) {
        task_add(task);
        longjmp(sched, 1);
    }
    char input;
    task = cur_task;

    while (user_mode) {
        if (setjmp(task->env) == 0) {
            if (read(STDIN_FILENO, &input, 1) == 1) {
                switch (input) {
                case 16: /* Ctrl-P */
                    user_display = !user_display;
                    if (!user_display)
                        printf("Stopping to display the chess board...\n");
                    break;
                case 17: /* Ctrl-Q */
                    end_attr = true;
                    user_mode = false;
                    printf("Stopping the user space tic-tac-toe game...\n");
                    user_move_seq[16] = user_move_step;
                    if (user_head >= 0) {
                        int count = 0;
                        unsigned long long temp[17];
                        for (size_t i = user_head; i < 16; i++) {
                            temp[count++] = user_move_seq[i];
                        }
                        for (size_t j = 0; j < user_head; j++) {
                            temp[count++] = user_move_seq[j];
                        }
                        temp[16] = user_move_step;
                        memcpy(user_move_seq, temp, sizeof(user_move_seq));
                    }
                    print_moves(user_move_seq);
                    break;
                case 18: /* Ctrl-R */
                    user_mode = false;
                    break;
                }
            }
            task_add(task);
            task_switch();
        }
        task = cur_task;
    }

    free(task);
    longjmp(sched, 1);
}



#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))


int main(int argc, char *argv[])
{
    if (!status_check())
        exit(1);

    raw_mode_enable();
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    unsigned int display_buf;
    user_mode = false;
    switch_counting = false;

    fd_set readset;
    int device_fd = open(XO_DEVICE_FILE, O_RDONLY);
    int max_fd = device_fd > STDIN_FILENO ? device_fd : STDIN_FILENO;
    read_attr = true;
    end_attr = false;


    while (!end_attr) {
        if (!user_mode) {
            if (switch_counting) {
                device_fd = open(XO_DEVICE_FILE, O_RDONLY);
                max_fd = device_fd > STDIN_FILENO ? device_fd : STDIN_FILENO;
                switch_counting = false;
            }
            display_buf = 0;
            FD_ZERO(&readset);
            FD_SET(STDIN_FILENO, &readset);
            FD_SET(device_fd, &readset);
            time(&current_time);
            time_info = localtime(&current_time);


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
                printf(
                    "\033[H\033[J"); /* ASCII escape code to clear the screen */
                read(device_fd, &display_buf, 4);
                draw_board(&display_buf);
                printf("%s\n", draw_buffer);
                printf("\rCurrent time: %04d-%02d-%02d %02d:%02d:%02d \n",
                       time_info->tm_year + 1900, time_info->tm_mon + 1,
                       time_info->tm_mday, time_info->tm_hour,
                       time_info->tm_min, time_info->tm_sec);
            }
        } else {
            if (!switch_counting) {
                close(device_fd);
                printf("\033[H\033[J");
            }
            switch_counting = true;
            user_display = true;
            memset(table, ' ', N_GRIDS);
#ifdef USE_RL
            CALC_STATE_NUM(state_num);
            init_rl_agent(&agent, state_num, 'O');
            load_model(&agent, state_num, MODEL_NAME);
#else
            mcts_init();
#endif
            negamax_init();
            printf("Start the user space tic-tac-toe game...\n");
            INIT_LIST_HEAD(&tasklist);
            void (*registered_task[])(void) = {task0, task2, task1, task2};
            tasks = registered_task;
            ntasks = ARRAY_SIZE(registered_task);
            schedule();
        }
    }

    raw_mode_disable();
    fcntl(STDIN_FILENO, F_SETFL, flags);
    close(device_fd);

    return 0;
}
