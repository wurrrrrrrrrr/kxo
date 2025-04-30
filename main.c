/* kxo: A Tic-Tac-Toe Game Engine implemented as Linux kernel module */

#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/sched/loadavg.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "game.h"
#include "mcts.h"
#include "negamax.h"

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("National Cheng Kung University, Taiwan");
MODULE_DESCRIPTION("In-kernel Tic-Tac-Toe game engine");

/* Macro DECLARE_TASKLET_OLD exists for compatibility.
 * See https://lwn.net/Articles/830964/
 */
#ifndef DECLARE_TASKLET_OLD
#define DECLARE_TASKLET_OLD(arg1, arg2) DECLARE_TASKLET(arg1, arg2, 0L)
#endif

#define DEV_NAME "kxo"

#define NR_KMLDRV 1

static int delay = 100; /* time (in ms) to generate an event */

/* Declare kernel module attribute for sysfs */

#define FLAG_DISPLAY (1 << 0)
#define FLAG_RESUME (1 << 1)
#define FLAG_END (1 << 2)

struct kxo_attr {
    char flags;
    rwlock_t lock;
};

static struct kxo_attr attr_obj;

static ssize_t kxo_state_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    read_lock(&attr_obj.lock);
    int ret = snprintf(buf, 1, "%c", attr_obj.flags);
    read_unlock(&attr_obj.lock);
    return ret;
}

static ssize_t kxo_state_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    write_lock(&attr_obj.lock);
    sscanf(buf, "%c", &(attr_obj.flags));
    write_unlock(&attr_obj.lock);
    return count;
}

static DEVICE_ATTR_RW(kxo_state);

/* Data produced by the simulated device */

/* Timer to simulate a periodic IRQ */
static struct timer_list timer;

/* Character device stuff */
static int major;
static struct class *kxo_class;
static struct cdev kxo_cdev;

static char table[N_GRIDS];
static char ai_one_buf[32];
static char ai_two_buf[32];

unsigned long long ai_one_load_avg;
unsigned long long ai_two_load_avg;

static unsigned int btable = 0;

static char turn;
static int finish;

/* Data are stored into a kfifo buffer before passing them to the userspace */
static DECLARE_KFIFO_PTR(rx_fifo, unsigned char);

/* NOTE: the usage of kfifo is safe (no need for extra locking), until there is
 * only one concurrent reader and one concurrent writer. Writes are serialized
 * from the interrupt context, readers are serialized using this mutex.
 */
static DEFINE_MUTEX(read_lock);

/* Wait queue to implement blocking I/O from userspace */
static DECLARE_WAIT_QUEUE_HEAD(rx_wait);

/* Insert the whole chess board into the kfifo buffer */
static void produce_board(void)
{
    for (int i = N_GRIDS - 1; i >= 0; i--) {
        if (table[i] == ' ') {
            btable = btable << 2 | 0x2;
        } else if (table[i] == 'X') {
            btable = btable << 2 | 1;
        } else {
            btable = btable << 2;
        }
        pr_info("%d\n", btable);
    }

    snprintf(ai_one_buf, sizeof(ai_one_buf), "%llu", ai_one_load_avg);
    snprintf(ai_two_buf, sizeof(ai_two_buf), "%llu", ai_two_load_avg);

    unsigned int len =
        kfifo_in(&rx_fifo, (unsigned char *) &btable, sizeof(btable));
    if (unlikely(len < sizeof(btable)) && printk_ratelimit())
        pr_warn("%s: %zu bytes dropped\n", __func__, sizeof(btable) - len);

    len = kfifo_in(&rx_fifo, (unsigned char *) &ai_one_buf, sizeof(ai_one_buf));
    if (unlikely(len < sizeof(ai_one_buf)) && printk_ratelimit())
        pr_warn("%s: %zu bytes dropped\n", __func__, sizeof(ai_one_buf) - len);

    len = kfifo_in(&rx_fifo, (unsigned char *) &ai_two_buf, sizeof(ai_two_buf));
    if (unlikely(len < sizeof(ai_two_buf)) && printk_ratelimit())
        pr_warn("%s: %zu bytes dropped\n", __func__, sizeof(ai_two_buf) - len);

    pr_debug("kxo: %s: in %u/%u bytes\n", __func__, len, kfifo_len(&rx_fifo));
}

/* Mutex to serialize kfifo writers within the workqueue handler */
static DEFINE_MUTEX(producer_lock);

/* Mutex to serialize fast_buf consumers: we can use a mutex because consumers
 * run in workqueue handler (kernel thread context).
 */
static DEFINE_MUTEX(consumer_lock);

/* We use an additional "faster" circular buffer to quickly store data from
 * interrupt context, before adding them to the kfifo.
 */
static struct circ_buf fast_buf;



static unsigned long long move_seq[17];
static unsigned long long tmp_move = 0;
static char tmp_move_step = 0;
static unsigned long long move_step = 0;
static unsigned int move_index = 0;

static char flag = 0;
static int head = -1;



/* Clear all data from the circular buffer fast_buf */
static void fast_buf_clear(void)
{
    fast_buf.head = fast_buf.tail = 0;
}

/* Workqueue handler: executed by a kernel thread */
static void drawboard_work_func(struct work_struct *w)
{
    int cpu;

    /* This code runs from a kernel thread, so softirqs and hard-irqs must
     * be enabled.
     */
    WARN_ON_ONCE(in_softirq());
    WARN_ON_ONCE(in_interrupt());

    /* Pretend to simulate access to per-CPU data, disabling preemption
     * during the pr_info().
     */
    cpu = get_cpu();
    pr_info("kxo: [CPU#%d] %s\n", cpu, __func__);
    put_cpu();

    read_lock(&attr_obj.lock);
    if (!(attr_obj.flags & FLAG_DISPLAY)) {
        read_unlock(&attr_obj.lock);
        return;
    }
    read_unlock(&attr_obj.lock);

    mutex_lock(&producer_lock);
    /* Store data to the kfifo buffer */
    mutex_lock(&consumer_lock);
    produce_board();
    mutex_unlock(&consumer_lock);
    mutex_unlock(&producer_lock);
    wake_up_interruptible(&rx_wait);
}


static void ai_one_work_func(struct work_struct *w)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    int cpu;

    WARN_ON_ONCE(in_softirq());
    WARN_ON_ONCE(in_interrupt());

    cpu = get_cpu();
    pr_info("kxo: [CPU#%d] start doing %s\n", cpu, __func__);
    tv_start = ktime_get();
    mutex_lock(&producer_lock);
    int move;
    WRITE_ONCE(move, mcts(table, 'O'));

    smp_mb();

    if (move != -1) {
        WRITE_ONCE(table[move], 'O');
        tmp_move = (tmp_move << 4) | (move & 0xF);
        tmp_move_step = tmp_move_step + 1;
    }

    WRITE_ONCE(turn, 'X');
    WRITE_ONCE(finish, 1);
    smp_wmb();
    mutex_unlock(&producer_lock);
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));
    ai_one_load_avg = calc_load(ai_one_load_avg, EXP_5, nsecs >> 10);
    pr_info("kxo: [CPU#%d] %s completed in %llu usec\n", cpu, __func__,
            (unsigned long long) nsecs >> 10);
    put_cpu();
}

static void ai_two_work_func(struct work_struct *w)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    int cpu;

    WARN_ON_ONCE(in_softirq());
    WARN_ON_ONCE(in_interrupt());

    cpu = get_cpu();
    pr_info("kxo: [CPU#%d] start doing %s\n", cpu, __func__);
    tv_start = ktime_get();
    mutex_lock(&producer_lock);
    int move;
    WRITE_ONCE(move, negamax_predict(table, 'X').move);

    smp_mb();

    if (move != -1) {
        WRITE_ONCE(table[move], 'X');
        tmp_move = (tmp_move << 4) | (move & 0xF);
        tmp_move_step = tmp_move_step + 1;
    }

    WRITE_ONCE(turn, 'O');
    WRITE_ONCE(finish, 1);
    smp_wmb();
    mutex_unlock(&producer_lock);
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));
    ai_two_load_avg = calc_load(ai_two_load_avg, EXP_5, nsecs >> 10);
    pr_info("kxo: [CPU#%d] %s completed in %llu usec\n", cpu, __func__,
            (unsigned long long) nsecs >> 10);
    put_cpu();
}

/* Workqueue for asynchronous bottom-half processing */
static struct workqueue_struct *kxo_workqueue;

/* Work item: holds a pointer to the function that is going to be executed
 * asynchronously.
 */
static DECLARE_WORK(drawboard_work, drawboard_work_func);
static DECLARE_WORK(ai_one_work, ai_one_work_func);
static DECLARE_WORK(ai_two_work, ai_two_work_func);

/* Tasklet handler.
 *
 * NOTE: different tasklets can run concurrently on different processors, but
 * two of the same type of tasklet cannot run simultaneously. Moreover, a
 * tasklet always runs on the same CPU that schedules it.
 */
static void game_tasklet_func(unsigned long __data)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    WARN_ON_ONCE(!in_interrupt());
    WARN_ON_ONCE(!in_softirq());

    tv_start = ktime_get();

    READ_ONCE(finish);
    READ_ONCE(turn);
    smp_rmb();

    if (finish && turn == 'O') {
        WRITE_ONCE(finish, 0);
        smp_wmb();
        queue_work(kxo_workqueue, &ai_one_work);
    } else if (finish && turn == 'X') {
        WRITE_ONCE(finish, 0);
        smp_wmb();
        queue_work(kxo_workqueue, &ai_two_work);
    }
    queue_work(kxo_workqueue, &drawboard_work);
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));

    pr_info("kxo: [CPU#%d] %s in_softirq: %llu usec\n", smp_processor_id(),
            __func__, (unsigned long long) nsecs >> 10);
}

/* Tasklet for asynchronous bottom-half processing in softirq context */
static DECLARE_TASKLET_OLD(game_tasklet, game_tasklet_func);

static void ai_game(void)
{
    WARN_ON_ONCE(!irqs_disabled());

    pr_info("kxo: [CPU#%d] doing AI game\n", smp_processor_id());
    pr_info("kxo: [CPU#%d] scheduling tasklet\n", smp_processor_id());
    tasklet_schedule(&game_tasklet);
}

static int count = 0;

static void timer_handler(struct timer_list *__timer)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    pr_info("kxo: [CPU#%d] enter %s\n", smp_processor_id(), __func__);
    /* We are using a kernel timer to simulate a hard-irq, so we must expect
     * to be in softirq context here.
     */
    WARN_ON_ONCE(!in_softirq());

    /* Disable interrupts for this CPU to simulate real interrupt context */
    local_irq_disable();

    tv_start = ktime_get();

    char win = ' ';

    if (count >= 2) {
        win = check_win(table);
    }

    if (win == ' ') {
        ai_game();
        count = count + 1;
        mod_timer(&timer, jiffies + msecs_to_jiffies(delay));
    } else {
        count = 0;
        read_lock(&attr_obj.lock);
        if (attr_obj.flags & FLAG_DISPLAY) {
            int cpu = get_cpu();
            pr_info("kxo: [CPU#%d] Drawing final board\n", cpu);
            put_cpu();
            mutex_lock(&producer_lock);
            /* Store data to the kfifo buffer */
            mutex_lock(&consumer_lock);
            produce_board();
            mutex_unlock(&producer_lock);
            mutex_unlock(&consumer_lock);


            wake_up_interruptible(&rx_wait);
        }

        if (!(attr_obj.flags & FLAG_END)) {
            memset(table, ' ',
                   N_GRIDS); /* Reset the table so the game restart */
            btable = 0;
            mod_timer(&timer, jiffies + msecs_to_jiffies(delay));
        }

        read_unlock(&attr_obj.lock);
        move_seq[move_index] = tmp_move;
        tmp_move = 0;
        if ((move_index + 1) == 16)
            flag = 1;

        move_index = (move_index + 1) & 0xF;
        move_step = (move_step << 4) | tmp_move_step;
        tmp_move_step = 0;
        if (flag)
            head = (head + 1) & 0xF;
        pr_info("kxo: %c win!!!\n", win);
    }
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));

    pr_info("kxo: [CPU#%d] %s in_irq: %llu usec\n", smp_processor_id(),
            __func__, (unsigned long long) nsecs >> 10);

    local_irq_enable();
}

static ssize_t kxo_read(struct file *file,
                        char __user *buf,
                        size_t count,
                        loff_t *ppos)
{
    unsigned int read;
    int ret;

    pr_debug("kxo: %s(%p, %zd, %lld)\n", __func__, buf, count, *ppos);

    if (unlikely(!access_ok(buf, count)))
        return -EFAULT;

    if (mutex_lock_interruptible(&read_lock))
        return -ERESTARTSYS;

    do {
        ret = kfifo_to_user(&rx_fifo, buf, count, &read);
        if (unlikely(ret < 0))
            break;
        if (read)
            break;
        if (file->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }
        ret = wait_event_interruptible(rx_wait, kfifo_len(&rx_fifo));
    } while (ret == 0);
    pr_debug("kxo: %s: out %u/%u bytes\n", __func__, read, kfifo_len(&rx_fifo));

    mutex_unlock(&read_lock);

    return ret ? ret : read;
}

static atomic_t open_cnt;

static int kxo_open(struct inode *inode, struct file *filp)
{
    pr_debug("kxo: %s\n", __func__);
    write_lock(&attr_obj.lock);
    if (attr_obj.flags & FLAG_END) {
        pr_info("kxo: Restarting game\n");

        attr_obj.flags &= ~FLAG_END;
        attr_obj.flags |= FLAG_DISPLAY;

        memset(table, ' ', N_GRIDS);
        turn = 'O';
        finish = 1;
        count = 0;
        memset(move_seq, 0, sizeof(move_seq));
        tmp_move = 0;
        move_index = 0;
        tmp_move_step = 0;
        move_step = 0;
        flag = 0;
        head = -1;
        ai_one_load_avg = 0;
        ai_two_load_avg = 0;
    }

    write_unlock(&attr_obj.lock);
    if (atomic_inc_return(&open_cnt) == 1)
        mod_timer(&timer, jiffies + msecs_to_jiffies(delay));
    pr_info("openm current cnt: %d\n", atomic_read(&open_cnt));

    return 0;
}

static int kxo_release(struct inode *inode, struct file *filp)
{
    pr_debug("kxo: %s\n", __func__);
    if (atomic_dec_and_test(&open_cnt)) {
        del_timer_sync(&timer);
        flush_workqueue(kxo_workqueue);
        fast_buf_clear();
    }
    pr_info("release, current cnt: %d\n", atomic_read(&open_cnt));

    return 0;
}
typedef struct {
    unsigned long long moves[17];
} kxo_move_seq_t;

#define MY_IOCTL_GET_DATA _IOR('m', 1, kxo_move_seq_t)

static long my_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    if (cmd == MY_IOCTL_GET_DATA) {
        kxo_move_seq_t data;
        move_seq[16] = move_step;

        if (flag) {
            unsigned long long temp[17];
            int count = 0;

            // Ensure head is in a valid range
            for (size_t i = head; i < 16; i++) {
                temp[count++] = move_seq[i];
            }
            for (size_t j = 0; j < head; j++) {
                temp[count++] = move_seq[j];
            }
            temp[16] = move_step;  // Add move_step at the end

            memcpy(data.moves, temp,
                   sizeof(data.moves));  // Copy the array to data
            if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
                return -EFAULT;
            }

        } else {
            memcpy(data.moves, move_seq, sizeof(data.moves));
            if (copy_to_user((void __user *) arg, &data, sizeof(data)))
                return -EFAULT;
        }
        return 0;
    }

    return -EINVAL;
}


static const struct file_operations kxo_fops = {
    .read = kxo_read,
    .llseek = no_llseek,
    .open = kxo_open,
    .release = kxo_release,
    .unlocked_ioctl = my_ioctl,
    .owner = THIS_MODULE,
};

static int __init kxo_init(void)
{
    dev_t dev_id;
    int ret;

    if (kfifo_alloc(&rx_fifo, PAGE_SIZE, GFP_KERNEL) < 0)
        return -ENOMEM;

    /* Register major/minor numbers */
    ret = alloc_chrdev_region(&dev_id, 0, NR_KMLDRV, DEV_NAME);
    if (ret)
        goto error_alloc;
    major = MAJOR(dev_id);

    /* Add the character device to the system */
    cdev_init(&kxo_cdev, &kxo_fops);
    ret = cdev_add(&kxo_cdev, dev_id, NR_KMLDRV);
    if (ret) {
        kobject_put(&kxo_cdev.kobj);
        goto error_region;
    }

    /* Create a class structure */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    kxo_class = class_create(THIS_MODULE, DEV_NAME);
#else
    kxo_class = class_create(DEV_NAME);
#endif
    if (IS_ERR(kxo_class)) {
        printk(KERN_ERR "error creating kxo class\n");
        ret = PTR_ERR(kxo_class);
        goto error_cdev;
    }

    /* Register the device with sysfs */
    struct device *kxo_dev =
        device_create(kxo_class, NULL, MKDEV(major, 0), NULL, DEV_NAME);

    ret = device_create_file(kxo_dev, &dev_attr_kxo_state);
    if (ret < 0) {
        printk(KERN_ERR "failed to create sysfs file kxo_state\n");
        goto error_cdev;
    }

    /* Allocate fast circular buffer */
    fast_buf.buf = vmalloc(PAGE_SIZE);
    if (!fast_buf.buf) {
        device_destroy(kxo_class, dev_id);
        class_destroy(kxo_class);
        ret = -ENOMEM;
        goto error_cdev;
    }

    /* Create the workqueue */
    kxo_workqueue = alloc_workqueue("kxod", WQ_UNBOUND, WQ_MAX_ACTIVE);
    if (!kxo_workqueue) {
        vfree(fast_buf.buf);
        device_destroy(kxo_class, dev_id);
        class_destroy(kxo_class);
        ret = -ENOMEM;
        goto error_cdev;
    }

    negamax_init();
    mcts_init();
    memset(table, ' ', N_GRIDS);
    turn = 'O';
    finish = 1;
    ai_one_load_avg = 0;
    ai_two_load_avg = 0;

    attr_obj.flags |= FLAG_DISPLAY;
    attr_obj.flags |= FLAG_RESUME;
    rwlock_init(&attr_obj.lock);
    /* Setup the timer */
    timer_setup(&timer, timer_handler, 0);
    atomic_set(&open_cnt, 0);

    pr_info("kxo: registered new kxo device: %d,%d\n", major, 0);
out:
    return ret;
error_cdev:
    cdev_del(&kxo_cdev);
error_region:
    unregister_chrdev_region(dev_id, NR_KMLDRV);
error_alloc:
    kfifo_free(&rx_fifo);
    goto out;
}

static void __exit kxo_exit(void)
{
    dev_t dev_id = MKDEV(major, 0);

    del_timer_sync(&timer);
    tasklet_kill(&game_tasklet);
    flush_workqueue(kxo_workqueue);
    destroy_workqueue(kxo_workqueue);
    vfree(fast_buf.buf);
    device_destroy(kxo_class, dev_id);
    class_destroy(kxo_class);
    cdev_del(&kxo_cdev);
    unregister_chrdev_region(dev_id, NR_KMLDRV);

    kfifo_free(&rx_fifo);
    pr_info("kxo: unloaded\n");
}

module_init(kxo_init);
module_exit(kxo_exit);
