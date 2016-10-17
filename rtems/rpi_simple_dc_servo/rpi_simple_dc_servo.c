/*
 * Example how to build simple DC motor servosystem with RPi
 *
 * Copyright (C) 2015 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *                               <ppisa@pikron.com>
 *
 * Department of Control Engineering
 * Faculty of Electrical Engineering
 * Czech Technical University in Prague (CTU)
 * http://dce.fel.cvut.cz/
 *
 * PiKRON s.r.o.
 * http://www.pikron.com/
 *
 * The code demonstrates simple motion control
 * system build for Raspberry Pi. Hardware can be wirewrapped
 * design documented on LinTarget project page
 *
 *   http://lintarget.sourceforge.net/rpi-motor-control/index.html
 *
 * or more advanced hardware provided by PiKRON company.
 * This hardware consists of FPGA interface board RPI-MI-1
 * and 3P-MOTOR-DRIVER designed by Petr Porazil for
 * PiKRON company.
 *
 * The demonstration is based on next ideas sources
 *  - motion control system designs at PiKRON company
 *  - Motor control semestral work in for subject
 *    Real-Time systems programming at DCE
 *    https://support.dce.felk.cvut.cz/psr/cviceni/semestralka/
 *    prevalent author Michal Sojka from DCE
 *  - diploma thesis of Radek Meciar who designed proof of concept
 *    impelmentation under author lead
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>  /* this provides mlockall() */
#include <pthread.h>
#include <signal.h>
#include <rtems.h>
#include <time.h>

#include "rpi_bidirpwm.h"
#include "app_def.h"

char *irc_dev_name = "/dev/irc0";
int irc_dev_fd = -1;
int base_task_prio;
int fifo_min_prio;
int fifo_max_prio;
volatile int64_t req_speed_fract;
volatile int32_t act_speed;
volatile uint64_t ref_pos_fract;
volatile uint32_t act_pos;
volatile uint32_t last_pos;
volatile int32_t pos_offset;
int32_t ctrl_p = 2000;
int32_t ctrl_i = 80;
int32_t ctrl_d = 10000;
int32_t ctrl_i_sum;
int32_t ctrl_err_last;
int32_t ctrl_action;
uint32_t pwm_max = 2000;
uint32_t sample_period_nsec = 1000 * 1000;
struct timespec sample_period_time;
struct timespec monitor_period_time;
volatile int speed_controller_started_fl;
volatile int environment_setup_done_fl;

int irc_dev_init(void)
{
    if (irc_dev_fd >= 0)
      return 0;

    irc_dev_fd = open(irc_dev_name, O_RDONLY);
    if (irc_dev_fd == -1) {
        return -1;
    }
    return 0;
}

int irc_dev_read(uint32_t *irc_val)
{
    if (read(irc_dev_fd, irc_val, sizeof(uint32_t)) != sizeof(uint32_t)) {
        return -1;
    }
    return 0;
}

int create_rt_task(pthread_t *thread, int prio, void *(*start_routine) (void *), void *arg)
{
    int ret ;

    pthread_attr_t attr;
    struct sched_param schparam;

    /* Initialize pthread attribute structure by default parameters */
    if (pthread_attr_init(&attr) != 0) {
        fprintf(stderr, "pthread_attr_init failed\n");
        return -1;
    }

    /* Request to switch to specified policy at thread start */
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0) {
        fprintf(stderr, "pthread_attr_setinheritsched failed\n");
        return -1;
    }

    /* Select RT aware policy */
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0) {
        fprintf(stderr, "pthread_attr_setschedpolicy SCHED_FIFO failed\n");
        return -1;
    }

    /* Select priority within selected policy */
    schparam.sched_priority = prio;

    /* Check priority parameter setup to the structure */
    if (pthread_attr_setschedparam(&attr, &schparam) != 0) {
        fprintf(stderr, "pthread_attr_setschedparam failed\n");
        return -1;
    }

    /* Create new thread with specified parameters and start it */
    ret = pthread_create(thread, &attr, start_routine, arg);

    /* Release unneeded parameters structure */
    pthread_attr_destroy(&attr);

    return ret;
}

int controler_step(uint32_t rp)
{
    uint32_t ap, lp;
    int32_t err;
    int32_t action;
    int fract_bits = 8;
    uint32_t act_max = pwm_max << fract_bits;

    irc_dev_read(&ap);
    ap += pos_offset;
    lp = act_pos;
    last_pos = lp;
    act_pos = ap;
    act_speed = (int32_t)(ap - lp);

    /* Difference between setpoint and plant state */
    err = (int32_t)(rp - ap);

    /* Limit error to not overflow 32-bit arithmetic later */
    if (err > 0x7fff)
        err = 0x7fff;
    else if (err < -0x7fff)
        err = -0x7fff;

    /*
     * Accumulation of error value for
     * PSD (proportional-summation-difference)
     * controller (same role as integral for
     * continuous PID).
    */
    if (ctrl_i == 0) {
        ctrl_i_sum = 0;
    } else {
        ctrl_i_sum += err * ctrl_i;
    }

    /* Compute control action */
    action = ctrl_p * err + /* proportional component */
             ctrl_i_sum +   /* "integral" component */
                 /* difference/"derivative" component */
             ctrl_d * (err - ctrl_err_last);

    /* Store error value for difference computation in next iteration */
    ctrl_err_last = err;

    /* Anti-windup algorithm */
    if (action >= 0) {
        if (action > act_max) {
            ctrl_i_sum -= action - act_max;
            action = act_max;
        }
    } else {
        if (-action > act_max) {
            ctrl_i_sum -= action + act_max;
            action = -act_max;
        }
    }

   /*
    * Scale the action value to enable computation
    * in fixed/integer arithmetic format
    */
    ctrl_action = action >> fract_bits;
    rpi_bidirpwm_set(action >> fract_bits);

    return 0;
}


void wait_next_period(void)
{
    sample_period_time.tv_nsec += sample_period_nsec;
    if (sample_period_time.tv_nsec > 1000*1000*1000) {
        sample_period_time.tv_nsec -= 1000*1000*1000;
        sample_period_time.tv_sec += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sample_period_time, NULL);
}

void stop_motor(void)
{
    rpi_bidirpwm_set(0);
}

void sig_handler(int sig)
{
    stop_motor();
    exit(1);
}

int setup_environment(const char *argv0)
{
    struct sigaction sigact;

    if (environment_setup_done_fl)
        return 0;

    fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
    fifo_max_prio = sched_get_priority_max(SCHED_FIFO);

    base_task_prio = fifo_max_prio - 20;
    if (base_task_prio < fifo_min_prio)
        base_task_prio = fifo_min_prio;

    if (rpi_bidirpwm_init() < 0) {
        fprintf(stderr, "%s: setpwm cannot initialize hardware\n", argv0);
        return -1;
    }

    if (irc_dev_init() < 0) {
        fprintf(stderr, "%s: readirc device init error\n"
                        "try: modprobe rpi_gpio_irc_module\n",
                        argv0);
        return -1;
    }

  #if 0
    if (mlockall(MCL_FUTURE | MCL_CURRENT) < 0) {
        fprintf(stderr, "%s: mlockall failed - cannot lock application in memory\n", argv0);
        exit(1);
    }
  #endif

    atexit(stop_motor);

    memset(&sigact, 0, sizeof(sigact));
    sigact.sa_handler = sig_handler;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    environment_setup_done_fl = 1;

    return 0;
}

void *speed_controller(void *arg)
{
    uint64_t rp_frac;

    do {
        rp_frac = ref_pos_fract;
        rp_frac += req_speed_fract;
        ref_pos_fract = rp_frac;

        controler_step(rp_frac >> 32);
        wait_next_period();
    } while(1);
}

void set_speed(int speed)
{
    req_speed_fract = speed * (uint64_t)(0x100000000LL / 1000.0 * 2000 / 1000.0);
}

void *monitor_thread(void *arg)
{
    int32_t ap;
    do {
        monitor_period_time.tv_sec += 1;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &monitor_period_time, NULL);

        if (req_speed_fract == 0)
            continue;

        ap = (int32_t)act_pos;
        printf("ap=%8ld act=%5ld i_sum=%8ld\n", (long)ap, (long)ctrl_action, (long)ctrl_i_sum);
    } while(1);
}

int run_speed_controller(int speed)
{
    uint32_t pos;
    pthread_t thread_id;
    int monitor_prio;

    set_speed(speed);

    if (speed_controller_started_fl)
        return 0;

    irc_dev_read(&pos);
    pos_offset = -pos;

    clock_gettime(CLOCK_MONOTONIC, &sample_period_time);
    monitor_period_time = sample_period_time;

    ref_pos_fract = 500LL << 32;

    if (create_rt_task(&thread_id, base_task_prio, speed_controller, NULL) != 0) {
        fprintf(stderr, "cannot start realtime speed_controller task\n");
        return -1;
    }

    monitor_prio = fifo_min_prio + 10;

    if (monitor_prio > base_task_prio)
        monitor_prio = base_task_prio - 1;

    if (monitor_prio < fifo_min_prio)
        monitor_prio = fifo_min_prio;

    if (create_rt_task(&thread_id, monitor_prio, monitor_thread, NULL) != 0) {
        fprintf(stderr, "cannot start realtime speed_controller task\n");
        return -1;
    }

    speed_controller_started_fl = 1;

    return 0;
}

int servo_setpwm_forshell(int argc, char **argv)
{
    char *p;
    long value;

    if (argc < 2) {
        fprintf(stderr, "%s: setpwm requires argument\n", argv[0]);
        return -1;
    }
    if (rpi_bidirpwm_init() < 0) {
        fprintf(stderr, "%s: setpwm cannot initialize hardware\n", argv[0]);
        return -1;
    }
    value = strtol(argv[1], &p, 0);
    if (argv[1] == p) {
        fprintf(stderr, "%s: setpwm value parse error\n", argv[0]);
        return -1;
    }
    rpi_bidirpwm_set(value);

    return 0;
}

int servo_readirc_forshell(int argc, char **argv)
{
    uint32_t irc_val;
    if (irc_dev_init() < 0) {
        fprintf(stderr, "%s: readirc device init error\n"
                "try: modprobe rpi_gpio_irc_module\n",
                argv[0]);
        return -1;
    }
    if (irc_dev_read(&irc_val) < 0) {
        fprintf(stderr, "%s: readirc device read failed\n",
                argv[0]);
        return -1;
    }
    printf("IRC value %ld\n", (long)(int32_t)irc_val);

    return 0;
}

int servo_runspeed_forshell(int argc, char **argv)
{
    char *p;
    long value;

    if (argc < 2) {
        fprintf(stderr, "%s: setspeed requires argument\n", argv[0]);
        return -1;
    }

    if (setup_environment(argv[0]) < 0) {
        fprintf(stderr, "%s: setup_environment failed\n", argv[0]);
        return -1;
    }

    value = strtol(argv[1], &p, 0);
    if (argv[1] == p) {
        fprintf(stderr, "%s: setpwm value parse error\n", argv[0]);
        return -1;
    }

    if (run_speed_controller(value) < 0) {
        fprintf(stderr, "%s: run_speed_controller failed\n", argv[0]);
        return -1;
    }

    return 0;
}
