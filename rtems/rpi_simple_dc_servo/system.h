/*  system.h
 *
 *  This include file contains information that is included in every
 *  function in the test set.
 *
 *  COPYRIGHT (c) 1989-1999.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id: system.h,v 1.13.6.1 2003/09/04 18:46:30 joel Exp $
 */

#include <rtems.h>

/* functions */

rtems_task Init(
  rtems_task_argument argument
);

rtems_status_code drv_gpio_irc_init(void);

/* configuration information */

#include <bsp.h> /* for device driver prototypes */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK

#define TICKS_PER_SECOND 1000

#define CONFIGURE_MAXIMUM_TIMERS                 32
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES         32
#define CONFIGURE_MAXIMUM_SEMAPHORES             32
#define CONFIGURE_MAXIMUM_TASKS                  32
#define CONFIGURE_MAXIMUM_PERIODS                4
#define CONFIGURE_MAXIMUM_USER_EXTENSIONS        2
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
/*#define CONFIGURE_MAXIMUM_DRIVERS (CONFIGURE_NUMBER_OF_DRIVERS+10)*/
#define CONFIGURE_MAXIMUM_DRIVERS                32

#ifdef RTEMS_POSIX_API
#define CONFIGURE_MAXIMUM_POSIX_THREADS          32
#define CONFIGURE_MAXIMUM_POSIX_MUTEXES          20
#define CONFIGURE_MAXIMUM_POSIX_SEMAPHORES       10
#define CONFIGURE_MAXIMUM_POSIX_KEYS             4
#define CONFIGURE_MAXIMUM_POSIX_KEY_VALUE_PAIRS  8
#endif /*RTEMS_POSIX_API*/

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_APPLICATION_NEEDS_NULL_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_ZERO_DRIVER

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM
/*#define CONFIGURE_USE_MINIIMFS_AS_BASE_FILESYSTEM*/

#define CONFIGURE_MICROSECONDS_PER_TICK 1000

#define CONFIGURE_EXTRA_TASK_STACKS     (10 * (RTEMS_MINIMUM_STACK_SIZE + 2 * 1024))

#define CONFIGURE_INIT_TASK_STACK_SIZE  (10*1024)
#define CONFIGURE_INIT_TASK_PRIORITY    120
#define CONFIGURE_INIT_TASK_INITIAL_MODES (RTEMS_PREEMPT | \
                                           RTEMS_NO_TIMESLICE | \
                                           RTEMS_NO_ASR | \
                                           RTEMS_INTERRUPT_LEVEL(0))

#include <rtems/confdefs.h>

/* end of include file */
