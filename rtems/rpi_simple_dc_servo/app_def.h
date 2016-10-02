#ifndef _APP_DEF_H
#define _APP_DEF_H

#ifndef COND_EXTERN
  #ifdef CONFIGURE_INIT
    #define COND_EXTERN
  #else
    #define COND_EXTERN extern
  #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define APP_VER_ID  "rpi_simple_dc_servo"

COND_EXTERN rtems_id   Task_1_id;           /* Task 1 id */
COND_EXTERN rtems_name Task_1_name;         /* Task 1 name */

rtems_task Task_1(
  rtems_task_argument argument
);

void bad_rtems_status(rtems_status_code status, int fail_level, const char *text);

static inline
void check_rtems_status(rtems_status_code status, int fail_level, const char *text)
{
   if(!rtems_is_status_successful(status))
     bad_rtems_status(status, fail_level, text);
}

#define TASK_1_PRIORITY     30
#define SHELL_TASK_PRIORITY 50

int servo_setpwm_forshell(int argc, char **argv);
int servo_readirc_forshell(int argc, char **argv);
int servo_runspeed_forshell(int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif /*_APP_DEF_H*/
