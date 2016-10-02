/*  Init
 *
 *  This routine is the initialization task for this test program.
 *  It is called from init_exec and has the responsibility for creating
 *  and starting the tasks that make up the test.  If the time of day
 *  clock is required for the test, it should also be set to a known
 *  value by this function.
 *
 *  Input parameters:  NONE
 *
 *  Output parameters:  NONE
 *
 *  COPYRIGHT (c) 1989-1999.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id: init.c,v 1.12.4.1 2003/09/04 18:46:30 joel Exp $
 */

#define CONFIGURE_INIT
#include "system.h"
#include "app_def.h"
#include <stdio.h>
#include <stdlib.h>
#include <rtems/error.h>
#include <rtems/monitor.h>
#include <rtems/shell.h>
#include <bsp/gpio.h>

#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#define CONFIGURE_SHELL_MOUNT_MSDOS

#include <rtems/shellconfig.h>

#define BUILD_VERSION_STRING(major,minor,patch) \
        __XSTRING(major) "." __XSTRING(minor) "." __XSTRING(patch)

void 
bad_rtems_status(rtems_status_code status, int fail_level, const char *text)
{
  printf("ERROR: %s status %s", text, rtems_status_text(status));
  status = rtems_task_delete( RTEMS_SELF );
}

int testcmd_forshell(int argc, char **argv)
{
  int i;
  printf("Command %s called\n",argv[0]);
  for(i=1;i<argc;i++)
    if(argv[i])
      printf("%s",argv[i]);
  printf("\n");
  return 0;
}

rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_status_code status;

  printf( "\n\nRTEMS v "
          BUILD_VERSION_STRING(__RTEMS_MAJOR__ ,__RTEMS_MINOR__ ,__RTEMS_REVISION__)
	  "\n");

  rtems_monitor_init(RTEMS_MONITOR_SUSPEND|RTEMS_MONITOR_GLOBAL);
  /*rtems_capture_cli_init (0);*/

  printf( "Starting application " APP_VER_ID "\n" );

  rtems_gpio_initialize();

  printf( "RTEMS GPIO initialized\n" );


  status = drv_gpio_irc_init();
  check_rtems_status(status, 0, "drv_gpio_irc_initialize");


 #if 0
  Task_1_name = rtems_build_name( 'T', 'S', 'K', '1' );

  status = rtems_task_create(
     Task_1_name,
     TASK_1_PRIORITY,
     RTEMS_MINIMUM_STACK_SIZE+0x10000,
     RTEMS_DEFAULT_MODES /*& ~(RTEMS_TIMESLICE_MASK) | RTEMS_TIMESLICE*/,
     RTEMS_DEFAULT_ATTRIBUTES,
     &Task_1_id
  );
  check_rtems_status(status, 0, "rtems_task_create of Task_1");

  status = rtems_task_start( Task_1_id, Task_1, 0 );
  check_rtems_status(status, 0, "rtems_task_start of Task_1\n");
 #endif

  rtems_shell_init("SHLL",RTEMS_MINIMUM_STACK_SIZE+0x1000,
              SHELL_TASK_PRIORITY, "/dev/console", 1, 0, NULL);

  rtems_shell_add_cmd("testcmd", "app",
                "test command for shell",
                testcmd_forshell);

  rtems_shell_add_cmd("setpwm", "app",
                "setpwm <value>",
                servo_setpwm_forshell);

  rtems_shell_add_cmd("readirc", "app",
                "readirc",
                servo_readirc_forshell);

  rtems_shell_add_cmd("runspeed", "app",
                "runspeed <value>",
                servo_runspeed_forshell);

  //rtems_monitor_wakeup();

  status = rtems_task_delete( RTEMS_SELF );

  printf( "*** END OF TEST2 ***\n" );
  exit( 0 );
}
