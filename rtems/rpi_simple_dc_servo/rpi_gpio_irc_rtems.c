/*
 *  file: rpi_gpio_irc_rtems.c
 *
 *  RTEMS device driver for processing events on GPIO inputs
 *  and evaluation quadrature encoded signals to position value
 *
 *  Copyright (C) 2016 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 *  Based on same HW and ideas as dested by Radek Meciar's bachelor thesis
 *    Motor control with Raspberry Pi board and Linux
 *    https://support.dce.felk.cvut.cz/mediawiki/images/1/10/Bp_2014_meciar_radek.pdf
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive
 *  for more details.
 */

/*
IRC inputs are mapped to GPIO 7, 8, 23, 24 on RPI P1.
The channel A is mapped to inputs 7 and 8, channel B to 23 and 24.
Mapping of each signal to two inputs (one configured for IRQ on
signal rising edge and another for falling edge)
simplifies evaluation because there is no need to read actual
GPIO values which posses considerable overhead through
RTEMS generic GPIO infrastructue.
*/

#include <stdio.h>
#include <stdlib.h>
#include <rtems.h>
#include <rtems/error.h>
#include <rtems/libio.h>
#include <bsp/gpio.h>
#include <string.h>
#include <stdatomic.h>

#define IRC1_GPIO	23 /* GPIO 3 -> IRC channel A */
#define IRC3_GPIO	24

#if 1
#define IRC2_GPIO	25 /* GPIO 2 -> IRC channel B */
#define IRC4_GPIO	27
#else
#define IRC2_GPIO	7 /* GPIO 2 -> IRC channel B */
#define IRC4_GPIO	8
#endif

/* #define IRQ_GPIO	25 input not used for this variant of processing */

#define IRC1_NAME	"GPIO23_irc1_chA"
#define IRC2_NAME	"GPIO7_irc2_chB"
#define IRC3_NAME	"GPIO24_irc3_chA"
#define IRC4_NAME	"GPI08_irc4_chB"
/* #define IRQ_name	"GPIO23_irq" not used */

#define IRC_DIRECTION_DOWN	-1
#define IRC_DIRECTION_UP	1

#define IRC_INPUT_LOW		0

#define DEVICE_NAME	"irc"

struct gpio_irc_state {
  atomic_int used_count;
  volatile uint32_t position;

  volatile char prev_phase;
  volatile char direction;

  int irc_gpio[4];

  const char *irc_gpio_name[4];

  unsigned int irc_irq_num[4];
};

static struct gpio_irc_state gpio_irc_0 = {
  .irc_gpio =      {IRC1_GPIO, IRC2_GPIO, IRC3_GPIO, IRC4_GPIO},
  .irc_gpio_name = {IRC1_NAME, IRC2_NAME, IRC3_NAME, IRC4_NAME},
};

/*
 * drv_gpio_irc_irq_handlerAR:
 *  GPIO IRC 1 (= 3) rising edge handler - direction determined from IRC 2 (= 4).
 */
static rtems_gpio_irq_state drv_gpio_irc_irq_handlerAR(void * arg)
{
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)arg;

  if (ircst->prev_phase == 0) {
    ircst->position++;
    ircst->prev_phase = 1;
    ircst->direction = IRC_DIRECTION_UP;
    return IRQ_HANDLED;
  }
  if (ircst->prev_phase == 3) {
    ircst->position--;
    ircst->prev_phase = 2;
    ircst->direction = IRC_DIRECTION_DOWN;
    return IRQ_HANDLED;
  }

  if (rtems_gpio_get_value(ircst->irc_gpio[1]) == IRC_INPUT_LOW) {
    ircst->position++;
    ircst->prev_phase = 1;
    ircst->direction = IRC_DIRECTION_UP;
  } else {
    ircst->position--;
    ircst->prev_phase = 2;
    ircst->direction = IRC_DIRECTION_DOWN;
  }
  return IRQ_HANDLED;
}

/*
 * drv_gpio_irc_irq_handlerAF:
 *  GPIO IRC 3 (= 1) faling edge handler - direction determined from IRC 2 (= 4).
 */
static rtems_gpio_irq_state drv_gpio_irc_irq_handlerAF(void * arg)
{
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)arg;

  if (ircst->prev_phase == 2) {
    ircst->position++;
    ircst->prev_phase = 3;
    ircst->direction = IRC_DIRECTION_UP;
    return IRQ_HANDLED;
  }
  if (ircst->prev_phase == 1) {
    ircst->position--;
    ircst->prev_phase = 0;
    ircst->direction = IRC_DIRECTION_DOWN;
    return IRQ_HANDLED;
  }

  if (rtems_gpio_get_value(ircst->irc_gpio[1]) != IRC_INPUT_LOW) {
    ircst->position++;
    ircst->prev_phase = 3;
    ircst->direction = IRC_DIRECTION_UP;
  } else {
    ircst->position--;
    ircst->prev_phase = 0;
    ircst->direction = IRC_DIRECTION_DOWN;
  }
  return IRQ_HANDLED;
}

/*
 * drv_gpio_irc_irq_handlerBF:
 *  GPIO IRC 2 (= 4) falling edge handler - direction determined from IRC 1 (= 3).
 */
static rtems_gpio_irq_state drv_gpio_irc_irq_handlerBF(void * arg)
{
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)arg;

  if (ircst->prev_phase == 3) {
    ircst->position++;
    ircst->prev_phase = 0;
    ircst->direction = IRC_DIRECTION_UP;
    return IRQ_HANDLED;
  }
  if (ircst->prev_phase == 2) {
    ircst->position--;
    ircst->prev_phase = 1;
    ircst->direction = IRC_DIRECTION_DOWN;
    return IRQ_HANDLED;
  }

  if (rtems_gpio_get_value(ircst->irc_gpio[0]) == IRC_INPUT_LOW) {
    ircst->position++;
    ircst->prev_phase = 0;
    ircst->direction = IRC_DIRECTION_UP;
  } else {
    ircst->position--;
    ircst->prev_phase = 1;
    ircst->direction = IRC_DIRECTION_DOWN;
  }
  return IRQ_HANDLED;
}

/*
 * drv_gpio_irc_irq_handlerBR:
 *  GPIO IRC 4 (= 2) rising edge handler - direction determined from IRC 1 (= 3).
 */
static rtems_gpio_irq_state drv_gpio_irc_irq_handlerBR(void * arg)
{
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)arg;

  if (ircst->prev_phase == 1) {
    ircst->position++;
    ircst->prev_phase = 2;
    ircst->direction = IRC_DIRECTION_UP;
    return IRQ_HANDLED;
  }
  if (ircst->prev_phase == 0) {
    ircst->position--;
    ircst->prev_phase = 3;
    ircst->direction = IRC_DIRECTION_DOWN;
    return IRQ_HANDLED;
  }

  if (rtems_gpio_get_value(ircst->irc_gpio[0]) != IRC_INPUT_LOW) {
    ircst->position++;
    ircst->prev_phase = 2;
    ircst->direction = IRC_DIRECTION_UP;
  } else {
    ircst->position--;
    ircst->prev_phase = 3;
    ircst->direction = IRC_DIRECTION_DOWN;
  }
  return IRQ_HANDLED;
}

/*
 * irc_read:
 *  file operation processing read systemcall for /dev/irc0 device
 *  it returns accumulated position to the calling process buffer
 */

static rtems_device_driver drv_gpio_irc_read(rtems_device_major_number major,
                               rtems_device_minor_number minor, void *arg)
{
  rtems_libio_rw_args_t *parms = (rtems_libio_rw_args_t *) arg;
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)parms->iop->data1;

  if ( minor != 0)
    return RTEMS_UNSATISFIED;

  if ( parms->count < sizeof(uint32_t) ) {
    parms->bytes_moved = 0;
    return RTEMS_SUCCESSFUL;
  }

  *(uint32_t*)parms->buffer = ircst->position;

  parms->bytes_moved = sizeof(uint32_t);

  return RTEMS_SUCCESSFUL;
}

static rtems_device_driver drv_gpio_irc_write(rtems_device_major_number major,
                               rtems_device_minor_number minor, void *arg)
{
  rtems_libio_rw_args_t *parms = (rtems_libio_rw_args_t *) arg;
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)parms->iop->data1;
  (void)ircst;

  if ( minor != 0)
    return RTEMS_UNSATISFIED;

  parms->bytes_moved = 0;

  return RTEMS_IO_ERROR;
}


/*
 * irc_open:
 *  file operation called at /dev/irc0 device open
 *  it records number of active device users
 */
static rtems_device_driver drv_gpio_irc_open(rtems_device_major_number major,
                               rtems_device_minor_number minor, void *arg)
{
  rtems_libio_open_close_args_t *parms = (rtems_libio_open_close_args_t *) arg;
  struct gpio_irc_state *ircst;

  parms->iop->data1 = &gpio_irc_0;

  ircst = (struct gpio_irc_state *)parms->iop->data1;

  if ( minor != 0)
    return RTEMS_UNSATISFIED;

  if (atomic_fetch_add(&ircst->used_count, 1) == 0)
    printf("the first irc user open\n");

  return RTEMS_SUCCESSFUL;
}

static rtems_device_driver drv_gpio_irc_close(rtems_device_major_number major,
                               rtems_device_minor_number minor, void *arg)
{
  rtems_libio_open_close_args_t *parms = (rtems_libio_open_close_args_t *) arg;
  struct gpio_irc_state *ircst = (struct gpio_irc_state *)parms->iop->data1;

  if ( minor != 0)
    return RTEMS_UNSATISFIED;

  if (atomic_fetch_sub(&ircst->used_count, 1) == 1)
    printf("the last irc user finished\n");

  return RTEMS_SUCCESSFUL;
}

static rtems_device_driver drv_gpio_irc_ioctl(rtems_device_major_number major,
                               rtems_device_minor_number minor, void *arg)
{
  rtems_libio_ioctl_args_t *parms = (rtems_libio_ioctl_args_t *) arg;
  (void)parms;

  if ( minor != 0)
    return RTEMS_UNSATISFIED;

  return RTEMS_SUCCESSFUL;
}

static void drv_gpio_irc_free_irq_fn(struct gpio_irc_state *ircst)
{
  rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[0], drv_gpio_irc_irq_handlerAR, ircst);
  rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[2], drv_gpio_irc_irq_handlerAF, ircst);
  rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[1], drv_gpio_irc_irq_handlerBF, ircst);
  rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[3], drv_gpio_irc_irq_handlerBR, ircst);
}

static void drv_gpio_irc_free_fn(struct gpio_irc_state *ircst)
{
  int i;

  for (i = 0; i < 4; i++)
    rtems_gpio_release_pin(ircst->irc_gpio[i]);
}

/*
 * drv_gpio_irc_setup_inputs:
 *  Configure inputs as sources and connect interrupt handlers
 *  GPIO 2, 3, 4, 23 and 24 are configured as inputs
 */
static int drv_gpio_irc_setup_inputs(struct gpio_irc_state *ircst)
{
  int i;

  for (i = 0; i < 4; i++) {
    if (rtems_gpio_request_pin(ircst->irc_gpio[i], DIGITAL_INPUT,
        false, false, NULL) != RTEMS_SUCCESSFUL) {
      printf("failed request %s\n", ircst->irc_gpio_name[i]);
      goto error_gpio_request;
    }
  }

  return 0;

error_gpio_request:

  while (i > 0)
    rtems_gpio_release_pin(ircst->irc_gpio[--i]);

  return -1;
}

static rtems_status_code drv_gpio_irc_initialize(rtems_device_major_number major,
                                          rtems_device_minor_number minor, void *arg)
{
  int i;
  int res;
  struct gpio_irc_state *ircst = &gpio_irc_0;
  rtems_status_code status;
  char device_name[20];

  printf("gpio_irc init started\n");
  printf("variant without table (4x IRQ on 4 GPIO) - FAST\n");
  printf("for peripheral variant 2\n");

  snprintf(device_name, sizeof(device_name), "/dev/%s%d",
           DEVICE_NAME, (int)minor);


  if ((status = rtems_io_register_name(device_name, major, minor)) !=
       RTEMS_SUCCESSFUL) {
    return status;
  }

  for (i = 0; i < 4; i++) {
    ircst->irc_gpio[i] = gpio_irc_0.irc_gpio[i];
    ircst->irc_gpio_name[i] = gpio_irc_0.irc_gpio_name[i];
  }

  res = drv_gpio_irc_setup_inputs(ircst);
  if (res == -1) {
    printf("Inicializace GPIO se nezdarila");
    return RTEMS_IO_ERROR ;
  }

  ircst->prev_phase = -1;

  if (rtems_gpio_enable_interrupt(ircst->irc_gpio[0], RISING_EDGE,
    UNIQUE_HANDLER, false, drv_gpio_irc_irq_handlerAR, ircst) != RTEMS_SUCCESSFUL) {
    printf("failed request IRQ for %s\n", ircst->irc_gpio_name[0]);
    drv_gpio_irc_free_fn(ircst);
    return RTEMS_IO_ERROR ;
  }
  if (rtems_gpio_enable_interrupt(ircst->irc_gpio[2], FALLING_EDGE,
    UNIQUE_HANDLER, false, drv_gpio_irc_irq_handlerAF, ircst) != RTEMS_SUCCESSFUL) {
    printf("failed request IRQ for %s\n", ircst->irc_gpio_name[2]);
    rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[0], drv_gpio_irc_irq_handlerAR, ircst);
    drv_gpio_irc_free_fn(ircst);
    return RTEMS_IO_ERROR ;
  }
  if (rtems_gpio_enable_interrupt(ircst->irc_gpio[1], FALLING_EDGE,
    UNIQUE_HANDLER, false, drv_gpio_irc_irq_handlerBF, ircst) != RTEMS_SUCCESSFUL) {
    printf("failed request IRQ for %s\n", ircst->irc_gpio_name[1]);
    rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[0], drv_gpio_irc_irq_handlerAR, ircst);
    rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[2], drv_gpio_irc_irq_handlerAF, ircst);
    drv_gpio_irc_free_fn(ircst);
    return RTEMS_IO_ERROR ;
  }
  if (rtems_gpio_enable_interrupt(ircst->irc_gpio[3], RISING_EDGE,
    UNIQUE_HANDLER, false, drv_gpio_irc_irq_handlerBR, ircst) != RTEMS_SUCCESSFUL) {
    printf("failed request IRQ for %s\n", ircst->irc_gpio_name[3]);
    rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[0], drv_gpio_irc_irq_handlerAR, ircst);
    rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[2], drv_gpio_irc_irq_handlerAF, ircst);
    rtems_gpio_interrupt_handler_remove(ircst->irc_gpio[1], drv_gpio_irc_irq_handlerBF, ircst);
    drv_gpio_irc_free_fn(ircst);
    return RTEMS_IO_ERROR ;
  }
  printf("gpio_irc init done\n");

  return RTEMS_SUCCESSFUL;
}

static rtems_device_major_number gpio_irc_major;

static rtems_driver_address_table gpio_irc_driver_table =
{
  drv_gpio_irc_initialize, drv_gpio_irc_open, drv_gpio_irc_close,
  drv_gpio_irc_read, drv_gpio_irc_write, drv_gpio_irc_ioctl
};


rtems_status_code drv_gpio_irc_init(void)
{
  rtems_status_code status;

  status = rtems_io_register_driver(0, &gpio_irc_driver_table, &gpio_irc_major);
  if (status != RTEMS_SUCCESSFUL){
     printf("caninit: rtems_io_register_driver %s\n",rtems_status_text(status));
     return status;
  }
  return RTEMS_SUCCESSFUL;
}

#if 0
/*
 * gpio_irc_exist:
 *  Called when module is removed.
 */
static void gpio_irc_exit(void)
{
  struct gpio_irc_state *ircst = &gpio_irc_0;
  int dev_minor = 0;

  drv_gpio_irc_free_irq_fn(ircst);
  drv_gpio_irc_free_fn(ircst);
  device_destroy(irc_class, MKDEV(dev_major, dev_minor));
  class_destroy(irc_class);
  unregister_chrdev(dev_major, DEVICE_NAME);

  printf("gpio_irc modul closed\n");
}
#endif