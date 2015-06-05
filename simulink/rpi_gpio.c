/*
 * Bidirectional pwm_base on Raspberry Pi module
 *
 * Copyright (C) 2014 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * Copyright (C) 2014 Radek Meciar
 *
 * Department of Control Engineering
 * Faculty of Electrical Engineering
 * Czech Technical University in Prague (CTU)
 *
 * Next exception is granted in addition to GPL.
 * Instantiating or linking compiled version of this code
 * to produce an application image/executable, does not
 * by itself cause the resulting application image/executable
 * to be covered by the GNU General Public License.
 * This exception does not however invalidate any other reasons
 * why the executable file might be covered by the GNU Public License.
 * Publication of enhanced or derived S-function files is required
 * although.
 */

#include "rpi_gpio.h"

/*Based on bachelor thesis work Meciar Radek: Motor control with Raspberry Pi board and Linux*/

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#define BASE		0x20000000		/* registers common base address */
#define GPIO_BASE 	(BASE + 0x200000)	/* gpio_base registers base address */
#define PWM_BASE	(BASE + 0x20C000)	/* pwm_base registers base address */
#define CLK_BASE	(BASE + 0x101000)	/* clk_base register base address */

#define PAGE_SIZE 	(4*1024)
#define BLOCK_SIZE	(4*1024)

rpi_registers_mapping_t rpi_registers_mapping;

/* Based on infromation from: http://elinux.org/RPi_Low-level_peripherals */

static int rpi_gpio_fnc_setup(unsigned gpio, unsigned fnc)
{
    volatile unsigned *reg;
    unsigned mask;

    if (gpio >= 32)
        return -1;

    mask = 7 << ((gpio % 10) * 3);
    fnc = fnc << ((gpio % 10) * 3);
    fnc &= mask;
    reg = rpi_registers_mapping.gpio_base + (gpio /10);

    if ((*reg & mask) != fnc) {
      *reg &= ~mask;
      *reg |= fnc;
    }
    return 0;
}

/* Configure gpio_base pin for input */
int rpi_gpio_direction_input(unsigned gpio)
{
    return rpi_gpio_fnc_setup(gpio, 0);
}

/* Configure gpio_base pin for output */
int rpi_gpio_direction_output(unsigned gpio, int value)
{
    if (gpio >= 32)
        return -1;

    rpi_gpio_set_value(gpio, value);
    if (rpi_gpio_fnc_setup(gpio, 1) < 0)
        return -1;
    rpi_gpio_set_value(gpio, value);
    return 0;
}

/* Configure gpio_base pin for alternate function */
int rpi_gpio_alt_fnc(unsigned gpio, int alt_fnc)
{
    return rpi_gpio_fnc_setup(gpio, alt_fnc <= 3? alt_fnc + 4: alt_fnc == 4? 3: 2);
}

/*
peripheral_registers_map:

Maps registers into virtual address space and sets  *gpio_base, *pwm_base, *clk_base poiners
*/
int rpi_peripheral_registers_map(void)
{
    rpi_registers_mapping_t *rrmap = &rpi_registers_mapping;
    if (rrmap->mapping_initialized)
        return rrmap->mapping_initialized;

    rrmap->mapping_initialized = -1;

    if ((rrmap->mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        return -1;
    }

    rrmap->gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, rrmap->mem_fd, GPIO_BASE);

    if (rrmap->gpio_map == MAP_FAILED) {
        return -1;
    }

    rrmap->gpio_base = (volatile unsigned *)rrmap->gpio_map;

    rrmap->pwm_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, rrmap->mem_fd, PWM_BASE);

    if (rrmap->pwm_map == MAP_FAILED) {
        return -1;
    }

    rrmap->pwm_base = (volatile unsigned *)rrmap->pwm_map;

    rrmap->clk_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, rrmap->mem_fd, CLK_BASE);

    if (rrmap->clk_map == MAP_FAILED) {
        return -1;
    }

    rrmap->clk_base = (volatile unsigned *)rrmap->clk_map;

    close(rrmap->mem_fd);

    rrmap->mapping_initialized = 1;

    return rrmap->mapping_initialized;
} /* peripheral_registers_map */
