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

#include <stdint.h>

#include "rpi_bidirpwm.h"
#include "rpi_gpio.h"

/*Based on bachelor thesis work Meciar Radek: Motor control with Raspberry Pi board and Linux*/

/* CTL - pwm_base control register, select clock source and pwm_base mode */
#define PWM_CTL		*(rpi_registers_mapping.pwm_base)

/* RNG1 - pwm_base divider register, cycle count and duty resolution */
#define PWM_RNG1	*(rpi_registers_mapping.pwm_base+4)

/* DAT1 - pwm_base duty value register*/
#define PWM_DAT1	*(rpi_registers_mapping.pwm_base+5)

/* CLK_CNTL - control clock for pwm_base (on/off) */
#define PWM_CLK_CNTL	*(rpi_registers_mapping.clk_base+40)

/* CLK_DIV - divisor (bits 11:0 are *quantized* floating part, 31:12 integer */
#define PWM_CLK_DIV	*(rpi_registers_mapping.clk_base+41)

#define LEFT		1
#define RIGHT		-1

#define GPIO_PWM	18	/* pwm_base pin corresponding gpio_base number (ALT fn 5) */
#define GPIO_DIR	22

static int pwm_period = 4000;

/*
pwm_output_init:

Setup pwm_base frequency to 25kHz
duty selectable in range 0-4000 (0-100%)
initial PWM_MODE 0%
*/
static void rpi_pwm_output_init(void){
    rpi_gpio_direction_output(GPIO_PWM, 0);
    rpi_gpio_alt_fnc(GPIO_PWM, 5);
    /* initial PWM_MODE set */
    PWM_CTL = 0;
    /* disable pwm_base */
    PWM_CLK_CNTL = (PWM_CLK_CNTL&~0x10)|0x5a000000;
    /* disable clock */
    while(PWM_CLK_CNTL&0x80);
    /* wait while BUSY not 0 */
    PWM_CLK_DIV = 0x5a000000|(5<<12);
    /* divider setup */
    PWM_CLK_CNTL = 0x5a000016;
    /* chanel enable and set source to PLLD (500MHz) */
    while(!(PWM_CLK_CNTL&0x80));
    /* wait for BUSY to signal 1 */
    PWM_RNG1 = pwm_period;
    /* external counter limit - duty levels */
    PWM_DAT1 = 0;
    /* initial duty PWM_MODE 0 */
    PWM_CTL = 0x81;
    /* enable MSEN=1 & ENA=1 */
} /* inicializace pwm_base */

/*
pwm_base direction bit control
*/
static void rpi_bidirpwm_output_direction_set(int action){
    rpi_gpio_set_value(GPIO_DIR, action >= 0? 0: 1);
} /* pwm_output_direction_set */

/*
pwm_output_set_width:

set duty PWM_MODE and direction bit
input range limited to vlaue from -4000 to 4000
*/
static void rpi_bidirpwm_output_set_width(int value){
    if(value < 0){
        rpi_bidirpwm_output_direction_set(-1);
        value = -value;
    }else if(value > 0){
        rpi_bidirpwm_output_direction_set(1);
    }

    if(value > pwm_period){
        PWM_DAT1 = pwm_period;
    }else if(value < 0){
        PWM_DAT1 = 0;
    }else{
        PWM_DAT1 = value;
    }
} /* pwm_output_set_width */

int rpi_bidirpwm_init(void)
{
    if (rpi_peripheral_registers_map() <= 0) {
        return -1;
    }
    rpi_pwm_output_init();
    rpi_gpio_direction_output(GPIO_DIR, 0);

    return 0;
}

int rpi_bidirpwm_set(int value)
{
    if (rpi_registers_mapping.mapping_initialized <= 0)
            return -1;

    rpi_bidirpwm_output_set_width(value);

    return 0;
}
