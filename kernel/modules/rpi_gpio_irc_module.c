/*
 *  file: rpi_gpio_irc_module.c
 *
 *  Driver for processing events on GPIO inputs and evaluation
 *  quadrature encoded signals to position value
 *
 *  Copyright (C) 2014 Radek Meciar
 *  Copyright (C) 2014 Pavel Pisa
 *
 *  More information in bachelor thesis
 *    Motor control with Raspberry Pi board and Linux
 *    https://support.dce.felk.cvut.cz/mediawiki/images/1/10/Bp_2014_meciar_radek.pdf
 *  Supervisor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
Linux generic GPIO infrastructue.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

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
	atomic_t used_count;
	volatile uint32_t position;

	volatile char prev_phase;
	volatile char direction;

	int irc_gpio[4];

	const char *irc_gpio_name[4];

	unsigned int irc_irq_num[4];
};

struct gpio_irc_state gpio_irc_0 = {
	.irc_gpio =      {IRC1_GPIO, IRC2_GPIO, IRC3_GPIO, IRC4_GPIO},
	.irc_gpio_name = {IRC1_NAME, IRC2_NAME, IRC3_NAME, IRC4_NAME},
};

int dev_major;

static struct class *irc_class;

/*
 * irc_irq_handlerAR:
 *	GPIO IRC 1 (= 3) rising edge handler - direction determined from IRC 2 (= 4).
 */
static irqreturn_t irc_irq_handlerAR(int irq, void *dev)
{
	struct gpio_irc_state *ircst = (struct gpio_irc_state *)dev;

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

	if (gpio_get_value(ircst->irc_gpio[1]) == IRC_INPUT_LOW) {
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
 * irc_irq_handlerAF:
 *	GPIO IRC 3 (= 1) faling edge handler - direction determined from IRC 2 (= 4).
 */
static irqreturn_t irc_irq_handlerAF(int irq, void *dev)
{
	struct gpio_irc_state *ircst = (struct gpio_irc_state *)dev;

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

	if (gpio_get_value(ircst->irc_gpio[1]) != IRC_INPUT_LOW) {
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
 * irc_irq_handlerBF:
 *	GPIO IRC 2 (= 4) falling edge handler - direction determined from IRC 1 (= 3).
 */
static irqreturn_t irc_irq_handlerBF(int irq, void *dev)
{
	struct gpio_irc_state *ircst = (struct gpio_irc_state *)dev;

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

	if (gpio_get_value(ircst->irc_gpio[0]) == IRC_INPUT_LOW) {
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
 * irc_irq_handlerBR:
 *	GPIO IRC 4 (= 2) rising edge handler - direction determined from IRC 1 (= 3).
 */
static irqreturn_t irc_irq_handlerBR(int irq, void *dev)
{
	struct gpio_irc_state *ircst = (struct gpio_irc_state *)dev;

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

	if (gpio_get_value(ircst->irc_gpio[0]) != IRC_INPUT_LOW) {
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
 *	file operation processing read systemcall for /dev/irc0 device
 *	it returns accumulated position to the calling process buffer
 */
ssize_t irc_read(struct file *file, char *buffer, size_t length, loff_t *offset)
{
	struct gpio_irc_state *ircst = (struct gpio_irc_state *)file->private_data;
	int bytes_to_copy;
	int ret;
	uint32_t pos;

	if (length < sizeof(uint32_t)) {
		pr_debug("Trying to read less bytes than a irc message,\n");
		pr_debug("this will always return zero.\n");
		return 0;
	}

	pos = ircst->position;

	ret = copy_to_user(buffer, &pos, sizeof(uint32_t));

	buffer += sizeof(uint32_t);

	bytes_to_copy = length-sizeof(uint32_t);
	if (ret)
		return -EFAULT;

	return length - bytes_to_copy;
}

/*
 * irc_open:
 *	file operation called at /dev/irc0 device open
 *	it records number of active device users
 */
int irc_open(struct inode *inode, struct file *file)
{
	int dev_minor = MINOR(file->f_dentry->d_inode->i_rdev);
	struct gpio_irc_state *ircst = &gpio_irc_0;

	if (dev_minor > 0)
		pr_err("There is no hardware support for the device file with minor nr.: %d\n",
			dev_minor);

	atomic_inc(&ircst->used_count);

	file->private_data = ircst;
	return 0;
}

/*
 *irc_relese:
 *	file operation called at /dev/irc0 device close/release time
 */
int irc_relase(struct inode *inode, struct file *file)
{
	struct gpio_irc_state *ircst = (struct gpio_irc_state *)file->private_data;

	if (atomic_dec_and_test(&ircst->used_count))
		pr_debug("Last irc user finished\n");

	return 0;
}

/*
 *Define file operations for device IRC
 */
const struct file_operations irc_fops = {
	.owner = THIS_MODULE,
	.read = irc_read,
	.write = NULL,
/*	.poll = irc_poll,*/
	.open = irc_open,
	.release = irc_relase,
};

void gpio_irc_free_irq_fn(struct gpio_irc_state *ircst)
{
	int i;

	for (i = 0; i < 4; i++)
		free_irq(ircst->irc_irq_num[i], ircst);
}

void gpio_irc_free_fn(struct gpio_irc_state *ircst)
{
	int i;

	for (i = 0; i < 4; i++)
		gpio_free(ircst->irc_gpio[i]);
}

/*
 * gpio_irc_setup_inputs:
 *	Configure inputs as sources and connect interrupt handlers
 *	GPIO 2, 3, 4, 23 and 24 are configured as inputs
 */
int gpio_irc_setup_inputs(struct gpio_irc_state *ircst)
{
	int i;

	for (i = 0; i < 4; i++) {
		if (gpio_request(ircst->irc_gpio[i], ircst->irc_gpio_name[i]) != 0) {
			pr_err("failed request %s\n", ircst->irc_gpio_name[i]);
			goto error_gpio_request;
		}
	}

	for (i = 0; i < 4; i++) {
		if (gpio_direction_input(ircst->irc_gpio[i]) != 0) {
			pr_err("failed set direction input %s\n", ircst->irc_gpio_name[i]);
			gpio_irc_free_fn(ircst);
			return (-1);
		}
	}

	return 0;

error_gpio_request:

	while (i > 0)
		gpio_free(ircst->irc_gpio[--i]);

	return -1;
}

/*
 * gpio_irc_init:
 *	Module initialization.
 */
static int gpio_irc_init(void)
{
	int i;
	int res;
	int dev_minor = 0;
	int pom = 0;
	struct gpio_irc_state *ircst = &gpio_irc_0;
	struct device *this_dev;

	pr_notice("gpio_irc init started\n");
	pr_notice("variant without table (4x IRQ on 4 GPIO) - FAST\n");
	pr_notice("for peripheral variant 2\n");

	irc_class = class_create(THIS_MODULE, DEVICE_NAME);
	res = register_chrdev(dev_major, DEVICE_NAME, &irc_fops);
	if (res < 0) {
		pr_err("Error registering driver.\n");
		class_destroy(irc_class);
		return -ENODEV;
		/*goto register_error;*/
	}
	if (dev_major == 0)
		dev_major = res;

	this_dev = device_create(irc_class, NULL, MKDEV(dev_major, dev_minor),
				NULL,  "irc%d", dev_minor);

	if (IS_ERR(this_dev)) {
		pr_err("problem to create device \"irc%d\" in the class \"irc\"\n",
			dev_minor);
		return (-1);
	}

	pom = gpio_irc_setup_inputs(ircst);
	if (pom == -1) {
		pr_err("Inicializace GPIO se nezdarila");
		return (-1);
	}

	ircst->prev_phase = -1;

	for (i = 0; i < 4; i++) {
		int irq_num;

		irq_num = gpio_to_irq(ircst->irc_gpio[i]);
		if (irq_num < 0) {
			pr_err("failed get IRQ number %s\n", ircst->irc_gpio_name[i]);
			gpio_irc_free_fn(ircst);
			return (-1);
		}
		ircst->irc_irq_num[i] = (unsigned int)irq_num;
	}

	if (request_irq(ircst->irc_irq_num[0], irc_irq_handlerAR,
			IRQF_TRIGGER_RISING, "irc1_irqAS", ircst) != 0) {
		pr_err("failed request IRQ for %s\n", ircst->irc_gpio_name[0]);
		gpio_irc_free_fn(ircst);
		return (-1);
	}
	if (request_irq(ircst->irc_irq_num[2], irc_irq_handlerAF,
			IRQF_TRIGGER_FALLING, "irc3_irqAN", ircst) != 0) {
		pr_err("failed request IRQ for %s\n", ircst->irc_gpio_name[2]);
		free_irq(ircst->irc_irq_num[0], ircst);
		gpio_irc_free_fn(ircst);
		return (-1);
	}
	if (request_irq(ircst->irc_irq_num[1], irc_irq_handlerBF,
			IRQF_TRIGGER_FALLING, "irc2_irqBS", ircst) != 0) {
		pr_err("failed request IRQ for %s\n", ircst->irc_gpio_name[1]);
		free_irq(ircst->irc_irq_num[0], ircst);
		free_irq(ircst->irc_irq_num[2], ircst);
		gpio_irc_free_fn(ircst);
		return (-1);
	}
	if (request_irq(ircst->irc_irq_num[3], irc_irq_handlerBR,
			IRQF_TRIGGER_RISING, "irc4_irqBN", ircst) != 0) {
		pr_err("failed request IRQ for %s\n", ircst->irc_gpio_name[3]);
		free_irq(ircst->irc_irq_num[0], ircst);
		free_irq(ircst->irc_irq_num[2], ircst);
		free_irq(ircst->irc_irq_num[1], ircst);
		gpio_irc_free_fn(ircst);
		return (-1);
	}
	pr_notice("gpio_irc init done\n");
	return 0;
}

/*
 * gpio_irc_exist:
 *	Called when module is removed.
 */
static void gpio_irc_exit(void)
{
	struct gpio_irc_state *ircst = &gpio_irc_0;
	int dev_minor = 0;

	gpio_irc_free_irq_fn(ircst);
	gpio_irc_free_fn(ircst);
	device_destroy(irc_class, MKDEV(dev_major, dev_minor));
	class_destroy(irc_class);
	unregister_chrdev(dev_major, DEVICE_NAME);

	pr_notice("gpio_irc modul closed\n");
}

module_init(gpio_irc_init);
module_exit(gpio_irc_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
MODULE_DESCRIPTION("gpio_irc module for incremetal/quadrature signals input processing");
MODULE_AUTHOR("Radek Meciar");
