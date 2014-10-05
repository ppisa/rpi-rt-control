/*
 *  file: rpi_gpio_irc_module.c
 *
 *  Driver for processing events on GPIO inputs and evaluation
 *  quadrature encoded signals to position value
 *
 *  Copyright (C) 2014 Radek Meciar
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
#include <asm/uaccess.h>
#include <linux/device.h>

#define IRC1	23 /* GPIO 3 -> IRC channel A */
#define IRC3	24

#define IRC2	7 /* GPIO 2 -> IRC channel B */
#define IRC4 	8

/* #define IRQ	25 input not used for this variant of processing */

#define IRC1_name	"GPIO23_irc1_chA"
#define IRC2_name	"GPIO7_irc2_chB"
#define IRC3_name	"GPIO24_irc3_chA"
#define IRC4_name	"GPI08_irc4_chB"
/* #define IRQ_name	"GPIO23_irq" not used */

#define LEFT	-1
#define RIGHT	1

#define HIGH	1
#define LOW	0

#define DEVICE_NAME 	"irc"


atomic_t used_count;
volatile uint32_t position = 0;

volatile char prev_val = 0;
volatile char direction = 1;

int dev_major=0;

int irc1_irq_num = 0;
int irc2_irq_num = 0;
int irc3_irq_num = 0;
int irc4_irq_num = 0;

static struct class *irc_class;

/*
irc_irq_handlerAR:
	GPIO IRC 1 (= 3) rising edge handler - direction determined from IRC 2 (= 4).
*/
static irqreturn_t irc_irq_handlerAR(int irq, void *dev){
	
	if(direction == RIGHT){
		if(prev_val == 4){
			position++;
			prev_val = 1;
			return IRQ_HANDLED;
		}
	}else{
		if(prev_val == 3){
			position--;
			prev_val = 1;
			return IRQ_HANDLED;
		}
	}
		
	if(gpio_get_value(IRC2) == HIGH){
		position++;
		direction = RIGHT;
	}else{
		position--;
		direction = LEFT;
	}
	prev_val = 1;
        return IRQ_HANDLED;
} /* irc_irq_handlerAR */

/*
irc_irq_handlerAF:
	GPIO IRC 3 (= 1) faling edge handler - direction determined from IRC 2 (= 4).
*/
static irqreturn_t irc_irq_handlerAF(int irq, void *dev){

	if(direction == RIGHT){
		if(prev_val == 3){
			position++;
			prev_val = 2;
			return IRQ_HANDLED;
		}
	}else{
		if(prev_val == 4){
			position--;
			prev_val = 2;
			return IRQ_HANDLED;
		}
	}
		
	if(gpio_get_value(IRC2) == LOW){
		position++;
		direction = RIGHT;
	}else{
		position--;
		direction = LEFT;
	}
	prev_val = 2;
        return IRQ_HANDLED;
} /* irc_irq_handlerAF */

/*
irc_irq_handlerBF:
	GPIO IRC 2 (= 4) falling edge handler - direction determined from IRC 1 (= 3).
*/
static irqreturn_t irc_irq_handlerBF(int irq, void *dev){
	if(direction == RIGHT){
		if(prev_val == 1){
			position++;
			prev_val = 3;
			return IRQ_HANDLED;
		}
	}else{
		if(prev_val == 2){
			position--;
			prev_val = 3;
			return IRQ_HANDLED;
		}
	}
	
	if(gpio_get_value(IRC1) == HIGH){
		position++;
		direction = RIGHT;
	}else{
		position--;
		direction = LEFT;
	}
	prev_val = 3;
        return IRQ_HANDLED;
} /* irc_irq_handlerBF */

/*
irc_irq_handlerBR:
	GPIO IRC 4 (= 2) rising edge handler - direction determined from IRC 1 (= 3).
*/
static irqreturn_t irc_irq_handlerBR(int irq, void *dev){
	if(direction == RIGHT){
		if(prev_val == 2){
			position++;
			prev_val = 4;
			return IRQ_HANDLED;
		}
	}else{
		if(prev_val == 1){
			position--;
			prev_val = 4;
			return IRQ_HANDLED;
		}
	}
		
	if(gpio_get_value(IRC1) == LOW){
		position++;
		direction = RIGHT;
	}else{
		position--;
		direction = LEFT;
	}
	prev_val = 4;
        return IRQ_HANDLED;
} /* irc_irq_handler */

/*
irc_read:
	file operation processing read systemcall for /dev/irc0 device
	it returns accumulated position to the calling process buffer
*/
ssize_t irc_read(struct file *file, char *buffer, size_t length, loff_t *offset) {
	int bytes_to_copy;
	int ret;
	uint32_t pos;

	if (length < sizeof(uint32_t)) {
		printk(KERN_DEBUG "Trying to read less bytes than a irc message, \n");
		printk(KERN_DEBUG "this will always return zero.\n");
		return 0;
	}
	
	pos = position;
	
	ret = copy_to_user(buffer, &pos, sizeof(uint32_t));

	buffer += sizeof(uint32_t);
	
	bytes_to_copy = length-sizeof(uint32_t);
	if(ret)	
		return -EFAULT;

	return length-bytes_to_copy;
} /* irc_read */

/*
irc_open:
	file operation called at /dev/irc0 device open
	it records number of active device users
*/
int irc_open(struct inode *inode, struct file *file) {
	int dev_minor = MINOR(file->f_dentry->d_inode->i_rdev);
	if(dev_minor > 0){
		printk(KERN_ERR "There is no hardware support for the device file with minor nr.: %d\n", dev_minor);
	}
	
	atomic_inc(&used_count);

	file->private_data = NULL;
	return 0;
} /* irc_open */

/*
irc_relese:
	file operation called at /dev/irc0 device close/release time
*/
int irc_relase(struct inode *inode, struct file *file) {

	if(atomic_dec_and_test(&used_count)){
		printk(KERN_DEBUG "Last irc user finished\n");
	}
	
	return 0;
} /* irc_relase */

/*
Define file operations for device IRC
*/
struct file_operations irc_fops={
	.owner=THIS_MODULE,
	.read=irc_read,
	.write=NULL,
/*	.poll=irc_poll,*/
	.open=irc_open,
	.release=irc_relase,
};

void free_irq_fn(void){
		free_irq((unsigned int)irc1_irq_num, NULL);
		free_irq((unsigned int)irc3_irq_num, NULL);
		free_irq((unsigned int)irc2_irq_num, NULL);
		free_irq((unsigned int)irc4_irq_num, NULL);
}

void free_fn(void){
	gpio_free(IRC1);
	gpio_free(IRC2);
	gpio_free(IRC3);
	gpio_free(IRC4);
	/*gpio_free(IRQ); */ /* Not used */
}

/*
gpio_irc_setup_inputs:
	Configure inputs as sources and connect interrupt handlers
	GPIO 2, 3, 4, 23 and 24 are configured as inputs
*/
int gpio_irc_setup_inputs(void){
	if(gpio_request(IRC1, IRC1_name) != 0){
		printk(KERN_ERR "failed request %s\n", IRC1_name);
		return (-1);
	}
	
	if(gpio_request(IRC2, IRC2_name) != 0){
		printk(KERN_ERR "failed request %s\n", IRC2_name);
		gpio_free(IRC1);
		return (-1);
	}
	
	if(gpio_request(IRC3, IRC3_name) != 0){
		printk(KERN_ERR "failed request %s\n", IRC3_name);
		gpio_free(IRC1);
		gpio_free(IRC2);
		return (-1);
	}
	
	if(gpio_request(IRC4, IRC4_name) != 0){
		printk(KERN_ERR "failed request %s\n", IRC4_name);
		gpio_free(IRC1);
		gpio_free(IRC2);
		gpio_free(IRC3);
		return (-1);
	}
	
	/*if(gpio_request(IRQ, IRQ_name) != 0){
		printk(KERN_ERR "failed request GPIO 23\n");
		gpio_free(IRC1);
		gpio_free(IRC2);
		gpio_free(IRC3);
		gpio_free(IRC4);
		return (-1);
	}*/
	
	if(gpio_direction_input(IRC1) != 0){
		printk(KERN_ERR "failed set direction input %s\n", IRC1_name);
		free_fn();
		return (-1);
	}

	if(gpio_direction_input(IRC2) != 0){
		printk(KERN_ERR "failed set direction input %s\n", IRC2_name);
		free_fn();
		return (-1);
	}
	
	if(gpio_direction_input(IRC3) != 0){
		printk(KERN_ERR "failed set direction input %s\n", IRC3_name);
		free_fn();
		return (-1);
	}
	if(gpio_direction_input(IRC4) != 0){
		printk(KERN_ERR "failed set direction input %s\n", IRC4_name);
		free_fn();
		return (-1);
	}
	/*if(gpio_direction_input(IRQ) != 0){
		printk(KERN_ERR "failed set direction input GPIO 23\n");
		free_fn();
		return (-1);
	}*/
	return 0;
}

/*
gpio_irc_init:
	inicializacni metoda modulu
*/
static int gpio_irc_init(void) {
	int res;
	int dev_minor = 0;
	int pom = 0;
	
	struct device *this_dev;
	
	printk(KERN_NOTICE "gpio_irc init started\n");
	printk(KERN_NOTICE "variant without table (4x IRQ on 4 GPIO) - FAST\n");
	printk(KERN_NOTICE "for peripheral variant 2\n");

	irc_class=class_create(THIS_MODULE, DEVICE_NAME);
	res=register_chrdev(dev_major,DEVICE_NAME, &irc_fops);
	if (res<0) {
		printk(KERN_ERR "Error registering driver.\n");
		class_destroy(irc_class);
		return -ENODEV;
		/*goto register_error;*/
	}
	if(dev_major == 0){
		dev_major = res;
	}
	this_dev=device_create(irc_class, NULL, MKDEV(dev_major, dev_minor), NULL,  "irc%d", dev_minor);

	if(IS_ERR(this_dev)){
		printk(KERN_ERR "problem to create device \"irc%d\" in the class \"irc\"\n", dev_minor);
		return (-1);
	}
	
	pom = gpio_irc_setup_inputs();
	if(pom == -1){
		printk(KERN_ERR "Inicializace GPIO se nezdarila");
		return (-1);
	}

	irc1_irq_num = gpio_to_irq(IRC1);
	if(irc1_irq_num < 0){
		printk(KERN_ERR "failed get IRQ number %s\n", IRC1_name);
		free_fn();
		return (-1);
	}
	
	irc2_irq_num = gpio_to_irq(IRC2);
	if(irc2_irq_num < 0){
		printk(KERN_ERR "failed get IRQ number %s\n", IRC2_name);
		free_fn();
		return (-1);
	}
	
	irc3_irq_num = gpio_to_irq(IRC3);
	if(irc3_irq_num < 0){
		printk(KERN_ERR "failed get IRQ number %s\n", IRC3_name);
		free_fn();
		return (-1);
	}
	
	irc4_irq_num = gpio_to_irq(IRC4);
	if(irc4_irq_num < 0){
		printk(KERN_ERR "failed get IRQ number %s\n", IRC4_name);
		free_fn();
		return (-1);
	}
	
	if(request_irq((unsigned int)irc1_irq_num, irc_irq_handlerAR, IRQF_TRIGGER_RISING, "irc1_irqAS", NULL) != 0){
		printk(KERN_ERR "failed request IRQ for %s\n", IRC1_name);
		free_fn();
		return (-1);
	}
	if(request_irq((unsigned int)irc3_irq_num, irc_irq_handlerAF, IRQF_TRIGGER_FALLING, "irc3_irqAN", NULL) != 0){
		printk(KERN_ERR "failed request IRQ for %s\n", IRC3_name);
		free_fn();
		free_irq((unsigned int)irc1_irq_num, NULL);
		return (-1);
	}
	if(request_irq((unsigned int)irc2_irq_num, irc_irq_handlerBF, IRQF_TRIGGER_FALLING, "irc2_irqBS", NULL) != 0){
		printk(KERN_ERR "failed request IRQ for %s\n", IRC2_name);
		free_fn();
		free_irq((unsigned int)irc1_irq_num, NULL);
		free_irq((unsigned int)irc3_irq_num, NULL);
		return (-1);
	}
	if(request_irq((unsigned int)irc4_irq_num, irc_irq_handlerBR, IRQF_TRIGGER_RISING, "irc4_irqBN", NULL) != 0){
		printk(KERN_ERR "failed request IRQ for %s\n", IRC4_name);
		free_fn();
		free_irq((unsigned int)irc1_irq_num, NULL);
		free_irq((unsigned int)irc3_irq_num, NULL);
		free_irq((unsigned int)irc2_irq_num, NULL);
		return (-1);
	}
	printk(KERN_NOTICE "gpio_irc init done\n");
	return 0;
	
} /* gpio_irc_init */

/*
gpio_irc_exist:
	metoda volaná při odstranení modulu
*/
static void gpio_irc_exit(void) {
	int dev_minor = 0;
	free_irq_fn();
	free_fn();
	device_destroy(irc_class, MKDEV(dev_major, dev_minor));
	class_destroy(irc_class);
	unregister_chrdev(dev_major,DEVICE_NAME);
	
	printk(KERN_NOTICE "gpio_irc modul closed\n");
} /* gpio_irc_exit */

module_init(gpio_irc_init);
module_exit(gpio_irc_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
MODULE_DESCRIPTION("gpio_irc module for incremetal/quadrature signals input processing");
MODULE_AUTHOR("Radek Meciar");
