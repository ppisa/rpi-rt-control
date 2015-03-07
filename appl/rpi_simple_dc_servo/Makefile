ifneq ("$(shell uname -m)","armv6l")
CC = arm-rpi-linux-gnueabihf-gcc
endif

CFLAGS += -Wall -O2 -ggdb
LOADLIBES = -lpthread -lrt

PROGRAM_NAME = rpi_simple_dc_servo
OBJS = rpi_simple_dc_servo.o rpi_bidirpwm.o rpi_gpio.o

all: $(PROGRAM_NAME)

$(PROGRAM_NAME) : $(OBJS)

.PHONY: all clean

clean:
	rm -f $(PROGRAM_NAME) $(OBJS)
