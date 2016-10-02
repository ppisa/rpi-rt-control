# Generic directory or leaf node makefile for OCERA make framework

ifndef MAKERULES_DIR
MAKERULES_DIR := $(shell ( old_pwd="" ;  while [ ! -e Makefile.rules ] ; do if [ "$$old_pwd" = `pwd`  ] ; then exit 1 ; else old_pwd=`pwd` ; cd -L .. 2>/dev/null ; fi ; done ; pwd ) )
endif

ifeq ($(MAKERULES_DIR),)
all : default
.DEFAULT::
	@echo -e "\nThe Makefile.rules has not been found in this or parent directory\n"
else
include $(MAKERULES_DIR)/Makefile.rules
endif

