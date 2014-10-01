#MEXFLAGS ?= -f ./mexopts.sh -g

# Build MF624 MEX functions without dependency on hardware access
WITHOUT_HW=y

ifeq ($(WITHOUT_HW),y)
MEXFLAGS += -DWITHOUT_HW=1
else
MF624_COMMON_C_FILES=mf624_SIMULINK.c
endif

all:
	#mex $(MEXFLAGS) sfAnalogInput.c   $(MF624_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfAnalogOutput.c  $(MF624_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfDigitalOutput.c $(MF624_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfDigitalInput.c  $(MF624_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfReadPWM.c       $(MF624_COMMON_C_FILES)
	mex $(MEXFLAGS) sfIRCInput.c      $(MF624_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfPWMOutput.c     $(MF624_COMMON_C_FILES)
	mex $(MEXFLAGS) sfPWMwDirOutput.c $(MF624_COMMON_C_FILES)
