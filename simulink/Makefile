#MEXFLAGS ?= -f ./mexopts.sh -g

# Build MF624 MEX functions without dependency on hardware access
WITHOUT_HW=y

ifeq ($(WITHOUT_HW),y)
MEXFLAGS += -DWITHOUT_HW=1
else
MEX_COMMON_C_FILES=mf624_SIMULINK.c
endif

all:
	#mex $(MEXFLAGS) sfAnalogInput.c   $(MEX_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfAnalogOutput.c  $(MEX_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfDigitalOutput.c $(MEX_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfDigitalInput.c  $(MEX_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfReadPWM.c       $(MEX_COMMON_C_FILES)
	mex $(MEXFLAGS) sfIRCInput.c      $(MEX_COMMON_C_FILES)
	#mex $(MEXFLAGS) sfPWMOutput.c     $(MEX_COMMON_C_FILES)
	mex $(MEXFLAGS) sfPWMwDirOutput.c $(MEX_COMMON_C_FILES)
	mex $(MEXFLAGS) sfPMSMonSPI.c     $(MEX_COMMON_C_FILES)
