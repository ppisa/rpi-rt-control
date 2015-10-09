/*
 * S-function to support PWM on Raspberry Pi module
 *
 * Copyright (C) 2014 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * Copyright (C) 2014 Radek Meciar
 *
 * Department of Control Engineering
 * Faculty of Electrical Engineering
 * Czech Technical University in Prague (CTU)
 *
 * The S-Function for ERT Linux can be distributed in compliance
 * with GNU General Public License (GPL) version 2 or later.
 * Other licence can negotiated with CTU.
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
 *
 * Linux ERT code is available from
 *    http://rtime.felk.cvut.cz/gitweb/ert_linux.git
 * More CTU Linux target for Simulink components are available at
 *    http://lintarget.sourceforge.net/
 *
 * sfuntmpl_basic.c by The MathWorks, Inc. has been used to accomplish
 * required S-function structure.
 */


#define S_FUNCTION_NAME  sfPWMwDirOutput
#define S_FUNCTION_LEVEL 2

/*
 * The S-function has next parameters
 *
 * PWM Channel number
 * PWM Frequency   - PWM_MODE -1 for external input
 * Associated digital output for direction
 */

#define PRM_CHANNEL(S)          (mxGetScalar(ssGetSFcnParam(S, 0)))
#define PRM_FREQUENCY(S)        (mxGetScalar(ssGetSFcnParam(S, 1)))
#define PRM_DIR_DO_BIT(S)       (mxGetScalar(ssGetSFcnParam(S, 2)))

#define PRM_COUNT                      3

#define IWORK_IDX_CHANNEL              0
#define IWORK_IDX_DIR_DO_BIT           1
#define IWORK_IDX_PWM_MODE             2
#define IWORK_IDX_USE_FREQUENCY_INPUT  3
#define IWORK_IDX_LAST_MODE            4
enum {PWM_MODE_ZERO = 0, PWM_MODE_PLUS_PWM = 1,
                          PWM_MODE_MINUS_PWM = -1};

#define IWORK_COUNT                    5

#define IWORK_CHANNEL(S)               (ssGetIWork(S)[IWORK_IDX_CHANNEL])
#define IWORK_DIR_DO_BIT(S)            (ssGetIWork(S)[IWORK_IDX_DIR_DO_BIT])
#define IWORK_PWM_MODE(S)              (ssGetIWork(S)[IWORK_IDX_PWM_MODE])
#define IWORK_USE_FREQUENCY_INPUT(S)   (ssGetIWork(S)[IWORK_IDX_USE_FREQUENCY_INPUT])
#define IWORK_LAST_MODE(S)             (ssGetIWork(S)[IWORK_IDX_LAST_MODE])

#define RWORK_IDX_BASE_FREQUENCY       0

#define RWORK_COUNT                    1

#define RWORK_BASE_FREQUENCY(S)        (ssGetRWork(S)[RWORK_IDX_BASE_FREQUENCY])


/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#define CTR_MAX_PWM_CHANNEL            0

#ifndef WITHOUT_HW

/*Based on bachelor thesis work Meciar Radek: Motor control with Raspberry Pi board and Linux*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
/*#include <inttypes.h>*/
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rpi_gpio.h"

#define PWM_CTL		(*(rpi_registers_mapping.pwm_base + 0))
	/* CTL - PWM control register, select clock source and PWM mode */
#define PWM_RNG1	(*(rpi_registers_mapping.pwm_base + 4))
	/* RNG1 - PWM divider register, cycle count and duty resolution */
#define PWM_DAT1	(*(rpi_registers_mapping.pwm_base + 5))
	/* DAT1 - PWM duty value register*/
#define PWM_CLK_CNTL	(*(rpi_registers_mapping.clk_base + 40))
	/* CLK_CNTL - control clock for PWM (on/off) */
#define PWM_CLK_DIV	(*(rpi_registers_mapping.clk_base + 41))
	/* CLK_DIV - divisor (bits 11:0 are *quantized* floating part, 31:12 integer */

#ifndef CLK_PASSWD
#define CLK_PASSWD  (0x5A<<24)
#define CLK_CTL_MASH(x)((x)<<9)
#define CLK_CTL_BUSY    (1 <<7)
#define CLK_CTL_KILL    (1 <<5)
#define CLK_CTL_ENAB    (1 <<4)
#define CLK_CTL_SRC(x) ((x)<<0)

#define CLK_DIV_DIVI(x) ((x)<<12)
#define CLK_DIV_DIVF(x) ((x)<< 0)
#endif /*CLK_PASSWD*/

#define LEFT		1
#define RIGHT		-1

#define GPIO_PWM	18			/* PWM pin corresponding GPIO number (ALT fn 5) */
#define GPIO_DIR	22

/*
pwm_output_init:

Setup PWM frequency to 25kHz
duty selectable in range 0-4000 (0-100%)
initial PWM_MODE 0%
*/
void pwm_output_init(void){
    rpi_gpio_direction_output(GPIO_PWM, 0);
    rpi_gpio_alt_fnc(GPIO_PWM, 5);
    /* initial PWM_MODE set */
    PWM_CTL = 0;
    /* disable PWM */
    PWM_CLK_CNTL = (PWM_CLK_CNTL & ~CLK_CTL_ENAB) | CLK_PASSWD;
    /* disable clock */
    while(PWM_CLK_CNTL & CLK_CTL_BUSY);
    /* wait while BUSY not 0 */
    PWM_CLK_DIV = CLK_DIV_DIVI(5) | CLK_DIV_DIVF(0) | CLK_PASSWD;
    /* divider setup */
    PWM_CLK_CNTL = CLK_CTL_SRC(6) | CLK_CTL_ENAB | CLK_PASSWD;
    /* chanel enable and set source to PLLD (500MHz) */
    while(!(PWM_CLK_CNTL & CLK_CTL_BUSY));
    /* wait for BUSY to signal 1 */
    PWM_RNG1 = 4000;
    /* external counter limit - duty levels */
    PWM_DAT1 = 0;
    /* initial duty PWM_MODE 0 */
    PWM_CTL = 0x81;
    /* enable MSEN=1 & ENA=1 */
} /* inicializace PWM */

/*
PWM direction bit control
*/
void pwm_output_direction_set(int action){
    if(action >=0){
        rpi_gpio_set_value(GPIO_DIR, 0);
    }else{
        rpi_gpio_set_value(GPIO_DIR, 1);
    }
} /* pwm_output_direction_set */

/*
pwm_output_set_width:

set duty PWM_MODE and direction bit
input range limited to vlaue from -4000 to 4000
*/
void pwm_output_set_width(int value){
    if(value < 0){
        pwm_output_direction_set(-1);
        value *= -1;
    }else if(value > 0){
        pwm_output_direction_set(1);
    }

    if(value > 4000){
        PWM_DAT1 = 4000;
    }else if(value < 0){
        PWM_DAT1 = 0;
    }else{
        PWM_DAT1 = value;
    }
} /* pwm_output_set_width */

#endif /*WITHOUT_HW*/

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    mdlCheckParameters verifies new parameter settings whenever parameter
   *    change or are re-evaluated during a simulation. When a simulation is
   *    running, changes to S-function parameters can occur at any time during
   *    the simulation loop.
   */
static void mdlCheckParameters(SimStruct *S)
{
    if ((PRM_CHANNEL(S) < 0) || (PRM_CHANNEL(S) > CTR_MAX_PWM_CHANNEL))
        ssSetErrorStatus(S, "valid PWM channel is 0, 1, 2, or 3");
    if ((PRM_FREQUENCY(S) <= 0) && (PRM_FREQUENCY(S) != -1))
        ssSetErrorStatus(S, "Frequency out of valid range");
    if ((PRM_DIR_DO_BIT(S) < 0) || (PRM_DIR_DO_BIT(S) > 7) )
        ssSetErrorStatus(S, "Invalid direction output specification (0 to 7 supported)");
}
#endif /* MDL_CHECK_PARAMETERS */



/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nInputPorts  = 1;

    ssSetNumSFcnParams(S, PRM_COUNT);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S, "2-parameters requited: Channel, PWM Frequncy or -1");
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

  #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
  #endif

    if (PRM_FREQUENCY(S) == -1)
        nInputPorts++;

    if (!ssSetNumInputPorts(S, nInputPorts)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/

    if (PRM_FREQUENCY(S) == -1) {
        ssSetInputPortWidth(S, 1, 1);
        ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    }

    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 0);

    if (!ssSetNumOutputPorts(S, 0)) return;

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, RWORK_COUNT);
    ssSetNumIWork(S, IWORK_COUNT);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
static void mdlStart(SimStruct *S)
{
  #ifndef WITHOUT_HW
    if (rpi_peripheral_registers_map() <= 0) {
        ssSetErrorStatus(S, "RPi low level peripherals mapping failed");
        return;
    }
    pwm_output_init();
    rpi_gpio_direction_output(GPIO_DIR, 0);

    IWORK_CHANNEL(S) = PRM_CHANNEL(S);
    IWORK_DIR_DO_BIT(S) = PRM_DIR_DO_BIT(S);

    IWORK_USE_FREQUENCY_INPUT(S) = (PRM_FREQUENCY(S) == -1)? 1: 0;

    RWORK_BASE_FREQUENCY(S) = 25e6;

    IWORK_PWM_MODE(S) = 0;

    IWORK_LAST_MODE(S) = PWM_MODE_ZERO;
  #endif /*WITHOUT_HW*/
}
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T duty = *(const real_T*) ssGetInputPortSignal(S, 0);
    real_T frequency;
    int_T new_mode;

  #ifndef WITHOUT_HW
    if (rpi_registers_mapping.mapping_initialized <= 0)
            return;

    if (IWORK_USE_FREQUENCY_INPUT(S))
        frequency = *(const real_T*) ssGetInputPortSignal(S, 1);
    else
        frequency = PRM_FREQUENCY(S);

    if (duty > 0)
        new_mode = PWM_MODE_PLUS_PWM;
        if (duty > 1)
            duty = 1;
    else if (duty < 0)
        new_mode = PWM_MODE_MINUS_PWM;
        if (duty < -1)
            duty = -1;
    else
        new_mode = PWM_MODE_ZERO;

    /* T = RWORK_BASE_FREQUENCY(S) / frequency; */

    if (IWORK_LAST_MODE(S) != new_mode) {
        IWORK_LAST_MODE(S) = new_mode;
    }

    pwm_output_set_width((int)(duty * 40000));

  #endif /*WITHOUT_HW*/
}



#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
  #ifndef WITHOUT_HW
    if (rpi_registers_mapping.mapping_initialized > 0)
            pwm_output_set_width(0);
    if (rpi_registers_mapping.mapping_initialized <= 0)
            return;
    /*FIXME: registers unmap has to be managed by central code */
  #endif /*WITHOUT_HW*/
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
