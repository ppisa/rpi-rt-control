/*
 * S-function to support IRC inputs on RaspberryPi through IRC device
 *
 * Copyright (C) 2014 Pavel Pisa <pisa@cmp.felk.cvut.cz>
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


#define S_FUNCTION_NAME  sfIRCInput
#define S_FUNCTION_LEVEL 2

/*
 * The S-function has next parameters
 *
 * Sample time     - sample time value or -1 for inherited
 * Counter Mode    -
 * Counter Gating
 * Reset Control
 * Digital Filter
 */

#define PRM_TS(S)               (mxGetScalar(ssGetSFcnParam(S, 0)))
#define PRM_CHANNEL(S)          (mxGetScalar(ssGetSFcnParam(S, 1)))
#define PRM_COUNTER_MODE(S)     (mxGetScalar(ssGetSFcnParam(S, 2)))
#define PRM_COUNTER_GATING(S)   (mxGetScalar(ssGetSFcnParam(S, 3)))
#define PRM_RESET_CONTROL(S)    (mxGetScalar(ssGetSFcnParam(S, 4)))
#define PRM_DIGITAL_FILTER(S)   (mxGetScalar(ssGetSFcnParam(S, 5)))
#define PRM_RESET_AT_STARTUP(S) (mxGetScalar(ssGetSFcnParam(S, 6)))

#define PRM_COUNT                   7

#define IWORK_IDX_CHANNEL           0
#define IWORK_IDX_USE_GATING_INPUT  1
#define IWORK_IDX_GATING_VALUE      2
#define IWORK_IDX_USE_RESET_INPUT   3
#define IWORK_IDX_RESET_VALUE       4
#define IWORK_IDX_IRC_DEV_FD        5
#define IWORK_IDX_IRC_OFFSET        6
#define IWORK_IDX_IRC_ACT_VAL       7

#define IWORK_COUNT                 8

#define IWORK_CHANNEL(S)            (ssGetIWork(S)[IWORK_IDX_CHANNEL])
#define IWORK_USE_GATING_INPUT(S)   (ssGetIWork(S)[IWORK_IDX_USE_GATING_INPUT])
#define IWORK_GATING_VALUE(S)       (ssGetIWork(S)[IWORK_IDX_GATING_VALUE])
#define IWORK_USE_RESET_INPUT(S)    (ssGetIWork(S)[IWORK_IDX_USE_RESET_INPUT])
#define IWORK_RESET_VALUE(S)        (ssGetIWork(S)[IWORK_IDX_RESET_VALUE])
#define IWORK_IRC_DEV_FD(S)         (ssGetIWork(S)[IWORK_IDX_IRC_DEV_FD])
#define IWORK_IRC_OFFSET(S)         (ssGetIWork(S)[IWORK_IDX_IRC_OFFSET])
#define IWORK_IRC_ACT_VAL(S)        (ssGetIWork(S)[IWORK_IDX_IRC_ACT_VAL])

#define IRC_RESET_SRC_DISABLED  0
#define IRC_RESET_SRC_ALWAYS    1
#define IRC_RESET_SRC_IF_IDX_LO 2
#define IRC_RESET_SRC_IF_IDX_HI 3
#define IRC_RESET_SRC_IF_IDX_RE 4
#define IRC_RESET_SRC_IF_IDX_FE 5
#define IRC_RESET_SRC_IF_IDX_BOTH 6
#define IRC_RESET_SRC_VAL_MAX   6

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#ifndef WITHOUT_HW

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

static const char *irc_dev_name_table[] = {
    "/dev/irc0",
    "/dev/irc1",
    "/dev/irc2",
    "/dev/irc3"
};
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
    if ((PRM_TS(S) < 0) && (PRM_TS(S) != -1))
        ssSetErrorStatus(S, "Ts has to be positive or -1 for automatic step");
    if ((PRM_CHANNEL(S) < 0) || (PRM_CHANNEL(S) > 3))
        ssSetErrorStatus(S, "valid IRC channel is 0, 1, 2, or 3");
    if ((PRM_RESET_CONTROL(S) < 0) || PRM_RESET_CONTROL(S) > IRC_RESET_SRC_VAL_MAX)
        if (PRM_RESET_CONTROL(S) != -1)
           ssSetErrorStatus(S, "Reset Control out of valid range and -1 for external input not set");
  #if 0
    if ((PRM_COUNTER_MODE(S) < 0) || PRM_COUNTER_MODE(S) >
            __mfld2val(IRCCTRL_IRC0MODE_mask, IRCCTRL_IRC0MODE_mask))
        ssSetErrorStatus(S, "Counter Mode out of valid range");
    if ((PRM_COUNTER_GATING(S) < 0) || PRM_COUNTER_GATING(S) >
            __mfld2val(IRCCTRL_IRC0COUNT_mask, IRCCTRL_IRC0COUNT_mask))
        if ((PRM_COUNTER_GATING(S) != -1))
           ssSetErrorStatus(S, "Counter Gating out of valid range and -1 for external input not set");
    if ((PRM_DIGITAL_FILTER(S) < 0) || PRM_DIGITAL_FILTER(S) >
            __mfld2val(IRCCTRL_IRC0FILTER_mask, IRCCTRL_IRC0FILTER_mask))
        ssSetErrorStatus(S, "Digital Filter out of valid range");
  #endif
    if ((PRM_RESET_AT_STARTUP(S) != 0) && (PRM_RESET_AT_STARTUP(S) != 1))
        ssSetErrorStatus(S, "Reset at startup can be only 0 or 1");
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nInputPorts  = 0;
    int_T i;

    ssSetNumSFcnParams(S, PRM_COUNT);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S, "6-parameters requited: Ts, Channel, Counter Mode, Counter Gating, Reset Control and Digital Filter");
        return;
    }

  #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
  #endif

    if (PRM_COUNTER_GATING(S) == -1)
        nInputPorts++;
    if (PRM_RESET_CONTROL(S) == -1)
        nInputPorts++;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, nInputPorts)) return;
    for (i = 0; i < nInputPorts; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDataType(S, i, SS_UINT8);
        ssSetInputPortRequiredContiguous(S, i, true); /*direct input signal access*/
        ssSetInputPortDirectFeedThrough(S, i, 0);
    }
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_INT32);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
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
    if (PRM_TS(S) == -1) {
        ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
        ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
    } else {
        ssSetSampleTime(S, 0, PRM_TS(S));
        ssSetOffsetTime(S, 0, 0.0);
    }
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
  #ifndef WITHOUT_HW
    uint32_t irc_val_raw = 0;
    int_T irc_dev_fd = IWORK_IRC_DEV_FD(S);

    if (irc_dev_fd == -1)
        return;

    if (read(irc_dev_fd, &irc_val_raw, sizeof(uint32_t)) != sizeof(uint32_t)) {
        ssSetErrorStatus(S, "/dev/ircX read failed");
    }

    IWORK_IRC_ACT_VAL(S) = (int32_t)irc_val_raw;
    if (PRM_RESET_AT_STARTUP(S)) {
        IWORK_IRC_OFFSET(S) = -(int32_t)irc_val_raw;
    } else {
        IWORK_IRC_OFFSET(S) = 0;
    }
  #endif /*WITHOUT_HW*/
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
    int_T irc_dev_fd;
    const char *irc_dev_name;

  #ifndef WITHOUT_HW
    IWORK_CHANNEL(S) = PRM_CHANNEL(S);
    IWORK_GATING_VALUE(S) = PRM_COUNTER_GATING(S);
    IWORK_USE_GATING_INPUT(S) = (IWORK_GATING_VALUE(S) == -1)? 1: 0;
    IWORK_RESET_VALUE(S) = PRM_RESET_CONTROL(S);
    IWORK_USE_RESET_INPUT(S) = (IWORK_RESET_VALUE(S) == -1)? 1: 0;

    irc_dev_name = irc_dev_name_table[IWORK_CHANNEL(S)];
    irc_dev_fd = open(irc_dev_name, O_RDONLY);
    if (irc_dev_fd == -1) {
        ssSetErrorStatus(S, "/dev/ircX open failed");
    }
    IWORK_IRC_DEV_FD(S) = irc_dev_fd;

  #endif /*WITHOUT_HW*/

    mdlInitializeConditions(S);
}
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int32_T *y = ssGetOutputPortSignal(S,0);

  #ifndef WITHOUT_HW
    y[0] = (int32_t)(IWORK_IRC_ACT_VAL(S) + IWORK_IRC_OFFSET(S));
  #else /*WITHOUT_HW*/
    y[0] = 0;
  #endif /*WITHOUT_HW*/
}



#define MDL_UPDATE  /* Change to #undef to remove function */
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
    int_T inp_num = 0;

  #ifndef WITHOUT_HW
    uint32_t irc_val_raw = 0;
    int_T irc_dev_fd = IWORK_IRC_DEV_FD(S);

    if (irc_dev_fd == -1)
        return;

    if (read(irc_dev_fd, &irc_val_raw, sizeof(uint32_t)) != sizeof(uint32_t)) {
        ssSetErrorStatus(S, "/dev/ircX read failed");
    }

    IWORK_IRC_ACT_VAL(S) = (int32_t)irc_val_raw;

    if (IWORK_USE_GATING_INPUT(S)) {
        const uint8_T *u = (const uint8_T*) ssGetInputPortSignal(S, inp_num);
        if (*u != IWORK_GATING_VALUE(S)) {
            IWORK_GATING_VALUE(S) = *u;
            /* IRC Mode Gating Changed */
        }
        inp_num++;
    }

    if (IWORK_USE_RESET_INPUT(S)) {
        const uint8_T *u = (const uint8_T*) ssGetInputPortSignal(S, inp_num);
        if (*u != IWORK_RESET_VALUE(S)) {
            IWORK_RESET_VALUE(S) = *u;
        }
        if (IWORK_RESET_VALUE(S) == IRC_RESET_SRC_ALWAYS)
            IWORK_IRC_OFFSET(S) = -IWORK_IRC_ACT_VAL(S);
        inp_num++;
    }
  #endif /*WITHOUT_HW*/
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
    int_T irc_dev_fd = IWORK_IRC_DEV_FD(S);
    if (irc_dev_fd != -1)
      close(irc_dev_fd);
    IWORK_IRC_DEV_FD(S) = -1;
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
