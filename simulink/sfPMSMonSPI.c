/*
 * S-function to support SPI Connected PMSM Driver Board from RaspberryPi
 *
 * Copyright (C) 2015 Pavel Pisa <pisa@cmp.felk.cvut.cz>
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


#define S_FUNCTION_NAME  sfPMSMonSPI
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

#define PRM_COUNT                   1

#define PWORK_IDX_SPIMC_STATE       0

#define PWORK_COUNT                 1

#define PWORK_SPIMC_STATE(S)        (ssGetPWork(S)[PWORK_IDX_SPIMC_STATE])

enum {
    sIn_N_PWM_VAL = 0,  /* PWM value [3 x 1]  */
    sIn_N_PWM_EN,       /* PWM enable [3 x 1] */
    sIn_N_NUM
};

/* Enumerated constants for output ports ******************************************** */
enum {
    sOut_N_Cur_ADC = 0, /* Current ADC [3 x 1] */
    sOut_N_IRC_Pos,     /* IRC position [1 x 1] */
    sOut_N_IRC_Idx,     /* IRC index [1 x 1] */
    sOut_N_IRC_Occur,   /* IRC index occurence [1 x 1] */
    sOut_N_HAL_Sector,  /* Hal sector [1 x 1] <0 .. 5>  and -1 ) */
    sOut_N_NUM
};

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

#include "rpi_gpio.h"
#include "rpi_spimc.h"
#include "rpi_gpclk.h"

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
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, PRM_COUNT);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S, "1-parameter requited: Ts");
        return;
    }

  #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
  #endif

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, sIn_N_NUM)) return;

    ssSetInputPortWidth(S, sIn_N_PWM_VAL, 3);
    ssSetInputPortWidth(S, sIn_N_PWM_EN, 3);

    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */

    if (!ssSetNumOutputPorts(S, sOut_N_NUM)) return;
    ssSetOutputPortWidth(S, sOut_N_Cur_ADC, 3);
    ssSetOutputPortWidth(S, sOut_N_IRC_Pos, 1);
    ssSetOutputPortDataType(S, sOut_N_IRC_Pos, SS_INT32);
    ssSetOutputPortWidth(S, sOut_N_IRC_Idx, 1);
    ssSetOutputPortDataType(S, sOut_N_IRC_Idx, SS_INT32);
    ssSetOutputPortWidth(S, sOut_N_IRC_Occur, 1);
    ssSetOutputPortDataType(S, sOut_N_IRC_Occur, SS_INT32);
    ssSetOutputPortWidth(S, sOut_N_HAL_Sector, 1);
    ssSetOutputPortDataType(S, sOut_N_HAL_Sector, SS_INT32);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, PWORK_COUNT);
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
    spimc_state_t *spimcst = (spimc_state_t *)PWORK_SPIMC_STATE(S);

    spimcst->curadc_offs[0] = 0; /*2072*/
    spimcst->curadc_offs[1] = 0; /*2077*/
    spimcst->curadc_offs[2] = 0; /*2051*/

    spimcst->pos_offset = -spimcst->act_pos;

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
  #ifndef WITHOUT_HW
    spimc_state_t *spimcst;

    PWORK_SPIMC_STATE(S) = NULL;

    if (rpi_peripheral_registers_map() <= 0) {
        ssSetErrorStatus(S, "rpi_peripheral_registers_map failed");
        return;
    }

    spimcst = malloc(sizeof(*spimcst));
    if (spimcst == NULL) {
        ssSetErrorStatus(S, "malloc spimcst failed");
        return;
    }
    memset(spimcst, sizeof(*spimcst), 0);

    spimcst->spi_dev = "/dev/spidev0.1";

    if (spimc_init(spimcst) < 0) {
        ssSetErrorStatus(S, "spimc_init spimcst failed");
        return;
    }

    PWORK_SPIMC_STATE(S) = spimcst;

    if (rpi_gpclk_setup(0, RPI_GPCLK_PLLD_500_MHZ, 10, 0) < 0) {
        ssSetErrorStatus(S, "rpi_gpclk_setup failed\n");
        return;
    }

    if (rpi_gpio_alt_fnc(4 /*gpio*/, 0/*alt_fnc*/) < 0) {
        ssSetErrorStatus(S, "rpi_gpio_alt_fnc failed\n");
        return;
    }

    spimc_transfer(spimcst);

  #endif /*WITHOUT_HW*/

    mdlInitializeConditions(S);
}
#endif /*  MDL_START */


#ifndef WITHOUT_HW
const unsigned char pxmc_lpc_bdc_hal_pos_table[8] =
{
  [0] = 0xff,
  [7] = 0xff,
  [1] = 0, /*0*/
  [5] = 1, /*1*/
  [4] = 2, /*2*/
  [6] = 3, /*3*/
  [2] = 4, /*4*/
  [3] = 5, /*5*/
};
#endif /*WITHOUT_HW*/

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *cur_adc = ssGetOutputPortSignal(S, sOut_N_Cur_ADC);
    int32_T *irc_pos = ssGetOutputPortSignal(S, sOut_N_IRC_Pos);
    int32_T *irc_idx = ssGetOutputPortSignal(S, sOut_N_IRC_Idx);
    int32_T *irc_idx_occ = ssGetOutputPortSignal(S, sOut_N_IRC_Occur);
    int32_T *hal_sec = ssGetOutputPortSignal(S, sOut_N_HAL_Sector);

  #ifndef WITHOUT_HW
    spimc_state_t *spimcst = (spimc_state_t *)PWORK_SPIMC_STATE(S);
    uint32_t curadc_sqn_diff;
    uint32_t curadc_val_diff;
    int i;
    int diff_to_last_fl = 0;
   #if 0
    static unsigned long sqn_accum;
    static unsigned long sqn_accum_over;
    static unsigned long runs;
    static unsigned long runs_over;
    static unsigned long runs_miss;
    static unsigned int hist[512];
   #endif
    curadc_sqn_diff = spimcst->curadc_sqn;
    if (diff_to_last_fl) {
      curadc_sqn_diff -= spimcst->curadc_sqn_last;
      curadc_sqn_diff &= 0x1ff;
    }

    if ((curadc_sqn_diff > 1) && (curadc_sqn_diff <= 450)) {
        for (i = 0; i < SPIMC_CHAN_COUNT; i++) {
            curadc_val_diff = spimcst->curadc_cumsum[i];
            if (diff_to_last_fl) {
              curadc_val_diff -= spimcst->curadc_cumsum_last[i];
              curadc_val_diff &= 0xffffff;
            }
            cur_adc[i] = (real_T)curadc_val_diff / curadc_sqn_diff -
                            spimcst->curadc_offs[i];
        }
       #if 0
        runs++;
        sqn_accum += curadc_sqn_diff;
       #endif
    }
   #if 0
    else {
        if (curadc_sqn_diff) {
          sqn_accum_over += curadc_sqn_diff;
          runs_over++;
        } else runs_miss++;
    }

    hist[curadc_sqn_diff]++;
    if (runs >= 1000) {
      fprintf(stderr, "aver sqn diff %ld runs %ld\n", sqn_accum / runs, runs);
      if (runs_over)
        fprintf(stderr, "over sqn diff %ld runs %ld\n", sqn_accum_over / runs_over, runs_over);
      if (runs_miss)
        fprintf(stderr, "missed runs %ld\n", runs_miss);
      for (i = 0; i < 512; i++) {
        fprintf(stderr, " %d", hist[i]);
        hist[i] = 0;
      }
      fprintf(stderr, "\n");
      runs = 0;
      runs_over = 0;
      runs_miss = 0;
      sqn_accum = 0;
      sqn_accum_over = 0;
    }
   #endif

    irc_pos[0] = spimcst->act_pos + spimcst->pos_offset;
    irc_idx[0] = spimcst->index_pos + spimcst->pos_offset;
    irc_idx_occ[0] = spimcst->index_occur;
    hal_sec[0] = pxmc_lpc_bdc_hal_pos_table[spimcst->hal_sensors];
  #else /*WITHOUT_HW*/
    cur_adc[0] = 0;
    cur_adc[1] = 0;
    cur_adc[2] = 0;
    irc_pos[0] = 0;
    irc_idx[0] = 0;
    irc_idx_occ[0] = 0;
    hal_sec[0] = 0;
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
    InputRealPtrsType pwm_val = ssGetInputPortRealSignalPtrs(S, sIn_N_PWM_VAL);
    InputRealPtrsType pwm_en = ssGetInputPortRealSignalPtrs(S, sIn_N_PWM_EN);

  #ifndef WITHOUT_HW
    spimc_state_t *spimcst = (spimc_state_t *)PWORK_SPIMC_STATE(S);
    int i;
    real_T pwm;

    spimcst->curadc_sqn_last = spimcst->curadc_sqn;

    for (i = 0; i < SPIMC_CHAN_COUNT; i++)
        spimcst->curadc_cumsum_last[i] = spimcst->curadc_cumsum[i];

    for (i = 0; i < SPIMC_CHAN_COUNT; i++) {
        if (*pwm_en[i]) {
            pwm = *pwm_val[i] * 2048;
            if (pwm > 2047)
                pwm = 2047;
            if (pwm < 0)
                pwm = 0;
            spimcst->pwm[i] = (uint32_t)pwm | SPIMC_PWM_ENABLE;
        } else {
            spimcst->pwm[i] = 0 | SPIMC_PWM_SHUTDOWN;
        }
    }

    spimc_transfer(spimcst);

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
    spimc_state_t *spimcst = (spimc_state_t *)PWORK_SPIMC_STATE(S);

    if (spimcst != NULL) {
        PWORK_SPIMC_STATE(S) = NULL;
        free(spimcst);
        rpi_gpio_direction_output(4, 0);
    }
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
