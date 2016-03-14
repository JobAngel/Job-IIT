#define S_FUNCTION_NAME CylinderDynamic
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 8);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    /* Number of continuous states*/
    ssSetNumContStates(S, 2);

    if (!ssSetNumInputPorts(S, 3)) return; /*Number of INPUTS*/
    ssSetInputPortWidth(S, 0, 1);          /* In0.0: x */
    ssSetInputPortWidth(S, 1, 1);          /* In1.0: dx*/
    ssSetInputPortWidth(S, 2, 2);          /* In2.0: qvA ; In2.1: qvB */
//     ssSetInputPortWidth(S, 3, 1);          /* In1.0: qvB*/
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
//     ssSetInputPortDirectFeedThrough(S, 3, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;/* Number of OUTPUTS*/
    ssSetOutputPortWidth(S, 0, 1);         /* Out0.0: deltaF*/
    ssSetOutputPortWidth(S, 1, 2);         /* Out1.0: pA & pB*/

    ssSetNumSampleTimes(S, 1);
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

//     /* Take care when specifying exception free code - see sfuntmpl_doc.c */
//     ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we have a continuous sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);         
}

#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
/*
 * Set the initial conditions to [0 2]
 */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetContStates(S);

    /* Initialize to the inputs */
    x0[0] = 0.0; /*pA*/
    x0[1] = 0.0; /*pB*/

}
#endif /* MDL_INITIALIZE_CONDITIONS */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = Cx + Du 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{

    double Aa  = ( mxGetPr(ssGetSFcnParam(S,3)) )[0];
    double Ab  = ( mxGetPr(ssGetSFcnParam(S,4)) )[0];
    double fv  = ( mxGetPr(ssGetSFcnParam(S,6)) )[0];

    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    UNUSED_ARG(tid); /* not used in single tasking mode */

    /* deltaF equation*/
    y[0] = (Aa*x[0] - Ab*x[1]) - fv*U(1);
    y[1] = x[0];
    y[2] = x[1];
}


#define MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      xdot = Ax + Bu
 */
static void mdlDerivatives(SimStruct *S)
{

    double beta  = ( mxGetPr(ssGetSFcnParam(S,0)) )[0];
    double Va0   = ( mxGetPr(ssGetSFcnParam(S,1)) )[0];
    double Vb0   = ( mxGetPr(ssGetSFcnParam(S,2)) )[0];
    double Aa    = ( mxGetPr(ssGetSFcnParam(S,3)) )[0];
    double Ab    = ( mxGetPr(ssGetSFcnParam(S,4)) )[0];
    double L     = ( mxGetPr(ssGetSFcnParam(S,5)) )[0];
    double leak  = ( mxGetPr(ssGetSFcnParam(S,7)) )[0];
    
    real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    /* dpA */
    dx[0]= (beta/(Va0 + Aa*U(0)))*(U(2) - Aa*U(1) - leak*(x[0]-x[1]));
    /* dpB */
    dx[1]= (beta/(Vb0 + Ab*(L - U(0))))*(Ab*U(1) - U(3) + leak*(x[0]-x[1]));
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
