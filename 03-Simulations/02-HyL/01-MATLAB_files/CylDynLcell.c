#define S_FUNCTION_NAME CylDynLcell
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
    ssSetNumSFcnParams(S, 10);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    /* Number of continuous states*/
    ssSetNumContStates(S, 4);

    if (!ssSetNumInputPorts(S, 2)) return; /*Number of INPUTS*/
//     ssSetInputPortWidth(S, 0, 2);          /* In0.0: xL ; In0.1: xL0 */
    ssSetInputPortWidth(S, 0, 2);          /* In0.0: Fe ; In0.1: x0 */
    ssSetInputPortWidth(S, 1, 2);          /* In1.0: qvA ; In1.1: qvB */
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;/* Number of OUTPUTS*/
    ssSetOutputPortWidth(S, 0, 1);         /* Out0.0: x*/
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
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    /* Initialize to the inputs */
    x0[0] = 0.0;  /*pA0*/
    x0[1] = 0.0;  /*pB0*/
    x0[2] = U(1); /*x0*/
    x0[3] = 0.0;  /*dx0*/


}
#endif /* MDL_INITIALIZE_CONDITIONS */



/* Function: mdlOutputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{

    double Ks    = ( mxGetPr(ssGetSFcnParam(S,8)) )[0];
    double L     = ( mxGetPr(ssGetSFcnParam(S,5)) )[0];
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    UNUSED_ARG(tid); 
    /* Fe equation*/
//     y[0] = Ks*(x[2] - U(0));

    if ((x[2] >= 0) && (x[2] <= L)) {y[0] = x[2];}
    else if (x[2] > L) {y[0] = L;}
    else {y[0] = 0;}
// 
//     y[0] = x[2];
//     
    y[1] = x[0];
    y[2] = x[1];
}


#define MDL_DERIVATIVES
/* Function: mdlDerivatives */
static void mdlDerivatives(SimStruct *S)
{

    double beta  = ( mxGetPr(ssGetSFcnParam(S,0)) )[0];
    double Va0   = ( mxGetPr(ssGetSFcnParam(S,1)) )[0];
    double Vb0   = ( mxGetPr(ssGetSFcnParam(S,2)) )[0];
    double Aa    = ( mxGetPr(ssGetSFcnParam(S,3)) )[0];
    double Ab    = ( mxGetPr(ssGetSFcnParam(S,4)) )[0];
    double L     = ( mxGetPr(ssGetSFcnParam(S,5)) )[0];
    double fv    = ( mxGetPr(ssGetSFcnParam(S,6)) )[0];
    double leak  = ( mxGetPr(ssGetSFcnParam(S,7)) )[0];
    double Ks    = ( mxGetPr(ssGetSFcnParam(S,8)) )[0];
    double Mp    = ( mxGetPr(ssGetSFcnParam(S,9)) )[0];

    
    real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    /* dpA */
    dx[0]= (beta/(Va0 + Aa*x[2]))*(U(2) - Aa*x[3] - leak*(x[0]-x[1]));
    /* dpB */
    dx[1]= (beta/(Vb0 + Ab*(L - x[2])))*(Ab*x[3] - U(3) + leak*(x[0]-x[1]));
    /* dx */
    dx[2]= x[3];
    /* ddx */
    dx[3]= (1/Mp)*(x[0]*Aa - x[1]*Ab - fv*x[3] - U(0));
//     dx[3]= (1/Mp)*(Ks*(x[2]-U(0)) - fv*x[3] - x[0]*Aa + x[1]*Ab);

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
