#define S_FUNCTION_NAME ValveDynamic
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */

double sat(double val,double min, double max){
    if (val < min) return min;
    else if (val > max) return max;
    else return val;
}

int sgn(double val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

double abso(double x){
if (x < 0) return -x;
else return x;
}

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
    ssSetNumSFcnParams(S, 11);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    /* Number of continuous states*/
    ssSetNumContStates(S, 2);

    if (!ssSetNumInputPorts(S, 2)) return; /*Number of INPUTS*/
    ssSetInputPortWidth(S, 0, 1);          /* In0.0: I */
    ssSetInputPortWidth(S, 1, 2);          /* In1.0: pA   , In1.1: pB*/
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;/* Number of OUTPUTS*/
    ssSetOutputPortWidth(S, 0, 2);         /* Out0.0: qvA  ,  Out0.1: qvB*/

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
    x0[0] = 0.0;
    x0[1] = 0.0;

}
#endif /* MDL_INITIALIZE_CONDITIONS */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = Cx + Du 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    double In    = ( mxGetPr(ssGetSFcnParam(S,2)) )[0];
    double Kva   = ( mxGetPr(ssGetSFcnParam(S,3)) )[0];
    double Kvb   = ( mxGetPr(ssGetSFcnParam(S,4)) )[0];
    double KvlkA = ( mxGetPr(ssGetSFcnParam(S,5)) )[0];
    double KvlkB = ( mxGetPr(ssGetSFcnParam(S,6)) )[0];
    double ps    = ( mxGetPr(ssGetSFcnParam(S,7)) )[0];
    double pt    = ( mxGetPr(ssGetSFcnParam(S,8)) )[0];
    double up_dz = ( mxGetPr(ssGetSFcnParam(S,9)) )[0];
    double lw_dz = ( mxGetPr(ssGetSFcnParam(S,10)) )[0];
    
//     double root;
    
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    UNUSED_ARG(tid); /* not used in single tasking mode */

   
if ((sat(U(0),-In,In) >= up_dz) && (sat(U(0),-In,In) <= In))
    {
    y[0] = (Kva *(x[0]/In) + KvlkA) * sgn(ps-U(1)) * sqrt(abso(ps-U(1))) - KvlkA * sgn(U(1)-pt) * sqrt(abso(U(1)-pt));
    y[1] = (Kvb *(x[0]/In) + KvlkB) * sgn(U(2)-pt) * sqrt(abso(U(2)-pt)) - KvlkB * sgn(ps-U(2)) * sqrt(abso(ps-U(2)));
    }
else 
    if ((sat(U(0),-In,In) >= -In) && (sat(U(0),-In,In) <= lw_dz))
    {
    y[0] = - (Kva *(abso(x[0])/In) + KvlkA) * sgn(U(1)-pt) * sqrt(abso(U(1)-pt)) + KvlkA * sgn(ps-U(1)) * sqrt(abso(ps-U(1)));
    y[1] = - (Kvb *(abso(x[0])/In) + KvlkB) * sgn(ps-U(2)) * sqrt(abso(ps-U(2))) + KvlkB * sgn(U(2)-pt) * sqrt(abso(U(2)-pt));
    }
else
    {
    y[0] = 0;
    y[1] = 0;
    }
}


#define MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      xdot = Ax + Bu
 */
static void mdlDerivatives(SimStruct *S)
{
    double wn = ( mxGetPr(ssGetSFcnParam(S,0)) )[0];
    double E  = ( mxGetPr(ssGetSFcnParam(S,1)) )[0];
    double In = ( mxGetPr(ssGetSFcnParam(S,2)) )[0];

    real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    /* xdot=Ax+Bu */
    dx[0]= x[1];
    dx[1]= -wn*wn*x[0] - 2*E*wn*x[1] + wn*wn*sat(U(0),-In,In);
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
