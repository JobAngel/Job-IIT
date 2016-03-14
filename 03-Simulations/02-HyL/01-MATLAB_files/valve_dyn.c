#define S_FUNCTION_NAME valve_dyn
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */

int sign(double A){

if (A>= 0) {return 1;}
else {return -1;}
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
    ssSetNumSFcnParams(S, 9);  /* Number of expected parameters */
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
    double In    = ( mxGetPr(ssGetSFcnParam(S,0)) )[0];
    double Kva   = ( mxGetPr(ssGetSFcnParam(S,1)) )[0];
    double Kvb   = ( mxGetPr(ssGetSFcnParam(S,2)) )[0];
    double KvlkA = ( mxGetPr(ssGetSFcnParam(S,3)) )[0];
    double KvlkB = ( mxGetPr(ssGetSFcnParam(S,4)) )[0];
    double ps    = ( mxGetPr(ssGetSFcnParam(S,5)) )[0];
    double pt    = ( mxGetPr(ssGetSFcnParam(S,6)) )[0];
    
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    UNUSED_ARG(tid); /* not used in single tasking mode */

// if (I>=up_dz) && (I<=In),
//     qvA = (Kva*(I/In) + KvlkA) * sign(ps-pA)*sqrt(abs(ps-pA)) - KvlkA*sign(pA-pt)*sqrt(abs(pA-pt)) ;
//     qvB = (Kvb*(I/In) + KvlkB) * sign(pB-pt)*sqrt(abs(pB-pt)) - KvlkB*sign(ps-pB)*sqrt(abs(ps-pB)) ;
// elseif (I>=-In) && (I<=lw_dz), 
//     qvA = KvlkA*sign(ps-pA)*sqrt(abs(ps-pA)) - (Kva*(abs(I)/In) + KvlkA) * sign(pA-pt)*sqrt(abs(pA-pt));
//      qvB = KvlkB*sign(pB-pt)*sqrt(abs(pB-pt)) - (Kvb*(abs(I)/In) + KvlkB) * sign(ps-pB)*sqrt(abs(ps-pB));
// else
//     qvA = 0;
//     qvB = 0;
// end
    
    /* qvA equation*/
    y[0] = (Kva*(x[0]/In) + KvlkA) * sign(ps-U(1))*sqrt(abs(ps-U(1))) - KvlkA*sign(U(1)-pt)*sqrt(abs(U(1)-pt)) ;
    /* qvB equation*/
    y[1] = (Kvb*(x[0]/In) + KvlkB) * sign(U(2)-pt)*sqrt(abs(U(2)-pt)) - KvlkB*sign(ps-U(2))*sqrt(abs(ps-U(2))) ;
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
 
    real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    /* xdot=Ax+Bu */
    dx[0]= x[1];
    dx[1]= -wn*wn*x[0] - 2*E*wn*x[1] + wn*wn*U(0);
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
