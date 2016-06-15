#define S_FUNCTION_NAME HydActuation
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */

/*=================*
 * EXTRA Functions *
 *=================*/

/* SATURATION of the valve (DEAD ZONE)*/
double sat(double val,double min, double max){
    if (val < min) return min;
    else if (val > max) return max;
    else return val;
}

/* SIGNUM function*/
int sgn(double val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

/* ABSOLUTE value function*/
double abso(double x){
if (x < 0) return -x;
else return x;
}

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 19);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetNumContStates(S, 4); /* Number of continuous states*/

    if (!ssSetNumInputPorts(S, 3)) return; /*Number of INPUTS*/
    ssSetInputPortWidth(S, 0, 1);          /* In0.0: I */
    ssSetInputPortWidth(S, 1, 1);          /* In1.0: x */
    ssSetInputPortWidth(S, 2, 1);          /* In1.0: dx */
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    
    if (!ssSetNumOutputPorts(S, 1)) return;/* Number of OUTPUTS */
    ssSetOutputPortWidth(S, 0, 1);         /* Out0.0: deltaF */

    ssSetNumSampleTimes(S, 1);
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);         
}

#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetContStates(S);
    x0[0] = 0.0; /* Ics  */
    x0[1] = 0.0; /* dIcs */
    x0[2] = 0.0; /* pA   */
    x0[3] = 0.0; /* pB   */

}
#endif /* MDL_INITIALIZE_CONDITIONS */



/* Function: mdlOutputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    double Aa    = ( mxGetPr(ssGetSFcnParam(S,14)) )[0];
    double Ab    = ( mxGetPr(ssGetSFcnParam(S,15)) )[0];
    double fv    = ( mxGetPr(ssGetSFcnParam(S,18)) )[0];

    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    
    UNUSED_ARG(tid); 

    /* OUTPUT equation*/
    y[0] = (Aa*x[2] - Ab*x[3]) - fv*U(2);
}


#define MDL_DERIVATIVES
/* Function: mdlDerivatives */
static void mdlDerivatives(SimStruct *S)
{
    double wn    = ( mxGetPr(ssGetSFcnParam(S,0)) )[0];
    double E     = ( mxGetPr(ssGetSFcnParam(S,1)) )[0];
    double In    = ( mxGetPr(ssGetSFcnParam(S,2)) )[0];
    double Kva   = ( mxGetPr(ssGetSFcnParam(S,3)) )[0];
    double Kvb   = ( mxGetPr(ssGetSFcnParam(S,4)) )[0];
    double KvlkA = ( mxGetPr(ssGetSFcnParam(S,5)) )[0];
    double KvlkB = ( mxGetPr(ssGetSFcnParam(S,6)) )[0];
    double ps    = ( mxGetPr(ssGetSFcnParam(S,7)) )[0];
    double pt    = ( mxGetPr(ssGetSFcnParam(S,8)) )[0];
    double up_dz = ( mxGetPr(ssGetSFcnParam(S,9)) )[0];
    double lw_dz = ( mxGetPr(ssGetSFcnParam(S,10)) )[0];
    double beta  = ( mxGetPr(ssGetSFcnParam(S,11)) )[0];
    double Va0   = ( mxGetPr(ssGetSFcnParam(S,12)) )[0];
    double Vb0   = ( mxGetPr(ssGetSFcnParam(S,13)) )[0];
    double Aa    = ( mxGetPr(ssGetSFcnParam(S,14)) )[0];
    double Ab    = ( mxGetPr(ssGetSFcnParam(S,15)) )[0];
    double L     = ( mxGetPr(ssGetSFcnParam(S,16)) )[0];
    double leak  = ( mxGetPr(ssGetSFcnParam(S,17)) )[0];

    real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    dx[0]= x[1];
    dx[1]= -wn*wn*x[0] - 2*E*wn*x[1] + wn*wn*sat(U(0),-In,In);
    
    if ((sat(U(0),-In,In) >= up_dz) && (sat(U(0),-In,In) <= In))
        {
        dx[2] = (beta/(Va0 + Aa*U(1))) * (((Kva *(x[0]/In) + KvlkA) * sgn(ps-x[2]) * sqrt(abso(ps-x[2])) - KvlkA * sgn(x[2]-pt) * sqrt(abso(x[2]-pt))) - Aa*U(2)- leak*(x[2]-x[3]));
        dx[3] = (beta/(Vb0 + Ab*(L - U(1)))) * (Ab*U(2) - ((Kvb *(x[0]/In) + KvlkB) * sgn(x[3]-pt) * sqrt(abso(x[3]-pt)) - KvlkB * sgn(ps-x[3]) * sqrt(abso(ps-x[3]))) + leak*(x[2]-x[3]));
        }
    else 
        if ((sat(U(0),-In,In) >= -In) && (sat(U(0),-In,In) <= lw_dz))
        {
        dx[2] = (beta/(Va0 + Aa*U(1))) * ((-(Kva *(abso(x[0])/In) + KvlkA) * sgn(x[2]-pt) * sqrt(abso(x[2]-pt)) + KvlkA * sgn(ps-x[2]) * sqrt(abso(ps-x[2]))) - Aa*U(2)- leak*(x[2]-x[3]));
        dx[3] = (beta/(Vb0 + Ab*(L - U(1)))) * (Ab*U(2) - (-(Kvb *(abso(x[0])/In) + KvlkB) * sgn(ps-x[3]) * sqrt(abso(ps-x[3])) + KvlkB * sgn(x[3]-pt) * sqrt(abso(x[3]-pt))) + leak*(x[2]-x[3]));
        }
    else
        {
        dx[2] = 0;
        dx[3] = 0;
        }

}

/* Function: mdlTerminate */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
