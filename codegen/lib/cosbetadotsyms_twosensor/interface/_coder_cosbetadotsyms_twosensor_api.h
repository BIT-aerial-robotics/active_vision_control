/*
 * File: _coder_cosbetadotsyms_twosensor_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 23-Nov-2021 12:07:43
 */

#ifndef _CODER_COSBETADOTSYMS_TWOSENSOR_API_H
#define _CODER_COSBETADOTSYMS_TWOSENSOR_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_cosbetadotsyms_twosensor_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void cosbetadotsyms_twosensor(real_T u[45], real_T *cosbeta_dot, real_T
  *cosbeta_dot_l);
extern void cosbetadotsyms_twosensor_api(const mxArray *prhs[1], const mxArray
  *plhs[2]);
extern void cosbetadotsyms_twosensor_atexit(void);
extern void cosbetadotsyms_twosensor_initialize(void);
extern void cosbetadotsyms_twosensor_terminate(void);
extern void cosbetadotsyms_twosensor_xil_terminate(void);

#endif

/*
 * File trailer for _coder_cosbetadotsyms_twosensor_api.h
 *
 * [EOF]
 */
