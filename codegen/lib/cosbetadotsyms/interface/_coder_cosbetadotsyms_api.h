/*
 * File: _coder_cosbetadotsyms_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 15-Nov-2021 04:45:05
 */

#ifndef _CODER_COSBETADOTSYMS_API_H
#define _CODER_COSBETADOTSYMS_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_cosbetadotsyms_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern real_T cosbetadotsyms(real_T u[21]);
extern void cosbetadotsyms_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void cosbetadotsyms_atexit(void);
extern void cosbetadotsyms_initialize(void);
extern void cosbetadotsyms_terminate(void);
extern void cosbetadotsyms_xil_terminate(void);

#endif

/*
 * File trailer for _coder_cosbetadotsyms_api.h
 *
 * [EOF]
 */
