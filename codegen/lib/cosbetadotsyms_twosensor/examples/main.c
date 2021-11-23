/*
 * File: main.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 23-Nov-2021 12:07:43
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include Files */
#include "rt_nonfinite.h"
#include "cosbetadotsyms_twosensor.h"
#include "main.h"
#include "cosbetadotsyms_twosensor_terminate.h"
#include "cosbetadotsyms_twosensor_initialize.h"

/* Function Declarations */
static void argInit_45x1_real_T(double result[45]);
static double argInit_real_T(void);
static void main_cosbetadotsyms_twosensor(void);

/* Function Definitions */

/*
 * Arguments    : double result[45]
 * Return Type  : void
 */
static void argInit_45x1_real_T(double result[45])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 45; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_cosbetadotsyms_twosensor(void)
{
  double dv0[45];
  double cosbeta_dot;
  double cosbeta_dot_l;

  /* Initialize function 'cosbetadotsyms_twosensor' input arguments. */
  /* Initialize function input argument 'u'. */
  /* Call the entry-point 'cosbetadotsyms_twosensor'. */
  argInit_45x1_real_T(dv0);
  cosbetadotsyms_twosensor(dv0, &cosbeta_dot, &cosbeta_dot_l);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  cosbetadotsyms_twosensor_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_cosbetadotsyms_twosensor();

  /* Terminate the application.
     You do not need to do this more than one time. */
  cosbetadotsyms_twosensor_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
