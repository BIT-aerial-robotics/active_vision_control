/*
 * File: cosbetadotsyms.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 15-Nov-2021 04:45:05
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "cosbetadotsyms.h"

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * from the symbol expression
 * input is the feature point and the state:
 * Arguments    : const double u[21]
 * Return Type  : double
 */
double cosbetadotsyms(const double u[21])
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double m_a;
  double n_a;
  double o_a;
  double p_a;
  double q_a;
  double r_a;
  double s_a;
  double t_a;
  double u_a;
  double v_a;
  double w_a;
  double x_a;
  double y_a;
  double ab_a;
  double bb_a;
  double cb_a;
  double db_a;
  double eb_a;
  double fb_a;
  double gb_a;
  double hb_a;
  double ib_a;
  double jb_a;
  double kb_a;
  double lb_a;
  double mb_a;
  double nb_a;
  double ob_a;
  double pb_a;
  double qb_a;
  double rb_a;
  double sb_a;
  double tb_a;
  double ub_a;
  double vb_a;
  double wb_a;
  double xb_a;
  double yb_a;
  double ac_a;
  double bc_a;
  double cc_a;
  double dc_a;
  double ec_a;
  double fc_a;
  double gc_a;

  /* the position of the vehicle: */
  /*  global x0_x  x0_y  x0_z;  */
  /* the velocity of the vehicle, in earth frame: */
  /*  global v0_x v0_y v0_z;  */
  /*  global r0_11 r0_12 r0_13 r0_21 r0_22 r0_23 r0_31 r0_32 r0_33; */
  /*  global omega0_x omega0_y omega0_z; */
  a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
       u[1]) - u[12] * u[2];
  b_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  c_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  d_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  e_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  f_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  g_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  h_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  i_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  j_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  k_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  l_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  m_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  n_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  o_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  p_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  q_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  r_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  s_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  t_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  u_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  v_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  w_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  x_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  y_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  ab_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  bb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  cb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  db_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  eb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  fb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  gb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  hb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  ib_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  jb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  kb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  lb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  mb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  nb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  ob_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  pb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  qb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  rb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  sb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  tb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  ub_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  vb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  wb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  xb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  yb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  ac_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  bc_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  cc_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  dc_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  ec_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  fc_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  gc_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  return (((((((((((u[18] - u[0]) * (u[15] * u[7] - u[16] * u[6]) * (((((u[18] *
    u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] *
    u[2]) * (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) -
              u[11] * u[1]) - u[14] * u[2]) / rt_powd_snf((a * a + b_a * b_a) +
    c_a * c_a, 1.5) - ((u[19] - u[1]) / sqrt((d_a * d_a + e_a * e_a) + f_a * f_a)
                       - (u[19] - u[1]) * (g_a * g_a) / rt_powd_snf((h_a * h_a +
    i_a * i_a) + j_a * j_a, 1.5)) * (u[16] * u[11] - u[17] * u[10])) - ((u[20] -
    u[2]) / sqrt((k_a * k_a + l_a * l_a) + m_a * m_a) - (u[20] - u[2]) * (n_a *
    n_a) / rt_powd_snf((o_a * o_a + p_a * p_a) + q_a * q_a, 1.5)) * (u[16] * u
    [14] - u[17] * u[13])) - u[3] * (u[6] / sqrt((r_a * r_a + s_a * s_a) + t_a *
    t_a) - ((2.0 * u[6] * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) -
    u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) + 2.0 * u[7] * (((((u[18] * u[7]
    + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] *
    u[2])) + 2.0 * u[8] * (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) -
    u[8] * u[0]) - u[11] * u[1]) - u[14] * u[2])) * (((((u[18] * u[6] + u[19] *
    u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) / (2.0 *
    rt_powd_snf((u_a * u_a + v_a * v_a) + w_a * w_a, 1.5)))) - u[4] * (u[9] /
    sqrt((x_a * x_a + y_a * y_a) + ab_a * ab_a) - ((2.0 * u[9] * (((((u[18] * u
    [6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] *
    u[2]) + 2.0 * u[10] * (((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) -
    u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2])) + 2.0 * u[11] * (((((u[18] *
    u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11] * u[1]) - u[14]
    * u[2])) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0])
                 - u[9] * u[1]) - u[12] * u[2]) / (2.0 * rt_powd_snf((bb_a *
    bb_a + cb_a * cb_a) + db_a * db_a, 1.5)))) - u[5] * (u[12] / sqrt((eb_a *
    eb_a + fb_a * fb_a) + gb_a * gb_a) - ((2.0 * u[12] * (((((u[18] * u[6] + u
    [19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2])
    + 2.0 * u[13] * (((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] *
                       u[0]) - u[10] * u[1]) - u[13] * u[2])) + 2.0 * u[14] *
    (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11] *
      u[1]) - u[14] * u[2])) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12])
    - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) / (2.0 * rt_powd_snf((hb_a *
    hb_a + ib_a * ib_a) + jb_a * jb_a, 1.5)))) - (u[18] - u[0]) * (u[15] * u[8]
    - u[17] * u[6]) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] *
    u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] * u[7] + u[19] * u[10]) +
    u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2]) / rt_powd_snf
              ((kb_a * kb_a + lb_a * lb_a) + mb_a * mb_a, 1.5)) - ((u[18] - u[0])
              / sqrt((nb_a * nb_a + ob_a * ob_a) + pb_a * pb_a) - (u[18] - u[0])
              * (qb_a * qb_a) / rt_powd_snf((rb_a * rb_a + sb_a * sb_a) + tb_a *
    tb_a, 1.5)) * (u[16] * u[8] - u[17] * u[7])) - (u[19] - u[1]) * (u[15] * u
             [11] - u[17] * u[9]) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] *
    u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] * u[7] +
    u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2])
            / rt_powd_snf((ub_a * ub_a + vb_a * vb_a) + wb_a * wb_a, 1.5)) + (u
            [19] - u[1]) * (u[15] * u[10] - u[16] * u[9]) * (((((u[18] * u[6] +
    u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2])
           * (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0])
               - u[11] * u[1]) - u[14] * u[2]) / rt_powd_snf((xb_a * xb_a + yb_a
             * yb_a) + ac_a * ac_a, 1.5)) - (u[20] - u[2]) * (u[15] * u[14] - u
           [17] * u[12]) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) -
             u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] * u[7] +
    u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2])
          / rt_powd_snf((bc_a * bc_a + cc_a * cc_a) + dc_a * dc_a, 1.5)) + (u[20]
    - u[2]) * (u[15] * u[13] - u[16] * u[12]) * (((((u[18] * u[6] + u[19] * u[9])
    + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] *
    u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11] * u[1]) - u[14]
    * u[2]) / rt_powd_snf((ec_a * ec_a + fc_a * fc_a) + gc_a * gc_a, 1.5);
}

/*
 * File trailer for cosbetadotsyms.c
 *
 * [EOF]
 */
