/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include "quadrotor_load_dynamics_L.cpp"
#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 31){ 
      mexErrMsgTxt("This problem expects 31 right hand side argument(s) since you have defined 31 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState x0_x;
    DifferentialState x0_y;
    DifferentialState x0_z;
    DifferentialState v0_x;
    DifferentialState v0_y;
    DifferentialState v0_z;
    DifferentialState r0_11;
    DifferentialState r0_12;
    DifferentialState r0_13;
    DifferentialState r0_21;
    DifferentialState r0_22;
    DifferentialState r0_23;
    DifferentialState r0_31;
    DifferentialState r0_32;
    DifferentialState r0_33;
    DifferentialState omega0_x;
    DifferentialState omega0_y;
    DifferentialState omega0_z;
    DifferentialState cosbeta;
    DifferentialState rBC_11;
    DifferentialState rBC_12;
    DifferentialState rBC_13;
    DifferentialState rBC_21;
    DifferentialState rBC_22;
    DifferentialState rBC_23;
    DifferentialState rBC_31;
    DifferentialState rBC_32;
    DifferentialState rBC_33;
    DifferentialState const_L;
    Control f_x;
    Control f_y;
    Control f_z;
    Control tau_x;
    Control tau_y;
    Control tau_z;
    Control omegaBC_x;
    Control omegaBC_y;
    Control omegaBC_z;
    double *mexinput0_temp = NULL; 
    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || !(mxGetM(prhs[0])==1 && mxGetN(prhs[0])==1) ) { 
      mexErrMsgTxt("Input 0 must be a noncomplex scalar double.");
    } 
    mexinput0_temp = mxGetPr(prhs[0]); 
    double mexinput0 = *mexinput0_temp; 

    double *mexinput1_temp = NULL; 
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || !(mxGetM(prhs[1])==1 && mxGetN(prhs[1])==1) ) { 
      mexErrMsgTxt("Input 1 must be a noncomplex scalar double.");
    } 
    mexinput1_temp = mxGetPr(prhs[1]); 
    double mexinput1 = *mexinput1_temp; 

    double *mexinput2_temp = NULL; 
    if( !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || !(mxGetM(prhs[2])==1 && mxGetN(prhs[2])==1) ) { 
      mexErrMsgTxt("Input 2 must be a noncomplex scalar double.");
    } 
    mexinput2_temp = mxGetPr(prhs[2]); 
    double mexinput2 = *mexinput2_temp; 

    double *mexinput3_temp = NULL; 
    if( !mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || !(mxGetM(prhs[3])==1 && mxGetN(prhs[3])==1) ) { 
      mexErrMsgTxt("Input 3 must be a noncomplex scalar double.");
    } 
    mexinput3_temp = mxGetPr(prhs[3]); 
    double mexinput3 = *mexinput3_temp; 

    double *mexinput4_temp = NULL; 
    if( !mxIsDouble(prhs[4]) || mxIsComplex(prhs[4]) || !(mxGetM(prhs[4])==1 && mxGetN(prhs[4])==1) ) { 
      mexErrMsgTxt("Input 4 must be a noncomplex scalar double.");
    } 
    mexinput4_temp = mxGetPr(prhs[4]); 
    double mexinput4 = *mexinput4_temp; 

    double *mexinput5_temp = NULL; 
    if( !mxIsDouble(prhs[5]) || mxIsComplex(prhs[5]) || !(mxGetM(prhs[5])==1 && mxGetN(prhs[5])==1) ) { 
      mexErrMsgTxt("Input 5 must be a noncomplex scalar double.");
    } 
    mexinput5_temp = mxGetPr(prhs[5]); 
    double mexinput5 = *mexinput5_temp; 

    double *mexinput6_temp = NULL; 
    if( !mxIsDouble(prhs[6]) || mxIsComplex(prhs[6]) || !(mxGetM(prhs[6])==1 && mxGetN(prhs[6])==1) ) { 
      mexErrMsgTxt("Input 6 must be a noncomplex scalar double.");
    } 
    mexinput6_temp = mxGetPr(prhs[6]); 
    double mexinput6 = *mexinput6_temp; 

    double *mexinput7_temp = NULL; 
    if( !mxIsDouble(prhs[7]) || mxIsComplex(prhs[7]) || !(mxGetM(prhs[7])==1 && mxGetN(prhs[7])==1) ) { 
      mexErrMsgTxt("Input 7 must be a noncomplex scalar double.");
    } 
    mexinput7_temp = mxGetPr(prhs[7]); 
    double mexinput7 = *mexinput7_temp; 

    double *mexinput8_temp = NULL; 
    if( !mxIsDouble(prhs[8]) || mxIsComplex(prhs[8]) || !(mxGetM(prhs[8])==1 && mxGetN(prhs[8])==1) ) { 
      mexErrMsgTxt("Input 8 must be a noncomplex scalar double.");
    } 
    mexinput8_temp = mxGetPr(prhs[8]); 
    double mexinput8 = *mexinput8_temp; 

    double *mexinput9_temp = NULL; 
    if( !mxIsDouble(prhs[9]) || mxIsComplex(prhs[9]) || !(mxGetM(prhs[9])==1 && mxGetN(prhs[9])==1) ) { 
      mexErrMsgTxt("Input 9 must be a noncomplex scalar double.");
    } 
    mexinput9_temp = mxGetPr(prhs[9]); 
    double mexinput9 = *mexinput9_temp; 

    double *mexinput10_temp = NULL; 
    if( !mxIsDouble(prhs[10]) || mxIsComplex(prhs[10]) || !(mxGetM(prhs[10])==1 && mxGetN(prhs[10])==1) ) { 
      mexErrMsgTxt("Input 10 must be a noncomplex scalar double.");
    } 
    mexinput10_temp = mxGetPr(prhs[10]); 
    double mexinput10 = *mexinput10_temp; 

    double *mexinput11_temp = NULL; 
    if( !mxIsDouble(prhs[11]) || mxIsComplex(prhs[11]) || !(mxGetM(prhs[11])==1 && mxGetN(prhs[11])==1) ) { 
      mexErrMsgTxt("Input 11 must be a noncomplex scalar double.");
    } 
    mexinput11_temp = mxGetPr(prhs[11]); 
    double mexinput11 = *mexinput11_temp; 

    double *mexinput12_temp = NULL; 
    if( !mxIsDouble(prhs[12]) || mxIsComplex(prhs[12]) || !(mxGetM(prhs[12])==1 && mxGetN(prhs[12])==1) ) { 
      mexErrMsgTxt("Input 12 must be a noncomplex scalar double.");
    } 
    mexinput12_temp = mxGetPr(prhs[12]); 
    double mexinput12 = *mexinput12_temp; 

    double *mexinput13_temp = NULL; 
    if( !mxIsDouble(prhs[13]) || mxIsComplex(prhs[13]) || !(mxGetM(prhs[13])==1 && mxGetN(prhs[13])==1) ) { 
      mexErrMsgTxt("Input 13 must be a noncomplex scalar double.");
    } 
    mexinput13_temp = mxGetPr(prhs[13]); 
    double mexinput13 = *mexinput13_temp; 

    double *mexinput14_temp = NULL; 
    if( !mxIsDouble(prhs[14]) || mxIsComplex(prhs[14]) || !(mxGetM(prhs[14])==1 && mxGetN(prhs[14])==1) ) { 
      mexErrMsgTxt("Input 14 must be a noncomplex scalar double.");
    } 
    mexinput14_temp = mxGetPr(prhs[14]); 
    double mexinput14 = *mexinput14_temp; 

    double *mexinput15_temp = NULL; 
    if( !mxIsDouble(prhs[15]) || mxIsComplex(prhs[15]) || !(mxGetM(prhs[15])==1 && mxGetN(prhs[15])==1) ) { 
      mexErrMsgTxt("Input 15 must be a noncomplex scalar double.");
    } 
    mexinput15_temp = mxGetPr(prhs[15]); 
    double mexinput15 = *mexinput15_temp; 

    double *mexinput16_temp = NULL; 
    if( !mxIsDouble(prhs[16]) || mxIsComplex(prhs[16]) || !(mxGetM(prhs[16])==1 && mxGetN(prhs[16])==1) ) { 
      mexErrMsgTxt("Input 16 must be a noncomplex scalar double.");
    } 
    mexinput16_temp = mxGetPr(prhs[16]); 
    double mexinput16 = *mexinput16_temp; 

    double *mexinput17_temp = NULL; 
    if( !mxIsDouble(prhs[17]) || mxIsComplex(prhs[17]) || !(mxGetM(prhs[17])==1 && mxGetN(prhs[17])==1) ) { 
      mexErrMsgTxt("Input 17 must be a noncomplex scalar double.");
    } 
    mexinput17_temp = mxGetPr(prhs[17]); 
    double mexinput17 = *mexinput17_temp; 

    double *mexinput18_temp = NULL; 
    if( !mxIsDouble(prhs[18]) || mxIsComplex(prhs[18]) || !(mxGetM(prhs[18])==1 && mxGetN(prhs[18])==1) ) { 
      mexErrMsgTxt("Input 18 must be a noncomplex scalar double.");
    } 
    mexinput18_temp = mxGetPr(prhs[18]); 
    double mexinput18 = *mexinput18_temp; 

    double *mexinput19_temp = NULL; 
    if( !mxIsDouble(prhs[19]) || mxIsComplex(prhs[19]) || !(mxGetM(prhs[19])==1 && mxGetN(prhs[19])==1) ) { 
      mexErrMsgTxt("Input 19 must be a noncomplex scalar double.");
    } 
    mexinput19_temp = mxGetPr(prhs[19]); 
    double mexinput19 = *mexinput19_temp; 

    double *mexinput20_temp = NULL; 
    if( !mxIsDouble(prhs[20]) || mxIsComplex(prhs[20]) || !(mxGetM(prhs[20])==1 && mxGetN(prhs[20])==1) ) { 
      mexErrMsgTxt("Input 20 must be a noncomplex scalar double.");
    } 
    mexinput20_temp = mxGetPr(prhs[20]); 
    double mexinput20 = *mexinput20_temp; 

    double *mexinput21_temp = NULL; 
    if( !mxIsDouble(prhs[21]) || mxIsComplex(prhs[21]) || !(mxGetM(prhs[21])==1 && mxGetN(prhs[21])==1) ) { 
      mexErrMsgTxt("Input 21 must be a noncomplex scalar double.");
    } 
    mexinput21_temp = mxGetPr(prhs[21]); 
    double mexinput21 = *mexinput21_temp; 

    double *mexinput22_temp = NULL; 
    if( !mxIsDouble(prhs[22]) || mxIsComplex(prhs[22]) || !(mxGetM(prhs[22])==1 && mxGetN(prhs[22])==1) ) { 
      mexErrMsgTxt("Input 22 must be a noncomplex scalar double.");
    } 
    mexinput22_temp = mxGetPr(prhs[22]); 
    double mexinput22 = *mexinput22_temp; 

    double *mexinput23_temp = NULL; 
    if( !mxIsDouble(prhs[23]) || mxIsComplex(prhs[23]) || !(mxGetM(prhs[23])==1 && mxGetN(prhs[23])==1) ) { 
      mexErrMsgTxt("Input 23 must be a noncomplex scalar double.");
    } 
    mexinput23_temp = mxGetPr(prhs[23]); 
    double mexinput23 = *mexinput23_temp; 

    double *mexinput24_temp = NULL; 
    if( !mxIsDouble(prhs[24]) || mxIsComplex(prhs[24]) || !(mxGetM(prhs[24])==1 && mxGetN(prhs[24])==1) ) { 
      mexErrMsgTxt("Input 24 must be a noncomplex scalar double.");
    } 
    mexinput24_temp = mxGetPr(prhs[24]); 
    double mexinput24 = *mexinput24_temp; 

    double *mexinput25_temp = NULL; 
    if( !mxIsDouble(prhs[25]) || mxIsComplex(prhs[25]) || !(mxGetM(prhs[25])==1 && mxGetN(prhs[25])==1) ) { 
      mexErrMsgTxt("Input 25 must be a noncomplex scalar double.");
    } 
    mexinput25_temp = mxGetPr(prhs[25]); 
    double mexinput25 = *mexinput25_temp; 

    double *mexinput26_temp = NULL; 
    if( !mxIsDouble(prhs[26]) || mxIsComplex(prhs[26]) || !(mxGetM(prhs[26])==1 && mxGetN(prhs[26])==1) ) { 
      mexErrMsgTxt("Input 26 must be a noncomplex scalar double.");
    } 
    mexinput26_temp = mxGetPr(prhs[26]); 
    double mexinput26 = *mexinput26_temp; 

    double *mexinput27_temp = NULL; 
    if( !mxIsDouble(prhs[27]) || mxIsComplex(prhs[27]) || !(mxGetM(prhs[27])==1 && mxGetN(prhs[27])==1) ) { 
      mexErrMsgTxt("Input 27 must be a noncomplex scalar double.");
    } 
    mexinput27_temp = mxGetPr(prhs[27]); 
    double mexinput27 = *mexinput27_temp; 

    double *mexinput28_temp = NULL; 
    if( !mxIsDouble(prhs[28]) || mxIsComplex(prhs[28]) || !(mxGetM(prhs[28])==1 && mxGetN(prhs[28])==1) ) { 
      mexErrMsgTxt("Input 28 must be a noncomplex scalar double.");
    } 
    mexinput28_temp = mxGetPr(prhs[28]); 
    double mexinput28 = *mexinput28_temp; 

    double *mexinput29_temp = NULL; 
    if( !mxIsDouble(prhs[29]) || mxIsComplex(prhs[29]) || !(mxGetM(prhs[29])==1 && mxGetN(prhs[29])==1) ) { 
      mexErrMsgTxt("Input 29 must be a noncomplex scalar double.");
    } 
    mexinput29_temp = mxGetPr(prhs[29]); 
    double mexinput29 = *mexinput29_temp; 

    double *mexinput30_temp = NULL; 
    if( !mxIsDouble(prhs[30]) || mxIsComplex(prhs[30]) || !(mxGetM(prhs[30])==1 && mxGetN(prhs[30])==1) ) { 
      mexErrMsgTxt("Input 30 must be a noncomplex scalar double.");
    } 
    mexinput30_temp = mxGetPr(prhs[30]); 
    double mexinput30 = *mexinput30_temp; 

    DifferentialEquation acadodata_f1;
    IntermediateState setc_is_1(39);
    setc_is_1(0) = autotime;
    setc_is_1(1) = x0_x;
    setc_is_1(2) = x0_y;
    setc_is_1(3) = x0_z;
    setc_is_1(4) = v0_x;
    setc_is_1(5) = v0_y;
    setc_is_1(6) = v0_z;
    setc_is_1(7) = r0_11;
    setc_is_1(8) = r0_12;
    setc_is_1(9) = r0_13;
    setc_is_1(10) = r0_21;
    setc_is_1(11) = r0_22;
    setc_is_1(12) = r0_23;
    setc_is_1(13) = r0_31;
    setc_is_1(14) = r0_32;
    setc_is_1(15) = r0_33;
    setc_is_1(16) = omega0_x;
    setc_is_1(17) = omega0_y;
    setc_is_1(18) = omega0_z;
    setc_is_1(19) = cosbeta;
    setc_is_1(20) = rBC_11;
    setc_is_1(21) = rBC_12;
    setc_is_1(22) = rBC_13;
    setc_is_1(23) = rBC_21;
    setc_is_1(24) = rBC_22;
    setc_is_1(25) = rBC_23;
    setc_is_1(26) = rBC_31;
    setc_is_1(27) = rBC_32;
    setc_is_1(28) = rBC_33;
    setc_is_1(29) = const_L;
    setc_is_1(30) = f_x;
    setc_is_1(31) = f_y;
    setc_is_1(32) = f_z;
    setc_is_1(33) = tau_x;
    setc_is_1(34) = tau_y;
    setc_is_1(35) = tau_z;
    setc_is_1(36) = omegaBC_x;
    setc_is_1(37) = omegaBC_y;
    setc_is_1(38) = omegaBC_z;
    CFunction cLinkModel_1( 29, quadrotor_load_dynamics_L ); 
    acadodata_f1 << cLinkModel_1(setc_is_1); 

    OCP ocp1(mexinput0, mexinput1, 10);
    ocp1.minimizeMayerTerm(const_L);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, x0_x == mexinput2);
    ocp1.subjectTo(AT_START, x0_y == mexinput3);
    ocp1.subjectTo(AT_START, x0_z == mexinput4);
    ocp1.subjectTo(AT_START, v0_x == mexinput5);
    ocp1.subjectTo(AT_START, v0_y == mexinput6);
    ocp1.subjectTo(AT_START, v0_z == mexinput7);
    ocp1.subjectTo(AT_START, r0_11 == mexinput8);
    ocp1.subjectTo(AT_START, r0_12 == mexinput9);
    ocp1.subjectTo(AT_START, r0_13 == mexinput10);
    ocp1.subjectTo(AT_START, r0_21 == mexinput11);
    ocp1.subjectTo(AT_START, r0_22 == mexinput12);
    ocp1.subjectTo(AT_START, r0_23 == mexinput13);
    ocp1.subjectTo(AT_START, r0_31 == mexinput14);
    ocp1.subjectTo(AT_START, r0_32 == mexinput15);
    ocp1.subjectTo(AT_START, r0_33 == mexinput16);
    ocp1.subjectTo(AT_START, omega0_x == mexinput17);
    ocp1.subjectTo(AT_START, omega0_y == mexinput18);
    ocp1.subjectTo(AT_START, omega0_z == mexinput19);
    ocp1.subjectTo(AT_START, cosbeta == mexinput20);
    ocp1.subjectTo(AT_START, rBC_11 == mexinput21);
    ocp1.subjectTo(AT_START, rBC_12 == mexinput22);
    ocp1.subjectTo(AT_START, rBC_13 == mexinput23);
    ocp1.subjectTo(AT_START, rBC_21 == mexinput24);
    ocp1.subjectTo(AT_START, rBC_22 == mexinput25);
    ocp1.subjectTo(AT_START, rBC_23 == mexinput26);
    ocp1.subjectTo(AT_START, rBC_31 == mexinput27);
    ocp1.subjectTo(AT_START, rBC_32 == mexinput28);
    ocp1.subjectTo(AT_START, rBC_33 == mexinput29);
    ocp1.subjectTo(AT_START, const_L == mexinput30);
    ocp1.subjectTo((-9.00000000000000000000e+01) <= f_z <= (-5.00000000000000000000e+01));
    ocp1.subjectTo((-1.00000000000000000000e+04+pow(f_x,2.00000000000000000000e+00)+pow(f_y,2.00000000000000000000e+00)+pow(f_z,2.00000000000000000000e+00)) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+01+pow(tau_x,2.00000000000000000000e+00)+pow(tau_y,2.00000000000000000000e+00)+pow(tau_z,2.00000000000000000000e+00)) <= 0.00000000000000000000e+00);
    ocp1.subjectTo(5.00000000000000111022e-01 <= (2.00000000000000000000e+00*r0_11*rBC_13+2.00000000000000000000e+00*r0_12*rBC_23+2.00000000000000000000e+00*r0_13*rBC_33-4.00000000000000000000e+00*pow(r0_11,2.00000000000000000000e+00)*rBC_13-4.00000000000000000000e+00*pow(r0_12,2.00000000000000000000e+00)*rBC_23-4.00000000000000000000e+00*pow(r0_13,2.00000000000000000000e+00)*rBC_33-4.00000000000000000000e+00*pow(r0_21,2.00000000000000000000e+00)*rBC_13-4.00000000000000000000e+00*pow(r0_22,2.00000000000000000000e+00)*rBC_23-4.00000000000000000000e+00*pow(r0_23,2.00000000000000000000e+00)*rBC_33-4.00000000000000000000e+00*pow(r0_31,2.00000000000000000000e+00)*rBC_13-4.00000000000000000000e+00*pow(r0_32,2.00000000000000000000e+00)*rBC_23-4.00000000000000000000e+00*pow(r0_33,2.00000000000000000000e+00)*rBC_33-4.00000000000000000000e+00*r0_11*r0_12*rBC_13-4.00000000000000000000e+00*r0_11*r0_12*rBC_23-4.00000000000000000000e+00*r0_11*r0_13*rBC_13-4.00000000000000000000e+00*r0_11*r0_13*rBC_33-4.00000000000000000000e+00*r0_12*r0_13*rBC_23-4.00000000000000000000e+00*r0_12*r0_13*rBC_33-4.00000000000000000000e+00*r0_21*r0_22*rBC_13-4.00000000000000000000e+00*r0_21*r0_22*rBC_23-4.00000000000000000000e+00*r0_21*r0_23*rBC_13-4.00000000000000000000e+00*r0_21*r0_23*rBC_33-4.00000000000000000000e+00*r0_22*r0_23*rBC_23-4.00000000000000000000e+00*r0_22*r0_23*rBC_33-4.00000000000000000000e+00*r0_31*r0_32*rBC_13-4.00000000000000000000e+00*r0_31*r0_32*rBC_23-4.00000000000000000000e+00*r0_31*r0_33*rBC_13-4.00000000000000000000e+00*r0_31*r0_33*rBC_33-4.00000000000000000000e+00*r0_32*r0_33*rBC_23-4.00000000000000000000e+00*r0_32*r0_33*rBC_33-r0_11*rBC_13*x0_x-r0_12*rBC_23*x0_x-r0_13*rBC_33*x0_x-r0_21*rBC_13*x0_y-r0_22*rBC_23*x0_y-r0_23*rBC_33*x0_y-r0_31*rBC_13*x0_z-r0_32*rBC_23*x0_z-r0_33*rBC_33*x0_z)/sqrt((pow((-(4.00000000000000000000e+00*r0_11+4.00000000000000000000e+00*r0_12+4.00000000000000000000e+00*r0_13+x0_x)*(r0_11*rBC_11+r0_12*rBC_21+r0_13*rBC_31)-(4.00000000000000000000e+00*r0_21+4.00000000000000000000e+00*r0_22+4.00000000000000000000e+00*r0_23+x0_y)*(r0_21*rBC_11+r0_22*rBC_21+r0_23*rBC_31)-(4.00000000000000000000e+00*r0_31+4.00000000000000000000e+00*r0_32+4.00000000000000000000e+00*r0_33+x0_z)*(r0_31*rBC_11+r0_32*rBC_21+r0_33*rBC_31)+(r0_11*rBC_11+r0_12*rBC_21+r0_13*rBC_31)*2.00000000000000000000e+00),2.00000000000000000000e+00)+pow((-(4.00000000000000000000e+00*r0_11+4.00000000000000000000e+00*r0_12+4.00000000000000000000e+00*r0_13+x0_x)*(r0_11*rBC_12+r0_12*rBC_22+r0_13*rBC_32)-(4.00000000000000000000e+00*r0_21+4.00000000000000000000e+00*r0_22+4.00000000000000000000e+00*r0_23+x0_y)*(r0_21*rBC_12+r0_22*rBC_22+r0_23*rBC_32)-(4.00000000000000000000e+00*r0_31+4.00000000000000000000e+00*r0_32+4.00000000000000000000e+00*r0_33+x0_z)*(r0_31*rBC_12+r0_32*rBC_22+r0_33*rBC_32)+(r0_11*rBC_12+r0_12*rBC_22+r0_13*rBC_32)*2.00000000000000000000e+00),2.00000000000000000000e+00)+pow((-(4.00000000000000000000e+00*r0_11+4.00000000000000000000e+00*r0_12+4.00000000000000000000e+00*r0_13+x0_x)*(r0_11*rBC_13+r0_12*rBC_23+r0_13*rBC_33)-(4.00000000000000000000e+00*r0_21+4.00000000000000000000e+00*r0_22+4.00000000000000000000e+00*r0_23+x0_y)*(r0_21*rBC_13+r0_22*rBC_23+r0_23*rBC_33)-(4.00000000000000000000e+00*r0_31+4.00000000000000000000e+00*r0_32+4.00000000000000000000e+00*r0_33+x0_z)*(r0_31*rBC_13+r0_32*rBC_23+r0_33*rBC_33)+(r0_11*rBC_13+r0_12*rBC_23+r0_13*rBC_33)*2.00000000000000000000e+00),2.00000000000000000000e+00))) <= 1.00000000000000000000e+00);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-04 );
    algo1.set( MAX_NUM_ITERATIONS, 20 );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

