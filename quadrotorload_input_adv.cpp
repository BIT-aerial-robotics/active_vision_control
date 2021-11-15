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

#include "quadrotor_dynamics_L.cpp"
#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 19){ 
      mexErrMsgTxt("This problem expects 19 right hand side argument(s) since you have defined 19 MexInput(s)");
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
    DifferentialState cbeta;
    DifferentialState const_L;
    Control omega0_x;
    Control omega0_y;
    Control omega0_z;
    Control T_net;
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

    DifferentialEquation acadodata_f1;
    IntermediateState setc_is_1(22);
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
    setc_is_1(16) = cbeta;
    setc_is_1(17) = const_L;
    setc_is_1(18) = omega0_x;
    setc_is_1(19) = omega0_y;
    setc_is_1(20) = omega0_z;
    setc_is_1(21) = T_net;
    CFunction cLinkModel_1( 17, quadrotor_dynamics_L ); 
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
    ocp1.subjectTo(AT_START, cbeta == mexinput17);
    ocp1.subjectTo(AT_START, const_L == mexinput18);
    ocp1.subjectTo((-1.50000000000000000000e+00) <= omega0_x <= 1.50000000000000000000e+00);
    ocp1.subjectTo((-1.50000000000000000000e+00) <= omega0_y <= 1.50000000000000000000e+00);
    ocp1.subjectTo((-1.50000000000000000000e+00) <= omega0_z <= 1.50000000000000000000e+00);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-04 );
    algo1.set( MAX_NUM_ITERATIONS, 200 );
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

