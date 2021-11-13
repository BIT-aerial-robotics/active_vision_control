clear all; clc; close all;

%%%%%%controller parameters:
global omega_trans_x_out omega_trans_y_out omega_trans_z_out  ksi_x_out ksi_y_out ksi_z_out;
omega_trans_x_out=0.1;
omega_trans_y_out=0.1;
omega_trans_z_out=0.5;
ksi_x_out =3;  ksi_y_out =3;  ksi_z_out =2;

global omega_trans_x_in omega_trans_y_in omega_trans_z_in ksi_x_in ksi_y_in  ksi_z_in;
omega_trans_x_in=0.4;
omega_trans_y_in=0.4;
omega_trans_z_in=0.8;
ksi_x_in =1.414; ksi_y_in=1.414;  ksi_z_in=2;

global omega_roll_out omega_pitch_out omega_yaw_out ksi_roll_out ksi_pitch_out ksi_yaw_out;
omega_roll_out=6;
omega_pitch_out=6;
omega_yaw_out=6;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 

global  omega_roll_in  omega_pitch_in omega_yaw_in ksi_roll_in ksi_pitch_in ksi_yaw_in;
omega_roll_in=8;
omega_pitch_in=8;
omega_yaw_in=8;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 


global omega_trans_x_out_fil omega_trans_y_out_fil omega_trans_z_out_fil  ksi_x_out_fil ksi_y_out_fil ksi_z_out_fil;
omega_trans_x_out_fil=10;
omega_trans_y_out_fil=10;
omega_trans_z_out_fil=10;
ksi_x_out_fil =1.414;  ksi_y_out_fil =1.414;  ksi_z_out_fil =1.414;

global omega_trans_x_in_fil omega_trans_y_in_fil omega_trans_z_in_fil ksi_x_in_fil ksi_y_in_fil  ksi_z_in_fil;
omega_trans_x_in_fil=5;
omega_trans_y_in_fil=5;
omega_trans_z_in_fil=5;
ksi_x_in_fil =1.414; ksi_y_in_fil=1.414;  ksi_z_in_fil=1.414;

global omega_rot_x_out_fil omega_rot_y_out_fil omega_rot_z_out_fil  ksi_phi_fil ksi_pitch_fil ksi_yaw_fil;
omega_rot_x_out_fil=10;
omega_rot_y_out_fil=10;
omega_rot_z_out_fil=10;
ksi_phi_fil =1.414;  ksi_pitch_fil =1.414;  ksi_yaw_fil =1.414;

global omega_rot_x_in_fil omega_rot_y_in_fil omega_rot_z_in_fil  ksi_p_fil ksi_q_fil ksi_r_fil;
omega_rot_x_in_fil=10;
omega_rot_y_in_fil=10;
omega_rot_z_in_fil=10;
ksi_p_fil =1.414;  ksi_q_fil =1.414;  ksi_r_fil =1.414;

global XP XI XD YP YI YD ZP ZI ZD; 
XP =0.1;  XI = 0.1;  XD = 1.4;  YP = 0.8;  YI= 0.002;  YD = 0.4;  ZP = 1;  ZI=0.02;  ZD=2; 
XP =0.1;  XI = 0.1;  XD = 1.4;  YP = 0.8;  YI= 0.1;  YD = 0.4;  ZP = 1;  ZI=0.02;  ZD=2; 

global Ixx Iyy Izz Ixz;
Ixx=0.0294853730990492 * 1;
Iyy=0.0636259128602231 * 1;
Izz=0.0809362287249525 * 1;
Ixz=0.0000;

global m_ctrl g_ctrl;
m_ctrl=2.1;
g_ctrl=9.8;

global l_ctrl;%length of cable 
l_ctrl=1;


%%%%%%plant parameters:
global I J_r m g Cd;
I=diag([0.0294853730990492; 0.0636259128602231; 0.0809362287249525]);  %inertia parameters
J_r = 22895.55/1e9; %�����ת������
m=2.1; %mass
g=9.8; %accelaration due to gravity
Cd= 0.02; %drag force coefficient

global mL; %the mass of the load 
global l_load; %the length of load

mL=2;
l_load=2;

global omega_n_m xi_m; %%the frequency and the damping ration of the actuation dynamics of motors
omega_n_m =30; 
xi_m=0.707;

global kx_lp kv_lp;

%tlc control for load attitude 
global omegan1 omegan2 omegan3; 
global xi1 xi2 xi3; 
omegan1=0.4; %a little ok
omegan2=0.4;
omegan3=0.4;
xi1=0.707;
xi2=0.707;
xi3=1.414;

% omegan1=2;
% omegan2=2;
% omegan3=2;
% xi1=0.707;
% xi2=0.707;
% xi3=1.414;

omegan1=0.8; %a little ok
omegan2=0.8;
omegan3=0.8;
xi1=0.707;
xi2=0.707;
xi3=1.414;

omegan1=1; %adafe little ok
omegan2=1;
omegan3=1;
xi1=1.414;
xi2=1.414;
xi3=1.414;


global omega_l_x_in omega_l_y_in omega_l_z_in  xi_l_x_in xi_l_y_in xi_l_z_in;
omega_l_x_in=1; %adafe little ok
omega_l_y_in=1;
omega_l_z_in=1;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;



omegan1=0.8;  
omegan2=0.8;
omegan3=0.8;
xi1=1.414;
xi2=1.414;
xi3=1.414;
omega_l_x_in=0.8; 
omega_l_y_in=0.8;
omega_l_z_in=0.8;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;


%a little ok
omegan1=0.8;  
omegan2=0.8;
omegan3=0.8;
xi1=1.414;xi2=1.414;xi3=1.414;
omega_l_x_in=0.5; 
omega_l_y_in=0.5;
omega_l_z_in=0.5;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;
omega_roll_out=2;
omega_pitch_out=2;
omega_yaw_out=2;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 
omega_roll_in=6;
omega_pitch_in=6;
omega_yaw_in=6;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 


%a little ok, load position control a little ok, 22, April, 2017
omegan1=0.8;  
omegan2=0.8;
omegan3=0.8;
xi1=1.414;xi2=1.414;xi3=1.414;
omega_l_x_in=1; 
omega_l_y_in=1;
omega_l_z_in=1;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;
omega_roll_out=3;
omega_pitch_out=3;
omega_yaw_out=3;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 
omega_roll_in=6;
omega_pitch_in=6;
omega_yaw_in=6;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 

%  load position control ok, 22 April, 2017
omegan1=0.5;  
omegan2=0.5;
omegan3=0.5;
xi1=1.414;xi2=1.414;xi3=1.414;
omega_l_x_in=0.8; 
omega_l_y_in=0.8;
omega_l_z_in=0.8;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;
omega_roll_out=2;
omega_pitch_out=2;
omega_yaw_out=2;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 
omega_roll_in=6;
omega_pitch_in=6;
omega_yaw_in=6;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 
kx_lp=1; kv_lp=1;


%finer than above one 
omegan1=0.5;  
omegan2=0.5;
omegan3=0.5;
xi1=1.414;xi2=1.414;xi3=1.414;
omega_l_x_in=0.8; 
omega_l_y_in=0.8;
omega_l_z_in=0.8;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;
omega_roll_out=2;
omega_pitch_out=2;
omega_yaw_out=2;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 
omega_roll_in=6;
omega_pitch_in=6;
omega_yaw_in=6;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 
kx_lp=0.8; kv_lp=1;

%in test
omegan1=0.5;  
omegan2=0.5;
omegan3=0.5;
xi1=1.414;xi2=1.414;xi3=1.414;
omega_l_x_in=0.8; 
omega_l_y_in=0.8;
omega_l_z_in=0.8;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;
omega_roll_out=2;
omega_pitch_out=2;
omega_yaw_out=2;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 
omega_roll_in=6;
omega_pitch_in=6;
omega_yaw_in=6;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 
kx_lp=0.8; kv_lp=1;



%in test, May 18, 2017
omegan1=0.5;  
omegan2=0.5;
omegan3=0.5;
xi1=1.414;xi2=1.414;xi3=1.414;
omega_l_x_in=0.8; 
omega_l_y_in=0.8;
omega_l_z_in=0.8;
xi_l_x_in =1.414;  xi_l_y_in =1.414;  xi_l_z_in =1.414;
omega_roll_out=2;
omega_pitch_out=2;
omega_yaw_out=2;
ksi_roll_out=1.414; ksi_pitch_out=1.414;  ksi_yaw_out=1.414; 
omega_roll_in=6;
omega_pitch_in=6;
omega_yaw_in=6;
ksi_roll_in=1.414;  ksi_pitch_in =1.414;  ksi_yaw_in =1.414; 
kx_lp=1; kv_lp=1;



%parameters in global control
global k_delta;
k_delta=10;

global  omega_R xi_R;
omega_R=20;
xi_R=1.414;

global omega_Rdot xi_Rdot;
omega_Rdot=20;
xi_Rdot=1.414;

global kxi komega;
kxi=0.8*diag([4;4;4]);
komega=2*diag([1;1;1]);


global I0_x I0_y I0_z; %the inertia paramters of the load
global m0; %the mass of the load 

global I1_x I1_y I1_z; 
global m1;  

global I2_x I2_y I2_z;  
global m2;  

global I3_x I3_y I3_z;  
global m3;  

global I4_x I4_y I4_z;  
global m4;  

global I5_x I5_y I5_z;  
global m5;  

global I6_x I6_y I6_z;  
global m6;  

global rou1_x rou1_y rou1_z; 
global rou2_x rou2_y rou2_z; 
global rou3_x rou3_y rou3_z; 
global rou4_x rou4_y rou4_z; 
global rou5_x rou5_y rou5_z; 
global rou6_x rou6_y rou6_z; 

global l1 l2 l3 l4 l5 l6; %the length of the links 


I0_x = 1; I0_y = 1; I0_z= 1;  %the inertia paramters of the load
m0 = 1;   %the mass of the load 

 I1_x  = 1;  I1_y  = 1;  I1_z  = 1;  
 m1  = 1;   

 I2_x  = 1;  I2_y = 1; I2_z  = 1;   
 m2 = 1; 

 I3_x = 1;  I3_y = 1;  I3_z = 1;  
 m3  = 1;   

 I4_x  = 1;  I4_y  = 1;  I4_z  = 1;   
 m4 = 1;  

 I5_x  = 1;  I5_y  = 1;  I5_z = 1;   
 m5  = 1;   

 I6_x  = 1;  I6_y = 1;  I6_z = 1;   
 m6 = 1;  

 rou1_x  = 1;  rou1_y  = 0;  rou1_z  = 0;  
 rou2_x  = 1;  rou2_y  = -1;  rou2_z  = 0;  
 rou3_x  = -1;  rou3_y  = -1;  rou3_z = 0;  
 rou4_x  = -1;  rou4_y   = 0; rou4_z  = 0;  
 rou5_x  = -1;  rou5_y  = 1; rou5_z = 0;  
 rou6_x = 1;  rou6_y = 1;  rou6_z = 0;  

 l1 = 1;  l2 = 1;  l3 = 1;  l4 = 1;  l5  = 1; l6 = 1;  %the length of the links 

 
 global n_quadrotor;
 n_quadrotor = 6;
 

