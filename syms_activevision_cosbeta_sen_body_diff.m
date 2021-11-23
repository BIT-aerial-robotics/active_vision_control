
clear all; 

%the configuration of body frame relative to camera frame  
syms p_b_cx p_b_cy p_b_cz; 
syms ex_b_sx ex_b_sy ex_b_sz; 

%the configuration of body frame relative to lidar frame
syms p_b_lx p_b_ly p_b_lz; 
syms ex_b_lx ex_b_ly ex_b_lz; 

%the rotation matrix of body frame relative to lidar frame
syms rbl_11 rbl_12 rbl_13  rbl_21 rbl_22 rbl_23 rbl_31 rbl_32 rbl_33;
Rbl = [rbl_11, rbl_12, rbl_13; rbl_21, rbl_22, rbl_23; rbl_31, rbl_32, rbl_33];

%the rotation matrix of body frame relative to camera frame
syms rbc_11 rbc_12 rbc_13  rbc_21 rbc_22 rbc_23 rbc_31 rbc_32 rbc_33;
Rbc = [rbc_11, rbc_12, rbc_13; rbc_21, rbc_22, rbc_23; rbc_31, rbc_32, rbc_33];

%the transformation matrix of body frame relative to lidar frame  
H_b_l = [Rbl, [p_b_lx; p_b_ly; p_b_lz]; [0, 0, 0, 1]];

%the transformation matrix of body frame relative to camera frame  
H_b_c = [Rbc, [p_b_cx; p_b_cy; p_b_cz]; [0, 0, 0, 1]];


%the position of the vehicle:
syms x0_x  x0_y  x0_z; 
p0 = [x0_x; x0_y; x0_z]; 

%the velocity of the vehicle, in earth frame:
syms v0_x v0_y v0_z;
v0 = [v0_x; v0_y; v0_z];

% syms Fx Fy Fz m_ctrl g_ctrl;
% syms psi; 
% % psi = 0;
% e_1 = [1; 0; 0];
% e_3 = [0; 0; 1];
% F =[Fx; Fy; Fz]; 
% F_exG = norm_syms(F-[0; 0; m_ctrl*g_ctrl]);
%  
% z_b= -(F- [0; 0; m_ctrl*g_ctrl] )/ norm_syms( F- [0; 0; m_ctrl*g_ctrl]);   
% 
% a_1= [cos(psi); sin(psi); 0];
% y_b = cross(z_b, a_1) / norm_syms(cross(z_b, a_1));  
% x_b = cross(y_b, z_b);  

syms r0_11 r0_12 r0_13 r0_21 r0_22 r0_23 r0_31 r0_32 r0_33;

R_b = [r0_11, r0_12, r0_13; r0_21, r0_22, r0_23; r0_31, r0_32, r0_33];
R_b = simplify(R_b);

R_b_col = [r0_11; r0_12; r0_13; r0_21; r0_22; r0_23; r0_31; r0_32; r0_33];

% syms omega1 omega2 omega3; 
syms omega0_x omega0_y omega0_z
omega0 = [omega0_x; omega0_y; omega0_z];

R_b_dot = R_b*anti_symmetric(omega0);
R_b_dot_col = [R_b_dot(1,1); R_b_dot(1,2); R_b_dot(1,3); R_b_dot(2,1); R_b_dot(2,2); R_b_dot(2,3); R_b_dot(3,1); R_b_dot(3,2); R_b_dot(3,3)];

%Earth frame relative to body-fixed frame 
R_b_E = R_b.';
 
%the transformation matrix of body frame relative to earth frame 
H_b = [R_b, p0; [0, 0, 0, 1]];

%the transformation matrix of earth frame relative to body frame  
H_E_b = [R_b_E, -R_b_E*p0; [0, 0, 0, 1]];
H_E_b = simplify(H_E_b);


%the position of feature, in earth frame 
syms p_fx p_fy p_fz;
p_f = [p_fx; p_fy; p_fz]; 

%the position of feature, in body frame 
p_f_b = H_E_b*[p_f;1];
p_f_b = simplify(p_f_b);

%the position of feature, in camera frame 
p_f_c = H_b_c*p_f_b;
p_f_c = simplify(p_f_c);

%the position of feature, in lidar frame 
p_f_l = H_b_c*p_f_b;
p_f_l = simplify(p_f_l);


%the main axis of the camera:
e_1 = [1; 0; 0];

cosbeta = e_1'*p_f_c(1:3)/norm_syms(p_f_c(1:3));
cosbeta = simplify(cosbeta);

cosbeta_dot = jacobian(cosbeta, [p0; R_b_col])*[v0; R_b_dot_col]; 
cosbeta_dot = simplify(cosbeta_dot);


%the main axis of the lidar:
e_1 = [1; 0; 0];

cosbeta_l = e_1'*p_f_l(1:3)/norm_syms(p_f_l(1:3));
cosbeta_l = simplify(cosbeta_l);

cosbeta_dot_l = jacobian(cosbeta_l, [p0; R_b_col])*[v0; R_b_dot_col]; 
cosbeta_dot_l = simplify(cosbeta_dot_l);


