

%the configuration of camera relative to the body frame 
syms p_b_sx p_b_sy p_b_sz; 
syms ex_b_sx ex_b_sy ex_b_sz; 

%the configuration of lidar relative to the body frame 
syms p_b_lx p_b_ly p_b_lz; 
syms ex_b_lx ex_b_ly ex_b_lz; 


%the position of the vehicle:
syms x0_x  x0_y  x0_z; 
p0 = [x0_x; x0_y; x0_z]; 

syms Fx Fy Fz m_ctrl g_ctrl;
syms psi; 
% psi = 0;
e_1 = [1; 0; 0];
e_3 = [0; 0; 1];
F =[Fx; Fy; Fz]; 
F_exG = norm_syms(F-[0; 0; m_ctrl*g_ctrl]);
 
z_b= -(F- [0; 0; m_ctrl*g_ctrl] )/ norm_syms( F- [0; 0; m_ctrl*g_ctrl]);   

a_1= [cos(psi); sin(psi); 0];
y_b = cross(z_b, a_1) / norm_syms(cross(z_b, a_1));  
x_b = cross(y_b, z_b);  

R_b = [x_b, y_b, z_b];
R_b = simplify(R_b);

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

cosbeta = e_1'*p_f_b(1:3)/norm_syms(p_f_b(1:3));
cosbeta = simplify(cosbeta);

