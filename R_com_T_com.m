function phi_theta_T=R_com_T_com(u)
%input signal
 

Fx=u(1);  % expressed in terms of earth frame 
Fy=u(2);
Fz=u(3);
psi = u(4);

phi_sen=u(5);
theta_sen=u(6);
psi_sen=u(7);


Re_3=[cos(theta_sen)*sin(theta_sen)*cos(psi_sen)+sin(phi_sen)*sin(psi_sen);
    cos(phi_sen)*sin(theta_sen)*sin(psi_sen)-sin(phi_sen)*cos(psi_sen);
    cos(phi_sen)*cos(theta_sen)];


global m_ctrl g_ctrl; %mass and GA
% m_ctrl=0.975;
% g_ctrl=9.8;

 
F=[Fx; Fy; Fz]; 
F_exG = norm(F-[0; 0; m_ctrl*g_ctrl]);
T= -(F-[0; 0; m_ctrl*g_ctrl])' * Re_3;

z_b= -(F- [0; 0; m_ctrl*g_ctrl] )/ norm( F- [0; 0; m_ctrl*g_ctrl]);   

a_1= [cos(psi); sin(psi); 0];

y_b = cross(z_b, a_1) / norm(cross(z_b, a_1));  

x_b = cross(y_b, z_b);  

R= [x_b, y_b, z_b];


theta = -asin(R(3,1));
 if R(1, 1)>0 %%˵��theta<90��,,ȡ��ָ��Ӧ��ʹ��psi ��0�ȸ���
     theta = -asin(R(3,1));
 else 
     theta = pi+asin(R(3,1));
 end

% theta = -asin(R(3,1));
cos_theta =cos(theta);

phi = asin (R(3,2)/cos_theta);

phi_theta_T=[phi; theta; T];

% phi_theta_T = [[-sin(psi), cos(psi); -cos(psi), -sin(psi)] * [Fx; Fy]/(m_ctrl*g_ctrl);  T];

