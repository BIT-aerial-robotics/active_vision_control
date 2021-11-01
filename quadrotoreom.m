function [sys,x0,str,ts]= quadrotoreom(t,x,u,flag)
switch flag
    case 0   %���ó�ʼ������
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);
    case 1   %��������״̬�ĸ��º���
        sys=mdlDerivatives(t,x,u);
    case 3   %����������ļ��㺯��?
        sys=mdlOutputs(t,x,u);
    case {2,4,9}  %δʹ�ù��flagֵ
        sys=[];
    otherwise %�������?
        error(['Unhandled flag=',num2str(flag)]);
end
%=======================================================================
%��flagΪ0ʱ�������ϵͳ�ĳ�ʼ��?
%=======================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u)
%���ȵ���simsizes����ó�ϵͳ��ģ����sizes,�������ɢϵͳ��ʵ���������
%sizes����
global n_quadrotor;
sizes=simsizes;
sizes.NumContStates= 18;
sizes.NumDiscStates=0;
sizes.NumOutputs= 18;
sizes.NumInputs= n_quadrotor*3;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
 

x0 = [  [0;0;0];  ...  % the position of the load
[0;0;0]; ... %the velocity of the load
[1;0;0;0;1;0;0;0;1] ;...
[0;0;0]]; %the rotation matrix of the load


%q0=s2_traj_gen(0);
%xl_out0=load_position_gen(0);

str=[];
ts=[0 0];

%=======================================================================
%��flagΪ1ʱ,�����������ϵͳ��״̬����?
%=======================================================================
function sys=mdlDerivatives(t,x,u)

%the number of the quadrotors:
global n_quadrotor;
n_quadrotor = 6; 

%the state of the links:
q_i = zeros(3*n_quadrotor,1);
omega_i = zeros(3*n_quadrotor,1);
q_i_dot = zeros(3*n_quadrotor, 1); 

%the state of the quadrotors, should be vectors, in order to be output in
%this function 
R_quadrotor_i = zeros(3*n_quadrotor, 1); 
omega_quadrotor_i = zeros(3*n_quadrotor, 1); 
R_quadrotor_i_dot = zeros(9*n_quadrotor, 1); 
omega_quadrotor_i_dot = zeros(3*n_quadrotor, 1);

%% state variables
%state of the load: 
xL=x(1:3);
vL=x(4:6);
RL=[x(7), x(8), x(9); 
   x(10), x(11), x(12);
   x(13), x(14), x(15)];
omegaL=x(16:18);

%in order to allocate the state and input varibles to the symbols, which
%are used in the symbol calculation 
q_i_matrix = zeros(3,n_quadrotor);
omega_i_matrix = zeros(3,n_quadrotor); 
 
 

t1_x = u(1); t1_y = u(2); t1_z = u(3);  
t2_x = u(4); t2_y = u(5); t2_z = u(6);  
t3_x = u(7); t3_y = u(8); t3_z = u(9);  
t4_x = u(10); t4_y = u(11); t4_z = u(12);  
t5_x = u(13); t5_y = u(14); t5_z = u(15);  
t6_x = u(16); t6_y = u(17); t6_z = u(18);  
thrust_matrix = [t1_x; t1_y;  t1_z;
t2_x;  t2_y;  t2_z;
t3_x;  t3_y;  t3_z;  
t4_x;  t4_y;  t4_z;  
t5_x;  t5_y;  t5_z;  
t6_x;  t6_y;  t6_z]; 


%% parameters:
I_n = zeros(3,3,n_quadrotor);   %store the inertia matrix of the multiple quadrotors, in order to write the dynamics 
tau_n = zeros(3,n_quadrotor);  %store the input torque for the quadrotor, in order to write the dynamics 
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

%allocate the inertia parameters to the total matrix of In in this program 
I_n(:,:,1) = diag([ I1_x I1_y I1_z]); 
I_n(:,:,2) = diag([ I2_x I2_y I2_z]); 
I_n(:,:,3) = diag([ I3_x I3_y I3_z]); 
I_n(:,:,4) = diag([ I4_x I4_y I4_z]); 
I_n(:,:,5) = diag([ I5_x I5_y I5_z]); 
I_n(:,:,6) = diag([ I6_x I6_y I6_z]); 
% tau_n(:,1) = [tau1_x; tau1_y; tau1_z];
% tau_n(:,2) = [tau2_x; tau2_y; tau2_z];
% tau_n(:,3) = [tau3_x; tau3_y; tau3_z];
% tau_n(:,4) = [tau4_x; tau4_y; tau4_z];
% tau_n(:,5) = [tau5_x; tau5_y; tau5_z];
% tau_n(:,6) = [tau6_x; tau6_y; tau6_z];

% kinematics of the load:
RL_dot = RL*anti_symmetric(omegaL); 
RL_dot_vector = [RL_dot(1,:)'; RL_dot(2,:)'; RL_dot(3,:)']; %used for output 

%the position of the COM of the entire system, expressed in the body frame of the load: 
m_vector = [m0; m1; m2; m3; m4; m5; m6];
mass_sum = m0+m1+m2+m3+m4+m5+m6;
I_0 = diag([I0_x I0_y I0_z]);
rou_i = [ rou1_x rou1_y rou1_z; 
 rou2_x rou2_y rou2_z; 
 rou3_x rou3_y rou3_z; 
 rou4_x rou4_y rou4_z; 
 rou5_x rou5_y rou5_z; 
 rou6_x rou6_y rou6_z];

rou_i = rou_i';  

sum_pos_mass = [zeros(3,1), rou_i]*m_vector; 
x_c_0 = sum_pos_mass/mass_sum; 


I_sum = zeros(3,3); 
for i = 1: n_quadrotor
	I_sum = m_vector(i+1)*anti_symmetric(rou_i(:,i))*anti_symmetric(rou_i(:,i) - x_c_0) + I_sum; 
end

I_sum  = I_0 - I_sum;  %the summary inertia matrix

e3=[0;0;1];
acc_g = 9.8;
mass_matrix = [mass_sum * eye(3,3), zeros(3,3); zeros(3,3), I_sum];
C_matrix = [anti_symmetric(mass_sum*omegaL), zeros(3,3); zeros(3,3), -anti_symmetric(I_sum*omegaL)]; 
G_matrix = [-mass_sum*acc_g*RL'*e3; zeros(3,1)]; 
input_equi_allocation = [eye(3,3), eye(3,3), eye(3,3), eye(3,3), eye(3,3), eye(3,3);...
    anti_symmetric(rou_i(:,1)) , anti_symmetric(rou_i(:,2)) , anti_symmetric(rou_i(:,3)) , anti_symmetric(rou_i(:,4)) ,anti_symmetric(rou_i(:,5)) , anti_symmetric(rou_i(:,6)) ]; 
input_equi = -input_equi_allocation*thrust_matrix; 

%test if the input is directly the force and torque: 
% input_equi = u(1:6);



v_dot = mass_matrix\(input_equi - C_matrix*[vL; omegaL] - G_matrix); 

%the output of thins function:
sys = [vL; v_dot(1:3); RL_dot_vector; v_dot(4:6)]; 
%=======================================================================
%��flagΪ3ʱ,����ϵͳ���������?:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys=x;
