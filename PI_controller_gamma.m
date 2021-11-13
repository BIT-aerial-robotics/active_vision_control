function [sys,x0,str,ts]=PI_controller_gamma(t,x,u,flag)
switch flag
    case 0   %���ó�ʼ������
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);      
    case 3   %����������ļ��㺯��
        sys=mdlOutputs(t,x,u);
    case {2,4,9}  %δʹ�ù��flagֵ
        sys=[];
    otherwise %�������
        error(['Unhandled flag=',num2str(flag)]);
end
%=======================================================================
%��flagΪ0ʱ�������ϵͳ�ĳ�ʼ��
%=======================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u)
%���ȵ���simsizes����ó�ϵͳ��ģ����sizes,�������ɢϵͳ��ʵ���������
%sizes����
sizes=simsizes;
sizes.NumDiscStates=0;
sizes.NumOutputs=3;
sizes.NumInputs=12;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[0 0];
%=======================================================================
%��flagΪ3ʱ,����ϵͳ���������:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
%Servo system Parameters

global omega_roll_out omega_pitch_out omega_yaw_out ksi_roll_out ksi_pitch_out ksi_yaw_out;

% omega_roll_out=1.5;
% omega_pitch_out=1.125;
% omega_yaw_out=1.4;

% global omega_trans_x_out omega_trans_y_out omega_trans_z_out;
% omega_roll_out=4*4*omega_trans_x_out;
% omega_pitch_out=4*4*omega_trans_y_out;
% omega_yaw_out=4*4*omega_trans_z_out;

phi_nom=u(1);
theta_nom=u(2);
psi_nom=u(3);
p_nom=u(4);
q_nom=u(5);
r_nom=u(6);
phi_err=u(7);
theta_err=u(8);
psi_err=u(9);
phi_err_int=u(10);
theta_err_int=u(11);
psi_err_int=u(12);

%only track yaw angle:
% psi_err = 0;
% psi_err_int = 0;


%alfa (from the damping and bandwidth)
alfa_112=2*ksi_roll_out*omega_roll_out;
alfa_122=2*ksi_pitch_out*omega_pitch_out;
alfa_132=2*ksi_yaw_out*omega_yaw_out;
alfa_111=omega_roll_out^2;
alfa_121=omega_pitch_out^2;
alfa_131=omega_yaw_out^2;



%compute output variable also the controller variable
%K_I1=[0,0,0;0,0,0;0,0,0];


K_I1=[alfa_111,0,-alfa_131*sin(theta_nom);...
0,alfa_121*cos(phi_nom),alfa_131*sin(phi_nom)*cos(theta_nom);...
0,-alfa_121*sin(phi_nom),alfa_131*cos(psi_nom)*cos(theta_nom)];

%K_p1=[1,0,0;0,1,0;0,0,1];


K_p1=[alfa_112,q_nom*sin(phi_nom)+r_nom*cos(phi_nom),-alfa_132*sin(theta_nom);...
    -r_nom,alfa_122*cos(phi_nom)+(q_nom*sin(phi_nom)+r_nom*cos(phi_nom))*sin(phi_nom)*sin(theta_nom)/cos(theta_nom),alfa_132*sin(phi_nom)*cos(theta_nom);...
    q_nom,-alfa_122*sin(phi_nom)+(q_nom*sin(phi_nom)+r_nom*cos(phi_nom))*cos(phi_nom)*sin(theta_nom)/cos(theta_nom),alfa_132*cos(phi_nom)*cos(theta_nom)];

u1=K_p1*[phi_err;theta_err;psi_err]+K_I1*[phi_err_int;theta_err_int;psi_err_int];

%output 
sys(1)=u1(1);
sys(2)=u1(2);
sys(3)=u1(3);

