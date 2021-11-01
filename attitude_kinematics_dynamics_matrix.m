function [sys,x0,str,ts]=attitude_kinematics_dynamics_matrix(t,x,u,flag)
switch flag
    case 0   %���ó�ʼ������
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);
    case 1   %��������״̬�ĸ��º���
        sys=mdlDerivatives(t,x,u);
    case 3   %����������ļ��㺯��
        sys=mdlOutputs(t,x,u);
    case {2,4,9}  %δʹ�ù���flagֵ
        sys=[];
    otherwise %�������
        error(['Unhandled flag=',num2str(flag)]);
end
%=======================================================================
%��flagΪ0ʱ��������ϵͳ�ĳ�ʼ��
%=======================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u)
%���ȵ���simsizes�����ó�ϵͳ��ģ����sizes,��������ɢϵͳ��ʵ���������
%sizes����
sizes=simsizes;
sizes.NumContStates=12;
%=======================================================================
%x1=phi,x2=theta,x3=psi
%=======================================================================
sizes.NumDiscStates=0;
sizes.NumOutputs=12;
sizes.NumInputs=7;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[[1; 0; 0; 0; 1; 0; 0; 0; 1]; [0; 0; 0]];  %x= R, omega
str=[];
ts=[0 0];

%=======================================================================
%��flagΪ1ʱ,������������ϵͳ��״̬����
%=======================================================================
function sys=mdlDerivatives(t,x,u)
 %state variables
R=[x(1), x(2), x(3); 
   x(4), x(5), x(6);
   x(7), x(8), x(9)];

omega=[x(10); x(11); x(12)]; 

hat_omega=[0    -x(12)  x(11);
           x(12)  0    -x(10);
           -x(11) x(10)    0];

global I J_r;

% I=diag([0.010914387; 0.011174571; 0.020874735]);  %inertia parameters

tau_a= u(1:3); %%����������Ť�ز��������� 
W1 = u(4); W2 = u(5); W3 = u(6); W4 = u(7);

% J_r = 22895.55/1e9; %�����ת������

tau_G=J_r*cross(omega, [0;0;1])* (-W1 + W2 - W3 + W4); %gyroscopic torque

R_dot=R*hat_omega;   
omega_dot=inv(I)*( tau_a - tau_G -hat_omega*I*omega);

sys=[R_dot(1,1); R_dot(1,2); R_dot(1,3); R_dot(2,1);  R_dot(2,2); R_dot(2,3); R_dot(3,1); R_dot(3,2); R_dot(3,3); omega_dot];

%=======================================================================
%��flagΪ3ʱ,����ϵͳ���������:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys=x;




