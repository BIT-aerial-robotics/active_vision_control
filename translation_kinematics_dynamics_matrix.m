function [sys,x0,str,ts]=translation_kinematics_dynamics_matrix(t,x,u,flag)
switch flag
    case 0   %���ó�ʼ������
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);
    case 1   %��������״̬�ĸ��º���
        sys=mdlDerivatives(t,x,u);
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
sizes.NumContStates=6;
sizes.NumDiscStates=0;
sizes.NumOutputs=6;
sizes.NumInputs=10;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[[0.5; 0; 0]; [0; 0; 0]];  %x= P, v, expressed in earth frame
str=[];
ts=[0 0];

%=======================================================================
%��flagΪ1ʱ,�����������ϵͳ��״̬����
%=======================================================================
function sys=mdlDerivatives(t,x,u)
 %state variables
R=[u(2), u(3), u(4); 
   u(5), u(6), u(7);
   u(8), u(9), u(10)];

T=u(1); %total thrust

v = x(4:6);  %x= P, v, expressed in earth frame

global m g Cd;
% m=0.975; %mass
% g=9.8; %accelaration due to gravity
% 
% Cd= 0.02; %drag force coefficient

Fd = -Cd*v.*v; 

% v_dot = R/m*[0; 0; -T]+ g*[0; 0; 1] + Fd;

v_dot = R/m*[0; 0; -T]+ g*[0; 0; 1];

P_dot = v;
sys=[P_dot; v_dot];

%=======================================================================
%��flagΪ3ʱ,����ϵͳ���������:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys=x;




