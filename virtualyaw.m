function [sys,x0,str,ts]=virtualyaw(t,x,u,flag)
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
sizes.NumContStates=7;
sizes.NumDiscStates=0;
sizes.NumOutputs=7;
sizes.NumInputs=4;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[6.5; 0; 0; 0; 0; 0; 0];  %x= P, v, expressed in earth frame
str=[];
ts=[0 0];

%=======================================================================
%��flagΪ1ʱ,�����������ϵͳ��״̬����
%=======================================================================
function sys=mdlDerivatives(t,x,u)
 %state variables
 p = x(1:3);
 v = x(4:6);
 yaw = x(7);
 
 
 F = u(1:3);
 yawrate = u(4);
sys= [v; F; yawrate];

%=======================================================================
%��flagΪ3ʱ,����ϵͳ���������:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys=x;




