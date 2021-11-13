function [sys,x0,str,ts]=PI_controller_vz(t,x,u,flag)
switch flag
    case 0   %���ó�ʼ������
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);      
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
sizes.NumDiscStates=0;
sizes.NumOutputs=1;
sizes.NumInputs=7;
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
xi=1.414*2;
omega_n=2;

m=0.975 * 1;

R_33=u(1);
P_z_err=u(4);
v_z_err=u(7);

%alfa (from the damping and bandwidth)
a_1=omega_n^2;
a_2=2*xi*omega_n;

%the gain matrix
%K_I1=[0,0,0;0,0,0;0,0,0];
%K_p1=[1,0,0;0,1,0;0,0,1];

if R_33~=0
    K_I1=-a_1*m/R_33;
    K_p1=-a_2*m/R_33;
else 
    K_I1=-a_1*m/0.01;
    K_p1=-a_2*m/0.01;
end

sys=K_p1*v_z_err+K_I1*P_z_err;


