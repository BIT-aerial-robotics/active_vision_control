function [sys,x0,str,ts]=PI_controller_vz(t,x,u,flag)
switch flag
    case 0   %调用初始化函数
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);      
    case 3   %调用输出量的计算函数
        sys=mdlOutputs(t,x,u);
    case {2,4,9}  %未使用过的flag值
        sys=[];
    otherwise %处理错误
        error(['Unhandled flag=',num2str(flag)]);
end
%=======================================================================
%当flag为0时进行整个系统的初始化
%=======================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u)
%首先调用simsizes函数得出系统规模参数sizes,并根据离散系统的实际情况设置
%sizes变量
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
%当flag为3时,计算系统的输出变量:返回两个状态
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


