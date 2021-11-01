function [sys,x0,str,ts]=torque_nom(t,x,u,flag)
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
sizes.NumOutputs=3;
sizes.NumInputs=6;
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
% I=diag([0.010914387; 0.011174571; 0.020874735]);  %inertia parameters
% Ixx=I(1,1);
% Iyy=I(2,2);
% Izz=I(3,3);
% Ixz=0.000;
global Ixx Iyy Izz Ixz;%inertia parameters
p_nom=u(1);
q_nom=u(2);
r_nom=u(3);
d_p_nom=u(4);
d_q_nom=u(5);
d_r_nom=u(6);

%compute output variable also the controller variable
Tl_nom=Ixx*d_p_nom+(Izz-Iyy)*q_nom*r_nom-Ixz*(d_r_nom+q_nom*p_nom);
Tm_nom=Iyy*d_q_nom+(Ixx-Izz)*p_nom*r_nom+Ixz*(p_nom^2-r_nom^2);
Tn_nom=Izz*d_r_nom+(Iyy-Ixx)*q_nom*p_nom+Ixz*(q_nom*r_nom-d_p_nom);


%output 
sys(1)=Tl_nom;
sys(2)=Tm_nom;
sys(3)=Tn_nom;








