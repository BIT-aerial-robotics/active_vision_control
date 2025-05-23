function [sys,x0,str,ts]=omega_nom(t,x,u,flag)
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
phi_nom=u(1);
theta_nom=u(2);
psi_nom=u(3);
d_phi_nom=u(4);
d_theta_nom=u(5);
d_psi_nom=u(6);

%compute output variable also the controller variable
p_nom=d_phi_nom-d_psi_nom*sin(theta_nom);
q_nom=d_theta_nom*cos(phi_nom)+d_psi_nom*sin(phi_nom)*cos(theta_nom);
r_nom=-d_theta_nom*sin(phi_nom)+d_psi_nom*cos(phi_nom)*cos(theta_nom);


%output 
sys(1)=p_nom;
sys(2)=q_nom;
sys(3)=r_nom;








