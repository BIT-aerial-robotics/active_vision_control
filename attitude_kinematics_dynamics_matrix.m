function [sys,x0,str,ts]=attitude_kinematics_dynamics_matrix(t,x,u,flag)
switch flag
    case 0   %调用初始化函数
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);
    case 1   %调用连续状态的更新函数
        sys=mdlDerivatives(t,x,u);
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
%当flag为1时,更新整个连续系统的状态变量
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

tau_a= u(1:3); %%旋翼拉力与扭矩产生的力矩 
W1 = u(4); W2 = u(5); W3 = u(6); W4 = u(7);

% J_r = 22895.55/1e9; %旋翼的转动惯量

tau_G=J_r*cross(omega, [0;0;1])* (-W1 + W2 - W3 + W4); %gyroscopic torque

R_dot=R*hat_omega;   
omega_dot=inv(I)*( tau_a - tau_G -hat_omega*I*omega);

sys=[R_dot(1,1); R_dot(1,2); R_dot(1,3); R_dot(2,1);  R_dot(2,2); R_dot(2,3); R_dot(3,1); R_dot(3,2); R_dot(3,3); omega_dot];

%=======================================================================
%当flag为3时,计算系统的输出变量:返回两个状态
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys=x;




