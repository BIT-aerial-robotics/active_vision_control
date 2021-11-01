function [sys,x0,str,ts]=plant_outer(t,x,u,flag)
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
sizes.NumContStates=3;
%=======================================================================
%x1=phi,x2=theta,x3=psi
%=======================================================================
sizes.NumDiscStates=0;
sizes.NumOutputs=3;
sizes.NumInputs=3;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[0;0;0];
str=[];
ts=[0 0];

%=======================================================================
%当flag为1时,更新整个连续系统的状态变量
%=======================================================================
function sys=mdlDerivatives(t,x,u)

%input signal
p=u(1);
q=u(2);
r=u(3);

sys(1)=p+q*sin(x(1))*tan(x(2))+r*cos(x(1))*tan(x(2));      %state variables derivatives
sys(2)=q*cos(x(1))-r*sin(x(2));
sys(3)=q*sin(x(1))*sec(x(2))+r*cos(x(1))*sec(x(2));

%=======================================================================
%当flag为3时,计算系统的输出变量:返回两个状态
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);%output1=phi
sys(2)=x(2);%output2=theta
sys(3)=x(3);%output3=psi




