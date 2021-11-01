function [sys,x0,str,ts]=T_nom(t,x,u,flag)
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
m =0.975 * 1; 
g =9.8;
v_z_nom=u(6);

R_33=u(7);

%compute output variable also the controller variable
if R_33~=0
    T_nom = (m*g-m*v_z_nom) / R_33; %the nominal total thrust
else 
    T_nom = (m*g-m*v_z_nom) / 0.01;
end

%output 
sys=T_nom;








