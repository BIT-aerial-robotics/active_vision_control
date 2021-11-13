function [sys,x0,str,ts]=rotational_kinematics_SO3(t,x,u,flag)
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
x0=[0;0;0];  %the exponential coordinate of rotation matrix
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

a=x(1); b=x(2);  c=x(3); 

if a==0 && b==0 && c==0 
    inv_dexp_neg_ksi=eye(3);
else   

inv_dexp_neg_ksi=[[1+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*(-c^2-b^2),-1/2*c+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*b*a,1/2*b+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*c*a]
[1/2*c+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*b*a,1+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*(-c^2-a^2),-1/2*a+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*c*b]
[-1/2*b+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*c*a,1/2*a+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*c*b,1+(1-1/2*(a^2+b^2+c^2)^(1/2)/tan(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*(-b^2-a^2)]];

end

sys= inv_dexp_neg_ksi*[p; q; r];

%=======================================================================
%当flag为3时,计算系统的输出变量:返回两个状态
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);%output1=phi
sys(2)=x(2);%output2=theta
sys(3)=x(3);%output3=psi




