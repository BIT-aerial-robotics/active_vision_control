function [sys,x0,str,ts]=omega_nom_SO3(t,x,u,flag)
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
a=u(1);
b=u(2);
c=u(3);
d_a_nom=u(4);
d_b_nom=u(5);
d_c_nom=u(6);

if a==0 && b==0 && c==0 
    dexp_neg_ksi=eye(3);
else  

%compute output variable also the controller variable
dexp_neg_ksi=[[1+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*(-c^2-b^2),2*sin(1/2*(a^2+b^2+c^2)^(1/2))^2/(a^2+b^2+c^2)*c+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*b*a,-2*sin(1/2*(a^2+b^2+c^2)^(1/2))^2/(a^2+b^2+c^2)*b+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*c*a]
[-2*sin(1/2*(a^2+b^2+c^2)^(1/2))^2/(a^2+b^2+c^2)*c+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*b*a,1+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*(-c^2-a^2),2*sin(1/2*(a^2+b^2+c^2)^(1/2))^2/(a^2+b^2+c^2)*a+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*b*c]
[2*sin(1/2*(a^2+b^2+c^2)^(1/2))^2/(a^2+b^2+c^2)*b+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*c*a,-2*sin(1/2*(a^2+b^2+c^2)^(1/2))^2/(a^2+b^2+c^2)*a+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*b*c,1+(1-2*sin(1/2*(a^2+b^2+c^2)^(1/2))/(a^2+b^2+c^2)^(1/2)*cos(1/2*(a^2+b^2+c^2)^(1/2)))/(a^2+b^2+c^2)*(-b^2-a^2)]];
end

%output 
sys=dexp_neg_ksi*[d_a_nom;  d_b_nom;  d_c_nom];


