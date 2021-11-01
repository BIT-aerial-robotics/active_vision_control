function [sys,x0,str,ts]=PI_controller_omega(t,x,u,flag)
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
sizes.NumInputs=21;
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
global Ixx Iyy Izz Ixz;
global  omega_roll_in  omega_pitch_in omega_yaw_in ksi_roll_in ksi_pitch_in ksi_yaw_in;
 

% ksi_in=0.707;
% omega_roll_in=6;
% omega_pitch_in=6;
% omega_yaw_in=5.6;
% global omega_trans_x_out omega_trans_y_out omega_trans_z_out;
% omega_roll_in=3*4*4*omega_trans_x_out;
% omega_pitch_in=3*4*4*omega_trans_y_out;
% omega_yaw_in=3*4*4*omega_trans_z_out;

%input signal
p_nom=u(1);
q_nom=u(2);
r_nom=u(3);
p_err=u(4);
q_err=u(5);
r_err=u(6);
p_err_int=u(7);
q_err_int=u(8);
r_err_int=u(9);
Ippq=u(10);
Ipqr=u(11);
Iqpp=u(12);
Iqpr=u(13);
Irpq=u(14);
gpl=u(15);
gpn=u(16);
gqm=u(17);
gm=u(18);
Irqr=u(19);
Iqrr=u(20);
grl=u(21);



%alfa (from the damping and bandwidth)
alfa_212=2*ksi_roll_in*omega_roll_in;
alfa_222=2*ksi_pitch_in*omega_pitch_in;
alfa_232=2*ksi_yaw_in*omega_yaw_in;
alfa_211=omega_roll_in^2;
alfa_221=omega_pitch_in^2;
alfa_231=omega_yaw_in^2;


%compute output variable also the controller variable

K_I2=[Ixx*alfa_211,0,-Ixz*alfa_231;...
0,Iyy*alfa_221,0;...
-Ixz*alfa_211,0,Izz*alfa_231];

K_p2=[Ixx*(Ippq*q_nom+alfa_212)-Ixz*Irpq*q_nom,Ixx*(Ippq*p_nom+Ipqr*r_nom)-Ixz*(Irpq*p_nom+Irqr*r_nom),Ixx*Ipqr*q_nom-Ixz*(Irqr*q_nom+alfa_232);...
    Iyy*(2*Iqrr*p_nom+Iqpr*r_nom),Iyy*alfa_222,Iyy*(2*Iqrr*r_nom+Iqpr*p_nom);...
    -Ixz*(Ippq*q_nom+alfa_212)+Izz*Irpq*q_nom,-Ixz*(Ippq*p_nom+Ipqr*r_nom)+Izz*(Irpq*p_nom+Irqr*r_nom),-Ixz*Ipqr*q_nom+Izz*(Irqr*q_nom+alfa_232)];

u2=K_p2*[p_err;q_err;r_err]+K_I2*[p_err_int;q_err_int;r_err_int];    

%output 
sys=u2;

