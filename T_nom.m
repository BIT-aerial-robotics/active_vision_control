function [sys,x0,str,ts]=T_nom(t,x,u,flag)
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








