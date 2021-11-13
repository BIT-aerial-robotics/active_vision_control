function [sys,x0,str,ts]=omega_nom(t,x,u,flag)
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
sizes.NumOutputs=3;
sizes.NumInputs=6;
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








