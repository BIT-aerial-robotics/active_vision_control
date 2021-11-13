function [sys,x0,str,ts]=plant_outer(t,x,u,flag)
switch flag
    case 0   %���ó�ʼ������
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);
    case 1   %��������״̬�ĸ��º���
        sys=mdlDerivatives(t,x,u);
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
%��flagΪ1ʱ,������������ϵͳ��״̬����
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
%��flagΪ3ʱ,����ϵͳ���������:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);%output1=phi
sys(2)=x(2);%output2=theta
sys(3)=x(3);%output3=psi




