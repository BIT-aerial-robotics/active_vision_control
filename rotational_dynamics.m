function [sys,x0,str,ts]=plant_inner(t,x,u,flag)
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
%x1=p,x2=q,x3=r
%=======================================================================
sizes.NumDiscStates=0;
sizes.NumOutputs=3;
sizes.NumInputs=15;
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
Tl=u(1);
Tm=u(2);
Tn=u(3);
Ippq=u(4);
Ipqr=u(5);
Iqpp=u(6);
Iqpr=u(7);
Irpq=u(8);
gpl=u(9);
gpn=u(10);
gqm=u(11);
grn=u(12);
Irqr=u(13);
Iqrr=u(14);
grl=u(15);

sys(1)=Ippq*x(1)*x(2)+Ipqr*x(2)*x(3)+gpl*Tl+gpn*Tn;      %state variables derivatives
sys(2)=Iqpp*x(1)^2+Iqrr*x(3)^2+Iqpr*x(1)*x(3)+gqm*Tm;
sys(3)=Irpq*x(1)*x(2)+Irqr*x(2)*x(3)+grl*Tl+grn*Tn;

%=======================================================================
%��flagΪ3ʱ,����ϵͳ���������:��������״̬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);%output1=p
sys(2)=x(2);%output2=q
sys(3)=x(3);%output3=r




