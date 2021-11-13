function [sys,x0,str,ts]=omega_nom_SO3(t,x,u,flag)
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


