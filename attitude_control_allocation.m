function W=attitude_control_allocation(Mx,My,Mz,T)

%��������ϵ�Ķ���Ӧ�����ı���һ��,
%�������Ϊ�ĸ�������������ת��

dr=0.45;     %������֮��ľ���
kt= 1 * 1.34e-7;    %����ϵ��
kd=1* 2.27e-9;   %ת��ϵ��

A=[ -1.414*dr/4*kt, -1.414*dr/4*kt, 1.414*dr/4*kt, 1.414*dr/4*kt;...
    1.414*dr/4*kt, -1.414*dr/4*kt, -1.414*dr/4*kt, 1.414*dr/4*kt;...   
    -kd,           kd,             -kd,            kd;...
    kt,            kt,             kt,             kt];
B=inv(A);

W=(B*[Mx;My;Mz;T]).^0.5;%���Ϊ�ĸ������ת��