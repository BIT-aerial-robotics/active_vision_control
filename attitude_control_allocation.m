function W=attitude_control_allocation(Mx,My,Mz,T)

%机体坐标系的定义应与论文保持一致,
%计算输出为四个电机带动旋翼的转速

dr=0.45;     %两旋翼之间的距离
kt= 1 * 1.34e-7;    %升力系数
kd=1* 2.27e-9;   %转矩系数

A=[ -1.414*dr/4*kt, -1.414*dr/4*kt, 1.414*dr/4*kt, 1.414*dr/4*kt;...
    1.414*dr/4*kt, -1.414*dr/4*kt, -1.414*dr/4*kt, 1.414*dr/4*kt;...   
    -kd,           kd,             -kd,            kd;...
    kt,            kt,             kt,             kt];
B=inv(A);

W=(B*[Mx;My;Mz;T]).^0.5;%输出为四个电机的转速