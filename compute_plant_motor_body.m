function M_T=compute_plant_motor_body(u)

%机体坐标系的定义应与论文保持一致,
%计算输出为机体系下的力矩以及总推力

dr=0.45;     %两旋翼之间的距离
kt=1.34e-7;    %升力系数
kd=2.27e-9;   %转矩系数

A=[ -1.414*dr/4*kt, -1.414*dr/4*kt, 1.414*dr/4*kt, 1.414*dr/4*kt;...
    1.414*dr/4*kt, -1.414*dr/4*kt, -1.414*dr/4*kt, 1.414*dr/4*kt;...   
    -kd,           kd,             -kd,            kd;...
    kt,            kt,             kt,             kt];
B=inv(A);

M_T=A*(u.^2);%输出为机体坐标系下的力矩以及T（因旋翼推力之和)