
function com_position=compute_com_position(t,x_end,y_end,z_end)

%计算得到航线,及位置随时间的指令
%以下所有单位为m

% t;     %输入量为时间
% x_end=150;
% y_end=10;
% z_end=20;%终点

x0=0;
y0=0;
z0=0;%起始点

% 垂直起飞航线
x_vt=0;
y_vt=0;
z_vt=z_end;

v_vt=3;%垂直起飞平均速度
v_h=5;%巡航平均速度
v_vl=3;%垂直降落平均速度

t_t=z_vt/v_vt;%各阶段消耗时间,非累加时间
t_h=(x_end^2+y_end^2)^0.5/v_h;
t_l=z_vt/v_vl;


if t>=0 & t<(z_vt/v_vt)
%垂直起飞阶段时间规划
% v_vt=3;%垂直起飞平均速度
x_com=0;
y_com=0;
z_com=z_end/t_t^2*t^2;
com_position=[x_com;y_com;z_com];


elseif t>=(z_vt/v_vt) & t<(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt))
        %平飞阶段规划
        angle=atan(y_end/x_end);
        % v_h=3;%巡航平均速度
        v_h_x=x_end/abs(x_end)*v_h*cos(angle);
        v_h_y=y_end/abs(y_end)*v_h*sin(angle);
        x_com=v_h_x*(t-t_t);
        y_com=v_h_y*(t-t_t);
        z_com=z_end;
        com_position=[x_com;y_com;z_com];
        
elseif t>=(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)) & t<(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)+(z_vt/v_vl))       
%     if t>=(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)) & t<(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)+(z_vt/v_vl))
        %垂直降落阶段时间规划
        % v_vl=3;%垂直降落平均速度
        x_com=x_end;
        y_com=y_end;
        z_com=-(v_vl^2/z_end)*(t-t_t-t_h)^2+z_end;
        com_position=[x_com;y_com;z_com];

else
        x_com=x_end;
        y_com=y_end;
        z_com=0;
        com_position=[x_com;y_com;z_com];
end







% x1=10;
% y1=10;%控制点1
% 
% x2=20;
% y2=20;%控制点2
