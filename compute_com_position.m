
function com_position=compute_com_position(t,x_end,y_end,z_end)

%����õ�����,��λ����ʱ���ָ��
%�������е�λΪm

% t;     %������Ϊʱ��
% x_end=150;
% y_end=10;
% z_end=20;%�յ�

x0=0;
y0=0;
z0=0;%��ʼ��

% ��ֱ��ɺ���
x_vt=0;
y_vt=0;
z_vt=z_end;

v_vt=3;%��ֱ���ƽ���ٶ�
v_h=5;%Ѳ��ƽ���ٶ�
v_vl=3;%��ֱ����ƽ���ٶ�

t_t=z_vt/v_vt;%���׶�����ʱ��,���ۼ�ʱ��
t_h=(x_end^2+y_end^2)^0.5/v_h;
t_l=z_vt/v_vl;


if t>=0 & t<(z_vt/v_vt)
%��ֱ��ɽ׶�ʱ��滮
% v_vt=3;%��ֱ���ƽ���ٶ�
x_com=0;
y_com=0;
z_com=z_end/t_t^2*t^2;
com_position=[x_com;y_com;z_com];


elseif t>=(z_vt/v_vt) & t<(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt))
        %ƽ�ɽ׶ι滮
        angle=atan(y_end/x_end);
        % v_h=3;%Ѳ��ƽ���ٶ�
        v_h_x=x_end/abs(x_end)*v_h*cos(angle);
        v_h_y=y_end/abs(y_end)*v_h*sin(angle);
        x_com=v_h_x*(t-t_t);
        y_com=v_h_y*(t-t_t);
        z_com=z_end;
        com_position=[x_com;y_com;z_com];
        
elseif t>=(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)) & t<(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)+(z_vt/v_vl))       
%     if t>=(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)) & t<(((x_end^2+y_end^2)^0.5/v_h)+(z_vt/v_vt)+(z_vt/v_vl))
        %��ֱ����׶�ʱ��滮
        % v_vl=3;%��ֱ����ƽ���ٶ�
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
% y1=10;%���Ƶ�1
% 
% x2=20;
% y2=20;%���Ƶ�2
