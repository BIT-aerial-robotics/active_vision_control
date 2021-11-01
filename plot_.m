 close all;

figure(1);    %位置轨迹图
plot3(P_com(:,1),P_com(:,2),P_com(:,3),'-.',P_sens(:,1),P_sens(:,2),P_sens(:,3)),grid;
axis equal;
xlabel('X(m)');ylabel('Y(m)');zlabel('H(m)');
title('POSITION TRACKIN:SENSED VS COMMAND');
legend('track_c_o_m','track_s_e_n');
% 
figure(2);        %位置图
subplot(3,1,1);
plot(t,P_com(:,1),'-.',t,P_sens(:,1), t,P_sim(:,1)),grid;
ylabel('X(m)');
title('POSITION:SENSED VS COMMAND');
legend('P_c_o_m','P_s_e_n');

subplot(3,1,2);
plot(t,P_com(:,2),'-.',t,P_sens(:,2),t,P_sim(:,2)),grid;
ylabel('Y(m)');

subplot(3,1,3);
plot(t,P_com(:,3),'-.',t,P_sens(:,3),t,P_sim(:,3)),grid;
ylabel('H=-Z(m)');xlabel('time(s)');

figure(3);        %机体坐标系下的速度图
subplot(3,1,1);
plot(t,V_com(:,1),'-.',t,V_sens(:,1)),grid;
ylabel('u(m/s)');
title('BODY VELOCITY: SENSED VS COMMAND');
legend('V_c_o_m','V_s_e_n');

subplot(3,1,2);
plot(t,V_com(:,2),'-.',t,V_sens(:,2)),grid;
ylabel('v(m/s)');

subplot(3,1,3);
plot(t,V_com(:,3),'-.',t,V_sens(:,3)),grid;
ylabel('w(m/s)');xlabel('time(s)');

figure(4);        %欧拉角
subplot(3,1,1);
plot(t,gamma_com(:,1),'-.',t,gamma_sens(:,1)),grid;
ylabel('Φ(rad)');
title('Euler angles: Actual VS Commanded');
legend('γ_c_o_m','γ_s_e_n');

subplot(3,1,2);
plot(t,gamma_com(:,2),'-.',t,gamma_sens(:,2)),grid;
ylabel('θ(rad)');

subplot(3,1,3);
plot(t,gamma_com(:,3),'-.',t,gamma_sens(:,3)),grid;
ylabel('Ψ(rad)');xlabel('time(s)');

% figure(40);        %指数坐标
% subplot(3,1,1);
% plot(t,xi_com(:,1),'-.',t,xi_sens(:,1)),grid;
% ylabel('\xi_1');
% title('Euler angles: Actual VS Commanded');
% legend('\xi_c_o_m','\xi_s_e_n');
% 
% subplot(3,1,2);
% plot(t,xi_com(:,2),'-.',t,xi_sens(:,2)),grid;
% ylabel('\xi_2');
% 
% subplot(3,1,3);
% plot(t,xi_com(:,3),'-.',t,xi_sens(:,3)),grid;
% ylabel('\xi_3');xlabel('time(s)');

figure(5);        %机体坐标系下的角速度图
subplot(3,1,1);
plot(t,omega_com(:,1),'-.',t,omega_sens(:,1)),grid;
ylabel('p(rad/s)');
title('BODY RATE: SENSED VS COMMAND');
legend('ω_c_o_m','ω_s_e_n');

subplot(3,1,2);
plot(t,omega_com(:,2),'-.',t,omega_sens(:,2)),grid;
ylabel('q(rad/s)');

subplot(3,1,3);
plot(t,omega_com(:,3),'-.',t,omega_sens(:,3)),grid;
ylabel('r(rad/s)');xlabel('time(s)');

figure(6);        %四个电机转速
subplot(4,1,1);
plot(t,o_com(:,1)),grid;
ylabel('o1(rad/s)');
% title('motor rotate velocity contorl');

subplot(4,1,2);
plot(t,o_com(:,2)),grid;
ylabel('o2(rad/s)');

subplot(4,1,3);
plot(t,o_com(:,3)),grid;
ylabel('o3(rad/s)');

subplot(4,1,4);
plot(t,o_com(:,4)),grid;
ylabel('o4(rad/s)');xlabel('time(s)');

figure(7);        %机体坐标系下的力主矢以及力矩主矢
subplot(2,1,1);
plot(t,F_com(:,1),'--',t,F_com(:,2),'-.',t,F_com(:,3)),grid;
ylabel('FORCE(N)');
title('BODY FORCE AND TORQUE');
legend('F_c_o_m_x','F_c_o_m_y','F_c_o_m_z');

subplot(2,1,2);
plot(t,M_com(:,1),'--',t,M_com(:,2),'-.',t,M_com(:,3)),grid;
ylabel('TORQUE(N.m)');
legend('M_c_o_m_x','M_c_o_m_y','M_c_o_m_z');


figure(8);   %机体系下因旋翼运动而产生的z轴力

plot(t,T_com),grid;
ylabel('fz(N)');
xlabel('t/s');
% legend('M_c_o_m_x','M_c_o_m_y','M_c_o_m_z');


figure(300);        %load attitude 
subplot(3,1,1);
plot(t,qL_com(:,1),'-.',t,qL(:,1)),grid;
ylabel('q1');
title('Load attitude: SENSED VS COMMAND');
legend('q_c_o_m','q_s_e_n');

subplot(3,1,2);
plot(t,qL_com(:,2),'-.',t,qL(:,2)),grid;
ylabel('q2');

subplot(3,1,3);
plot(t,qL_com(:,3),'-.',t,qL(:,3)),grid;
ylabel('q3');xlabel('time(s)');

figure(301);        %load angular velocity  
subplot(3,1,1);
plot(t,omegal_com(:,1),'-.',t,omegal_sen(:,1)),grid;
ylabel('p_L');
title('Load attitude: SENSED VS COMMAND');
legend('q_c_o_m','q_s_e_n');

subplot(3,1,2);
plot(t,omegal_com(:,2),'-.',t,omegal_sen(:,2)),grid;
ylabel('q_L');

subplot(3,1,3);
plot(t,omegal_com(:,3),'-.',t,omegal_sen(:,3)),grid;
ylabel('r_L');xlabel('time(s)');


figure(302);        %load position tracking  
subplot(3,1,1);
plot(t,xl_com(:,1),'-.',t,pl_sen(:,1)),grid;
ylabel('x_L');
title('Load position: SENSED VS COMMAND');
legend('pL_{com}','pL_{sen}');

subplot(3,1,2);
plot(t,xl_com(:,2),'-.',t,pl_sen(:,2)),grid;
ylabel('x_L');

subplot(3,1,3);
plot(t,xl_com(:,3),'-.',t,pl_sen(:,3)),grid;
ylabel('y_L');xlabel('time(s)');



