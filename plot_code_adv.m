 close all;
 
 t =t1;
 
 P_com = y_ref_array(:,1:3);
 P_sens = y1_nom(:,1:3);
 P_sim = P_sens;
 
 V_com = y_ref_array(:,4:6);
 V_sens = y1_nom(:,4:6);
 
 psi_ref = y_ref_array(:, 7);
 psi_nom = y1_nom(:,7);
 
 
 gamma_nom = zeros(length(t), 3);
 gamma_ref = zeros(length(t), 3);
 cosbeta_arr = zeros(length(t), 1);
 
     for i =1:length(t)
         R_sen_i = y1_nom(i,7:15);
         R_ref_i = y_ref_array(i,7:15);
        R_i = [R_sen_i(1), R_sen_i(2),  R_sen_i(3); R_sen_i(4), R_sen_i(5), R_sen_i(6);  R_sen_i(7), R_sen_i(8), R_sen_i(9)];
        R_0 = [R_ref_i(1), R_ref_i(2),  R_ref_i(3); R_ref_i(4), R_ref_i(5), R_ref_i(6);  R_ref_i(7), R_ref_i(8), R_ref_i(9)];
        R_0_i = R_0'*R_i;
%         norm_R_array(i) = norm(R_i-R_d_i);
        gamma_nom(i,:) = (Euler_angles(R_i))';
        gamma_ref(i,:) = (Euler_angles(R_0))';     
        
        r0_11 = y1_nom(i, 7);
        r0_12 = y1_nom(i, 8);
    r0_13 = y1_nom(i, 9);
    r0_21 = y1_nom(i, 10);
    r0_22 = y1_nom(i, 11);
    r0_23 = y1_nom(i, 12);
    r0_31 = y1_nom(i, 13);
    r0_32 = y1_nom(i, 14);
    r0_33 = y1_nom(i, 15);
    
    %the position of the vehicle:
    x0_x = y1_nom(i, 1); 
    x0_y = y1_nom(i, 2);  
    x0_z = y1_nom(i, 3);
 
    p_fx = 10; p_fy =1;  p_fz = 0;
    cosbeta_arr(i) = (p_fx*r0_11 + p_fy*r0_21 + p_fz*r0_31 - r0_11*x0_x - r0_21*x0_y - r0_31*x0_z)/((p_fx*r0_11 + p_fy*r0_21 + p_fz*r0_31 - r0_11*x0_x - r0_21*x0_y - r0_31*x0_z)^2 + (p_fx*r0_12 + p_fy*r0_22 + p_fz*r0_32 - r0_12*x0_x - r0_22*x0_y - r0_32*x0_z)^2 + (p_fx*r0_13 + p_fy*r0_23 + p_fz*r0_33 - r0_13*x0_x - r0_23*x0_y - r0_33*x0_z)^2)^(1/2);
     
     end
     
     
 omega_nom = u_nom_arr(1:3, :)';
 T_net = u_nom_arr(4,:)';
   

figure(1);    %3D position
plot3(P_com(:,1),P_com(:,2),P_com(:,3),'-.',P_sens(:,1),P_sens(:,2),P_sens(:,3)),grid;
axis equal;
xlabel('X(m)');ylabel('Y(m)');zlabel('H(m)');
title('POSITION TRACKIN:SENSED VS COMMAND');
legend('track_c_o_m','track_s_e_n');
 
figure(2);        %position
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

figure(3);        %velocity
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

% figure(4);        %yaw angle 
% 
% plot(t,psi_ref,'-.',t, psi_nom),grid;
% ylabel('\psi(rad)');
% title('yaw angle');
% legend('\psi_{com}','\psi');

figure(4);        %Euler angles
subplot(3,1,1);
plot(t,gamma_ref(:,1),'-.', t,gamma_nom(:,1)),grid;
ylabel('\phi(rad)');
title('Euler angles: Actual VS Commanded');
legend('ref','nom');

subplot(3,1,2);
plot(t,gamma_ref(:,2),'-.',t,gamma_nom(:,2)),grid;
ylabel('\theta(rad)');

subplot(3,1,3);
plot(t,gamma_ref(:,3),'-.',t,gamma_nom(:,3)),grid;
ylabel('\psi(rad)');xlabel('time(s)');

figure(5);        %angular velocity
subplot(3,1,1);
plot(t,omega_nom(:,1) ),grid;
ylabel('p(rad/s)');
title('BODY RATE');
% legend('��_c_o_m','��_s_e_n');

subplot(3,1,2);
plot(t,omega_nom(:,2) ),grid;
ylabel('q(rad/s)');

subplot(3,1,3);
plot(t,omega_nom(:,3) ),grid;
ylabel('r(rad/s)');xlabel('time(s)');

figure(7);        %net thrust
% subplot(2,1,1);
plot(t,T_net),grid;
ylabel('FORCE(N)');
title('force');
xlabel('time(s)');
% legend('F_c_o_m_x','F_c_o_m_y','F_c_o_m_z');

figure(8);        %net thrust
% subplot(2,1,1);
plot(t,cosbeta_arr),grid;
ylabel('\cos \beta ');
xlabel('time(s)');
% legend('F_c_o_m_x','F_c_o_m_y','F_c_o_m_z');


% figure(4);        %ŷ����
% subplot(3,1,1);
% plot(t,gamma_com(:,1),'-.',t(:,1)),grid;
% ylabel('��(rad)');
% title('Euler angles: Actual VS Commanded');
% legend('��_c_o_m','��_s_e_n');
% 
% subplot(3,1,2);
% plot(t,gamma_com(:,2),'-.',t,gamma_sens(:,2)),grid;
% ylabel('��(rad)');
% 
% subplot(3,1,3);
% plot(t,gamma_com(:,3),'-.',t,gamma_sens(:,3)),grid;
% ylabel('��(rad)');xlabel('time(s)');
% 
% figure(40);        %ָ�����
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

% figure(5);        %�������ϵ�µĽ��ٶ�ͼ
% subplot(3,1,1);
% plot(t,omega_com(:,1),'-.',t,omega_sens(:,1)),grid;
% ylabel('p(rad/s)');
% title('BODY RATE: SENSED VS COMMAND');
% legend('��_c_o_m','��_s_e_n');
% 
% subplot(3,1,2);
% plot(t,omega_com(:,2),'-.',t,omega_sens(:,2)),grid;
% ylabel('q(rad/s)');
% 
% subplot(3,1,3);
% plot(t,omega_com(:,3),'-.',t,omega_sens(:,3)),grid;
% ylabel('r(rad/s)');xlabel('time(s)');
% 
% figure(6);        %�ĸ����ת��
% subplot(4,1,1);
% plot(t,o_com(:,1)),grid;
% ylabel('o1(rad/s)');
% % title('motor rotate velocity contorl');
% 
% subplot(4,1,2);
% plot(t,o_com(:,2)),grid;
% ylabel('o2(rad/s)');
% 
% subplot(4,1,3);
% plot(t,o_com(:,3)),grid;
% ylabel('o3(rad/s)');
% 
% subplot(4,1,4);
% plot(t,o_com(:,4)),grid;
% ylabel('o4(rad/s)');xlabel('time(s)');
% 
% figure(7);        %�������ϵ�µ�����ʸ�Լ�������ʸ
% subplot(2,1,1);
% plot(t,F_com(:,1),'--',t,F_com(:,2),'-.',t,F_com(:,3)),grid;
% ylabel('FORCE(N)');
% title('BODY FORCE AND TORQUE');
% legend('F_c_o_m_x','F_c_o_m_y','F_c_o_m_z');
% 
% subplot(2,1,2);
% plot(t,M_com(:,1),'--',t,M_com(:,2),'-.',t,M_com(:,3)),grid;
% ylabel('TORQUE(N.m)');
% legend('M_c_o_m_x','M_c_o_m_y','M_c_o_m_z');
% 
% 
% figure(8);   %����ϵ���������˶�������z����
% 
% plot(t,T_com),grid;
% ylabel('fz(N)');
% xlabel('t/s');
% % legend('M_c_o_m_x','M_c_o_m_y','M_c_o_m_z');
% 
% 
% figure(300);        %load attitude 
% subplot(3,1,1);
% plot(t,qL_com(:,1),'-.',t,qL(:,1)),grid;
% ylabel('q1');
% title('Load attitude: SENSED VS COMMAND');
% legend('q_c_o_m','q_s_e_n');
% 
% subplot(3,1,2);
% plot(t,qL_com(:,2),'-.',t,qL(:,2)),grid;
% ylabel('q2');
% 
% subplot(3,1,3);
% plot(t,qL_com(:,3),'-.',t,qL(:,3)),grid;
% ylabel('q3');xlabel('time(s)');
% 
% figure(301);        %load angular velocity  
% subplot(3,1,1);
% plot(t,omegal_com(:,1),'-.',t,omegal_sen(:,1)),grid;
% ylabel('p_L');
% title('Load attitude: SENSED VS COMMAND');
% legend('q_c_o_m','q_s_e_n');
% 
% subplot(3,1,2);
% plot(t,omegal_com(:,2),'-.',t,omegal_sen(:,2)),grid;
% ylabel('q_L');
% 
% subplot(3,1,3);
% plot(t,omegal_com(:,3),'-.',t,omegal_sen(:,3)),grid;
% ylabel('r_L');xlabel('time(s)');
% 
% 
% figure(302);        %load position tracking  
% subplot(3,1,1);
% plot(t,xl_com(:,1),'-.',t,pl_sen(:,1)),grid;
% ylabel('x_L');
% title('Load position: SENSED VS COMMAND');
% legend('pL_{com}','pL_{sen}');
% 
% subplot(3,1,2);
% plot(t,xl_com(:,2),'-.',t,pl_sen(:,2)),grid;
% ylabel('x_L');
% 
% subplot(3,1,3);
% plot(t,xl_com(:,3),'-.',t,pl_sen(:,3)),grid;
% ylabel('y_L');xlabel('time(s)');



