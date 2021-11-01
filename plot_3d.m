close all;


%calculate the thrust_vector array from the body wrench:
global rou1_x rou1_y rou1_z; 
global rou2_x rou2_y rou2_z; 
global rou3_x rou3_y rou3_z; 
global rou4_x rou4_y rou4_z; 
global rou5_x rou5_y rou5_z; 
global rou6_x rou6_y rou6_z; 
rou_i = [ rou1_x rou1_y rou1_z; 
 rou2_x rou2_y rou2_z; 
 rou3_x rou3_y rou3_z; 
 rou4_x rou4_y rou4_z; 
 rou5_x rou5_y rou5_z; 
 rou6_x rou6_y rou6_z];
rou_i = rou_i';   
input_equi_allocation = [eye(3,3), eye(3,3), eye(3,3), eye(3,3), eye(3,3), eye(3,3);...
    anti_symmetric(rou_i(:,1)) , anti_symmetric(rou_i(:,2)) , anti_symmetric(rou_i(:,3)) , anti_symmetric(rou_i(:,4)) ,anti_symmetric(rou_i(:,5)) , anti_symmetric(rou_i(:,6)) ]; 

t = t1;

length_t = length(t); 

P_sens = y1(:, 1:3);   P_nom = y1_nom(:, 1:3); 
P_com = y_ref_array(:, 1:3); 

V_sens = y1(:, 4:6);   V_nom = y1_nom(:, 4:6);
V_com = y_ref_array(:, 4:6);


R_sen = y1(:, 7:15);   R_nom = y1_nom(:, 7:15);
R_com = y_ref_array(:, 7:15); 

gamma_sens = zeros(length_t, 3);  
gamma_com = zeros(length_t, 3); 
gamma_nom = zeros(length_t, 3); 

omega_sens = y1(:, 16:18);    omega_nom = y1_nom(:, 16:18);  
omega_com = y_ref_array(:, 16:18); 

t_mag = zeros(length_t, 6); 
c_i_ang = zeros(length_t, 6); 

wrench_array = zeros(6, length_t);
wrench_array_nog_eath = zeros(6, length_t);

for aa = 1:6
t_mag(:,aa) =    sqrt( thrust_vector(:, 3*(aa-1) +1).^2 + thrust_vector(:, 3*(aa-1) +2).^2 + thrust_vector(:, 3*(aa-1) +3).^2);  
c_i_ang(:, aa) = thrust_vector(:, 3*(aa-1) +3)./t_mag(:,aa) ; 

end

for bb =1:length_t
    wrench_array(:, bb) = input_equi_allocation*thrust_vector(bb,:)'; %in body frame, does not include the gravity
    R = [R_sen(bb,1), R_sen(bb,2),  R_sen(bb,3); R_sen(bb,4), R_sen(bb,5), R_sen(bb,6);  R_sen(bb,7), R_sen(bb,8), R_sen(bb,9)];
    
    m_sum = 15.8; 
    acc_g = 9.8; 
%     f_0_earth = out_pos.CONTROLS(1, 2:4)';
%     f_body_nogra = R0' * (f_0_earth - [0; 0; m_sum*acc_g]); 
    
    f_0_earth = R*(wrench_array(1:3,bb) + [0; 0; m_sum*acc_g]); 
    wrench_array_nog_eath(:, bb) = wrench_array(:, bb);
    wrench_array_nog_eath(1:3, bb) = f_0_earth;
end

figure(1342);
plot(t, R_sen(:,9), t, R_nom(:,9), '-.');     legend('sens','nom');
ylabel('the third element of R');

%the point represents the three body axis:
% global axisscale;
% xbody=[[1;0;0]*axisscale; 1];
% ybody=[[0;1;0]*axisscale; 1];
% zbody=[[0;0;1]*axisscale; 1];
% xbodyE =E_b_T*xbody;
% ybodyE =E_b_T*ybody;
% zbodyE =E_b_T*zbody;

%the end point of axis z of the body frame:
end_z = zeros(3, length_t);
for aa =1:length_t
    R = [R_sen(aa,1), R_sen(aa,2),  R_sen(aa,3); R_sen(aa,4), R_sen(aa,5), R_sen(aa,6);  R_sen(aa,7), R_sen(aa,8), R_sen(aa,9)];
    end_z(:,aa) = R*[0;0;1];
end
figure(2120); 
plot3(end_z(1,:),end_z(2,:),end_z(3,:));grid on;
axis equal; 


% line([point_array(1,7); xbodyE(1)], [point_array(2,7); xbodyE(2)],  [point_array(3,7); xbodyE(3)],'color','y','LineWidth',2);hold on; %the x axis
% line([point_array(1,7); ybodyE(1)], [point_array(2,7); ybodyE(2)],  [point_array(3,7); ybodyE(3)],'color','g','LineWidth',2);hold on; %the y axis
% line([point_array(1,7); zbodyE(1)], [point_array(2,7); zbodyE(2)],  [point_array(3,7); zbodyE(3)],'color','b','LineWidth',2);hold on; %the z axis



figure(213);
plot(t, t_mag); 
ylabel('The magnitude of the thrust of each quadrotor'); 

figure(214);
plot(t, c_i_ang); 
ylabel('The third element of the thrust of each quadrotor'); 

figure(132);   %the earth force, no gravity
plot(t, wrench_array_nog_eath(1,:), t, wrench_array_nog_eath(2,:), t, wrench_array_nog_eath(3,:)); legend('f_x','f_y', 'f_z');
ylabel('control force'); 

figure(133);   %the torque
plot(t, wrench_array_nog_eath(4:6,:));
ylabel('control torque'); 


% thrust_vector  

for ii = 1:6
thrust_vector(3*(ii-1)+1:3*(ii-1)+3);  

    gamma_sens_quadrotor = zeros(length_t, 3);
    gamma_com_quadrotor = zeros(length_t, 3);
    norm_R_array=zeros(length(t),1);
    R_sen_i = y1(:, 9*(ii-1)+19:9*(ii-1)+27);
    for i =1:length(t)
        R_i = [R_sen_i(i,1), R_sen_i(i,2),  R_sen_i(i,3); R_sen_i(i,4), R_sen_i(i,5), R_sen_i(i,6);  R_sen_i(i,7), R_sen_i(i,8), R_sen_i(i,9)];
        R_d_i = R_ref(:,:,ii,i); 
        norm_R_array(i) = norm(R_i-R_d_i);
        gamma_com_quadrotor(i,:) = (Euler_angles(R_d_i))';
    end
    figure(5000+ii); %2-norm of  R-Rd of each quadrotor

    h=plot(t, gamma_com_quadrotor(:,1), '-.',  t, gamma_com_quadrotor(:,2), '--', t, gamma_com_quadrotor(:,3) ); grid;
    legend('\phi','\theta', '\psi');
    ylabel('Euler angles, quadrotor ','FontName','Times New Roman','FontSize',16);
    xlabel('Time (s)','FontName','Times New Roman','FontSize',16);
    set(gca,'FontSize',12, 'FontName','Times New Roman','LineWidth',1.5);
%     set(h, 'color',[0 0 0], 'LineWidth',1);
end

% the attitude tracking error of each quadrotor
for ii = 1:6
    gamma_sens_quadrotor = zeros(length_t, 3);
    gamma_com_quadrotor = zeros(length_t, 3);
    norm_R_array=zeros(length(t),1);
    R_sen_i = y1(:, 9*(ii-1)+19:9*(ii-1)+27);
    for i =1:length(t)
        R_i = [R_sen_i(i,1), R_sen_i(i,2),  R_sen_i(i,3); R_sen_i(i,4), R_sen_i(i,5), R_sen_i(i,6);  R_sen_i(i,7), R_sen_i(i,8), R_sen_i(i,9)];
        R_d_i = R_ref(:,:,ii,i); 
        norm_R_array(i) = norm(R_i-R_d_i);
        gamma_sens_quadrotor(i,:) = (Euler_angles(R_i))';
        gamma_com_quadrotor(i,:) = (Euler_angles(R_d_i))';
    end
    figure(4000+ii); %2-norm of  R-Rd of each quadrotor

    h=plot(t, norm_R_array); grid;
    ylabel('||{\sl R}- {\sl R}_d||, quadrotor','FontName','Times New Roman','FontSize',16);
    xlabel('Time (s)','FontName','Times New Roman','FontSize',16);
    set(gca,'FontSize',12, 'FontName','Times New Roman','LineWidth',1.5);
    set(h, 'color',[0 0 0], 'LineWidth',1);
        
end


% figure(132);   %the  
% plot(t, wrench0_earth(1:3,:));
% ylabel('control force'); 
% 
% 
% figure(133);   %the  
% plot(t, wrench0_earth(4:6,:));
% ylabel('control torque'); 

figure(40012);        %attitude tracking
subplot(3,1,1);
plot(t,gamma_com_quadrotor(:,1),'-.',t,gamma_sens_quadrotor(:,1)),grid;
ylabel('\phi(rad)');
title('Euler angles ofquadrotor:  Actual VS Commanded');
legend('\gamma_{com}','\gamma_{sen}');

subplot(3,1,2);
plot(t,gamma_com_quadrotor(:,2),'-.',t,gamma_sens_quadrotor(:,2)),grid;
ylabel('\theta(rad)');

subplot(3,1,3);
plot(t,gamma_com_quadrotor(:,3),'-.',t,gamma_sens_quadrotor(:,3)),grid;
ylabel('\psi(rad)');xlabel('time(s)');


figure(400); %2-norm of  R-Rd
norm_R_array=zeros(length(t),1);
for i =1:length(t)
    R = [R_sen(i,1), R_sen(i,2),  R_sen(i,3); R_sen(i,4), R_sen(i,5), R_sen(i,6);  R_sen(i,7), R_sen(i,8), R_sen(i,9)];
    R_d=[R_com(i,1), R_com(i,2),  R_com(i,3); R_com(i,4), R_com(i,5), R_com(i,6);  R_com(i,7), R_com(i,8), R_com(i,9)];
    R_nom_matrix = [R_nom(i,1), R_nom(i,2),  R_nom(i,3); R_nom(i,4), R_nom(i,5), R_nom(i,6);  R_nom(i,7), R_nom(i,8), R_nom(i,9)];
    norm_R_array(i) = norm(R-R_d);
    gamma_sens(i,:) = (Euler_angles(R))';
    gamma_com(i,:) = (Euler_angles(R_d))';
    gamma_nom(i,:) = (Euler_angles(R_nom_matrix))';
end
h=plot(t, norm_R_array); grid;
ylabel('||{\sl R}- {\sl R}_d||','FontName','Times New Roman','FontSize',16);
xlabel('Time (s)','FontName','Times New Roman','FontSize',16);
set(gca,'FontSize',12, 'FontName','Times New Roman','LineWidth',1.5);
set(h, 'color',[0 0 0], 'LineWidth',1);


figure(1);    %3-d position
  plot3(P_com(:,1),P_com(:,2),P_com(:,3),'-.',P_sens(:,1),P_sens(:,2),P_sens(:,3)); hold on; 
grid;
plot3(1, 0.05, 5, '*'); 
axis equal;
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
set (gca,'ZDir','reverse');
% title('POSITION TRACKIN:SENSED VS COMMAND');
% legend('track_c_o_m','track_s_e_n');

% the position constraints condition:
figure(2002); 
plot(t,  (P_sens(:,1)-1).^2+ (P_sens(:,2)-0.05).^2 + ( P_sens(:,3)-5).^2);grid; 
ylabel('Distance to obstacle(m)');


figure(2);        %position tracking
subplot(3,1,1);
plot(t,P_com(:,1),'-.',t,P_sens(:,1), t,P_nom(:,1), '--'),grid;
ylabel('X(m)');
% title('POSITION:SENSED VS COMMAND');
legend('P_c_o_m','P_s_e_n', 'P_{com}');

subplot(3,1,2);
plot(t,P_com(:,2),'-.',t,P_sens(:,2), t,P_nom(:,2), '--' ),grid;
ylabel('Y(m)');


subplot(3,1,3);
plot(t,P_com(:,3),'-.',t,P_sens(:,3), t,P_nom(:,3), '--' ),grid;
ylabel('H=-Z(m)');xlabel('time(s)');

figure(3);        %velocity tracking
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
ylabel('w(m/s)'); xlabel('time(s)');   

figure(4);        %attitude tracking
subplot(3,1,1);
plot(t,gamma_com(:,1),'-.',t,gamma_sens(:,1), t, gamma_nom(:,1), '--'),grid;
ylabel('\phi(rad)');
% title('Euler angles: Actual VS Commanded');
legend('\gamma_{com}','\gamma_{sen}', '\gamma_{nom}');

subplot(3,1,2);
plot(t,gamma_com(:,2),'-.',t,gamma_sens(:,2), t, gamma_nom(:,2), '--'),grid;
ylabel('\theta(rad)');

subplot(3,1,3);
plot(t,gamma_com(:,3),'-.',t,gamma_sens(:,3), t, gamma_nom(:,3), '--'),grid;
ylabel('\psi(rad)');xlabel('time(s)');

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

figure(5);        %angular velocity 
subplot(3,1,1);
plot(t,omega_com(:,1),'-.',t,omega_sens(:,1)),grid;
ylabel('p(rad/s)');
title('BODY RATE: SENSED VS COMMAND');
legend('��_c_o_m','��_s_e_n');

subplot(3,1,2);
plot(t,omega_com(:,2),'-.',t,omega_sens(:,2)),grid;
ylabel('q(rad/s)');

subplot(3,1,3);
plot(t,omega_com(:,3),'-.',t,omega_sens(:,3)),grid;
ylabel('r(rad/s)');xlabel('time(s)');

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
% figure(100);  %error of yaw angle
% for i=1:size(gamma_com,1)
% yaw_err(i)=error_yaw([gamma_com(i,3); gamma_sens(i,3)]);
% end
% plot(t,yaw_err),grid;
% ylabel('error of yaw (rad)');xlabel('time(s)');
% 
% 
% 
% figure(40);        %EC
% subplot(2,1,1);
% plot(t,xi_err),grid;
% ylabel('\xi');
% 
% for i=1:size(xi_err,1)
% xi_norm(i)=norm(xi_err(i,:));
% end
% 
% subplot(2,1,2);
% plot(t,xi_norm);grid;
% ylabel('norm of \xi');
% 
% 
% 
