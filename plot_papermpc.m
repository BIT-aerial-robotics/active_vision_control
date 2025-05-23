close all;


% load sim_data.mat;


t = t1;

P_sens = y1(:, [6,5]); 
P_com = P_sens;
% P_com=reference(:,1:2);


delta = u_ctrl(1,:);
dot_y = y1(:,2);
dot_x = y1(:,1);
dot_psi = y1(:,3); 

a = 1.41; 
b = 1.576; 

alpha_f= (dot_y+a*dot_psi)./dot_x - delta'; 
alpha_r = (dot_y -b*dot_psi)./dot_x; 

figure(100); 
subplot(2,1,1);
plot(t,alpha_f ),grid;
ylabel('\alpha_f');
title('slip ratios');
% legend('P_c_o_m','P_s_e_n');

subplot(2,1,2);
plot(t,alpha_r ),grid;
ylabel('\alpha_r');
 xlabel('time(s)');


len = length(t); 
h_plot = zeros(len,1);

for i=1:len
    h_plot(i) = (P_sens(i,1) - 80)^2 + (P_sens(i,2) - 0.5)^2 -1;
end
 



figure(1);   
h = plot(P_com(:,1),P_com(:,2),'-.',P_sens(:,1),P_sens(:,2)),grid; hold on;
% axis equal;
xlabel('s(m)');ylabel('e_y(m)');
% title('POSITION TRACKING:SENSED VS COMMAND');
% legend('traj_{com}','traj_{sen}');
axis equal; 

global pos_ob_array_pre radius_pre;
%pos_ob_array_pre(2, :) = - pos_ob_array_pre(2,:);  %only for plot the mpc data with mistake 
for i=1:size(pos_ob_array_pre,2)
    plot(pos_ob_array_pre(1,i), pos_ob_array_pre(2,i), '*r', 'LineWidth',1); hold on; 
    Ds =radius_pre(i);
    circle(Ds,pos_ob_array_pre(1,i), pos_ob_array_pre(2,i)); hold on; 
end
 

set(h, 'LineWidth',1);
set(gca,'FontSize',16, 'FontName','Times New Roman','LineWidth',1.5);

figure(2);   
subplot(4,1,1);
plot(t,P_com(:,1),'-.',t,P_sens(:,1)),grid;
ylabel('X(m)');
title('POSITION:SENSED VS COMMAND');
% legend('P_c_o_m','P_s_e_n');

subplot(4,1,2);
plot(t,P_com(:,2),'-.',t,P_sens(:,2)),grid;
ylabel('Y(m)');
 xlabel('time(s)');
 
 subplot(4,1,3);
plot(t,y1(:,1) ),grid;
ylabel('v(m/s)');
 xlabel('time(s)');
 
 subplot(4,1,4);
plot(t,y1(:,4)/pi*180 ),grid;
ylabel('\psi(degree)');
 xlabel('time(s)');
 
 
 figure(3);
 plot(t, h_plot); 
ylabel('distance');
 xlabel('time(s)');
 
 
 figure(4);   %control input
%   plot(t_ctrl, u_ctrl(1:2,:));    
  subplot(2,1,1);
  plot(t_ctrl, u_ctrl(1,:),  'LineWidth',1);  grid; 
ylabel('$\delta_f$ (rad)',   'Interpreter','latex','FontSize',16);
%  xlabel('time(s)'); 
 set(h, 'LineWidth',1);
set(gca,'FontSize',16, 'FontName','Times New Roman','LineWidth',1.5);
 
   subplot(2,1,2);
  plot(t_ctrl, u_ctrl(2, :),  'LineWidth',1 );  grid;  
%   ylim([-4 4]);
ylabel('$a_x (m/s^2)$' , 'Interpreter','latex','FontSize',16);
 xlabel('time(s)','Interpreter','latex','FontSize',16);
%  title('actual control'); 
%  set(h, 'LineWidth',1);
set(gca,'FontSize',16, 'FontName','Times New Roman','LineWidth',1.5);
 

% figure(4);
% plot(t,input),grid;
% ylabel('control input');xlabel('time(s)');


% figure(4);
% plot(t,h),grid;
% ylabel('h');xlabel('time(s)');


% figure(5);
% subplot(4,1,1);
% plot(t,test(:,1)),grid;  ylabel('An 1');
% 
% 
% subplot(4,1,2);
% plot(t,test(:,2)),grid;  ylabel('An 2');
% 
% subplot(4,1,3);
% plot(t,test(:,3)),grid; ylabel('nominal input 1');
% 
% subplot(4,1,4);
% plot(t,test(:,4)),grid;
% 
% ylabel('nominal input 2');xlabel('time(s)');


%%road side:
road_side_x = min(P_sens(:,1)):0.1:max(P_sens(:,1));
road_side_y1 = 3.7*ones(1, length(road_side_x));
road_side_y2 = -3.7*ones(1, length(road_side_x));
figure(1);
plot(road_side_x,road_side_y1,'-.k',road_side_x,road_side_y2,'-.k'); hold on;


