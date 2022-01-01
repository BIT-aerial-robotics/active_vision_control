%Example for kalman filter, Course of Autonomous Robots 

% In this example, the 3-d translational motion of a vehicle is simulated. 
% The GPS provides global position observation of the vehicle. 
% The accelerometer measures the acceleration of the vehicle. 
% Kalman filter is used to fuse the data reading from accelerometer and GPS.

%Yushu Yu, March, 2019

close all; 

t_samp = 0.01; 
T = 10000; 

t_array = 0:t_samp:T;
size_T = length(t_array);
u_array = repmat([0.0; 0.0; 0.0], 1, size_T); %input 
% u_array = repmat(rand(3,1), 1, size_T); %input 
x_k = zeros(6,1);
x_array = zeros(6, size_T); 


%% simulate the real motion: 
for k=1:size_T
    G_T = [eye(3,3), t_samp*eye(3,3); zeros(3,3), eye(3,3)];
    H_T = [0.5*t_samp*t_samp*eye(3,3); t_samp*eye(3,3)];
    
    u_k = u_array(: , k);
    x_k = G_T*x_k + H_T*u_k; 
    x_array(:,k) = x_k;  
end
%position 
figure(1);
plot(t_array, x_array(1:3,:));
xlabel('Time/s'); ylabel('Position, actual (m)');  legend('P_x','P_y','P_z'); grid on; 

%velocity 
figure(2);
plot(t_array, x_array(4:6,:));
xlabel('Time/s'); ylabel('velocity, actual (m/2)');  legend('V_x','V_y','V_z'); grid on; 

%acc  
figure(3);
plot(t_array, u_array);
xlabel('Time/s'); ylabel('acc, actual (m/s^2)');  legend('a_x','a_y','a_z'); grid on; 

%% global observation
t_ob_global = 0.5; 
t_array_ob = 0:t_ob_global:T;
size_T_ob = length(t_array_ob);
z_array = zeros(3, size_T_ob);  %observation, position
for k=1:size_T_ob
       k_motion = t_ob_global*k/t_samp;
       if k_motion > size_T
           z_k_mean = x_array(1:3, end);
       else
           z_k_mean = x_array(1:3, k_motion); 
       end
       Q =   2000.0 *...
         [0.0182    0.0024   -0.0010
          0.0024    0.0398   -0.0001
          -0.0010   -0.0001    0.1915]; 
%       Q =   0.25 *eye(3,3);
      
       r = mvnrnd(z_k_mean, Q, 1);
       z_array(:,k) = r';
end

%observed position 
figure(4);
plot(t_array_ob, z_array(1:3,:));
xlabel('Time/s'); ylabel('Position, GPS (m)');  legend('P_x','P_y','P_z'); grid on; 


%% integrate: 
t_input = 0.1; 
t_array_input = 0:t_input:T;
size_T_input = length(t_array_input);
u_array_noise = zeros(3, size_T_input); 
x_array_int = zeros(6, size_T_input);
x_k = zeros(6,1);
for k=1:size_T_input
    k_motion = t_input*k/t_samp;
    G_T = [eye(3,3), t_input*eye(3,3); zeros(3,3), eye(3,3)];
    H_T = [0.5*t_input*t_input*eye(3,3); t_input*eye(3,3)];
    if k_motion > size_T
           u_k_mean = u_array(1:3, end);
    else
           u_k_mean = u_array(1:3, round(k_motion)); 
    end
        R =   1.0* 1e-3 *...
         [0.0182    0.0024   -0.0010
          0.0024    0.0398   -0.0001
          -0.0010   -0.0001    0.1915]; 
      
      R =   0.04 *eye(3,3);
       r = mvnrnd(u_k_mean, R, 1);
    u_array_noise(:,k) = r';
    x_k = G_T*x_k + H_T*r'; 
    x_array_int(:,k) = x_k;  
end

%position 
figure(5);
plot(t_array_input, x_array_int(1:3,:));
xlabel('Time/s'); ylabel('Position, integrate  (m)');  legend('P_x','P_y','P_z'); grid on; 

%velocity 
figure(6);
plot(t_array_input, x_array_int(4:6,:));
xlabel('Time/s'); ylabel('velocity, integrate (m/2)');  legend('V_x','V_y','V_z'); grid on; 

%acc  
figure(7);
plot(t_array_input, u_array_noise);
xlabel('Time/s'); ylabel('acc with noise (m/s^2)');  legend('a_x','a_y','a_z'); grid on; 


%% filter:
t_est = 0.05; 
t_array_est = 0:t_est:T;
size_T_est = length(t_array_est);
P_k_k = zeros(6,6); 
x_k_k = zeros(6,1);
x_array_est = zeros(6, size_T_est);

for k=1:size_T_est
    G_T = [eye(3,3), t_est*eye(3,3); zeros(3,3), eye(3,3)]; %state transition matrix
    H_T = [0.5*t_est*t_est*eye(3,3); t_est*eye(3,3)];  %input matrix
    C_T = [eye(3,3), zeros(3,3)]; %output matrix 
    
    k_input = round(k*t_est/t_input);
    if k_input > size_T_input
        u_k = u_array_noise(1:3, end);
    else
        u_k = u_array_noise(1:3, k_input); 
    end
    
    k_ob = round(k*t_est/t_ob_global);
    if k_ob > size_T_ob
        z_k = z_array(1:3, end);
    elseif k_ob ==0 
        z_k = z_array(1:3, 1); 
    else
        z_k = z_array(1:3, k_ob); 
    end

    x_k_k_1 = G_T*x_k_k + H_T* u_k;
    %test:
    R  = zeros(3,3);  Q = 0.0*rand(3,3);  Q=eye(3,3);
    P_k_k_1 = G_T*P_k_k*G_T' + H_T*R*H_T';
    
    y = z_k -C_T*x_k_k_1;
    S=C_T*P_k_k_1*C_T'+ Q; %observation
    K=P_k_k_1*C_T'*inv(S);     
    x_k_k=x_k_k_1+K*y;
    P_k_k=(eye(6,6)-K*C_T)*P_k_k_1;
    x_array_est(:,k)=x_k_k;
end

%position 
figure;
plot(t_array_est, x_array_est(1:3,:));
xlabel('Time/s'); ylabel('Position, estimated  (m)');  legend('P_x','P_y','P_z'); grid on; 

%velocity 
figure;
plot(t_array_est, x_array_est(4:6,:));
xlabel('Time/s'); ylabel('velocity, estimated (m/2)');  legend('V_x','V_y','V_z'); grid on; 
