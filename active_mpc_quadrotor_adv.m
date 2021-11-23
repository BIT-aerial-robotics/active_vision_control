%% Perception awareness MPC for multiple aerial vehicle systems
%Chuanbeibei Shi, Yushu Yu
% ------------------------------------------------
% The corresponding data of system trajectory
% is stored in mat file 
% ------------------------------------------------
% T: the time duration of simulation 
%
% 

function active_mpc_quadrotor_adv(T)
clc;
close all;
if nargin < 1
    T = 10; 
end


%% Simulation based on ODE function
% control function handles
ctrl_hdl1 = @virtual_Control;
current_hdl = ctrl_hdl1;
ctrl_hdl_str = func2str(current_hdl);

tajector_ref = @traj_gen;


% initial condition of different trials
% -------------------------------------------------
    % x(1:3): position
    % x(4:6): velocity
    % x(7:15): rotation matrix
    % x(16:18): body angular velocity
% -------------------------------------------------
% initial condition
% r0=[0;0;0];
% dr0=[1;0;0];
% ddr0=[0;0;0];
% dddr0=[0;0;0];
axis_ini = [1;1;1]; axis_ini = [1;1;0];
axis_ini = axis_ini/norm(axis_ini);
R0_ini= expm( anti_symmetric (axis_ini*0.6));

R0_ini= R0_ini*expm( anti_symmetric ([0;0;1]*0.4));
R0_ini_ = [R0_ini(1,1); R0_ini(1,2); R0_ini(1,3); R0_ini(2,1); R0_ini(2,2);R0_ini(2,3); R0_ini(3,1); R0_ini(3,2);R0_ini(3,3)];

global R_des;
axis_des = [0;0.2;1];
axis_des = [0;1.2;1]; axis_des = [1;1;0];
axis_des = axis_des/norm(axis_des);
R_des = expm( anti_symmetric (axis_des*(-0.6)));

y0 = [[10;0.4; 1];  ...  % the position 
[0;0;0]; ... %the velocity  
[0;-1;0;1;0;0;0;0;1]; ...%the rotation matrix
];    

global t_ctrl; 
global u_ctrl; 
global x_nom_next; 
x_nom_next = y0; %nominal trajectory 
t_ctrl = [0];
u_ctrl = zeros(4,1);
global u_nom_arr; 
u_nom_arr = zeros(4,1);
 
global wrench0_earth;
wrench0_earth = zeros(4,1);   
global scale  u_global; 
scale = 0;
u_global = zeros(4,1);
global y_nom; 
global R_ref;
global Ti_ref; 
global count_control; 
count_control =1; 

global thrust_vector_array; %store the thrust vector output from MPC


global errRint; %the integer of the attitude error
global erromegaint; %the integer of the angualar velocity 
errRint = zeros(3,6);
erromegaint = zeros(3,6); 


global Rd_m1  Rd_m2; 
global Rd_lasttime; 
global omegad_m1  omegad_m2; 
global omegad_lasttime; 

%initialization: 
for aa = 1:6
    Rd_m1(:,:,aa) = eye(3,3);
    Rd_m2(:,:,aa) = eye(3,3); 
    Rd_lasttime(:,:,aa) = eye(3,3);
end

omegad_m1 = zeros(3,6); 
omegad_m2 = zeros(3,6); 
omegad_lasttime = zeros(3,6); 

% option of ode function 
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3);

% simulation process
disp('The simulation process has started.');
disp(strcat('Controller: ', ctrl_hdl_str));
disp('---------------------------------------------');
tspan = [0 T];
% [t1, y1] = ode15s(@quad_3d_ode, tspan, y0, options, current_hdl);
% [t1, y1] = ode45(@quad_3d_ode, tspan, y0, options, current_hdl);
[t1, y1, y1_nom, y_ref_array, quadrotorinput] = self_solverdynamics(@quad_3d_ode, tspan, y0, options, current_hdl, tajector_ref);

% save all the data into .mat file  
save('sim_data.mat');
clear u_global flag_mode scale; 
global name;  
global pos_ob_array_pre radius_pre;
save(name);
disp('Data successfully stored!');


end


% nominal MPC: 
function [u, wrench_nom, thrust_vector] = virtual_Control(t, y, y_nom, trajd)
%wrench_nom: the 6-by-1 vector of the wrench, force is expressed in body
%frame, the gravity of the dynamics is considered in the term 
%y_nom: the nominal value of the dynamics, used in the MPC design

global scale  u_global u_global_act;
global x_nom u_nom; 
global flag_mode; 
global mpccontrol; 
global count_control; 
global R_des;
global pos_state_mpc att_state_mpc;
global u_nom_earth; 
global u_nom_body; 

count_control = count_control+1; 
 
dt = 0.001; 
if (scale==0)
%run mpc   
 
%     L_0 = y_nom(1)^2 +  y_nom(2)^2 + y_nom(3)^2; 
    
    hori_T = 0.7;
    
    r0_11 = y_nom(7);
    r0_12 = y_nom(8);
    r0_13 = y_nom(9);
    r0_21 = y_nom(10);
    r0_22 = y_nom(11);
    r0_23 = y_nom(12);
    r0_31 = y_nom(13);
    r0_32 = y_nom(14);
    r0_33 = y_nom(15);
    
    %the position of the vehicle:
    x0_x = y_nom(1); 
    x0_y = y_nom(2);  
    x0_z = y_nom(3);
 
    p_fx = 5; p_fy =10;  p_fz = 0;
    cosbeta = (p_fx*r0_11 + p_fy*r0_21 + p_fz*r0_31 - r0_11*x0_x - r0_21*x0_y - r0_31*x0_z)/((p_fx*r0_11 + p_fy*r0_21 + p_fz*r0_31 - r0_11*x0_x - r0_21*x0_y - r0_31*x0_z)^2 + (p_fx*r0_12 + p_fy*r0_22 + p_fz*r0_32 - r0_12*x0_x - r0_22*x0_y - r0_32*x0_z)^2 + (p_fx*r0_13 + p_fy*r0_23 + p_fz*r0_33 - r0_13*x0_x - r0_23*x0_y - r0_33*x0_z)^2)^(1/2);
     
    out = quadrotorload_input_adv_RUN(t, t+hori_T, ...  %start time and final time 
    y_nom(1), y_nom(2), y_nom(3),...
    y_nom(4), y_nom(5), y_nom(6),... %feedback states of the linear velocity of the load
    y_nom(7), y_nom(8), y_nom(9), y_nom(10), y_nom(11), y_nom(12), y_nom(13), y_nom(14), y_nom(15),...     %feedback states of rotation matrix 
    cosbeta,... %cosbeta
    0); 

    mass = 2.1;
    g_ = 9.8;

    wrench_0 = out.CONTROLS(1, 2:5)'; %control from MPC: 3d force and yaw rate
    Tminusmg = wrench_0(4) + mass*g_;
    wrench_0 = [wrench_0(1:3); Tminusmg];
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %     thrust_vector = zeros(18,1); 
    u_global =wrench_0;
%     x_nom = out(19:end); 
    u_nom = wrench_0;
     
   
elseif (mod(scale, 10)==0)
    if (flag_mode==1)  %mpc and cbf
%         correct = cbf_seperate_mult_dynamic_complex_constraints_correct([y; t; u_global]);
%         u = correct.u;
%         u_global =u;
        wrench_0 = u_global; 
        
%         no_control = scale/10+1; 
%         wrench_0 = mpccontrol(no_control, 2:end)'; 
%         u_global = wrench_0; 
    else
        wrench_0 = u_global; 
    end
else 
    wrench_0 = u_global; 
end

wrench_nom = wrench_0; 

%feedback control on the wrench of load, then transform wrench to the
%thrust vector:
if (mod(scale, 1)==0)
% if ((scale)==0)
% closed loop control 
%     k =  0.1*[0, 0.1, 0.01, 0.01, 0.01, 0.0; 0.1, 0, 0, 0, 0.1, 0];
%     i_cur = floor(scale/20)+1; 
%     x_nom_cur = x_nom(i_cur,2:end-1)';
%     u_nom_cur = u_nom(i_cur,2:end)';
    
    
    %feedback state:
    v0_earth = [y(4); y(5); y(6)]; %body linear velocity 
    p0 = [y(1); y(2); y(3)];  %earth position 
    R_vec = y(7:15);
    R = [R_vec(1), R_vec(2), R_vec(3);...
    R_vec(4), R_vec(5), R_vec(6);...
    R_vec(7), R_vec(8), R_vec(9)];
     
    v0_earth_nom = [y_nom(4); y_nom(5); y_nom(6)]; %body linear velocity 
    p0_nom = [y_nom(1); y_nom(2); y_nom(3)];  %earth position 
    R_nom_vec = y_nom(7:15);
    R_nom = [R_nom_vec(1), R_nom_vec(2), R_nom_vec(3);...
    R_nom_vec(4), R_nom_vec(5), R_nom_vec(6);...
    R_nom_vec(7), R_nom_vec(8), R_nom_vec(9)];
    
    %nominal state: 
 
    %feedback control: should be tunned carefully  
    
    k_pos = -20;
    k_att = -8; 
    k_vel = -40; 
    k_anv = -10;
 
    e_p_earth = p0 - p0_nom;
    e_v_earth = v0_earth - v0_earth_nom; 
    e_R = R - R_nom; 
%     matrix=[0 -x(3) x(2);
%         x(3) 0 -x(1);
%         -x(2) x(1) 0];
%     feedbackf_earth = k_pos*e_p_earth +  k_vel*e_v_earth + k_anv * e_an_vel +  wrench_nom(1:3);
%     feedbackztorque =  + k_anv * e_an_vel +  wrench_nom(4);
%     
%     wrench_0 = [feedbackf_earth; feedbackztorque]; %the feedback control law  
    wrench_0 = zeros(4,1);
    u_global_act = wrench_0;
    
%     wrench_0(1:3)= k_pos* (p0 - p0_nom) + k_vel* (v0_body-v0_body_nom) +  wrench_nom(1:3); 
%     u_global_act = wrench_0;
        
%     if (coef_.C == 1)
%         %the feasible control space is empty
%         u = [0; -4]; 
%     else    
%         H= diag([1;1]);
%         f2 = -2* u;  %the optimal goal is for the entire control
%         optoption_1 = optimset('Display', 'off', 'TolFun', 1e-10);
%         alpha=[1.0; 4];
%         [x, FVAL, EXITFLAG] = quadprog(H, f2, coef_.A, coef_.B, [], [], -alpha, alpha, [], optoption_1);
%         u = x; 
%     end    
%         u_global = u;
%     u = u_global;

else     
%     u = u_global;
    wrench_0 = u_global_act; 
end
    u = wrench_0;
    thrust_vector = u;

scale = scale+1;
if (scale == 20)  %tune the sampling time
    scale = 0;
end
end


%% Ode Function of this vehicle
function [dy, dy_nom, u, u_nom, thrust_vector] = quad_3d_ode(t, y, y_nom, ctrl_hdl)

dy =zeros(15,1);

% get the current reference and control input 
trajd = traj_gen(t);
[u, u_nom, thrust_vector] = feval(ctrl_hdl, t, y, y_nom, trajd);

%% 2021-11-14, the nominal dynamics:
p_nom = y_nom(1:3);
v_nom = y_nom(4:6);
R_nom_vec = y_nom(7:15);
R_nom = [R_nom_vec(1), R_nom_vec(2), R_nom_vec(3);...
    R_nom_vec(4), R_nom_vec(5), R_nom_vec(6);...
    R_nom_vec(7), R_nom_vec(8), R_nom_vec(9)];

omega_nom = u_nom(1:3);
T_nom = u_nom(4);

mass = 2.1;
g_ =9.8;
e_3 = [0;0;1];

vdot_nom = 1/mass*(-R_nom*T_nom*e_3) + g_*e_3;
R_nom_dot = R_nom*anti_symmetric(omega_nom);
R_nom_dot_col = [R_nom_dot(1,1); R_nom_dot(1,2); R_nom_dot(1,3); R_nom_dot(2,1); R_nom_dot(2,2); R_nom_dot(2,3); R_nom_dot(3,1); R_nom_dot(3,2); R_nom_dot(3,3)];

dy_nom  =   [v_nom; vdot_nom; R_nom_dot_col];  

%% 2021-11-14, the actual dynamics:


p = y(1:3);
v = y(4:6);
R_vec = y(7:15);
R = [R_vec(1), R_vec(2), R_vec(3);...
    R_vec(4), R_vec(5), R_vec(6);...
    R_vec(7), R_vec(8), R_vec(9)];

omega = u(1:3);
T = u(4);

mass = 2.1;
g_ =9.8;
e_3 = [0;0;1];

vdot = 1/mass*(-R*T*e_3) + g_*e_3;
R_dot = R*anti_symmetric(omega);
R_dot_col = [R_dot(1,1); R_dot(1,2); R_dot(1,3); R_dot(2,1); R_dot(2,2); R_dot(2,3); R_dot(3,1); R_dot(3,2); R_dot(3,3)];

dy  =   [v; vdot; R_dot_col];  

delta = 1.0*[zeros(3,1);...
    10*[1.0*rand(1,1)-0.5; 1.0*rand(1,1)-0.5; 1.0*rand(1,1)-0.5];...
     0*rand(9,1);...
     ]; 

dy = dy+delta; 

%input signal% -----------------------------------------------
% check simulation time for stability property
debug = 1;
if debug == 1
    disp(['The current time is ', num2str(t)]);
 
end
% -----------------------------------------------

%record control: 
global t_ctrl; 
global u_ctrl;
global u_nom_arr;
t_ctrl = [t_ctrl; t];
u_ctrl = [u_ctrl, u];
u_nom_arr = [u_nom_arr, u_nom];

global wrench0_earth; 
wrench0_earth = [wrench0_earth; u]; 
end


function out = traj_gen(u)

% r=7.5;
% pos = [r*cos(1/r*t); r*sin(1/r*t); 0];
% vel = [-sin(1/r*t); cos(1/r*t); 0];
% acc = [-1/r*cos(1/r*t); -1/r*sin(1/r*t); 0];
% dacc =[1/r^2*sin(1/r*t); -1/r^2*cos(1/r*t); 0]; 
% d2acc = [1/r^3*cos(1/r*t); 1/r^3*sin(1/r*t); 0];
 t0 = 0;  %start time 
alpha = 20;  %duration time 

coe_x_pos = [0;0;0;0;1.536999999997807e+03;-5.531999999986896e+03;7.681999999969546e+03;-4.827999999965346e+03;1.151999999980595e+03;4.292660782390101e-09]; 
coe_x_vel = [0;0;0;6.147999999991229e+03;-2.765999999993448e+04;4.609199999981727e+04;-3.379599999975742e+04;9.215999999844762e+03;3.863394704151091e-08;0];
coe_x_acc = [0;0;1.844399999997369e+04;-1.106399999997379e+05;2.304599999990864e+05;-2.027759999985445e+05;6.451199999891334e+04;3.090715763320873e-07;0;0];

coe_y_pos = [1;0;0;0;1.605999999997880e+03;-6.311999999987353e+03;9.355999999970678e+03;-6.183999999966713e+03;1.535999999981404e+03;4.104722561848462e-09];
coe_y_vel = [0;0;0;6.423999999991518e+03;-3.155999999993677e+04;5.613599999982407e+04;-4.328799999976700e+04;1.228799999985123e+04;3.694250305663616e-08;0];
coe_y_acc = [0;0;1.927199999997456e+04;-1.262399999997471e+05;2.806799999991204e+05;-2.597279999986020e+05;8.601599999895860e+04;2.955400244530893e-07;0;0];

coe_z_pos = [2;0;0;0;-1.512999999997308e+03;5.883999999983917e+03;-8.657999999962642e+03;5.691999999957507e+03;-1.407999999976212e+03;-5.260956115193949e-09];
coe_z_vel = [0;0;0;-6.051999999989232e+03;2.941999999991958e+04;-5.194799999977585e+04;3.984399999970254e+04;-1.126399999980970e+04;-4.734860503674554e-08;0];
coe_z_acc = [0;0;-1.815599999996770e+04;1.176799999996783e+05;-2.597399999988792e+05;2.390639999982153e+05;-7.884799999866789e+04;-3.787888402939643e-07;0;0];


t= (u-t0)/alpha;
t_vector = [t^0; t^1; t^2; t^3; t^4; t^5; t^6; t^7; t^8; t^9];

pos = [coe_x_pos'*t_vector;  coe_y_pos'*t_vector; coe_z_pos'*t_vector];
vel = [coe_x_vel'*t_vector;  coe_y_vel'*t_vector; coe_z_vel'*t_vector];
acc = [coe_x_acc'*t_vector;  coe_y_acc'*t_vector; coe_z_acc'*t_vector];
vel = vel/alpha;
acc = acc/alpha/alpha;

out = [pos; vel; [1; 0; 0; 0; 1; 0; 0; 0; 1]; [0; 0; 0]];

% R_des = expm( anti_symmetric ([0;0;1]));
global R_des;
% R_des = eye(3,3); 
R_des_ = [R_des(1,1); R_des(1,2); R_des(1,3); R_des(2,1); R_des(2,2);R_des(2,3); R_des(3,1); R_des(3,2);R_des(3,3)];
out = [[0; 0; 0]; [0; 0; 0]; R_des_; [0; 0; 0]];  %constant destination

out = [0;0;0;0;0;0;1; 0; 0; 0; 1; 0; 0; 0; 1];
% out= ones(18,1); 
end





 
