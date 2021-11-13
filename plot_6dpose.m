function plot_6dpose(configuration) 
%ע�ⵥλ, mm 
%%plot the configuration of MMAR 
%%by Yushu Yu, revised on May 25th, 2020, units: mm
%input is the configuration of a rigidbody, where the attitude is
%expressed in rotation matrix 

%need the file rotor_plot.m 

% robotparameter:
% global l1 l2 dt dc m1 m2 m0 I_c1_x I_c1_y I_c1_z I_c2_x I_c2_y I_c2_z I_x_m I_y_m I_z_m;
% l1=1000*l1; l2=1000*l2; dt = 1000*dt; dc=1000*dt; %convert the unit from m to  mm 
% dv=20;  %the distance between the plane of propellers and the CG, along the positive direction of body z axis
% CCC=2^0.5/2;  %constant
% r=304.8/2;   %12 inch, propellers, 
% dr=550;  %distance bwtween a pair of propellers

% global zoomfactor;


% main_tune_gain; 


% x=P(1); y=P(2); z=P(3); %position of the CG
% phi=attitude(1); theta=attitude(2); psi=attitude(3); %orientation of Sigma_0
% alpha_I1= alpha(1);  alpha_I2= alpha(2); alpha_II1= alpha(3); alpha_II2= alpha(4); %the joint positions, units: rad
% 
% R=[cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
%   cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
%   -sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];  %rotation matrix, Sigma_0 relative to Sigma_E

x=configuration(1); y=configuration(2); z=configuration(3); %position of the CG
R= [configuration(4), configuration(5), configuration(6);
    configuration(7), configuration(8), configuration(9);
    configuration(10), configuration(11), configuration(12)];    
   
Sigma0_x=x; Sigma0_y=y; Sigma0_z=z;   %position of the origin of Sigma_0

E_b_T = [R, [Sigma0_x; Sigma0_y; Sigma0_z]; [0, 0, 0, 1]];  %transformation matrix, Sigma_0 relative to Sigma_E


%the point represents the three body axis:
global axisscale;
axisscale = 1; 
xbody=[[1;0;0]*axisscale; 1];
ybody=[[0;1;0]*axisscale; 1];
zbody=[[0;0;1]*axisscale; 1];
xbodyE =E_b_T*xbody;
ybodyE =E_b_T*ybody;
zbodyE =E_b_T*zbody;

line([E_b_T(1,4); xbodyE(1)], [E_b_T(2,4); xbodyE(2)],  [E_b_T(3,4); xbodyE(3)],'color','y','LineWidth',2);hold on; %the x axis
line([E_b_T(1,4); ybodyE(1)], [E_b_T(2,4); ybodyE(2)],  [E_b_T(3,4); ybodyE(3)],'color','g','LineWidth',2);hold on; %the y axis
line([E_b_T(1,4); zbodyE(1)], [E_b_T(2,4); zbodyE(2)],  [E_b_T(3,4); zbodyE(3)],'color','b','LineWidth',2);hold on; %the z axis

axis equal; grid on; set(gca,'ZDir','reverse'); set(gca,'YDir','reverse');

end

