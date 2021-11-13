function [t1, y1, y1_nom, y_ref_array, quadrotorinput, thrust_vector]=self_solverdynamics(ode, tspan, y0, options, current_hdl, tajector_ref_hdl)
% quad_3d_ode(t, y,  ctrl_hdl)
% [t1, y1] = ode45(@quad_3d_ode, tspan, y0, options, current_hdl);

dt=0.001;

n_state=length(y0);   %dimension of the states

t1=tspan(1):dt:tspan(end);
n_time=length(t1);

global R_ref; 
R_ref = zeros(3,3,6,n_time); 
for aa = 1:6
    R_ref(:,:,aa,1) = eye(3,3);
end

y1=zeros(n_time, n_state);
y1(1,:) = y0';
y=y0;

%the nominal dynamics
y1_nom=zeros(n_time, n_state);
y1_nom(1,:) = y0';
y_nom=y0;

y_ref_array = zeros(n_time, 18);
y_ref_array(1,1:18) = y0(1:18)';

quadrotorinput = zeros(n_time, 4*6);
quadrotorinput(1, :) = zeros(1, 24); 

thrust_vector = zeros(n_time, 3*6);
thrust_vector(1,:) = zeros(1,18); 


for i=2:n_time    
    t=t1(i);    
    y_ref = feval(tajector_ref_hdl, t); %reference trajectory
    y_ref_array(i,:) = y_ref'; 
    [dy,dy_nom, u,u_nom, thrust ] = feval(ode, t, y, y_nom,  current_hdl);
    thrust_vector(i,:) = thrust'; 
%     quadrotorinput(i,:) = u'; 
    quadrotorinput(i,:) = zeros(24,1)'; 
    y=y+dy*dt; 
    y1(i,:)=y';
    
    y_nom=y_nom+dy_nom*dt; 
    y1_nom(i,:)=y_nom';
    
end
