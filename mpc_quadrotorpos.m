
function f = mpc_quadrotorpos(u)
%u: feedback state, and current time,  7-by-1
% 
hori_T = 0.7;
out = quadrotorload_input_pos_RUN(u(7), u(7)+hori_T, ...  %start time and final time 
    u(1), u(2), u(3),...
    u(4), u(5), u(6),... %feedback states of the linear velocity of the load
    u(8),...% feedback yaw angle 
    0); 

f1 = out.CONTROLS(1, 2:5)'; %control from MPC: 3d force and yaw rate
f2 = out.STATES(1, 7)'; %the state of yaw angle output from MPC
f = [f1;f2];

end