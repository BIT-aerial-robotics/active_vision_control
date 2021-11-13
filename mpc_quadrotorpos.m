
function f = mpc_quadrotorpos(u)
%u: feedback state, and current time,  7-by-1
% 
hori_T = 0.7;
out = quadrotorload_input_pos_RUN(u(7), u(7)+hori_T, ...  %start time and final time 
    u(1), u(2), u(3),...
    u(4), u(5), u(6),... %feedback states of the linear velocity of the load
    0); 

f = out.CONTROLS(1, 2:4)';

end