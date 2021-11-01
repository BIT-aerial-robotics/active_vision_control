function xi=exponentials_R(u)

R=[u(1), u(2), u(3); 
   u(4), u(5), u(6);
   u(7), u(8), u(9)];

theta_xi = acos((R(1,1)+ R(2,2)+R(3,3)-1)/2);

if theta_xi~=0
    omega_xi = 1/(2*sin(theta_xi))*[R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
else
    omega_xi=[0;0;1];
end

xi = omega_xi*theta_xi;
