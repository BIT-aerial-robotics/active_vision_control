function  err=attitude_err(u)
%compute the equivalent angle and axis according to the given Euler angles
phi=u(1);  theta=u(2);  psi=u(3); 
% R=rotate(psi,'z')*rotate(theta,'y')*rotate(phi,'x');

R_com=[cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
  cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
  -sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];

hat_kxi=[0 -u(6) u(5);
        u(6) 0 -u(4);
        -u(5) u(4) 0];

R_sens=expm(hat_kxi);

matrix=R_com'*R_sens;

err=2-sqrt(1+matrix(1,1)+matrix(2,2)+matrix(3,3));






