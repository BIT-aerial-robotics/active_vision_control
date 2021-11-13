function  exponential=angle_axis(euler)
%compute the equivalent angle and axis according to the given Euler angles
phi=euler(1);  theta=euler(2);  psi=euler(3); 
% R=rotate(psi,'z')*rotate(theta,'y')*rotate(phi,'x');

R=[cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
  cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
  -sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];

% delta_x=(R(2,2)-1)*(R(3,3)-1)-R(2,3)*R(3,2);
% delta_y=(R(1,1)-1)*(R(3,3)-1)-R(1,3)*R(3,1);
% delta_z=(R(1,1)-1)*(R(2,2)-1)-R(1,2)*R(2,1);
% 
% if delta_x~=0
%     x=delta_x;
%     y=0;
% end
angle=acos((R(1,1)+R(2,2)+R(3, 3)-1)/2);
if angle~=0
    omega=1/(2*sin(angle))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
else 
    omega=[0; 0; 1];
end
exponential=angle*omega;

end