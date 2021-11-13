function ea=Euler_angles(u)
%calculate the Eulear angles frome rotation maxtrix
 

%rotation matrix
% R=[u(1), u(2), u(3); 
%    u(4), u(5), u(6);
%    u(7), u(8), u(9)]; 

R = u; 

ea_m = [atan(R(3,2)/R(3,3)); -asin(R(3,1)); atan(R(2,1)/R(1,1))];  %main Euler angles 

theta = ea_m(2); %pitch angle equals to the main pitch angle

if((R(3,3)==0) && (R(3,2)>=0))  %roll angle 
    phi = pi/2; 
elseif((R(3,3)==0) && (R(3,2)<0))
    phi = -pi/2; 
elseif(R(3,3)>0)
    phi = ea_m(1);
elseif ((R(3,3)<0) && (R(3,2)>=0))
    phi = ea_m(1)+pi;
elseif ((R(3,3)<0) && (R(3,2)<0))
    phi = ea_m(1)-pi;
end

if((R(1,1)==0) && (R(2,1)>=0)) %yaw angle
    psi = pi/2; 
elseif((R(1,1)==0) && (R(2,1)<0))
    psi = -pi/2; 
elseif(R(1,1)>0)
    psi = ea_m(3);
elseif ((R(1,1)<0) && (R(2,1)>=0))
    psi = ea_m(3)+pi;
elseif ((R(1,1)<0) && (R(2,1)<0))
    psi = ea_m(3)-pi;
end

ea = [phi; theta; psi];
end

