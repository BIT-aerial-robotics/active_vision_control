function gamma=Eulerangles_R(u)

R=[u(1), u(2), u(3); 
   u(4), u(5), u(6);
   u(7), u(8), u(9)];



 if R(1, 1)>0 %%˵��theta<90��,,ȡ��ָ��Ӧ��ʹ��psi ��0�ȸ���
     theta = -asin(R(3,1));
 else 
     theta = pi+asin(R(3,1));
 end

% theta = -asin(R(3,1));
cos_theta =cos(theta);

phi = asin (R(3,2)/cos_theta);

psi = asin (R(2,1)/cos_theta);

gamma=[phi; theta; psi];
