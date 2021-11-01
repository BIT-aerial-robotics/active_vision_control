


% DifferentialState xp_dot yp_dot  psi_dot epsi  ey s L;  %sometimes, the name of the variables induces errors, I don't know why. 
%     Control delta_f a_x;  
figure (1);    
subplot(3,1,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('p0_x');

subplot(3,1,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('p0_y');

subplot(3,1,3)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('p0_z');

figure(2);
subplot(3,1,1)
plot(out.STATES(:,1), out.STATES(:,5), 'r')
title('v0_x');

subplot(3,1,2)
plot(out.STATES(:,1), out.STATES(:,6), 'r')
title('v0_y');

subplot(3,1,3)
plot(out.STATES(:,1), out.STATES(:,7), 'r')
title('v0_z');

% figure(3);
% subplot(3,3,1)
% plot(out.STATES(:,1), out.STATES(:,8), 'r')
% title('r0_11');
% 
% subplot(3,3,2)
% plot(out.STATES(:,1), out.STATES(:,9), 'r')
% title('r0_12');
% 
% subplot(3,3,3)
% plot(out.STATES(:,1), out.STATES(:,10), 'r')
% title('r0_13');
% 
% subplot(3,3,4)
% plot(out.STATES(:,1), out.STATES(:,11), 'r')
% title('r0_21');
% 
% subplot(3,3,5)
% plot(out.STATES(:,1), out.STATES(:,12), 'r')
% title('r0_22');
% 
% subplot(3,3,6)
% plot(out.STATES(:,1), out.STATES(:,13), 'r')
% title('r0_23');
% 
% subplot(3,3,7)
% plot(out.STATES(:,1), out.STATES(:,14), 'r')
% title('r0_31');
% 
% subplot(3,3,8)
% plot(out.STATES(:,1), out.STATES(:,15), 'r')
% title('r0_32');
% 
% subplot(3,3,9)
% plot(out.STATES(:,1), out.STATES(:,16), 'r')
% title('r0_33');


% figure(4);
% subplot(3,1,1)
% plot(out.STATES(:,1), out.STATES(:,17), 'r')
% title('\omega 0_x');
% 
% subplot(3,1,2)
% plot(out.STATES(:,1), out.STATES(:,18), 'r')
% title('\omega 0_y');
% 
% subplot(3,1,3)
% plot(out.STATES(:,1), out.STATES(:,19), 'r')
% title('\omega 0_z');


figure(5);  
plot(out.STATES(:,1), out.STATES(:,8), 'r')
title('L');

figure(6);  
subplot(3,1,1)
plot(out.STATES(:,1), out.CONTROLS(:,2), 'r')
title('f_{0,x}');

subplot(3,1,2)
plot(out.STATES(:,1), out.CONTROLS(:,3), 'r')
title('f_{0,y}');

subplot(3,1,3)
plot(out.STATES(:,1), out.CONTROLS(:,4), 'r')
title('f_{0,z}');


title('f_0');

% figure(7);  
% subplot(3,1,1)
% plot(out.STATES(:,1), out.CONTROLS(:,5), 'r')
% title('\tau_{0,x}');
% 
% subplot(3,1,2)
% plot(out.STATES(:,1), out.CONTROLS(:,6), 'r')
% title('\tau_{0,y}');
% 
% subplot(3,1,3)
% plot(out.STATES(:,1), out.CONTROLS(:,7), 'r')
% title('\tau_{0,z}');
% 
% 
% title('\tau_0');

figure;
plot(out.STATES(:,1), 0.5.*out.STATES(:,2).*out.STATES(:,2), 'r')
title('Kinetic Engery');