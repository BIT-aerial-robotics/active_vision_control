 %This file should be run first, in order to obtain the mex file 

%input: 
%u: 5-by-1, including: 
%time: 1-by-1, the current time, 
%x_feedback: 4-by-1, the current feedback state of the system 

close all; clear variables;




BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'quadrotorload_input_adv'); 
    
     %sometimes, the name of the variables induces errors, I don't know why. 
    DifferentialState x0_x  x0_y  x0_z v0_x  v0_y  v0_z  r0_11 r0_12 r0_13 r0_21 r0_22 r0_23 r0_31 r0_32 r0_33 cbeta const_L;
    Control omega0_x omega0_y omega0_z T_net;   

    
    %constants:  
    
    %% input variables of the generated mex file: 
    starttime = acado.MexInput;   %start time   
	endtime = acado.MexInput;  %final time   
   
    
    %% Set default objects
    f = acado.DifferentialEquation();
    f.linkCFunction('quadrotor_dynamics_L.cpp', 'quadrotor_dynamics_L');
%     f.linkCFunction('dynamics_null.cpp', 'quadrotor_load_dynamics_L');
    
    %Set up optimal control problem, 
    %start at t0, control in 40 intervals to tf
    %parameters: 
 
 
    ocp = acado.OCP(starttime, endtime, 10);      
    ocp.minimizeMayerTerm(const_L);  % minimizeLagrange is not yet implemented for matlab ode calls!
                               % but you can define another differential
                               % state to get the same effect (L)
 

    ocp.subjectTo( f );
 

    ocp.subjectTo( 'AT_START', x0_x ==   acado.MexInput);
    ocp.subjectTo( 'AT_START', x0_y ==  acado.MexInput);
    ocp.subjectTo( 'AT_START', x0_z ==   acado.MexInput );
    ocp.subjectTo( 'AT_START', v0_x ==   acado.MexInput );
    ocp.subjectTo( 'AT_START', v0_y ==   acado.MexInput );
    ocp.subjectTo( 'AT_START', v0_z ==   acado.MexInput );
    ocp.subjectTo( 'AT_START', r0_11 ==   acado.MexInput  );
    ocp.subjectTo( 'AT_START', r0_12 ==   acado.MexInput);
    ocp.subjectTo( 'AT_START', r0_13  == acado.MexInput );
    ocp.subjectTo( 'AT_START', r0_21  == acado.MexInput );
    ocp.subjectTo( 'AT_START', r0_22  == acado.MexInput );
    ocp.subjectTo( 'AT_START', r0_23  == acado.MexInput );    
    ocp.subjectTo( 'AT_START', r0_31  == acado.MexInput );
    ocp.subjectTo( 'AT_START', r0_32  == acado.MexInput );
    ocp.subjectTo( 'AT_START', r0_33  == acado.MexInput );
    ocp.subjectTo( 'AT_START', cbeta  == acado.MexInput );
    ocp.subjectTo( 'AT_START', const_L  == acado.MexInput);
 
    % state constraints: 
%     ocp.subjectTo(  sqrt((x0_x - 10)^2  + (x0_y -8)^2 + ( x0_z + 5 )^2  )  -2  >= 0); 
%       ocp.subjectTo(  sqrt((x0_x - 1)^2  + (x0_y -2)^2 + ( x0_z -1 )^2  )  -1  >= 0); 
        
%             ocp.subjectTo(  sqrt((x0_x - 1)^2  + (x0_y -0.05)^2 + ( x0_z - 3)^2  )  -1  >= 0); 
%              ocp.subjectTo(  sqrt((x0_x - 1.6)^2  + (x0_y -0.08)^2 + ( x0_z - 8)^2  )  -2.5  >= 0); 
             
%              ocp.subjectTo(  sqrt((x0_x - 1)^2  + (x0_y -0.05)^2 + ( x0_z - 5)^2  )  -2.5  >= 0); 
%      ocp.subjectTo(  ((v0_x - 0)^2  + (v0_y -0.00)^2 + ( v0_z - 0)^2  )  -4 <= 0); 
%       ocp.subjectTo( -2.5 <= v0_x <= 2.5);
%       ocp.subjectTo( -2.5 <= v0_y <= 2.5);
%       ocp.subjectTo( -2.5 <= v0_z <= 2.5);
%       ocp.subjectTo( -0.5 <= psi_dot <= 0.5);

      ocp.subjectTo( -1.5 <= omega0_x <= 1.5);
      ocp.subjectTo( -1.5 <= omega0_y <= 1.5);
      ocp.subjectTo( -1.5 <= omega0_z <= 1.5);

    
% input constraints:        
%     ocp.subjectTo(  -10 <=  Fz <=  10);  %the input constraints should be set to appropriate range, here include gravity force, 
%     horif = 100;
%     ocp.subjectTo(    ( Fx^2  +  Fy^2  )  - horif<= 0); 
% perception constraints:    
    m_ctrl = 2.1;
    g_ctrl = 9.8;
    p_fx = 0;
    p_fy = 100;
    p_fz = 0; 
%     ocp.subjectTo( ((Fy^2*p_fx*cos(psi) + Fz^2*p_fx*cos(psi) - Fy^2*x0_x*cos(psi) - Fz^2*x0_x*cos(psi) + Fx^2*p_fy*sin(psi) + Fz^2*p_fy*sin(psi) - Fx^2*x0_y*sin(psi) - Fz^2*x0_y*sin(psi) - Fx*Fy*p_fy*cos(psi) - Fx*Fz*p_fz*cos(psi) + g_ctrl^2*m_ctrl^2*p_fx*cos(psi) + Fx*Fy*x0_y*cos(psi) + Fx*Fz*x0_z*cos(psi) - g_ctrl^2*m_ctrl^2*x0_x*cos(psi) - Fx*Fy*p_fx*sin(psi) - Fy*Fz*p_fz*sin(psi) + g_ctrl^2*m_ctrl^2*p_fy*sin(psi) + Fx*Fy*x0_x*sin(psi) + Fy*Fz*x0_z*sin(psi) - g_ctrl^2*m_ctrl^2*x0_y*sin(psi) + Fx*g_ctrl*m_ctrl*p_fz*cos(psi) - 2*Fz*g_ctrl*m_ctrl*p_fx*cos(psi) - Fx*g_ctrl*m_ctrl*x0_z*cos(psi) + 2*Fz*g_ctrl*m_ctrl*x0_x*cos(psi) + Fy*g_ctrl*m_ctrl*p_fz*sin(psi) - 2*Fz*g_ctrl*m_ctrl*p_fy*sin(psi) - Fy*g_ctrl*m_ctrl*x0_z*sin(psi) + 2*Fz*g_ctrl*m_ctrl*x0_y*sin(psi))/(((- Fx^2*cos(psi)^2 + Fy^2*cos(psi)^2 + Fx^2 + Fz^2 + g_ctrl^2*m_ctrl^2 - Fx*Fy*sin(2*psi) - 2*Fz*g_ctrl*m_ctrl)/(Fx^2 + Fy^2 + Fz^2 - 2*Fz*g_ctrl*m_ctrl + g_ctrl^2*m_ctrl^2))^(1/2)*(Fx^2 + Fy^2 + Fz^2 - 2*Fz*g_ctrl*m_ctrl + g_ctrl^2*m_ctrl^2)*(p_fx^2 - 2*p_fx*x0_x + p_fy^2 - 2*p_fy*x0_y + p_fz^2 - 2*p_fz*x0_z + x0_x^2 + x0_y^2 + x0_z^2)^(1/2))) - 0.8 >=0);
 

    algo = acado.OptimizationAlgorithm(ocp);
    
     
    % !!
%     algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!
    
    algo.set( 'KKT_TOLERANCE', 1e-4);
    algo.set( 'MAX_NUM_ITERATIONS', 200);
%     algo.set( 'PRINT_SCP_METHOD_PROFILE', 'YES' );


    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
% out = quadrotorload_input_RUN(0, 1);

out = quadrotorload_input_adv_RUN(0, 0.5, ...  %start time and final time 
    1, 0, 0,...
    0, 0, 0,... %feedback states of the linear velocity of the load
    1, 0, 0, 0, 1, 0,  0, 0, 1,...     %feedback states of rotation matrix 
    0,... %cosbeta
    0); 
    


% u_k = out.CONTROLS(1,2:end)'; 

draw_oponly;

% end
