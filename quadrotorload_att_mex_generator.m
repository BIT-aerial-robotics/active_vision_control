 %This file should be run first, in order to obtain the mex file 

%input: 
%u: 5-by-1, including: 
%time: 1-by-1, the current time, 
%x_feedback: 4-by-1, the current feedback state of the system 

close all; clear variables;




BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'quadrotorload_input_att'); 
    
     %sometimes, the name of the variables induces errors, I don't know why. 
    DifferentialState x0_x  x0_y  x0_z    	v0_x  v0_y  v0_z   	r0_11  r0_12  r0_13  r0_21  r0_22  	r0_23  r0_31  r0_32  r0_33     	omega0_x  omega0_y  omega0_z const_L;
    Control f_x f_y f_z tau_x tau_y tau_z; 

    
    %constants:  
    
    %% input variables of the generated mex file: 
    starttime = acado.MexInput;   %start time   
	endtime = acado.MexInput;  %final time   
    
    %%feedback states: 
%	x0_x_feedback = acado.MexInput; 
%	x0_y_feedback = acado.MexInput; 
%	x0_z_feedback = acado.MexInput; 
%	v0_x_feedback = acado.MexInput; 
%	v0_y_feedback = acado.MexInput; 
%	v0_z_feedback = acado.MexInput; 
%	r0_11_feedback = acado.MexInput; 
%	r0_12_feedback = acado.MexInput; 
%	r0_13_feedback = acado.MexInput; 
%	r0_21_feedback = acado.MexInput; 
%	r0_22_feedback = acado.MexInput; 
%	r0_23_feedback = acado.MexInput; 
%	r0_31_feedback = acado.MexInput; 
%	r0_32_feedback = acado.MexInput; 
%	r0_33_feedback = acado.MexInput; 
%	omega0_x_feedback = acado.MexInput; 
%	omega0_y_feedback = acado.MexInput; 
%	omega0_z_feedback = acado.MexInput; 
%	q1_x_feedback = acado.MexInput; 
%	q1_y_feedback = acado.MexInput; 
%	q1_z_feedback = acado.MexInput; 
%	q2_x_feedback = acado.MexInput; 
%	q2_y_feedback = acado.MexInput; 
%	q2_z_feedback = acado.MexInput; 
%	q3_x_feedback = acado.MexInput; 
%	q3_y_feedback = acado.MexInput; 
%	q3_z_feedback = acado.MexInput; 
%	q4_x_feedback = acado.MexInput; 
%	q4_y_feedback = acado.MexInput; 
%	q4_z_feedback = acado.MexInput; 
%	q5_x_feedback = acado.MexInput; 
%	q5_y_feedback = acado.MexInput; 
%	q5_z_feedback = acado.MexInput; 
%	q6_x_feedback = acado.MexInput; 
%	q6_y_feedback = acado.MexInput; 
%	q6_z_feedback = acado.MexInput; 
%	omega1_x_feedback = acado.MexInput; 
%	omega1_y_feedback = acado.MexInput; 
%	omega1_z_feedback = acado.MexInput; 
%	omega2_x_feedback = acado.MexInput; 
%	omega2_y_feedback = acado.MexInput; 
%	omega2_z_feedback = acado.MexInput; 
%	omega3_x_feedback = acado.MexInput; 
%	omega3_y_feedback = acado.MexInput; 
%	omega3_z_feedback = acado.MexInput; 
%	omega4_x_feedback = acado.MexInput; 
%	omega4_y_feedback = acado.MexInput; 
%	omega4_z_feedback = acado.MexInput; 
%	omega5_x_feedback = acado.MexInput; 
%	omega5_y_feedback = acado.MexInput; 
%	omega5_z_feedback = acado.MexInput; 
%	omega6_x_feedback = acado.MexInput; 
%	omega6_y_feedback = acado.MexInput; 
%	omega6_z_feedback = acado.MexInput; 
%	const_L_feedback = acado.MexInput;

% 
%     %%maximum 5 obstacles, each has 2 constraints, then Au<b, A is 10-by-2,
%     %b is 2-by-1, totally 30 inputs 
%     %coe_cbf*u <= remaining
%     coe_cbfqp_a11 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a12 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b1 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a21 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a22 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b2 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a31 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a32 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b3 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a41 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a42 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b4 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a51 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a52 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b5 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a61 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a62 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b6 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a71 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a72 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b7 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a81 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a82 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b8 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a91 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a92 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b9 = acado.MexInput;  %coeffecients of the cbf constraints
%     
%     coe_cbfqp_a101 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_a102 = acado.MexInput;  %coeffecients of the cbf constraints
%     coe_cbfqp_b10 = acado.MexInput;  %coeffecients of the cbf constraints
    
    
    %% Set default objects
    f = acado.DifferentialEquation();
    f.linkCFunction('quadrotor_load_dynamics_L_att.cpp', 'quadrotor_load_dynamics_L_att');
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
    ocp.subjectTo( 'AT_START', omega0_x  == acado.MexInput );
    ocp.subjectTo( 'AT_START', omega0_y  == acado.MexInput );
    ocp.subjectTo( 'AT_START', omega0_z  == acado.MexInput );       
    ocp.subjectTo( 'AT_START', const_L  == acado.MexInput);
    
%     ocp.subjectTo( 'AT_END', x0_x*x0_x + x0_y*x0_y + x0_z*x0_z<=   0.01);
%     ocp.subjectTo( 'AT_START', x0_y ==  acado.MexInput);
%     ocp.subjectTo( 'AT_START', x0_z ==   acado.MexInput );
%     ocp.subjectTo( 'AT_START', v0_x ==   acado.MexInput );
%     ocp.subjectTo( 'AT_START', v0_y ==   acado.MexInput );
%     ocp.subjectTo( 'AT_START', v0_z ==   acado.MexInput );
%     
%     ocp.subjectTo( 'AT_START', r0_11 ==   acado.MexInput  );
%     ocp.subjectTo( 'AT_START', r0_12 ==   acado.MexInput);
%     ocp.subjectTo( 'AT_START', r0_13  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', r0_21  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', r0_22  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', r0_23  == acado.MexInput );    
%     ocp.subjectTo( 'AT_START', r0_31  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', r0_32  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', r0_33  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', omega0_x  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', omega0_y  == acado.MexInput );
%     ocp.subjectTo( 'AT_START', omega0_z  == acado.MexInput );       
%     ocp.subjectTo( 'AT_START', const_L  == acado.MexInput);
 

% state constraints: 
%     ocp.subjectTo(  sqrt((x0_x - 10)^2  + (x0_y -8)^2 + ( x0_z + 5 )^2  )  -2  >= 0); 
%       ocp.subjectTo(  sqrt((x0_x - 1)^2  + (x0_y -2)^2 + ( x0_z -1 )^2  )  -1  >= 0); 
        
%             ocp.subjectTo(  sqrt((x0_x - 10)^2  + (x0_y -0.5)^2 + ( x0_z - 0.5)^2  )  -1  >= 0); 

ocp.subjectTo(  r0_33 >= 0.8); 
ocp.subjectTo(  r0_33 <= 0.85); 
    
% input constraints: 
 
     ocp.subjectTo(    ( tau_x^2  +  tau_y^2 +  tau_z^2  )  -8<= 0);
  
     
    
%CBF constraints: 
%very important for this problem, because there may be singular 
%sometimes, this constant should be big enough, in order to let the solver
%works 

%% if do not use CBF to generate constraints, uncomment the following: 
%     global pos_ob_array_pre radius_pre;
%     global no_ob;
%     
%     
%   
%     global no_ob_local pos_ob_local radius_local; 
%         no_ob_local= no_ob; 
%     pos_ob_local = pos_ob_array_pre;  
%     radius_local = radius_pre; 
%     
%     
%     global flag_mode; 
%     if(flag_mode == 2)  %MPC only 
%         for i=1:no_ob_local
%             ocp.subjectTo(  sqrt((s - pos_ob_local(1,i))^2  + (ey - pos_ob_local(2,i))^2) ...
%                 - (radius_local(i) + 1.414/3) >= -0.0); 
%         end
%     elseif(flag_mode == 1)  %cbf and mpc
%         for i=1:no_ob
%             ocp.subjectTo(  sqrt((s - pos_ob_array_pre(1,i))^2  + (ey - pos_ob_array_pre(2,i))^2) ...
%                 - (radius_pre(i) + 0.0) >= 0); 
%         end
%     end
        
            
%     ocp.subjectTo(  sqrt((s - pos_ob_array_pre(1,1))^2  + (ey - pos_ob_array_pre(2,1))^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - pos_ob_array_pre(1,2))^2  + (ey - pos_ob_array_pre(2,2))^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - pos_ob_array_pre(1,3))^2  + (ey -pos_ob_array_pre(2,3))^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - pos_ob_array_pre(1,4))^2  + (ey - pos_ob_array_pre(2,4))^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - pos_ob_array_pre(1,5))^2  + (ey - pos_ob_array_pre(2,5))^2)  -1  >= 0); 
    
%     ocp.subjectTo(  sqrt((s - 50)^2  + (ey- 1.2)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - 60.8)^2  + (ey + 2.8)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - 60.8)^2  + (ey - 0.0)^2)  -1  >= 0); 
%     ocp.subjectTo(  sqrt((s - 60.8)^2  + (ey - 2.8)^2)  -1  >= 0);       
%     ocp.subjectTo(  -3.7 + 1.414/3 <= ey <= 3.7-1.414/3); 
 
 %% if use CBF generate constraints, uncomment the following: 
 
     
    
    %mpc and cbf
%      if(flag_mode == 1)
%          ocp.subjectTo(  coe_cbfqp_a11*delta_f + coe_cbfqp_a12*a_x -coe_cbfqp_b1  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a21*delta_f + coe_cbfqp_a22*a_x -coe_cbfqp_b2  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a31*delta_f + coe_cbfqp_a32*a_x -coe_cbfqp_b3  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a41*delta_f + coe_cbfqp_a42*a_x -coe_cbfqp_b4  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a51*delta_f + coe_cbfqp_a52*a_x -coe_cbfqp_b5  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a61*delta_f + coe_cbfqp_a62*a_x -coe_cbfqp_b6  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a71*delta_f + coe_cbfqp_a72*a_x -coe_cbfqp_b7  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a81*delta_f + coe_cbfqp_a82*a_x -coe_cbfqp_b8  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a91*delta_f + coe_cbfqp_a92*a_x -coe_cbfqp_b9  <= 0 ); 
%          ocp.subjectTo(  coe_cbfqp_a101*delta_f + coe_cbfqp_a102*a_x -coe_cbfqp_b10  <= 0 ); 
%      end

    algo = acado.OptimizationAlgorithm(ocp);
    
     
    % !!
%     algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!
    
    algo.set( 'KKT_TOLERANCE', 1e-4);
    algo.set( 'MAX_NUM_ITERATIONS', 20);
%     algo.set( 'PRINT_SCP_METHOD_PROFILE', 'YES' );


    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
% out = quadrotorload_input_RUN(0, 1);

out = quadrotorload_input_att_RUN(0, 0.5, ...  %start time and final time 
    0, 0, 0,...
    0, 0, 0,... %feedback states of the linear velocity of the load
      0.8746,   -0.2750,    0.3993, 0.4358 ,   0.8066 ,  -0.3993,    -0.2123,    0.5232,    0.8253,....%     0, 1, 0, -1, 0, 0,  0, 0, 1,...     %feedback states of rotation matrix of the load 
    0, 0, 0, ... %feedback states of the angular velocity of the load
    0); 
    


% u_k = out.CONTROLS(1,2:end)'; 

draw_oponly;

% end
