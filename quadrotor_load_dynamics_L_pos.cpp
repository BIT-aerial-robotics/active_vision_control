#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <cmath> 

using namespace std; 
void quadrotor_load_dynamics_L_pos( double *x, double *f,  void  *user_data ){

//the position, velocity, and input force are all expressed in earth frame
    
    double f_x, f_y, f_z, tau_x, tau_y, tau_z; 
	
	double x0_x, x0_y, x0_z; // the position of the load
	double v0_x, v0_y, v0_z;  //the velocity of the load
	double r0_11,r0_12,r0_13,r0_21,r0_22;
	double r0_23,r0_31,r0_32,r0_33; //the rotation matrix of the load
 	 
	double t = x[0];
	x0_x= x[1]; x0_y= x[2]; x0_z = x[3]; // the position of the load
	v0_x = x[4]; v0_y = x[5]; v0_z = x[6];   //the velocity of the load
	Eigen::Vector3d vL; 
	vL << v0_x, v0_y, v0_z; 	

	double const_L = x[7]; //the cost function of the optimal control

	f_x = x[8]; f_y = x[9]; f_z = x[10]; 
	//tau_x = x[23]; tau_y = x[24]; tau_z = x[25]; //the input of the system
    
    double mass_sum = 2.1;
    Eigen::Vector3d u_vector1;
    u_vector1 << f_x, f_y, f_z;
	Eigen::Vector3d  vdot = u_vector1  /mass_sum; //earth  frame
 	
 
    double omega0_d[3];
    double x0_d[3];
    double v0_d[3]; 
    double a0_d[3]; 

	double p_err = x0_x*x0_x + x0_y*x0_y + x0_z*x0_z; 
    double v_err = v0_x*v0_x + v0_y*v0_y + v0_z*v0_z; 
    double delta_x = 5*p_err + 5*v_err; //the coefficient of the volocity can also be adjusted. 	
 
    Eigen::Vector3d u_gen_trans;
    u_gen_trans << 0, 0, 0;   //The force is expressed in earth frame, and does not include the gravity force. 
     
	double delta_u =  (f_x - u_gen_trans(0) )* (f_x - u_gen_trans(0) )+ (f_y - u_gen_trans(1))* (f_y - u_gen_trans(1))  + (f_z - u_gen_trans(2))* (f_z - u_gen_trans(2)); 

    f[0] = v0_x; 
	f[1] = v0_y;  
	f[2] = v0_z; 
	f[3] = vdot(0); 
	f[4] = vdot(1);
	f[5] = vdot(2);

    f[6] = 3*delta_x + 0.01*delta_u;   
    f[6] = 10*delta_x + 0.05*delta_u;  
    f[6] = 3*delta_x + 0.1*delta_u;  
    
        f[6] = 2*delta_x + 0.1*delta_u;  

    //cout << "x0: " << x[0] << endl; 
}


