#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <cmath> 
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */


USING_NAMESPACE_ACADO;
using namespace std; 
void quadrotor_load_dynamics_L( double *x, double *f,  void  *user_data ){
    	double t1_x, t1_y, t1_z;   //the input 
	double t2_x, t2_y, t2_z;   //the input 
	double t3_x, t3_y, t3_z;   //the input 
	double t4_x, t4_y, t4_z;   //the input 
	double t5_x, t5_y, t5_z;   //the input 
	double t6_x, t6_y, t6_z;   //the input 
	
	double x0_x, x0_y, x0_z; // the position of the load
	double v0_x, v0_y, v0_z;  //the velocity of the load
	double r0_11,r0_12,r0_13,r0_21,r0_22;
	double r0_23,r0_31,r0_32,r0_33; //the rotation matrix of the load
	double omega0_x,omega0_y,omega0_z;//the angular velocity of the load 
	double q1_x,q1_y,q1_z; //the configuration of the link
	double q2_x,q2_y,q2_z;//the configuration of the link
	double q3_x,q3_y,q3_z; //the configuration of the link
	double q4_x,q4_y,q4_z; //the configuration of the link
	double q5_x,q5_y,q5_z; //the configuration of the link
	double q6_x,q6_y,q6_z; //the configuration of the link
	double omega1_x,omega1_y, omega1_z;// the angular velocity of the links
	double omega2_x,omega2_y, omega2_z;// the angular velocity of the links
	double omega3_x,omega3_y, omega3_z;// the angular velocity of the links
	double omega4_x,omega4_y, omega4_z;// the angular velocity of the links
	double omega5_x,omega5_y, omega5_z;// the angular velocity of the links//parameters
	double omega6_x,omega6_y, omega6_z;// the angular velocity of the links

	double I0_x = 1,I0_y = 1,I0_z = 1;//the,inertia,paramters,of,the,load//parameters//parameters
	double m0 = 1;//the,mass,of,the,load,//parameters
	double I1_x =1,I1_y = 1,I1_z =1;//parameters
	double m1=1;//parameters
	double I2_x =1,I2_y =1,I2_z=1;//parameters
	double m2 =1;//parameters
	double I3_x = 1, I3_y = 1, I3_z = 1;//parameters
	double m3 = 1;//parametersb
	double I4_x = 1, I4_y = 1, I4_z = 1;//parameters
	double m4 = 1;//parameters
	double I5_x =1, I5_y = 1,I5_z = 1;//parameters
	double m5 = 1;//parameters
	double I6_x =1, I6_y = 1, I6_z = 1;//parameters
	double m6 = 1;//parameters
	double rou1_x = 1,rou1_y = 1,rou1_z = 1;//parameters
	double rou2_x = 1,rou2_y = 1,rou2_z = 1;//parameters
	double rou3_x = 1,rou3_y = 1,rou3_z = 1; //parameters
	double rou4_x = 1,rou4_y = 1,rou4_z = 1;//parameters
	double rou5_x = 1,rou5_y = 1,rou5_z = 1;//parameters
	double rou6_x = 1,rou6_y = 1,rou6_z = 1;//parameters
	double l1 = 1,l2 = 1,l3 = 1,l4 = 1,l5 = 1,l6 = 1;//the,length,of,the,links,//parameters

	 rou1_x  = 3;  rou1_y  = 1;  rou1_z  = 1;  
	 rou2_x  = -3;  rou2_y  = 1;  rou2_z  = 1;  
	 rou3_x  = 1;  rou3_y  = -1;  rou3_z = 1;  
	 rou4_x  = 1;  rou4_y   = 1; rou4_z  = 1;  
	 rou5_x  = 5;  rou5_y  = 1; rou5_z = 1;  
	 rou6_x = 6;  rou6_y = 1;  rou6_z = 1;  

	int n_quadrotor = 6;//the number of the quadrotors, in this case, we consider 6 quadrotors 
		//,x[0],->,time,t
    // x[1] -> px
    // x[2] -> py
    // x[3] -> v
    // x[4] -> psi
    // x[5] -> L
    // x[6] -> u(1), a
    // x[7] -> u(2), psi_dot
    //in order to allocate the state and input varibles to the symbols, which
//are used in the symbol calculation 
	//double q_i_matrix[3][n_quadrotor];
	//double omega_i_matrix[3][n_quadrotor];
	//R_quadrotor_i_matrixzeros(9,n_quadrotor);
	//double omega_quadrotor_i_matrix[3][n_quadrotor];
	//double R_quadrotor_i_matrix[3][3][n_quadrotor];
	
	double t = x[0];
	x0_x= x[1]; x0_y= x[2]; x0_z = x[3]; // the position of the load
	v0_x = x[4]; v0_y = x[5]; v0_z = x[6];   //the velocity of the load
	r0_11 = x[7]; r0_12 = x[8]; r0_13 = x[9]; r0_21 = x[10]; r0_22 = x[11];
	r0_23 = x[12]; r0_31 = x[13]; r0_32 = x[14]; r0_33 = x[15];  //the rotation matrix of the load
	omega0_x = x[16]; omega0_y = x[17]; omega0_z = x[18];   //the angular velocity of the load 
	q1_x = x[19]; q1_y = x[20]; q1_z = x[21];    //the configuration of the link
	q2_x = x[22]; q2_y = x[23]; q2_z = x[24];     //the configuration of the link
	q3_x = x[25]; q3_y = x[26]; q3_z = x[27];    //the configuration of the link
	q4_x = x[28]; q4_y = x[29]; q4_z = x[30];    //the configuration of the link
	q5_x = x[31]; q5_y = x[32]; q5_z = x[33];    //the configuration of the link
	q6_x = x[34]; q6_y = x[35]; q6_z = x[36];      //the configuration of the link
	omega1_x = x[37]; omega1_y = x[38];  omega1_z = x[39];     // the angular velocity of the links
	omega2_x = x[40]; omega2_y = x[41];  omega2_z = x[42];     // the angular velocity of the links
	omega3_x = x[43]; omega3_y = x[44];  omega3_z = x[45];     // the angular velocity of the links
	omega4_x = x[46]; omega4_y = x[47];  omega4_z = x[48];     // the angular velocity of the links
	omega5_x = x[49]; omega5_y = x[50];  omega5_z = x[51];     // the angular velocity of the links
	omega6_x = x[52]; omega6_y = x[53];  omega6_z = x[54];     // the angular velocity of the links

	double const_L = x[55]; //the cost function of the optimal control

	t1_x = x[56]; t1_y = x[57]; t1_z = x[58];  //the input of the system
	t2_x = x[59]; t2_y =x[60];  t2_z= x[61];  
	t3_x = x[62]; t3_y = x[63]; t3_z = x[64];  
	t4_x = x[65]; t4_y = x[66];  t4_z = x[67];  
	t5_x = x[68]; t5_y = x[69]; t5_z = x[70]; 
	t6_x = x[71]; t6_y = x[72]; t6_z = x[73];  
    
	for (int bb = 0; bb < 55; bb++){
		f[bb] = 0;
	
	}
}
