<<<<<<< HEAD

=======
>>>>>>> d25f2258be33da1ce168f4cffbbbef5a5a0df300
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <cmath> 

using namespace std; 
void quadrotor_load_dynamics_L_att( double *x, double *f,  void  *user_data ){

//	double t1_x, t1_y, t1_z;   //the input //
//	double t2_x, t2_y, t2_z;   //the input //
//	double t3_x, t3_y, t3_z;   //the input //
//	double t4_x, t4_y, t4_z;   //the input //
//	double t5_x, t5_y, t5_z;   //the input //
//	double t6_x, t6_y, t6_z;   //the input //
    
    double f_x, f_y, f_z, tau_x, tau_y, tau_z; 
	
	double x0_x, x0_y, x0_z; // the position of the load
	double v0_x, v0_y, v0_z;  //the velocity of the load
	double r0_11,r0_12,r0_13,r0_21,r0_22;
	double r0_23,r0_31,r0_32,r0_33; //the rotation matrix of the load
	double omega0_x,omega0_y,omega0_z;//the angular velocity of the load  

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

 rou1_x  = 1;  rou1_y  = 0;  rou1_z  = 0;  
 rou2_x  = 1;  rou2_y  = -1;  rou2_z  = 0;  
 rou3_x  = -1;  rou3_y  = -1;  rou3_z = 0;  
 rou4_x  = -1;  rou4_y   = 0; rou4_z  = 0;  
 rou5_x  = -1;  rou5_y  = 1; rou5_z = 0;  
 rou6_x = 1;  rou6_y = 1;  rou6_z = 0;

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
	Eigen::Vector3d omegaL, vL; 
	vL << v0_x, v0_y, v0_z; 
	omegaL << omega0_x, omega0_y, omega0_z; 	

	double const_L = x[19]; //the cost function of the optimal control

	f_x = x[20]; f_y = x[21]; f_z = x[22]; 
	tau_x = x[23]; tau_y = x[24]; tau_z = x[25]; //the input of the system
    
	Eigen::Matrix3d R0, R0_dot; 
	R0 <<  r0_11, r0_12, r0_13, 
	    r0_21, r0_22, r0_23, 
	    r0_31, r0_32, r0_33;
	//kinematics of the load: 
	Eigen::Matrix3d hat_omega0; 
	hat_omega0 << 0,  - omega0_z,  omega0_y, 
	 omega0_z, 0, -omega0_x, 
	-omega0_y,  omega0_x, 0;  

	R0_dot = R0*hat_omega0; 

	//cout << "100" << endl; 
    
    //the position of the COM of the entire system, expressed in the body frame of the load: 
    Eigen::VectorXd m_vector(7); 
    m_vector << m0, m1, m2, m3, m4, m5, m6;
    double mass_sum = m0+m1+m2+m3+m4+m5+m6;
    Eigen::Matrix3d I_0; 
    I_0 << I0_x, 0, 0,
            0, I0_y, 0,
            0, 0, I0_z;
	Eigen::MatrixXd rou_i_t(7,3);
	//cout << "111" << endl; 
   rou_i_t << 0, 0, 0, 
		 rou1_x, rou1_y, rou1_z, 
		 rou2_x, rou2_y, rou2_z, 
		 rou3_x, rou3_y, rou3_z, 
		 rou4_x, rou4_y, rou4_z, 
		 rou5_x, rou5_y, rou5_z, 
		 rou6_x, rou6_y, rou6_z;

	//cout << "119" << endl; 
	Eigen::MatrixXd rou_i(3,7);
    rou_i = rou_i_t.transpose();

	Eigen::Vector3d sum_pos_mass;
    sum_pos_mass =  rou_i*m_vector; 
	Eigen::Vector3d x_c_0 = sum_pos_mass/mass_sum; 

	//cout << "125" << endl; 

	Eigen::Matrix3d I_sum;
	/*I_sum.Zero(); 
	for (int i = 0; i < n_quadrotor; i++){
		Eigen::Vector3d rou_i_vector; 
		Eigen::Vector3d rou_i_vectorsubc; 
		rou_i_vector = rou_i.block(0, i+1, 3, 1); 
		rou_i_vectorsubc = rou_i_vector - x_c_0; 
		Eigen::Matrix3d hat_rou_i_vector, hat_rou_i_vectorsubc; 
		hat_rou_i_vector <<  0,  - rou_i_vector(2),  rou_i_vector(1), 
				 rou_i_vector(2), 0, -rou_i_vector(0), 
				-rou_i_vector(1),  rou_i_vector(0), 0;  

		hat_rou_i_vectorsubc <<  0,  - rou_i_vectorsubc(2),  rou_i_vectorsubc(1), 
				 rou_i_vectorsubc(2), 0, -rou_i_vectorsubc(0), 
				-rou_i_vectorsubc(1),  rou_i_vectorsubc(0), 0;  
		I_sum = m_vector(i+1)* hat_rou_i_vector * hat_rou_i_vectorsubc + I_sum; 
	}
//cout << "147" << endl; 
	I_sum  = I_0 - I_sum;  //the summary inertia matrix*/
    
    /*
  I_sum(0,0) = I0_x+m1*rou1_y*(rou1_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m2*rou2_y*(rou2_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m3*rou3_y*(rou3_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m1*rou1_z*(rou1_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m4*rou4_y*(rou4_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m2*rou2_z*(rou2_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m5*rou5_y*(rou5_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m3*rou3_z*(rou3_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m6*rou6_y*(rou6_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m4*rou4_z*(rou4_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m5*rou5_z*(rou5_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m6*rou6_z*(rou6_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(0,1) = -m1*rou1_y*(rou1_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m2*rou2_y*(rou2_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m3*rou3_y*(rou3_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m4*rou4_y*(rou4_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m5*rou5_y*(rou5_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m6*rou6_y*(rou6_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(0,2) = -m1*rou1_z*(rou1_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m2*rou2_z*(rou2_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m3*rou3_z*(rou3_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m4*rou4_z*(rou4_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m5*rou5_z*(rou5_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))-m6*rou6_z*(rou6_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(1,0) = -m1*rou1_x*(rou1_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m2*rou2_x*(rou2_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m3*rou3_x*(rou3_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m4*rou4_x*(rou4_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m5*rou5_x*(rou5_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m6*rou6_x*(rou6_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(1,1) = I0_y+m1*rou1_x*(rou1_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m2*rou2_x*(rou2_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m3*rou3_x*(rou3_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m4*rou4_x*(rou4_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m5*rou5_x*(rou5_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m6*rou6_x*(rou6_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m1*rou1_z*(rou1_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m2*rou2_z*(rou2_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m3*rou3_z*(rou3_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m4*rou4_z*(rou4_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m5*rou5_z*(rou5_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))+m6*rou6_z*(rou6_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(1,2) = -m1*rou1_z*(rou1_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m2*rou2_z*(rou2_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m3*rou3_z*(rou3_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m4*rou4_z*(rou4_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m5*rou5_z*(rou5_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))-m6*rou6_z*(rou6_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(2,0) = -m1*rou1_x*(rou1_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m2*rou2_x*(rou2_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m3*rou3_x*(rou3_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m4*rou4_x*(rou4_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m5*rou5_x*(rou5_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m6*rou6_x*(rou6_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(2,1) = -m1*rou1_y*(rou1_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m2*rou2_y*(rou2_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m3*rou3_y*(rou3_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m4*rou4_y*(rou4_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m5*rou5_y*(rou5_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6))-m6*rou6_y*(rou6_z-(m1*rou1_z+m2*rou2_z+m3*rou3_z+m4*rou4_z+m5*rou5_z+m6*rou6_z)/(m0+m1+m2+m3+m4+m5+m6));
  I_sum(2,2) = I0_z+m1*rou1_x*(rou1_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m2*rou2_x*(rou2_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m3*rou3_x*(rou3_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m1*rou1_y*(rou1_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m4*rou4_x*(rou4_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m2*rou2_y*(rou2_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m5*rou5_x*(rou5_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m3*rou3_y*(rou3_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m6*rou6_x*(rou6_x-(m1*rou1_x+m2*rou2_x+m3*rou3_x+m4*rou4_x+m5*rou5_x+m6*rou6_x)/(m0+m1+m2+m3+m4+m5+m6))+m4*rou4_y*(rou4_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m5*rou5_y*(rou5_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6))+m6*rou6_y*(rou6_y-(m1*rou1_y+m2*rou2_y+m3*rou3_y+m4*rou4_y+m5*rou5_y+m6*rou6_y)/(m0+m1+m2+m3+m4+m5+m6));
*/
  
//  I_sum << 5, 0, 0,
//          0, 7, 0, 
//          0, 0, 11; 
 
  I_sum << 2.263, 0, 0,
           0, 2.4705, 0, 
           0, 0, 4.7235;    
    
	double acc_g = 9.8;
	Eigen::Matrix3d m_matrix1, m_matrix2, C_matrix1, C_matrix2;
	Eigen::Vector3d G_vector1, u_vector1, u_vector2;
	u_vector2 << 0, 0, 0;
	m_matrix2 = I_sum; 
	Eigen::Matrix3d hat_omegaL, hat_j_omegaL;
	Eigen::Vector3d j_omegaL = I_sum * omegaL; 
//cout << "147" << endl; 
	hat_omegaL <<  0,  - omegaL(2),  omegaL(1), 
				 omegaL(2), 0, -omegaL(0), 
				-omegaL(1),  omegaL(0), 0;  
	hat_j_omegaL <<  0,  - j_omegaL(2),  j_omegaL(1), 
				 j_omegaL(2), 0, -j_omegaL(0), 
				-j_omegaL(1),  j_omegaL(0), 0;  
	C_matrix1 = mass_sum*hat_omegaL; 
	C_matrix2 = -hat_j_omegaL; 
	Eigen::Vector3d e3; 
	e3 << 0, 0, 1; 
//cout << "147" << endl; 
	G_vector1 = -mass_sum*acc_g*R0.transpose()*e3;
  //cout << "R0" << R0  << endl; 
	
	u_vector1 << f_x, f_y, f_z; 

//cout << "170" << endl; 
	/*for (int i = 0; i < n_quadrotor; i++){
		
		Eigen::Vector3d rou_i_vector; 
		rou_i_vector = rou_i.block(0, i+1, 3, 1); 
		Eigen::Matrix3d hat_rou_i_vector;
		hat_rou_i_vector <<  0,  - rou_i_vector(2),  rou_i_vector(1), 
				 rou_i_vector(2), 0, -rou_i_vector(0), 
				-rou_i_vector(1),  rou_i_vector(0), 0;  
		Eigen::Vector3d thrusti; 
		thrusti << x[3*i+20], x[3*i+21], x[3*i+22]; 
		u_vector2 = hat_rou_i_vector * thrusti + u_vector2;
	}*/

  //input_equi[0][0] = -t1_x-t2_x-t3_x-t4_x-t5_x-t6_x;
  //input_equi[1][0] = -t1_y-t2_y-t3_y-t4_y-t5_y-t6_y;
  //input_equi[2][0] = -t1_z-t2_z-t3_z-t4_z-t5_z-t6_z;
  u_vector2 << tau_x, tau_y, tau_z; 
  //test only, if the input is directly the force and torque: 
  	/*u_vector1 << -t1_x , 
		- t1_y  , 
		 -t1_z ; 
    	u_vector2 <<  -t2_x , 
		   -t2_y  , 
		 -t2_z  ; */

	Eigen::Vector3d vdot, omegadot; 
	vdot = (u_vector1 - G_vector1 - C_matrix1*vL)/mass_sum; //body-fix frame
	omegadot = I_sum.inverse()*(u_vector2 - C_matrix2*omegaL); 
	
    //cout <<   G_vector1(0) << "," << G_vector1(1) << "," << G_vector1(2)  << "," << endl; 
    Eigen::Vector3d vdot_earth;
    vdot_earth = R0*vdot;

    double R_err; 
    
    //reference signals of the system: 
    Eigen::Matrix3d R0d; //the desired rotation matrix
    R0d << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
   
    R0d <<    0.8776,    0.3069,   -0.3683,
   -0.3069 ,   0.9498,    0.0602,
    0.3683,    0.0602,    0.9278;
    
    R0d << 0.9388,    0.0612,   -0.3390,
    0.0612,    0.9388,    0.3390,
    0.3390,   -0.3390,    0.8776;
    
    
    R0d << 0.9127,    0.0873,   -0.3993,
    0.0873,    0.9127,    0.3993,
    0.3993,   -0.3993,    0.8253;
    
    
    Eigen::Matrix3d deltaR; 
    deltaR = R0d.transpose()*R0; 
    R_err = 0.5* ((1-deltaR(0,0))+ (1-deltaR(1,1)) + (1-deltaR(2,2)));
    
    double omega0_d[3];
    double x0_d[3];
    double v0_d[3]; 
    double a0_d[3]; 

    omega0_d[0] = 0; omega0_d[1] = 0; omega0_d[2] = 0; 
    x0_d[0] = 10* t; x0_d[1] = 0; x0_d[2] = 0; 
    v0_d[0] = 10; v0_d[1] = 0; v0_d[2] = 0; 

	//calculate the minimum snap trajectory for the position 
	double t0 = 0;  //start time 
	double alpha = 20;  //duration time 

	Eigen::VectorXd coe_x_pos(10), coe_x_vel(10), coe_x_acc(10),
	coe_y_pos(10), coe_y_vel(10), coe_y_acc(10),
	coe_z_pos(10), coe_z_vel(10), coe_z_acc(10); 

	coe_x_pos << 0,0,0,0,1.536999999997807e+03,-5.531999999986896e+03,7.681999999969546e+03,-4.827999999965346e+03,1.151999999980595e+03,4.292660782390101e-09;
	coe_x_vel << 0,0,0,6.147999999991229e+03,-2.765999999993448e+04,4.609199999981727e+04,-3.379599999975742e+04,9.215999999844762e+03,3.863394704151091e-08,0;
	coe_x_acc << 0,0,1.844399999997369e+04,-1.106399999997379e+05,2.304599999990864e+05,-2.027759999985445e+05,6.451199999891334e+04,3.090715763320873e-07,0,0;

	coe_y_pos << 1,0,0,0,1.605999999997880e+03,-6.311999999987353e+03,9.355999999970678e+03,-6.183999999966713e+03,1.535999999981404e+03,4.104722561848462e-09;
	coe_y_vel << 0,0,0,6.423999999991518e+03,-3.155999999993677e+04,5.613599999982407e+04,-4.328799999976700e+04,1.228799999985123e+04,3.694250305663616e-08,0;
	coe_y_acc << 0,0,1.927199999997456e+04,-1.262399999997471e+05,2.806799999991204e+05,-2.597279999986020e+05,8.601599999895860e+04,2.955400244530893e-07,0,0;

	coe_z_pos << 2,0,0,0,-1.512999999997308e+03,5.883999999983917e+03,-8.657999999962642e+03,5.691999999957507e+03,-1.407999999976212e+03,-5.260956115193949e-09;
	coe_z_vel << 0,0,0,-6.051999999989232e+03,2.941999999991958e+04,-5.194799999977585e+04,3.984399999970254e+04,-1.126399999980970e+04,-4.734860503674554e-08,0;
	coe_z_acc << 0,0,-1.815599999996770e+04,1.176799999996783e+05,-2.597399999988792e+05,2.390639999982153e+05,-7.884799999866789e+04,-3.787888402939643e-07,0,0;

	double time_current = (t-t0)/alpha;
	Eigen::VectorXd t_vector(10);

	t_vector[0] = 1.0;
	t_vector[1] = time_current;
	t_vector[2] = time_current*time_current;
	t_vector[3] = time_current*time_current*time_current;
	t_vector[4] = time_current*time_current*time_current*time_current;
	t_vector[5] = time_current*time_current*time_current*time_current*time_current;
	t_vector[6] = time_current*time_current*time_current*time_current*time_current*time_current;
	t_vector[7] = time_current*time_current*time_current*time_current*time_current*time_current*time_current;
	t_vector[8] = time_current*time_current*time_current*time_current*time_current*time_current*time_current*time_current;
	t_vector[9] = time_current*time_current*time_current*time_current*time_current*time_current*time_current*time_current*time_current;
	x0_d[0] = coe_x_pos.transpose()*t_vector; 
    x0_d[1] = coe_y_pos.transpose()*t_vector;
    x0_d[2] = coe_z_pos.transpose()*t_vector;
	v0_d[0] = coe_x_vel.transpose()*t_vector; 
    v0_d[1] = coe_y_vel.transpose()*t_vector;
    v0_d[2] = coe_z_vel.transpose()*t_vector;
	a0_d[0] = coe_x_acc.transpose()*t_vector; 
    a0_d[1] = coe_y_acc.transpose()*t_vector;
    a0_d[2] = coe_z_acc.transpose()*t_vector;
	v0_d[0] = v0_d[0]/alpha; 
    v0_d[1] = v0_d[1]/alpha;
    v0_d[2] = v0_d[2]/alpha;
	a0_d[0] = a0_d[0]/alpha/alpha; 
    a0_d[1] = a0_d[1]/alpha/alpha;
    a0_d[2] = a0_d[2]/alpha/alpha;

	Eigen::Vector3d v_ref_earth, v_ref_dot_earth;  //the reference velocity and acc, expressed in earth frame
	v_ref_earth << v0_d[0], v0_d[1], v0_d[2]; 
	v_ref_dot_earth << a0_d[0], a0_d[1], a0_d[2]; 

	//added on 2019-09-09, the reference input is added:
	Eigen::Matrix3d hat_omegaL_ref, hat_j_omegaL_ref;
	Eigen::Vector3d omegaL_ref, omegaL_ref_dot;
	Eigen::Matrix3d RL_ref, RL_ref_dot;
	RL_ref << 1, 0, 0, 
		      0, 1, 0, 
			  0, 0, 1;
	RL_ref_dot << 0, 0, 0, 
			     0, 0, 0, 
				 0, 0, 0; 
	omegaL_ref << 0, 0, 0; //the reference angular velocity of the load 
	omegaL_ref_dot << 0, 0, 0; //the reference derivative of the angular velocity of the load
	Eigen::Vector3d j_omegaL_ref = I_sum * omegaL_ref; 
//cout << "147" << endl; 
	hat_omegaL_ref <<  0,  - omegaL_ref(2),  omegaL_ref(1), 
				 omegaL_ref(2), 0, -omegaL_ref(0), 
				-omegaL_ref(1),  omegaL_ref(0), 0;  
	hat_j_omegaL_ref <<  0,  - j_omegaL_ref(2),  j_omegaL_ref(1), 
				 j_omegaL_ref(2), 0, -j_omegaL_ref(0), 
				-j_omegaL_ref(1),  j_omegaL_ref(0), 0;  
	Eigen::Vector3d G_vector1_ref;
	Eigen::Matrix3d C_matrix1_ref, C_matrix2_ref; 
	G_vector1_ref << 0, 0, -mass_sum*acc_g; //the reference gravity matrix, note this should be modified according to reference attitude 
	C_matrix1_ref = mass_sum*hat_omegaL_ref; 
	C_matrix2_ref = -hat_j_omegaL_ref; 

	Eigen::Vector3d u_gen_trans;
	Eigen::Vector3d u_gen_rot;
    Eigen::Vector3d v_ref, v_ref_dot;
	v_ref = RL_ref.transpose()*v_ref_earth; //transform from the earth frame to the body-fixed frame 
	v_ref_dot = RL_ref.transpose()*v_ref_dot_earth;//transform from the earth frame to the body-fixed frame  
	u_gen_trans = mass_sum*v_ref_dot + C_matrix1_ref*v_ref + G_vector1_ref;
	u_gen_rot = I_sum*omegaL_ref_dot + C_matrix2_ref*omegaL_ref; 

    //cout <<  G_vector1_ref  << endl; 
	/*vel = [coe_x_vel'*t_vector;  coe_y_vel'*t_vector; coe_z_vel'*t_vector];
	acc = [coe_x_acc'*t_vector;  coe_y_acc'*t_vector; coe_z_acc'*t_vector];
	vel = [coe_x_vel'*t_vector;  coe_y_vel'*t_vector; coe_z_vel'*t_vector];
	acc = [coe_x_acc'*t_vector;  coe_y_acc'*t_vector; coe_z_acc'*t_vector];
	vel = vel/alpha;
	acc = acc/alpha/alpha;
	out = [pos; vel; acc];*/
   
    omega0_d[0]= 0;
    omega0_d[1]= 0;
    omega0_d[2]= 0;
    x0_d[0] = 0;
    x0_d[1] = 0;
    x0_d[2] = 0;
    v0_d[0] = 0; 
    v0_d[1] = 0; 
    v0_d[2] = 0; 
    double omega_err = ( omega0_x - omega0_d[0] )*( omega0_x - omega0_d[0] ) + ( omega0_y - omega0_d[1] )* ( omega0_y - omega0_d[1] ) +  ( omega0_z - omega0_d[2] )* ( omega0_z - omega0_d[2] );  
    double x0_err = ( x0_x - x0_d[0] )*( x0_x - x0_d[0] ) + ( x0_y - x0_d[1] )* ( x0_y - x0_d[1] ) +  1* ( x0_z - x0_d[2] )* ( x0_z - x0_d[2] );  //notice the gain, should be appropriately tuned
    double v0_err = ( v0_x - v0_d[0] )*( v0_x - v0_d[0] ) + ( v0_y - v0_d[1] )* ( v0_y - v0_d[1] ) +  ( v0_z - v0_d[2] )* ( v0_z - v0_d[2] );  
    
	/*Eigen::MatrixXd B_pinv_1(9,6);
	Eigen::MatrixXd B_pinv_2(9,6);
	B_pinv_1 << 0.1667,0.0000,0,0,0,-0.0000, 
	0,0.1667,0,0,0,0.1000, 
	0,-0.0000,0.1667,0,-0.1667,0.0000, 
	0.1667,-0.0000,0,0,0,0.1000, 
	0,0.1667,0,0,0,0.1000, 
	0,-0.0000,0.1667,-0.2500,-0.1667,0.0000, 
	0.1667,-0.0000,0,0,0,0.1000, 
	0,0.1667,0,0,0,-0.1000, 
	0,-0.0000,0.1667,-0.2500,0.1667,0.0000; 
	B_pinv_2 << 0.1667,0.0000,0,0,0,-0.0000, 
	0,0.1667,0,0,0,-0.1000, 
	0,-0.0000,0.1667,0,0.1667,0.0000, 
	0.1667,0.0000,0,0,0,-0.1000, 
	0,0.1667,0,0,0,-0.1000, 
	0,0.0000,0.1667,0.2500,0.1667,-0.0000, 
	0.1667,0.0000,0,0,0,-0.1000, 
	0,0.1667,0,0,0,0.1000, 
	0,-0.0000,0.1667,0.2500,-0.1667,0.0000; */
	Eigen::MatrixXd B_pinv(18,6);
	B_pinv <<   0.1667, 0.0000, 0, 0, 0, -0.0000, 
	0, 0.1667, 0, 0, 0, 0.1000, 
	0, -0.0000, 0.1667, 0, -0.1667, 0.0000, 
	0.1667, -0.0000, 0, 0, 0, 0.1000, 
	0, 0.1667, 0, 0, 0, 0.1000, 
	0, -0.0000, 0.1667, -0.2500, -0.1667, 0.0000, 
	0.1667, -0.0000, 0, 0, 0, 0.1000, 
	0, 0.1667, 0, 0, 0, -0.1000, 
	0, -0.0000, 0.1667, -0.2500, 0.1667, 0.0000, 
	0.1667, 0.0000, 0, 0, 0, -0.0000, 
	0, 0.1667, 0, 0, 0, -0.1000, 
	0, -0.0000, 0.1667, 0, 0.1667, 0.0000, 
	0.1667, 0.0000, 0, 0, 0, -0.1000, 
	0, 0.1667, 0, 0, 0, -0.1000, 
	0, 0.0000, 0.1667, 0.2500, 0.1667, -0.0000, 
	0.1667, 0.0000, 0, 0, 0, -0.1000, 
	0, 0.1667, 0, 0, 0, 0.1000, 
	0, -0.0000, 0.1667, 0.2500, -0.1667, 0.0000;
	Eigen::VectorXd u_gen(6);
	u_gen << u_gen_trans(0), u_gen_trans(1), u_gen_trans(2), u_gen_rot(0),  u_gen_rot(1), u_gen_rot(2);
	Eigen::VectorXd input = -B_pinv*u_gen;
	 
//	double delta_x  = omega0_x*omega0_x + omega0_y*omega0_y + omega0_z*omega0_z +r0_11*r0_11+r0_12*r0_12+r0_13*r0_13+r0_21*r0_21+r0_22*r0_22+r0_23*r0_23+r0_31*r0_31+r0_32*r0_32+r0_33*r0_33+v0_x*v0_x+v0_y*v0_y+v0_z*v0_z+x0_x*x0_x+x0_y*x0_y+x0_z*x0_z;
	// double delta_x = 30*R_err + omega_err; //attitude only	
    double delta_x = 200*R_err + 10*omega_err; //attitude only	
//	double delta_u_1 = ( (t1_x - input(0))   )*( (t1_x - input(0))   ) + (t1_y - input(1))* (t1_y - input(1)) + (t1_z - input(2))* (t1_z - input(2));
//	double delta_u_2 = ( (t2_x - input(3))   )*( (t2_x - input(3))   ) + (t2_y - input(4))* (t2_y - input(4))+ (t2_z - input(5))* (t2_z - input(5));
//	double delta_u_3 = ( (t3_x - input(6))   )*( (t3_x - input(6))   ) + (t3_y - input(7))* (t3_y - input(7))+ (t3_z - input(8))* (t3_z - input(8));
//	double delta_u_4 = ( (t4_x - input(9))   )*( (t4_x - input(9))   ) + (t4_y - input(10))* (t4_y - input(10))+ (t4_z - input(11))* (t4_z - input(11));
//	double delta_u_5 = ( (t5_x - input(12))   )*( (t5_x - input(12))   ) + (t5_y - input(13))* (t5_y - input(13)) + (t5_z - input(14))* (t5_z - input(14));
//	double delta_u_6 = ( (t6_x - input(15))   )*( (t6_x - input(15))   ) + (t6_y - input(16))* (t6_y - input(16)) + (t6_z - input(17))* (t6_z - input(17));
//	double delta_u = delta_u_1 + delta_u_2 + delta_u_3 + delta_u_4 + delta_u_5 + delta_u_6; 	
    
    u_gen_trans << 0, 0, -68.6; 
    u_gen_rot << 0, 0, 0; 
    
	double delta_u =  (tau_x - u_gen_rot(0) )* (tau_x - u_gen_rot(0) )+ (tau_y - u_gen_rot(1))* (tau_y - u_gen_rot(1))  + (tau_z - u_gen_rot(2))* (tau_z - u_gen_rot(2)); 

    f[0] = vdot_earth(0); 
	f[1] = vdot_earth(1);  
	f[2] = vdot_earth(2); 
	f[3] = vdot(0); 
	f[4] = vdot(1);
	f[5] = vdot(2);

	f[6] = R0_dot(0,0);
	f[7] = R0_dot(0,1);
	f[8] = R0_dot(0,2);
	f[9] = R0_dot(1,0);
	f[10] = R0_dot(1,1);
	f[11] = R0_dot(1,2);
	f[12] = R0_dot(2,0);
	f[13] = R0_dot(2,1);
	f[14] = R0_dot(2,2);

	f[15] = omegadot(0); 
	f[16] = omegadot(1);
	f[17] = omegadot(2);

  
    f[18] = 3*delta_x + 0.0000001*delta_u;   
    
    f[18] = 1*delta_x + 0.1*delta_u;   
 
    f[18] = 1*delta_x + 10*delta_u;  
       f[18] = 2*delta_x + 0.1*delta_u; 

	//cout << "x0: " << x[0] << endl; 
}


