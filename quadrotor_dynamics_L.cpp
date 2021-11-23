#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <cmath> 
 

using namespace std; 
double cosbetadotsyms(const double u[21]);
void quadrotor_dynamics_L( double *x, double *f,  void  *user_data ){

//the position, velocity, and input force are all expressed in earth frame
    
    double f_x, f_y, f_z, tau_x, tau_y, tau_z; 
	
	double x0_x, x0_y, x0_z; // the position of the load
	double v0_x, v0_y, v0_z;  //the velocity of the load
	double r0_11,r0_12,r0_13,r0_21,r0_22;
	double r0_23,r0_31,r0_32,r0_33; //the rotation matrix of the load
	double omega0_x,omega0_y,omega0_z;//the angular velocity of the load  
 	 
	double t = x[0];
	x0_x= x[1]; x0_y= x[2]; x0_z = x[3]; // the position of the load
	v0_x = x[4]; v0_y = x[5]; v0_z = x[6];   //the velocity of the load
   	Eigen::Vector3d vL; 
	vL << v0_x, v0_y, v0_z; 
    
    Eigen::Vector3d omegaL;
	omegaL << omega0_x, omega0_y, omega0_z; 	
    
    r0_11 = x[7]; r0_12 = x[8]; r0_13 = x[9]; r0_21 = x[10]; r0_22 = x[11];
	r0_23 = x[12]; r0_31 = x[13]; r0_32 = x[14]; r0_33 = x[15];  //the rotation matrix of the load
    double cosbeta = x[16];  //cos beta
    double const_L = x[17]; //the cost function of the optimal control
    
	omega0_x = x[18]; omega0_y = x[19]; omega0_z = x[20];   //the angular velocity of the load, input
    double T_net_zero = x[21];  //net force - mg , input
 
    //f_x = x[9]; f_y = x[10]; f_z = x[11]; 
 	//tau_x = x[23]; tau_y = x[24]; tau_z = x[25]; //the input of the system
    
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
    
    Eigen::Vector3d e3;
    e3 << 0, 0, 1;
    
 
    double mass_sum = 2.1;
    double g_ = 9.8;
    
    double p_fx, p_fy, p_fz;
    p_fx = 5;
    p_fy = 10;  
    p_fz = 0;
    
    Eigen::Vector3d fbody; 
    double T_net = T_net_zero + mass_sum*g_ ;  //net force - mg , input
    fbody = -R0*T_net*e3 + mass_sum*g_*e3;
	Eigen::Vector3d  vdot = fbody  /mass_sum; //earth  frame
 	
    double cbeta_dot;
    
    double state_feature[21];
    for (int i = 0; i < 15; i++)
      state_feature[i]= x[i+1];
    state_feature[15]= x[18];
    state_feature[16]= x[19];
    state_feature[17]= x[20];
    state_feature[18]= p_fx;
    state_feature[19]= p_fy;
    state_feature[20]= p_fz;
    
    cbeta_dot = cosbetadotsyms(state_feature);
    
    double omega0_d[3];
    double x0_d[3];
    double v0_d[3]; 
    double a0_d[3]; 

	double p_err = x0_x*x0_x + x0_y*x0_y + x0_z*x0_z; 
    double v_err = v0_x*v0_x + v0_y*v0_y + v0_z*v0_z; 
    //double yaw_err = psi*psi;
    double cbeta_err = (cosbeta-1)*(cosbeta-1);
    
    Eigen::Matrix3d R0d; //the desired rotation matrix
    R0d << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;    
    
    Eigen::Matrix3d deltaR; 
    deltaR = R0d.transpose()*R0; 
    //double R_err = 0.5* ((1-deltaR(0,0))+ (1-deltaR(1,1)) + (1-deltaR(2,2)));  
    double R_err = 0.5* ( (1-deltaR(2,2)));   
    
    double delta_x = 5*p_err + 2*v_err + 80*R_err +  200* cbeta_err; //the coefficient of the volocity can also be adjusted. 	

 
    Eigen::Vector3d u_gen_trans;
    u_gen_trans << 0, 0, 0;   //The force is expressed in earth frame, and does not include the gravity force. 
     
	double delta_u =  (omega0_x - u_gen_trans(0) )* (omega0_x - u_gen_trans(0) )+ (omega0_y - u_gen_trans(1))* (omega0_y - u_gen_trans(1))  + (omega0_z - u_gen_trans(2))* (omega0_z - u_gen_trans(2)) +
            1* T_net_zero * T_net_zero;

    f[0] = v0_x; 
	f[1] = v0_y;  
	f[2] = v0_z; 
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

	f[15] = cbeta_dot;  

    f[16] = 3*delta_x + 0.01*delta_u;   
    f[16] = 10*delta_x + 0.05*delta_u;  
    f[16] = 3*delta_x + 0.1*delta_u;  
    
        f[16] = 2*delta_x + 0.1*delta_u;   //ok 
        
        f[16] = 2*delta_x + 0.1*delta_u;  

    //cout << "x0: " << x[0] << endl; 
}


double cosbetadotsyms(const double u[21])
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double m_a;
  double n_a;
  double o_a;
  double p_a;
  double q_a;
  double r_a;
  double s_a;
  double t_a;
  double u_a;
  double v_a;
  double w_a;
  double x_a;
  double y_a;
  double ab_a;
  double bb_a;
  double cb_a;
  double db_a;
  double eb_a;
  double fb_a;
  double gb_a;
  double hb_a;
  double ib_a;
  double jb_a;
  double kb_a;
  double lb_a;
  double mb_a;
  double nb_a;
  double ob_a;
  double pb_a;
  double qb_a;
  double rb_a;
  double sb_a;
  double tb_a;
  double ub_a;
  double vb_a;
  double wb_a;
  double xb_a;
  double yb_a;
  double ac_a;
  double bc_a;
  double cc_a;
  double dc_a;
  double ec_a;
  double fc_a;
  double gc_a;

  /* the position of the vehicle: */
  /*  global x0_x  x0_y  x0_z;  */
  /* the velocity of the vehicle, in earth frame: */
  /*  global v0_x v0_y v0_z;  */
  /*  global r0_11 r0_12 r0_13 r0_21 r0_22 r0_23 r0_31 r0_32 r0_33; */
  /*  global omega0_x omega0_y omega0_z; */
  a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
       u[1]) - u[12] * u[2];
  b_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  c_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  d_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  e_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  f_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  g_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  h_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  i_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  j_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  k_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  l_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  m_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  n_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  o_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  p_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  q_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  r_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  s_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  t_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  u_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  v_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  w_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11]
         * u[1]) - u[14] * u[2];
  x_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] *
         u[1]) - u[12] * u[2];
  y_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10]
         * u[1]) - u[13] * u[2];
  ab_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  bb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  cb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  db_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  eb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  fb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  gb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  hb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  ib_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  jb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  kb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  lb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  mb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  nb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  ob_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  pb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  qb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  rb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  sb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  tb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  ub_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  vb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  wb_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  xb_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  yb_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  ac_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  bc_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  cc_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  dc_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  ec_a = ((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9]
          * u[1]) - u[12] * u[2];
  fc_a = ((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u
          [10] * u[1]) - u[13] * u[2];
  gc_a = ((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u
          [11] * u[1]) - u[14] * u[2];
  return (((((((((((u[18] - u[0]) * (u[15] * u[7] - u[16] * u[6]) * (((((u[18] *
    u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] *
    u[2]) * (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) -
              u[11] * u[1]) - u[14] * u[2]) / pow((a * a + b_a * b_a) +
    c_a * c_a, 1.5) - ((u[19] - u[1]) / sqrt((d_a * d_a + e_a * e_a) + f_a * f_a)
                       - (u[19] - u[1]) * (g_a * g_a) / pow((h_a * h_a +
    i_a * i_a) + j_a * j_a, 1.5)) * (u[16] * u[11] - u[17] * u[10])) - ((u[20] -
    u[2]) / sqrt((k_a * k_a + l_a * l_a) + m_a * m_a) - (u[20] - u[2]) * (n_a *
    n_a) / pow((o_a * o_a + p_a * p_a) + q_a * q_a, 1.5)) * (u[16] * u
    [14] - u[17] * u[13])) - u[3] * (u[6] / sqrt((r_a * r_a + s_a * s_a) + t_a *
    t_a) - ((2.0 * u[6] * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) -
    u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) + 2.0 * u[7] * (((((u[18] * u[7]
    + u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] *
    u[2])) + 2.0 * u[8] * (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) -
    u[8] * u[0]) - u[11] * u[1]) - u[14] * u[2])) * (((((u[18] * u[6] + u[19] *
    u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) / (2.0 *
    pow((u_a * u_a + v_a * v_a) + w_a * w_a, 1.5)))) - u[4] * (u[9] /
    sqrt((x_a * x_a + y_a * y_a) + ab_a * ab_a) - ((2.0 * u[9] * (((((u[18] * u
    [6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] *
    u[2]) + 2.0 * u[10] * (((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) -
    u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2])) + 2.0 * u[11] * (((((u[18] *
    u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11] * u[1]) - u[14]
    * u[2])) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0])
                 - u[9] * u[1]) - u[12] * u[2]) / (2.0 * pow((bb_a *
    bb_a + cb_a * cb_a) + db_a * db_a, 1.5)))) - u[5] * (u[12] / sqrt((eb_a *
    eb_a + fb_a * fb_a) + gb_a * gb_a) - ((2.0 * u[12] * (((((u[18] * u[6] + u
    [19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2])
    + 2.0 * u[13] * (((((u[18] * u[7] + u[19] * u[10]) + u[20] * u[13]) - u[7] *
                       u[0]) - u[10] * u[1]) - u[13] * u[2])) + 2.0 * u[14] *
    (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11] *
      u[1]) - u[14] * u[2])) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12])
    - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) / (2.0 * pow((hb_a *
    hb_a + ib_a * ib_a) + jb_a * jb_a, 1.5)))) - (u[18] - u[0]) * (u[15] * u[8]
    - u[17] * u[6]) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) - u[6] *
    u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] * u[7] + u[19] * u[10]) +
    u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2]) / pow
              ((kb_a * kb_a + lb_a * lb_a) + mb_a * mb_a, 1.5)) - ((u[18] - u[0])
              / sqrt((nb_a * nb_a + ob_a * ob_a) + pb_a * pb_a) - (u[18] - u[0])
              * (qb_a * qb_a) / pow((rb_a * rb_a + sb_a * sb_a) + tb_a *
    tb_a, 1.5)) * (u[16] * u[8] - u[17] * u[7])) - (u[19] - u[1]) * (u[15] * u
             [11] - u[17] * u[9]) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] *
    u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] * u[7] +
    u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2])
            / pow((ub_a * ub_a + vb_a * vb_a) + wb_a * wb_a, 1.5)) + (u
            [19] - u[1]) * (u[15] * u[10] - u[16] * u[9]) * (((((u[18] * u[6] +
    u[19] * u[9]) + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2])
           * (((((u[18] * u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0])
               - u[11] * u[1]) - u[14] * u[2]) / pow((xb_a * xb_a + yb_a
             * yb_a) + ac_a * ac_a, 1.5)) - (u[20] - u[2]) * (u[15] * u[14] - u
           [17] * u[12]) * (((((u[18] * u[6] + u[19] * u[9]) + u[20] * u[12]) -
             u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] * u[7] +
    u[19] * u[10]) + u[20] * u[13]) - u[7] * u[0]) - u[10] * u[1]) - u[13] * u[2])
          / pow((bc_a * bc_a + cc_a * cc_a) + dc_a * dc_a, 1.5)) + (u[20]
    - u[2]) * (u[15] * u[13] - u[16] * u[12]) * (((((u[18] * u[6] + u[19] * u[9])
    + u[20] * u[12]) - u[6] * u[0]) - u[9] * u[1]) - u[12] * u[2]) * (((((u[18] *
    u[8] + u[19] * u[11]) + u[20] * u[14]) - u[8] * u[0]) - u[11] * u[1]) - u[14]
    * u[2]) / pow((ec_a * ec_a + fc_a * fc_a) + gc_a * gc_a, 1.5);
}


