// kaidi wang, zean zhen 2023.3.1
// linear ava controller
// define a class of linear ava controller
#ifndef LINEAR_AVA_CONTROLLER_H_
#define LINEAR_AVA_CONTROLLER_H_
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <cstring>

#include <controller/linear_ava_plant.h> // linear ava plant file
#include <controller/uav_normal_function.h>
//#include <controller/param.h>

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>


// define a linear ava controller class
class linear_ava_controller
{
private:
    // define a ros nodehandle nh
    ros::NodeHandle nh;

	double g;
    int count_loop;
    //define a controller_base class
    linear_ava_plant param_linear;

    Eigen::Matrix3f I33;
    Eigen::Matrix3f zero_33;
    Eigen::Vector3f vector3_zero;
    Eigen::Vector3f e3;
    Eigen::Matrix3f I;

    Eigen::Matrix3f pos_s1;      //position matrix
    Eigen::Matrix3f pos_s2;
    Eigen::Matrix3f pos_s3;
	
    //define Rotation matrix
    Eigen::Matrix3f r_mat;       //main_euler_angle rotation mat
    Eigen::Matrix3f r_mat_d;     //nominal_euler_angle rotation mat

	Eigen::Matrix<float, 6, 6> S_bar;
    Eigen::Matrix<float, 6, 6> S_bar_t;
    Eigen::Matrix3f J;
    Eigen::Matrix3f J_bar;
	Eigen::Matrix<float, 6, 6> M;
	Eigen::Matrix<float, 6, 6> C;
	Eigen::Matrix<float, 6, 1> G;

	Eigen::Matrix<float, 5, 5> Ma;
	Eigen::Matrix<float, 5, 5> Ca;
	Eigen::Matrix<float, 5, 1> Cau;
	Eigen::Matrix<float, 5, 1> ga;

	Eigen::Matrix<float, 1, 5> grad_phi;
    Eigen::Vector3f grad_v;      //calc grad function output
    Eigen::Vector3f grad_w;

	Eigen::Matrix<float, 6, 6> delta;
	Eigen::Matrix<float, 6, 5> delta_a;
	Eigen::Matrix<float, 6, 1> delta_u;
	Eigen::Matrix<float, 6, 6> Omega;
	Eigen::Matrix<float, 6, 5> omega_at;
	Eigen::Matrix<float, 6, 1> omega_ut;
    Eigen::Matrix<float, 6, 1> V;
	Eigen::Matrix<float, 6, 1> V_d;
	Eigen::Matrix<float, 6, 1> V_d_dot;
	Eigen::Matrix<float, 5, 1> Va;
	Eigen::Matrix<float, 5, 1> ua;
	Eigen::Matrix<float, 5, 1> Va_d;
	Eigen::Matrix<float, 5, 1> ua_d;
	Eigen::Matrix<float, 5, 1> Va_d_dot;
	Eigen::Matrix<float, 5, 1> ua_d_dot;
	// Eigen::Matrix<float, 6, 1> U_u;
	Eigen::Matrix<float, 6, 1> U;

	Eigen::Matrix<float, 5, 1> ea;          // ea = va-vad
    Eigen::Matrix<float, 5, 1> sum_ea; // sum of ea
    Eigen::Vector3f sum_ea_front; // sum of ea first three items
    Eigen::Vector3f sum_ea_end;   // sum of ea end three items
    Eigen::Vector3f sum_ea_front_ls;
    Eigen::Vector3f sum_ea_end_ls;
    Eigen::Vector3f ea_front; // ea first three items
    Eigen::Vector3f ea_end; // ea end three items


    Eigen::Matrix<float, 5, 1> sum_grad_phi_t; // sum of ea
    Eigen::Vector3f sum_grad_phi_t_front; // sum of ea first three items
    Eigen::Vector3f sum_grad_phi_t_end;   // sum of ea end three items
    Eigen::Vector3f sum_grad_phi_t_front_ls;
    Eigen::Vector3f sum_grad_phi_t_end_ls;
    Eigen::Vector3f grad_phi_t_front; // ea first three items
    Eigen::Vector3f grad_phi_t_end; // ea end three items

	Eigen::Matrix<float, 5, 5> kv;
	Eigen::Matrix<float, 5, 5> ki;
	Eigen::Matrix<float, 6, 6> kd;


    Eigen::Vector3f unit_inter1; // 
	Eigen::Vector3f pos1; // sub aircraft1 position
    Eigen::Vector3f pos2; // sub_aircraft2 position

    float uu;
	double Vu;
	double Vu_d;
	double Vu_d_dot;

    bool update_mian;
    // taux last value
    
	Eigen::Vector3f r_e0;
	Eigen::Vector3f r_ew;
    Eigen::Vector3f r_d_omega;
	Eigen::Matrix3f r_e0_s;
	Eigen::Matrix3f r_ew_s;

    Eigen::Vector3f err_xyz;     //main_position-nominal_position
    Eigen::Vector3f de_0;        //tool attach position in body frame
    Eigen::Matrix3f de_s;

    Eigen::Matrix3f kp_pos;      //p gain of position
    Eigen::Matrix3f ki_pos;      //i gain of position
    Eigen::Matrix3f kd_pos;      //d gain of position

    Eigen::Matrix3f kp_att;      //p gain of attitude
    Eigen::Matrix3f ki_att;      //i gain of attitude
    Eigen::Matrix3f kd_att;      //d gain of attitude

    /**********************autopilot control theory************************/
    Eigen::Matrix3f kp;
    double err_v_integral_x_ls;
    double err_v_integral_y_ls;
    double err_v_integral_z_ls;
    /**********************************************************************/
    
    //child function output list 
    Eigen::Vector3f position_err;//calc grad function output  
    Eigen::Vector3f att_err;     
    Eigen::Vector3f ev;          //input of calc_thrust function
    Eigen::Vector3f ew;          //input of calc_torques

    Eigen::Vector3f thrust_u;
    Eigen::Vector3f torques_u;

    Eigen::Vector3f lamda1;
    Eigen::Vector3f lamda2;
    Eigen::Vector3f lamda3;
    //grad_w 

    //rotation kinematic inverse
    Eigen::Vector3f omega_d;
    Eigen::Vector3f omege;
    //translational kinematic inverse
    Eigen::Vector3f vd;
	Eigen::Matrix<float, 6, 9> B;
	Eigen::Matrix<float, 5, 9> B_full;

	Eigen::Matrix<float, 6, 1>  epsilon;
	Eigen::Matrix<float, 6, 1>  epsilon_d;
	Eigen::Matrix<float, 6, 1>  epsilon_d_dot;
	
    //calc_epsilon_v function
    Eigen::Vector3f epsilon_d_v;
    Eigen::Vector3f epsilon_d_v_dot;
    //calc_epsilon_w function
    Eigen::Vector3f epsilon_d_w;
    Eigen::Vector3f epsilon_d_w_dot;

    //output list
    geometry_msgs::Point ang1;
    geometry_msgs::Point ang2;
    geometry_msgs::Point ang3;
    std_msgs::Float64 thu1;
    std_msgs::Float64 thu2;
    std_msgs::Float64 thu3;

    //publister list
    ros::Publisher angle1_pub;
    ros::Publisher angle2_pub;
    ros::Publisher angle3_pub;
    ros::Publisher thrust1_pub;
    ros::Publisher thrust2_pub;
    ros::Publisher thrust3_pub;

    ros::Publisher pos_sim_pub;
    ros::Publisher euler_sim_pub;
    ros::Publisher body_rate_sim_pub;
    ros::Publisher velocity_sim_pub;

    // publish thrust and torque
    ros::Publisher thrust_pub;
    ros::Publisher torque_pub;

    //subscriber list
	ros::Subscriber nominal_position_sub ;      //nominal_position
	ros::Subscriber nominal_eular_angles_sub ;  //nominal_eular_angles
    ros::Subscriber nominal_velocity_sub;       //nominal velocity
    ros::Subscriber nominal_body_rate_sub;      //nominal body rate
	ros::Subscriber main_position_sub ;	        //controller input of main_position
	ros::Subscriber main_velocity_sub ;	        //controller input of main_velocity
	ros::Subscriber main_eular_angles_sub ;	    //controller input of main_eular_angles
	ros::Subscriber main_body_rates_sub ;	    //controller input of main_body_rates
	ros::Subscriber init_euler_angles_cmd_sub ;	//init cmd publish: euler angles
	ros::Subscriber init_pos_cmd_sub ;	        //init cmd publish: position
	ros::Subscriber init_body_rates_cmd_sub ;	//init cmd publish: body_rates
	ros::Subscriber init_velocity_cmd_sub ;	    //init cmd publish: velocity
    ros::Subscriber mode_switch_sub;            //mode switch msg
	ros::Subscriber control_start_sub_att ;	    //control switch of when publish attitude to child node, besides control when start the controller  
	ros::Subscriber psi_bias_1_sub;
	ros::Subscriber psi_bias_2_sub;
	ros::Subscriber psi_bias_3_sub;

	//integrator Intermediate variables list
    double dot_u_integra_val_1;
    double dot_u_integra_val_2;
    double dot_v_integra_val_1;
    double dot_v_integra_val_2;
    double dot_w_integra_val_1;
    double dot_w_integra_val_2;

    double dot_p_integra_val_1;
    double dot_p_integra_val_2;
    double dot_q_integra_val_1;
    double dot_q_integra_val_2;
    double dot_r_integra_val_1;
    double dot_r_integra_val_2;

    double dot_phi_integra_val_1;
    double dot_phi_integra_val_2;
    double dot_theta_integra_val_1;
    double dot_theta_integra_val_2;
    double dot_psi_integra_val_1;
    double dot_psi_integra_val_2;

    double dot_xd_integra_val_1;
    double dot_xd_integra_val_2;
    double dot_yd_integra_val_1;
    double dot_yd_integra_val_2;
    double dot_zd_integra_val_1;
    double dot_zd_integra_val_2;

    double ang_err_init_x_integar_last;
    double ang_err_init_y_integar_last;
    double ang_err_init_z_integar_last;

    double pos_err_init_x_integar_last;
    double pos_err_init_y_integar_last;
    double pos_err_init_z_integar_last;


    //differ last number var
    double epsilon_d_v_dot_u_last;
    double epsilon_d_v_dot_v_last;
    double epsilon_d_v_dot_w_last;
    //epsilon_d_v_dot_u Runge Kutta method related val
    double epsilon_d_v_dot_u_x;
    double epsilon_d_v_dot_u_y;
    double epsilon_d_v_dot_u_z;
    double epsilon_d_v_dot_u;
    //epsilon_d_v_dot_v Runge Kutta method related val
    double epsilon_d_v_dot_v_x;
    double epsilon_d_v_dot_v_y;
    double epsilon_d_v_dot_v_z;
    double epsilon_d_v_dot_v;
    //epsilon_d_v_dot_w Runge Kutta method related val
    double epsilon_d_v_dot_w_x;
    double epsilon_d_v_dot_w_y;
    double epsilon_d_v_dot_w_z;
    double epsilon_d_v_dot_w;

    double epsilon_d_w_dot_p_last;
    double epsilon_d_w_dot_q_last;
    double epsilon_d_w_dot_r_last;
    //epsilon_d_v_dot_p Runge Kutta method related val
    double epsilon_d_w_dot_p_x;
    double epsilon_d_w_dot_p_y;
    double epsilon_d_w_dot_p_z;
    double epsilon_d_w_dot_p;
    //epsilon_d_v_dot_v Runge Kutta method related val
    double epsilon_d_w_dot_q_x;
    double epsilon_d_w_dot_q_y;
    double epsilon_d_w_dot_q_z;
    double epsilon_d_w_dot_q;
    //epsilon_d_v_dot_w Runge Kutta method related val
    double epsilon_d_w_dot_r_x;
    double epsilon_d_w_dot_r_y;
    double epsilon_d_w_dot_r_z;
    double epsilon_d_w_dot_r;

    double phi_bar_dot_last;
    double theta_bar_dot_last;
    double psi_bar_dot_last;

    double xd_dot_last;
    double yd_dot_last;
    double zd_dot_last;

    //phi Runge Kutta method related val
    double phi_bar_dot_x;
    double phi_bar_dot_y;
    double phi_bar_dot_z;
    double phi_bar_dot;
    //theta Runge Kutta method related val
    double theta_bar_dot_x;
    double theta_bar_dot_y;
    double theta_bar_dot_z;
    double theta_bar_dot;
    //psi Runge Kutta method related val
    double psi_bar_dot_x;
    double psi_bar_dot_y;
    double psi_bar_dot_z;
    double psi_bar_dot;
    //x Runge Kutta method related val
    double xd_dot_x;
    double xd_dot_y;
    double xd_dot_z;
    double xd_dot;
    //y Runge Kutta method related val
    double yd_dot_x;
    double yd_dot_y;
    double yd_dot_z;
    double yd_dot;
    //z Runge Kutta method related val
    double zd_dot_x;
    double zd_dot_y;
    double zd_dot_z;
    double zd_dot;

    // get subscriber callback variables
    geometry_msgs::Point nominal_position;      //setpoint position
    geometry_msgs::Point nominal_euler_angles;  //euler angle setpoint position
    geometry_msgs::Point nominal_velocity;      //setpoint position
    geometry_msgs::Point nominal_body_rate;     //euler angle setpoint position
    geometry_msgs::Point main_position;
    geometry_msgs::Point main_velocity;
    geometry_msgs::Point main_eular_angles;
    geometry_msgs::Point main_body_rates;
    geometry_msgs::Point init_euler_angles;
    geometry_msgs::Point init_position;
    geometry_msgs::Point init_body_rate;
    geometry_msgs::Point init_velocity;
    std_msgs::Bool mode_switch;
    std_msgs::Bool start_pub_att;
    std_msgs::Float64 node1_yaw;
    std_msgs::Float64 node2_yaw;
    std_msgs::Float64 node3_yaw;
	
    int run_times;
    int flag_sim_real;
private:
    // Interface functions 
    void init();
    void init_publisher();
    void init_subscriber();
    void output_publish(geometry_msgs::Point ang1,geometry_msgs::Point ang2,geometry_msgs::Point ang3,std_msgs::Float64 thu1,std_msgs::Float64 thu2,std_msgs::Float64 thu3);
    void time_run();
    geometry_msgs::Point range_judge(geometry_msgs::Point origin, double range);

    void controller(); // controller body
    void plant();

    void controller_body();
    void calc_grad_phi();
    void calc_epsilon_d();
    void calc_grad_v();
    void calc_grad_w();
    void calc_epsilon_v();      //v epsilon 
    void calc_epsilon_w();      //w epsilon

    void dynamics_eqution();
    void full_actuacted_dynamics_eqution();
    
    void calc_U();
    void calc_vd(double xd_dot, double yd_dot, double zd_dot);
    void calc_grad();
    
    

    void control_allocation();
    void ctrl_allocation();
    void ctrl_allocation_new();

    Eigen::Vector4f alloc(Eigen::Vector3f f,double psi_cmd);

	void RK4_02(double u,double omega,double epsilon,double *x,double *y,double *z,double h);
	void kinematics_pseudoinverse(double phi_bar, double phi_bar_dot,double theta_bar, double theta_bar_dot,double psi_bar, double psi_bar_dot);
	void rotation_kine_inerse();//kinematic inverse of rotation
    void trans_kine_inverse(); //translational kinematic inverse

	double gain_1(double u,double omega);
    void update_main_pos();
    void publish_main_pos();
	
private:
    // call back function list
    void nominal_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg );
    void nominal_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void nominal_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg );
    void nominal_body_rate_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_body_rates_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg);
    void control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg);
    void psi_bias_1_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void psi_bias_2_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void psi_bias_3_sub_cb(const std_msgs::Float64::ConstPtr& msg);

public:

    linear_ava_controller(ros::NodeHandle* nodehandle);
    //define a timer for period calc controller
    ros::Timer calc_timer;
    void calc_cb(const ros::TimerEvent&);

    ~linear_ava_controller();
};

// callback function 
void linear_ava_controller::nominal_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_position = *msg;
	//ROS_INFO_STREAM("nominal_position: ");
	//ROS_INFO_STREAM(nominal_position);
}
//setpoint attitude, eular angles, chk
void linear_ava_controller::nominal_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_euler_angles = *msg;
	// ROS_INFO_STREAM("nominal_euler_angles: ");
	// ROS_INFO_STREAM(nominal_euler_angles);
}

void linear_ava_controller::nominal_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg )
{
	nominal_velocity = *msg;
    // ROS_INFO_STREAM(nominal_velocity);
}
void linear_ava_controller::nominal_body_rate_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_body_rate = *msg;
    // ROS_INFO_STREAM(nominal_body_rate);
}


//position back, chk
void linear_ava_controller::main_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    if(flag_sim_real == 0)
    {
        main_position = *msg;
    }
	
	// ROS_INFO_STREAM("main_position: ");
	// ROS_INFO_STREAM(main_position);
}
//velocity back, chk
void linear_ava_controller::main_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    if(flag_sim_real == 0)
    {
	    main_velocity = *msg;
    }
	// ROS_INFO_STREAM("main_velocity: ");
	// ROS_INFO_STREAM(main_velocity);
}
//attitude back, chk
void linear_ava_controller::main_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    if(flag_sim_real == 0)
    {
	    main_eular_angles = *msg;
	}
    // ROS_INFO_STREAM("main_eular_angles: ");
	// ROS_INFO_STREAM(main_eular_angles);
}
//body rates back,chk
void linear_ava_controller::main_body_rates_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    if(flag_sim_real == 0)
    {
        main_body_rates = *msg;
    }
	// ROS_INFO_STREAM("main_body_rates: ");
	// ROS_INFO_STREAM(main_body_rates);
}
//init attitude,chk
void linear_ava_controller::init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{

	init_euler_angles = *msg;
    init_euler_angles.x = 0;
    // ROS_INFO_STREAM("init_euler_angles: ");
	// ROS_INFO_STREAM(init_euler_angles);
}
//init position,chk
void linear_ava_controller::init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_position = *msg;
	// ROS_INFO_STREAM("init_position: ");
	// ROS_INFO_STREAM(init_position);
}
//init body rate,chk
void linear_ava_controller::init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_body_rate = *msg;
	// ROS_INFO_STREAM("init_body_rate: ");
	// ROS_INFO_STREAM(init_body_rate);
}
//init velocity 
void linear_ava_controller::init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_velocity = *msg;
	// ROS_INFO_STREAM("init_velocity: ");
	// ROS_INFO_STREAM(init_velocity);
}
//mode switch callback function
void linear_ava_controller::mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
	mode_switch = *msg;
}

//control start chk
void linear_ava_controller::control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg)
{
	start_pub_att = *msg;
}
//psi_bias 1 chk
void linear_ava_controller::psi_bias_1_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node1_yaw = *msg;
}
//psi_bias 2 chk
void linear_ava_controller::psi_bias_2_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node2_yaw = *msg;
}
//psi_bias 3 chk
void linear_ava_controller::psi_bias_3_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node3_yaw = *msg;
}

linear_ava_controller::linear_ava_controller(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // define a timer 100hz
    calc_timer = nh.createTimer(ros::Duration(TIME), &linear_ava_controller::calc_cb, this);  //timer used to publish state, should be at least for some minimal frequency
    // init section start
    		
    run_times = 0;
    count_loop = 0;
	zero_33.setZero(3,3); 

	I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;


    pos_err_init_x_integar_last = pos_err_init_y_integar_last = pos_err_init_z_integar_last = 0;
	e3(0)=e3(1)=0;
	e3(2)=1;
    init_publisher();// inin publisher 
    init_subscriber();// init subscriber
    init(); // init some matrix and some temp variables

    // init section end
    
    ROS_INFO_STREAM("Whether this test is real flight experiment :Y/n");
	std::string str;
	std::getline(std::cin,str);
	if (str == "Y")
	{
		ROS_INFO_STREAM("this is real flight experiment.");
		flag_sim_real = 0;
        
		// ROS_INFO_STREAM("flag_real_sim:"<<flag_real_sim);
		// mpc_state_function();
		// ros::shutdown();// close the ros node
	}
	else
	{
		ROS_INFO_STREAM("this is simulation test.");
		flag_sim_real = 1;
        param_linear.ask_noise();
	}
}

linear_ava_controller::~linear_ava_controller()
{
}

#endif