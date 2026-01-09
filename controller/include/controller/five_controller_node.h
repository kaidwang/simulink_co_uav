//kaidi wang,
//2023.11.28
//define a class and decleartion of class member function
//change this code for normal path or mpc path
//four sub-aircraft platform
#ifndef  FOUR_CONTROLLER_NODE_H_
#define  FOUR_CONTROLLER_NODE_H_

#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <controller/four_param.h>
// #include <dynamic_reconfigure/server.h>
#include <math.h>
#include <dynamic_reconfigure/server.h> 
#include <controller/paramConfig.h>
//#include <eigen3/Eigen/Core>
#include <controller/config_param.h>

#include <ros/ros.h>
#define TIME 0.01
#define GRAVITY 9.8

class controller_class
{
private:
    //put private member data here: private data will only be available to member function of this class
    ros::NodeHandle nh;//we will need this, to pass between "main" and constructor

    double detla_time;// 0.01s
    int run_times;
    double g;

    //define a controller_base class
    controller_base param;

    Eigen::Matrix3f J;
    Eigen::Matrix3f pos_s1;      //position matrix
    Eigen::Matrix3f pos_s2;
    Eigen::Matrix3f pos_s3;
    Eigen::Matrix3f pos_s4;


    //define Rotation matrix
    Eigen::Matrix3f r_mat;       //main_euler_angle rotation mat
    Eigen::Matrix3f r_mat_d;     //nominal_euler_angle rotation mat

    Eigen::Vector3f err_xyz;     //main_position-nominal_position
    Eigen::Vector3f de_0;        //tool attach position in body frame
    
    Eigen::Matrix3f kp_pos;      //p gain of position
    Eigen::Matrix3f ki_pos;      //i gain of position
    Eigen::Matrix3f kd_pos;      //d gain of position

    Eigen::Matrix3f kp_att;      //p gain of attitude
    Eigen::Matrix3f ki_att;
    Eigen::Matrix3f kd_att;

    /**********************autopilot control theory************************/
    Eigen::Matrix3f kp;
    double err_v_integral_x_ls;
    double err_v_integral_y_ls;
    double err_v_integral_z_ls;
    /**********************************************************************/
    //child function output list 
    Eigen::Vector3f position_err;//calc grad function output
    Eigen::Vector3f grad_v;      //calc grad function output
    Eigen::Vector3f att_err;     
    Eigen::Vector3f grad_w;
    Eigen::Vector3f ev;          //input of calc_thrust function
    Eigen::Vector3f ew;          //input of calc_torques

    Eigen::Vector3f thrust_u;
    Eigen::Vector3f torques_u;

    Eigen::Vector3f lamda1;
    Eigen::Vector3f lamda2;
    Eigen::Vector3f lamda3;
    Eigen::Vector3f lamda4;
    //grad_w 

    //rotation kinematic inverse
    Eigen::Vector3f omega_d;
    Eigen::Vector3f omege;
    //translational kinematic inverse
    Eigen::Vector3f vd;

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
    geometry_msgs::Point ang4;

    std_msgs::Float64 thu1;
    std_msgs::Float64 thu2;
    std_msgs::Float64 thu3;
    std_msgs::Float64 thu4;

    //input list
    geometry_msgs::Point nominal_position;      //setpoint position
	geometry_msgs::Point nominal_euler_angles;  //euler angle setpoint position
    geometry_msgs::Point nominal_velocity;      //setpoint position
	geometry_msgs::Point nominal_body_rate;  //euler angle setpoint position

	geometry_msgs::Point main_position;
	geometry_msgs::Point main_velocity;
	geometry_msgs::Point main_eular_angles;
	geometry_msgs::Point main_body_rates;
	geometry_msgs::Point init_euler_angles;
	geometry_msgs::Point init_position;
	geometry_msgs::Point init_body_rate;
	geometry_msgs::Point init_velocity;

    //kaidi wang, 2021.10.13, mode switch variable
    std_msgs::Bool mode_switch;
    std_msgs::Bool start_pub_att;
    std_msgs::Float64 node1_yaw;
    std_msgs::Float64 node2_yaw;
    std_msgs::Float64 node3_yaw;

    //publister list
    ros::Publisher angle1_pub;
    ros::Publisher angle2_pub;
    ros::Publisher angle3_pub;
    ros::Publisher angle4_pub;

    ros::Publisher thrust1_pub;
    ros::Publisher thrust2_pub;
    ros::Publisher thrust3_pub;
    ros::Publisher thrust4_pub;


    //subscriber list
    //geometry_msgs::Point
	ros::Subscriber nominal_position_sub ;      //nominal_position
	ros::Subscriber nominal_eular_angles_sub ;  //nominal_eular_angles
    ros::Subscriber nominal_velocity_sub; // nominal velocity
    ros::Subscriber nominal_body_rate_sub;// nominal body rate

	ros::Subscriber main_position_sub ;	        //controller input of main_position
	ros::Subscriber main_velocity_sub ;	        //controller input of main_velocity
	ros::Subscriber main_eular_angles_sub ;	    //controller input of main_eular_angles
	ros::Subscriber main_body_rates_sub ;	    //controller input of main_body_rates
	ros::Subscriber init_euler_angles_cmd_sub ;	//init cmd publish: euler angles
	ros::Subscriber init_pos_cmd_sub ;	        //init cmd publish: position
	ros::Subscriber init_body_rates_cmd_sub ;	//init cmd publish: body_rates
	ros::Subscriber init_velocity_cmd_sub ;	    //init cmd publish: velocity

    //kaidi wang, 2021.10.13
    ros::Subscriber mode_switch_sub;            //mode switch msg

    //std_msgs::Bool
	ros::Subscriber control_start_sub_att ;	    //control switch of when publish attitude to child node, besides control when start the controller  
   
    //subscribe the bias of psi angle that each child flight needed
    //std_msgs::Float64
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


    //member funtion list 
    void init_publisher();
    void init_subscriber();
//member function list
    void output_publish(
	geometry_msgs::Point ang1,
	geometry_msgs::Point ang2,
	geometry_msgs::Point ang3,
	geometry_msgs::Point ang4,
	std_msgs::Float64 thu1,
	std_msgs::Float64 thu2,
	std_msgs::Float64 thu3,
	std_msgs::Float64 thu4);    //top level function

    void controller_body();//controller body
    void control_allocation();//controller allocation body

    //second level function
    void calc_grad();// controller body function
    void calc_epsilon_d();
    void calc_thrust();
    void calc_torques();

    void ctrl_allocation();//control allocation
    Eigen::Vector4f alloc(Eigen::Vector3f f,double psi_cmd);//alloc function

    //third level functions
    void calc_grad_v();
    void calc_grad_w();
    
    void rotation_kine_inerse();//kinematic inverse of rotation
    void trans_kine_inverse();//translational kinematic inverse
    void calc_epsilon_v();
    void calc_epsilon_w();

    //fourth level functions 
    double differentiator(
        double u, 
        double initial_angle,
        double omega,
        double epsilon,
        double *output_last,
        double *integral_val_1,
        double *integral_val_2);

    // Runge-Kutta method to solve the ODE
    void RK4_02(double u,double omega,double epsilon,
        double *x,double *y,double *z,double h);
    // differential equation lastest
    void differentiator_lastest(double *last,double *now,double *re,double h);


    void kinematics_pseudoinverse(
        double phi_bar, 
        double phi_bar_dot,
        double theta_bar, 
        double theta_bar_dot,
        double psi_bar, 
        double psi_bar_dot);
    void calc_vd( double xd_dot, double yd_dot, double zd_dot);


    //fiveth level functions
    double gain_1(double u,double omega);
    double gain_2(double u,double omega,double epsilon);
    //double integrator(double input,double orign,double last_sum, double t);//it's worked , int times
    double integrator(double input,double orign,double *ls,double t);//new integrator function
    Eigen::Matrix3f euler_to_rotation_mat(geometry_msgs::Point angle);

    // Runge-Kutta mathod child function
    double d2f(double u,double omega,double epsilon,double x,double y,double z);
    double df(double x,double y,double z);

    //other function
    Eigen::Matrix3f pos_mat(double x,double y,double z);
    void time_run();
    //Eigen::Vector3f calc_lamda();
    //callback function list
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

    geometry_msgs::Point range_judge(geometry_msgs::Point origin, double range);

    // add speed control loop before the position control loop
    Eigen::Vector3f calc_vd_p_controller(Eigen::Matrix3f p, geometry_msgs::Point nominal_pos,geometry_msgs::Point main_pos);
    void calc_thrust_autopilot_pid();
    // kaidi wang code this function, 6.9
    void calc_thrust_new();

    // reconfigure param function
    void reconfigure_param(config_param var_arg);
public:
    controller_class(ros::NodeHandle* nodehandle);//define a init node handle function 
    //define a timer for period calc controller
    ros::Timer calc_timer;
    void calc_cb(const ros::TimerEvent&);
    // void callback(controller::paramConfig &config, uint32_t level);
    ~controller_class();
};
controller_class::~controller_class()
{

}
#endif
