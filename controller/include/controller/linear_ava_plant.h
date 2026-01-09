// kaidi wang 
// 2023.3.1
// linear ava platform plant head file
#ifndef LINEAR_AVA_PLANT_H_
#define LINEAR_AVA_PLANT_H_
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
// uav normal function define 
#include <controller/uav_normal_function.h>
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
// define a xyz class
class xyz
{
private:
    /* data */
public:
    double x;
    double y;
    double z;
    xyz(/* args */);
    ~xyz();
};

// define input of plant dynamic model
class plant_input
{
private:
    
public:
    geometry_msgs::Point angle1; // sub1 angle 
    geometry_msgs::Point angle2; // sub2 angle
    geometry_msgs::Point angle3; // sub3 angle
    std_msgs::Float64 thrust1; // sub1 thrust
    std_msgs::Float64 thrust2; // sub2 thrust
    std_msgs::Float64 thrust3; // sub3 thrust

    geometry_msgs::Point init_pos; // init position 
    geometry_msgs::Point init_att; // init attitude
    geometry_msgs::Point init_vel; // init velocity
    geometry_msgs::Point init_body_rate; // init body rate


public:
    plant_input(/* args */);
    ~plant_input();
};




// define output of plant dynamic model
class plant_output
{
private:
    /* data */
public:
    // dynamic model output, position, attitude, velocity and body rate
    geometry_msgs::Point main_position; // ned,
    geometry_msgs::Point main_attitude; // ned, euler angle
    geometry_msgs::Point main_velocity; // linear velocity
    geometry_msgs::Point main_body_rate; // body rate 
    geometry_msgs::Point thrust; // thrust
    geometry_msgs::Point torque; //torque
public:
    plant_output(/* args */);
    ~plant_output();
};



// define linear plant class
class linear_ava_plant
{
private:
    /* data */
    Eigen::Matrix3f I;
    Eigen::Matrix3f zero;
    Eigen::Matrix3f J;
    Eigen::Vector3f e3;
    Eigen::Vector3f zero_vec;

    // integrator temp data
    double va0_dot_last,va1_dot_last,va2_dot_last,va3_dot_last,va4_dot_last;
    double vu_dot_last;
    double position_x_last,position_y_last,position_z_last;
    double attitude_x_last,attitude_y_last,attitude_z_last;
public:

    xyz fly1_pos;
    xyz fly2_pos;
    xyz fly3_pos;
    xyz I_platform; // inertia of linear platform
    double platform_mass; // mass of linear platform
    double fly1_mass;
    double fly2_mass;
    double fly3_mass;

    float node1_yaw;
    float node2_yaw;
    float node3_yaw;

    xyz I_sys; // inertia of entire system
    double sys_mass; // mass of entire system
    double g;
    //omege and epsilon param
    double omega_phi;
    double epsilon_phi;
    double omega_theta;
    double epsilon_theta;
    double omega_psi;
    double epsilon_psi;
    double omega_u;
    double epsilon_u;
    double omega_v;
    double epsilon_v;
    double omega_w;
    double epsilon_w;
    double omega_x;
    double epsilon_x;
    double omega_y;
    double epsilon_y;
    double omega_z;
    double epsilon_z;
    double omega_p;
    double epsilon_p;
    double omega_q;
    double epsilon_q;
    double omega_r;
    double epsilon_r;
    xyz tool_pos;
    //controller param
    double k_taux;
    double taux_last;

    double kp_x;
    double kp_y;
    double kp_z;

    double ki_x;
    double ki_y;
    double ki_z;

    double kd_x;
    double kd_y;
    double kd_z;
    /***********************autopilot control PID theory************************/
    double kpx;
    double kpy;
    double kpz;
  
    /***************************************************************************/
    double kp_phi;//x;
    double kp_theta;//y;
    double kp_psi;//z;

    double ki_phi;//x;
    double ki_theta;//y;
    double ki_psi;//z;

    double kd_phi;//x;
    double kd_theta;//y;
    double kd_psi;//z;

    bool flag_ask_noise;
    
    bool ideal_model;
    // define input and output
    plant_input input;
    plant_output output;
    Eigen::Vector3f thrust_u;     //thrust vector
    Eigen::Vector3f torques_u;    //torque vector

    Eigen::Vector3f fe; // thrust noise
    Eigen::Vector3f te; // torque noise
    Eigen::Matrix3f rotation_matrix;
    // define matrix of plant 
    Eigen::Vector3f body_rates;
    Eigen::Vector3f euler_angles;
    Eigen::Vector3f omega_dot;
    double p_dot;
    double q_dot;
    double r_dot;
    double p_dot_last;
    double q_dot_last;
    double r_dot_last;
    double p,q,r;

    double phi,theta,psi;
    double phi_dot,theta_dot,psi_dot;
    double phi_dot_last,theta_dot_last,psi_dot_last;

    Eigen::Vector3f x0_w;
    Eigen::Vector3f v0_dot;
    Eigen::Vector3f v0;
    
    double v1,v2,v3;
    double v1_dot,v2_dot,v3_dot;
    double v1_dot_last,v2_dot_last,v3_dot_last;

    double w_x         ,w_y         ,w_z;
    double w_x_dot     ,w_y_dot     ,w_z_dot;
    double w_x_dot_last,w_y_dot_last,w_z_dot_last;
    // Eigen::MatrixXf omega_u; // omega_u
    // Eigen::MatrixXf delta_u; // delta_u


    //output message variable
    geometry_msgs::Point position;
    geometry_msgs::Point euler_angle;
    geometry_msgs::Point body_rate;
    geometry_msgs::Point velocity;

    // init state
    void init_state(
        geometry_msgs::Point init_position,
        geometry_msgs::Point init_velocity,
        geometry_msgs::Point init_euler_angles,
        geometry_msgs::Point init_body_rate);

    // composition angle and thrust
    void composition(
        geometry_msgs::Point body_euler, 
        geometry_msgs::Point sub1_ang, 
        geometry_msgs::Point sub2_ang, 
        geometry_msgs::Point sub3_ang,
        std_msgs::Float64 sub1_thu,
        std_msgs::Float64 sub2_thu,
        std_msgs::Float64 sub3_thu,
        Eigen::Vector3f U_front,
        Eigen::Vector3f U_end);
    void init_mat();// calc delta_a matrix
    void update_mat(
        geometry_msgs::Point v, 
        geometry_msgs::Point body_rate,
        geometry_msgs::Point thrust,
        geometry_msgs::Point torque,
        geometry_msgs::Point main_euler);
    void ask_noise(); // ask noise function 
    double getRandData(double min,double max); // get rank data 

    void dynamic_model();// dynamic model function 
    void update_va(); // update va function , integrator vector x5
    void update_velocity_position(); // velocity and position
    void update_body_rate_attitude(); // body_rate and attitude


    // old plant code 
    void system_plant();

    //second level function list
    void rotation_kinematics();
    void translational_kinematics();
    void rotation_dynamics();
    void translational_dynamics();
    void output_publish();
    linear_ava_plant(/* args */);
    ~linear_ava_plant();

};



#endif