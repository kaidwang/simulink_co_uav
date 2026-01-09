//kaidi wang, 2021.8.1
//define a class and declearation of class member
#ifndef FOUR_PLANT_H__
#define FOUR_PLANT_H__

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <Eigen/Dense>
#include <math.h>
//include memory file
#include <memory>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>
//define usual param
#define PI 3.1415926
#define G 9.8
#define T 0.004//this param need to change 

class some_xyz
{
private:
    //none private param
public:
    double x;
    double y;
    double z;
    some_xyz(/* args */);
    ~some_xyz();
};
some_xyz::some_xyz(/* args */)
{
    x=0;
    y=0;
    z=0;
}
some_xyz::~some_xyz()
{

}

//class of controller_base
class controller_base
{
private:

public:
    //will be used in controller_node.cpp
    /* define data list */
    some_xyz fly1_pos;
    some_xyz fly2_pos;
    some_xyz fly3_pos;
    some_xyz fly4_pos;

    some_xyz I_center;

    double center_mass;
    double fly1_mass;
    double fly2_mass;
    double fly3_mass;
    double fly4_mass;

    some_xyz I_sys;
    double S3Q_mass;//define the mass param
    // Eigen::Matrix3f Mt;//define the inertia of the AVA platform

    some_xyz tool_pos;
    controller_base(/* args */);
    ~controller_base();
};
controller_base::controller_base(/* args */)
{
    float base_len = 0.452;
    // float outrigger_len = 0.51;
    float angle = 45*PI/180;
    float vertical_dis = base_len*cos(angle);
    float horizontal_dis = base_len*cos(angle);
    fly1_pos.x = vertical_dis;
    fly1_pos.y = horizontal_dis;
    fly1_pos.z = -0.046;

    //fly2_position
    fly2_pos.x = -fly1_pos.x ;
    fly2_pos.y = fly1_pos.y;
    fly2_pos.z = -0.046;

    //fly3_position
    fly3_pos.x = fly2_pos.x;
    fly3_pos.y = -fly2_pos.y;
    fly3_pos.z = -0.046;

    // fly4_position
    fly4_pos.x = -fly3_pos.x;
    fly4_pos.y = fly3_pos.y;
    fly4_pos.z = -0.046;

    //tool position
    tool_pos.x = 0;
    tool_pos.y = 0;
    tool_pos.z = 0;

    //mass param
    // fly1_mass = 1.949;//1.597;//1.95
    // fly2_mass = 1.967;//1.604;//1.97
    // fly3_mass = 1.940;//1.583;//1.94
    // center_mass = 1.607;//+1.5;

    fly1_mass = 1.3;//1.597,1.95;//
    fly2_mass = 1.3;//1.604,1.97;//
    fly3_mass = 1.3;//1.583,1.94;//
    fly4_mass = 1.3;
    center_mass = 1.2;//+1.283;

    //I center param
    I_center.x = 0.056804546;
    I_center.y = 0.057436539;
    I_center.z = 0.108610089;

    //calc mass of S3Q platform
    S3Q_mass = fly1_mass+fly2_mass+fly3_mass+fly4_mass+center_mass;
    I_sys.x =  I_center.x + 
               fly1_mass*(fly1_pos.y*fly1_pos.y+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.y*fly2_pos.y+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.y*fly3_pos.y+fly3_pos.z*fly3_pos.z)+
               fly4_mass*(fly4_pos.y*fly4_pos.y+fly4_pos.z*fly4_pos.z);

    I_sys.y =  I_center.y + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.z*fly3_pos.z)+
               fly4_mass*(fly4_pos.x*fly4_pos.x+fly4_pos.z*fly4_pos.z);

    I_sys.z =  I_center.z + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.y*fly1_pos.y)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.y*fly2_pos.y)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.y*fly3_pos.y)+
               fly4_mass*(fly4_pos.x*fly4_pos.x+fly4_pos.y*fly4_pos.y);
    //assignment to Mt matrix
}
controller_base::~controller_base()
{

}

//class of plant simulation
class plant_sim
{
private:
    //param list variable
    controller_base param;

    //whether use ideal model
    bool ideal_model;

    double detla_time;// 0.01s
    int run_times;
    double g;
    Eigen::Vector3f e3;//define z axis 
    // start the plant sim model
    std_msgs::Bool mode_switch;
    std_msgs::Bool start_pub_att;

    //define the param value of mass and moment of inertia
    Eigen::Matrix3f J;
    Eigen::Matrix3f pos_s1;      //position matrix
    Eigen::Matrix3f pos_s2;
    Eigen::Matrix3f pos_s3;
    Eigen::Matrix3f pos_s4;

    //distributor param list
    Eigen::Vector3f thrust_u;     //thrust vector
    Eigen::Vector3f torques_u;    //torque vector
    // for publish
    geometry_msgs::Point thrust_p;
    geometry_msgs::Point torque_p;

    Eigen::Vector3f fe;
    Eigen::Vector3f te;

    //useful param list
    //rotation dynamics inter param list
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
    //
    Eigen::Vector3f x0_w;
    Eigen::Vector3f v0_dot;
    Eigen::Vector3f v0;
    
    double v1,v2,v3;
    double v1_dot,v2_dot,v3_dot;
    double v1_dot_last,v2_dot_last,v3_dot_last;

    double w_x         ,w_y         ,w_z;
    double w_x_dot     ,w_y_dot     ,w_z_dot;
    double w_x_dot_last,w_y_dot_last,w_z_dot_last;

    //input message variable
    geometry_msgs::Point add_thrust;
    geometry_msgs::Point add_torque;

    geometry_msgs::Point ang1;
    geometry_msgs::Point ang2;
    geometry_msgs::Point ang3;
    geometry_msgs::Point ang4;

    std_msgs::Float64 thu1;
    std_msgs::Float64 thu2;
    std_msgs::Float64 thu3;
    std_msgs::Float64 thu4;

    geometry_msgs::Point init_euler_angles;
    geometry_msgs::Point init_position;
    geometry_msgs::Point init_body_rate;
    geometry_msgs::Point init_velocity;


    //output message variable
    geometry_msgs::Point position;
    geometry_msgs::Point euler_angle;
    geometry_msgs::Point body_rate;
    geometry_msgs::Point velocity;

    //subscirber define list
    ros::Subscriber add_thrust_sub;
    ros::Subscriber add_torque_sub;

    ros::Subscriber angle1_sub ;  //angle1_sub
    ros::Subscriber angle2_sub ;  //angle2_sub
    ros::Subscriber angle3_sub ;  //angle3_sub
    ros::Subscriber angle4_sub ;  //angle4_sub
    ros::Subscriber thrust1_sub ; //thrust1_sub
    ros::Subscriber thrust2_sub ; //thrust2_sub
    ros::Subscriber thrust3_sub ; //thrust3_sub
    ros::Subscriber thrust4_sub ; //thrust4_sub
    ros::Subscriber init_euler_angles_cmd_sub ;	//init cmd publish: euler angles
    ros::Subscriber init_pos_cmd_sub ;	        //init cmd publish: position
    ros::Subscriber init_body_rates_cmd_sub ;	//init cmd publish: body_rates
    ros::Subscriber init_velocity_cmd_sub ;	    //init cmd publish: velocity
    ros::Subscriber start_sub;
    ros::Subscriber control_start_sub_att;

    //publisher define list
    ros::Publisher pos_pub;
    ros::Publisher euler_pub;
    ros::Publisher body_rate_pub;
    ros::Publisher velocity_pub;

    ros::Publisher thrust_pub; 
    ros::Publisher torque_pub;

    void add_thrust_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void add_torque_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    
    void angle1_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void angle2_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void angle3_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void angle4_sub_cb(const geometry_msgs::Point::ConstPtr& msg);

    void thrust1_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void thrust2_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void thrust3_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void thrust4_sub_cb(const std_msgs::Float64::ConstPtr& msg);

    void init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg);
    void mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg);
    
    //basic function list
    void init_publisher();
    void init_subscriber();
    void output_publish();
    Eigen::Matrix3f euler_to_rotation_mat(geometry_msgs::Point angle);
    //own function list
    Eigen::Matrix3f pos_mat(double x,double y,double z);

    //useful member function list
    void composition();
    void system_plant();
    double integrator(double input,double orign,double *ls,double t);//new integrator function

    //second level function list
    void rotation_kinematics();
    void translational_kinematics();
    void rotation_dynamics();
    void translational_dynamics();

    //get rand data
    double getRandData(double min,double max);
    //third level function list
    // void kinematics();
    // void v2x_dot();
public:
    ros::NodeHandle nh;

    plant_sim(ros::NodeHandle* nodehandle);

    ros::Timer calc_timer;
    void calc_cb(const ros::TimerEvent&);
    ~plant_sim();
};

plant_sim::~plant_sim()
{
}

#endif
