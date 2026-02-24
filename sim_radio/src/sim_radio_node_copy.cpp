// kaidi wang , 2022.5.29
// simulate radio signal
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
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
// mavros include, radio input file
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sim_radio/get_master_make_m_node.h>

#define Time 0.01
// #define PI 3.1415;
double delta_time = Time;
// init data var list
geometry_msgs::Point pos_data;
geometry_msgs::Point euler_data;
geometry_msgs::Point vel_data;
geometry_msgs::Point body_rate_data;
// useful var list call in the while(1) section
geometry_msgs::Point pos_speed;
geometry_msgs::Point angle_speed;
// path output var
geometry_msgs::Point position_output;
geometry_msgs::Point angle_output;
// velocity output var
geometry_msgs::Point velocity_out;
geometry_msgs::Point body_rate_out;
// path output var
geometry_msgs::Point position_output_ls;
geometry_msgs::Point angle_output_ls;

// define the double val for store last output value
double p_x_ls =0;
double p_y_ls =0;
double p_z_ls =0;

double a_x_ls =0;
double a_y_ls =0;
double a_z_ls =0;
// end point var
geometry_msgs::Point end_position;
geometry_msgs::Point end_angle;
double rect_width = 4.0;      // 矩形宽度
double rect_height = 3.0;     // 矩形高度  
double rect_x_center = 0.0;   // 矩形中心x
double rect_y_center = 0.0;   // 矩形中心y
double rect_z_height = -1.0;  // 飞行高度
double rect_speed = 0.5;      // 飞行速度
double rect_progress = 0.0;    // 轨迹进度
double rect_last_progress = 0.0; // 上一次进度
// signal input var list
std_msgs::Bool start;
std_msgs::Bool mode;

// outloop trigger message
std_msgs::Bool outloop_tri;
// innerloop trigger message
std_msgs::Bool innerloop_tri;

// get rc value and 
mavros_msgs::RCIn get_rc_channel;
int sw_arm;
int sw_auto_land;
int sw_kill;
int thrust;
int switch_yaw_xyz[4];//xyz,yaw
int change_roll_pitch;//change roll pitch
int back_roll_pitch;
int control_mode_switch;
int switch_position_attitude=0;
sw_mapping sw;//define the switch class var

// global var for store raw position and attitude data
geometry_msgs::PoseStamped local_pos;
geometry_msgs::TwistStamped local_vel;
mavros_msgs::State current_state;

// geometry_msgs::Point calc_circle(double r, double x_ori, double y_ori, 
//double omega_theta, double origin_theta,double theta_ls, double fly_height,double t)
double theta=0;
double radius = 2;
double x_center = 1;
double y_center = 1; 
double origin_ang = 1.5*PI;
double angle_ls=0;
double fly_h = 0;
double omega_ang = 1.0;//0.5;
// integrator function
double integrator(
    double input,
    double orign,
    double *ls,
    double t)
{
	double output;
	*ls = *ls+input*t;
    // ROS_INFO_STREAM("last_sum:"<<*ls);
	//last_sum = input;
	output = *ls+orign;
	return output;
}

// outloop callback function 
void outloop_cb(const std_msgs::Bool::ConstPtr& msg)
{
	outloop_tri = *msg;
	// ROS_INFO_STREAM("outloop trigger signal :"<<outloop_tri);
}
//current state callback function
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}
// position callback function 
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
	//ROS_INFO_STREAM("enu pos x: "<<local_pos.pose.position.x);
	//ROS_INFO_STREAM("enu pos y: "<<local_pos.pose.position.y);
	//ROS_INFO_STREAM("enu pos z: "<<local_pos.pose.position.z);
}
//local velocity callback function
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	local_vel = *msg;
}

// the linear path generate function
geometry_msgs::Point calc_integral(
    geometry_msgs::Point input,
    geometry_msgs::Point orign,
    //geometry_msgs::Point *ls,
    double *x_ls,
    double *y_ls,
    double *z_ls,
    double t)
{
    geometry_msgs::Point int_result;
    // (*ls).x = (*ls).x+input.x*t;
    // (*ls).y = (*ls).y+input.y*t;
    // (*ls).z = (*ls).z+input.z*t;
    
    *x_ls = *x_ls+input.x*t;
    *y_ls = *y_ls+input.y*t;
    *z_ls = *z_ls+input.z*t;

    // int_result.x = (*ls).x+orign.x;
    // int_result.y = (*ls).y+orign.y;
    // int_result.z = (*ls).z+orign.z;

    int_result.x = *x_ls+orign.x;
    int_result.y = *y_ls+orign.y;
    int_result.z = *z_ls+orign.z;
    // ROS_INFO_STREAM("---------------------");
    // ROS_INFO_STREAM(orign.y);
    // ROS_INFO_STREAM("---------------------");
    // ROS_INFO_STREAM("---------------------");

    return int_result;
}

// generate circle path function
// parametric equation
geometry_msgs::Point calc_circle(double r, double x_ori, double y_ori, 
    double omega_theta, double origin_theta,double theta_ls, double fly_height,double t)
{
    geometry_msgs::Point circle_path;
    theta = integrator(omega_theta,origin_theta,&angle_ls,t);
    ROS_INFO_STREAM("theta: "<<theta);

    circle_path.x = x_ori + r*cos(theta);
    circle_path.y = y_ori + r*sin(theta);
    circle_path.z = -0.1;

    double v_linear=0;
    v_linear = r*omega_theta;
    // velocity output
    velocity_out.x = v_linear*cos(theta-origin_theta);
    velocity_out.y = v_linear*sin(theta-origin_theta);
    velocity_out.z = 0;
    // body rate set to zero
    body_rate_out.x = body_rate_out.y= body_rate_out.z = 0;
    return circle_path;
}
// 添加矩形轨迹生成函数
geometry_msgs::Point calc_rectangular(
    double width,
    double height,
    double x_center,
    double y_center,
    double z_height,
    double speed,
    double& progress,
    double& last_progress,
    double t)
{
    geometry_msgs::Point rect_path;

    const double perimeter = 2.0 * (width + height);

    // last_progress 作为沿周长走过的距离 s，单位 m
    last_progress += speed * t;
    while (last_progress >= perimeter) last_progress -= perimeter;
    while (last_progress < 0.0) last_progress += perimeter;

    const double s = last_progress;
    progress = s / perimeter * 4.0;  // 保留你原先的 0-4 表示法，方便观察

    int side = 0;
    double seg = 0.0;

    // 四条边长度依次为：height, width, height, width
    if (s < height) {
        side = 0;                       // 右边：下->上
        seg = s / height;
        rect_path.x = x_center + width / 2.0;
        rect_path.y = y_center - height / 2.0 + height * seg;
        velocity_out.x = 0.0;
        velocity_out.y = speed;
    } else if (s < height + width) {
        side = 1;                       // 上边：右->左
        seg = (s - height) / width;
        rect_path.x = x_center + width / 2.0 - width * seg;
        rect_path.y = y_center + height / 2.0;
        velocity_out.x = -speed;
        velocity_out.y = 0.0;
    } else if (s < 2.0 * height + width) {
        side = 2;                       // 左边：上->下
        seg = (s - height - width) / height;
        rect_path.x = x_center - width / 2.0;
        rect_path.y = y_center + height / 2.0 - height * seg;
        velocity_out.x = 0.0;
        velocity_out.y = -speed;
    } else {
        side = 3;                       // 下边：左->右
        seg = (s - 2.0 * height - width) / width;
        rect_path.x = x_center - width / 2.0 + width * seg;
        rect_path.y = y_center - height / 2.0;
        velocity_out.x = speed;
        velocity_out.y = 0.0;
    }

    rect_path.z = z_height;
    velocity_out.z = 0.0;

    body_rate_out.x = 0.0;
    body_rate_out.y = 0.0;
    body_rate_out.z = 0.0;



    return rect_path;
}
// this is not a class function 
// so define a new function for six-dimensional time-varying trajectory
void six_d_time_var_traj(
    double r, // x,y radius
    double x_ori, // x center coordinate
    double y_ori, // y center coordinate
    double omega_theta, // x,y angular velocity
    double origin_theta, // x,y origin angular
    double theta_ls,  // x,y theta last value
    double fly_height, 
    double t // delta time
    ) 
{
    // x,y circle trajectory
    theta = integrator(omega_theta,origin_theta,&angle_ls,t);
    // position nominal , x,y
    position_output.x = x_ori ;//+ r*cos(theta);
    position_output.y = y_ori ;//+ r*sin(theta);
    double v_linear=0;
    v_linear = r*omega_theta;
    // velocity nominal , x,y
    velocity_out.x = 0;//v_linear*cos(theta-origin_theta);
    velocity_out.y = 0;//v_linear*sin(theta-origin_theta);

    // z trajectory, sin
    position_output.z = sin(theta);
    velocity_out.z = cos(theta);


}

// calc time variable path 
void calc_time_var_path(
    geometry_msgs::Point linear_speed,
    geometry_msgs::Point rot_speed,
    geometry_msgs::Point init_p,
    geometry_msgs::Point init_a,
    geometry_msgs::Point ls_p,
    geometry_msgs::Point ls_a,
    geometry_msgs::Point end_p,
    geometry_msgs::Point end_a,
    double t
    )
{

    /*
    double p_x_ls =0;
    double p_y_ls =0;
    double p_z_ls =0;

    double a_x_ls =0;
    double a_y_ls =0;
    double a_z_ls =0;
    */
    velocity_out =linear_speed;
    body_rate_out=rot_speed ;

    position_output = calc_integral(linear_speed,init_p,&p_x_ls,&p_y_ls,&p_z_ls,t);
    angle_output = calc_integral(rot_speed,init_a,&a_x_ls,&a_y_ls,&a_z_ls,t);

    // ROS_INFO_STREAM("init pos:"<<init_p);
    // ROS_INFO_STREAM("init att:"<<init_a);

    // judge wether reach the position end point
    if(position_output.x>=end_p.x)
    {
        position_output.x = end_p.x;
        velocity_out.x=0;
    }
    if(position_output.y>=end_p.y)
    {
        position_output.y = end_p.y;
        velocity_out.y=0;
    }
    if(position_output.z>=end_p.z)
    {
        position_output.z = end_p.z;
        velocity_out.z=0;
    }

    // judge wether reach the euler angle end point
    if(angle_output.x>=end_a.x)
    {
        angle_output.x = end_a.x;
        body_rate_out.x=0;
    }
    if(angle_output.y>=end_a.y)
    {
        angle_output.y = end_a.y;
        body_rate_out.y=0;
    }
    if(angle_output.z>=end_a.z)
    {
        angle_output.z = end_a.z;
        body_rate_out.z=0;
    }
}


// init position, euler angle , velocity and body rate data
int init_data()
{
    int ret=1;
    // pos_data.x = pos_data.y = pos_data.z =0;//1
    // 矩形起点选右下角，与 calc_rectangular 的 side=0 起点一致
pos_data.x = rect_x_center + rect_width / 2.0;
pos_data.y = rect_y_center - rect_height / 2.0;
pos_data.z = rect_z_height;

    euler_data.x = euler_data.y = 0;
    euler_data.z = 0;//0.01

    vel_data.x = vel_data.y = vel_data.z =0;
    body_rate_data.x = body_rate_data.y = body_rate_data.z =0;
    
    pos_speed.x = pos_speed.y = pos_speed.z =0;
    angle_speed.x = angle_speed.y = angle_speed.z =0;
    position_output.x = position_output.y = position_output.z =0;
    angle_output.x = angle_output.y = angle_output.z =0;


    position_output_ls.x = position_output_ls.y = position_output_ls.z =0;
    angle_output_ls.x = angle_output_ls.y = angle_output_ls.z =0;

    pos_speed.x = 0.1;//0.5;
    pos_speed.y = 0.1;
    pos_speed.z = 0.2;

    angle_speed.x = 0.05;//0.05;
    angle_speed.y = 0.05;//0.1;
    angle_speed.z = 0.05;
    // 添加矩形轨迹参数
    rect_width = 4.0;      // 矩形宽度4米
    rect_height = 3.0;     // 矩形高度3米
    rect_x_center = 0.0;   // 中心在原点
    rect_y_center = 0.0;
    rect_z_height = -1.0;  // 飞行高度1米
    rect_speed = 0.5;      // 飞行速度0.5米/秒
    rect_progress = 0.0;
    rect_last_progress = 0.0;
    
    end_position.x= pos_data.x+0;//+5;
    end_position.y= pos_data.y+0;
    end_position.z= pos_data.z-1;

    end_angle.x = euler_data.x+0;//+0.1;
    end_angle.y = euler_data.y+0;//+0.8;
    end_angle.z = euler_data.z+0;
    return ret;
}

//callback function of get_rc_channel_cb
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;
	sw_arm = get_rc_channel.channels[6];
	// sw_auto_land = get_rc_channel.channels[5];
	sw_auto_land = get_rc_channel.channels[7];
	sw_kill = get_rc_channel.channels[6];

	//map a channel of RC to the vel, the channel number should be changed
	switch_position_attitude = get_rc_channel.channels[4];
	//get the thrust volume
	thrust = get_rc_channel.channels[2];
	for(int i =0 ; i<4; i++)
	{
		switch_yaw_xyz[i]=get_rc_channel.channels[i];
	}
	change_roll_pitch = get_rc_channel.channels[13];
	back_roll_pitch = get_rc_channel.channels[12];

	//kaidi wang 2021.10.13, map the get_rc_channel.channels[13] to control_mode_switch
	control_mode_switch = get_rc_channel.channels[8];
    // ROS_INFO_STREAM("-----strick value------");
	// ROS_INFO_STREAM("channel 1: "<<get_rc_channel.channels[0]<<" "<<switch_yaw_xyz[0]);	 	 
	// ROS_INFO_STREAM("channel 2: "<<get_rc_channel.channels[1]<<" "<<switch_yaw_xyz[1]);
	// ROS_INFO_STREAM("channel 3: "<<get_rc_channel.channels[2]<<" "<<switch_yaw_xyz[2]);
	// ROS_INFO_STREAM("channel 4: "<<get_rc_channel.channels[3]<<" "<<switch_yaw_xyz[3]);
	// ROS_INFO_STREAM("-----function value-----");
	// ROS_INFO_STREAM("channel 7: "<<get_rc_channel.channels[6]);
	// ROS_INFO_STREAM("channel 9: "<<get_rc_channel.channels[8]);
	// ROS_INFO_STREAM("channel 5: "<<get_rc_channel.channels[4]);
}


// main function 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_radio_node");
	ros::NodeHandle nh;

    //publish list init position and euler angle data
	ros::Publisher init_euler_angles_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_euler_angles_cmd",100);
	//init cmd publish: position
	ros::Publisher init_pos_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_position_cmd",100);
	//init cmd publish: body_rates
	ros::Publisher init_body_rates_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_body_rates_cmd",100);
	//init cmd publish: velocity
	ros::Publisher init_velocity_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_velocity_cmd",100);
	
    //nominal_position
	//ros::Publisher nominal_position_pub = nh.advertise<geometry_msgs::Point>("/original_position",100);
	//nominal_eular_angles
	//ros::Publisher nominal_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/original_euler_angles",100);

    //nominal_position
	ros::Publisher nominal_position_pub = nh.advertise<geometry_msgs::Point>("/nominal_position",100);
	//nominal_eular_angles
	ros::Publisher nominal_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/nominal_euler_angles",100);

    //nominal_position
	ros::Publisher nominal_velocity_pub = nh.advertise<geometry_msgs::Point>("/nominal_velocity",100);
	//nominal_eular_angles
	ros::Publisher nominal_body_rate_pub = nh.advertise<geometry_msgs::Point>("/nominal_body_rate",100);

    // start outer loop
    ros::Publisher control_start_pub_att = nh.advertise<std_msgs::Bool>("start_pub_att",100);
    // start inner loop
	ros::Publisher control_mode_switch_pub = nh.advertise<std_msgs::Bool>("/mode_switch",10);
    // get the outloop trigger signal
	ros::Subscriber outloop_sub = nh.subscribe<std_msgs::Bool>("/out_tri",10,outloop_cb);
    // subscribe RC channels value
    ros::Subscriber get_rc_channel_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",100,get_rc_channel_cb);
    // subscribe position and attitude value from the PX4 sensor
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
	("mavros/local_position/pose", 10, local_pos_cb);
	//ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose",10,local_pos_cb);
	//subscriber local_velocity , NWU frame of body rate
	ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
	("mavros/local_position/velocity_body",10,local_vel_cb);
    //PX4 state subscribe
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);
    ros::Rate rate(100.0);

    outloop_tri.data = false;
    innerloop_tri.data = false;
    mode.data = false;
    int init = init_data();

    // wait the PX4 to be connected
    // while (ros::ok() && !current_state.connected)
    // {
	// 	ROS_INFO("px4 is unconnected...");
	// 	ros::spinOnce();
	// 	rate.sleep();
    // }
    

    // input start signal
    // input a signal as a trigger input and then publish init data 
    while (ros::ok()&&outloop_tri.data == false)
    {
        ROS_INFO_STREAM("wait the outloop trigger signal and then publish init data...");
        init_pos_cmd_pub.publish(pos_data);
        init_euler_angles_cmd_pub.publish(euler_data);
        init_velocity_cmd_pub.publish(vel_data);
        init_body_rates_cmd_pub.publish(body_rate_data);
        rate.sleep();
        start.data = true;
        //mode.data = true;
        ros::spinOnce();
    }
    
    // ROS_INFO_STREAM("Whether input start_pub_att:Y/n");
    // std::string start_pub_str;
    // std::getline(std::cin,start_pub_str);
    // if (start_pub_str == "Y")
    // {
    //     start.data = true;
    // }
    // else
    // {
    //     start.data = false;
    // }
    // // input mode signal
    // ROS_INFO_STREAM("Whether input mode_switch_pub:Y/n");
    // std::string mode_pub_str;
    // std::getline(std::cin,mode_pub_str);
    // if (start_pub_str == "Y")
    // {
    //     mode.data = true;
    // }
    // else
    // {
    //     mode.data = false;
    // }


    // the main loop
    while (ros::ok()&&mode.data == false)
    {
        // call the calc path function and then publish it 
        control_start_pub_att.publish(start);
        //control_mode_switch_pub.publish(mode);

        // keep roll and pitch and we can change positon and yaw angle
        if(switch_position_attitude<1500)
        {
            sw.sw_map_vel(switch_yaw_xyz);
        }
        else{
            //keep x and y and we can change attitude and z height
            sw.sw_map_ang_vel(switch_yaw_xyz);
        }
        // ROS_INFO_STREAM(sw.vel_setpoint);// velocity setpoint
        // ROS_INFO_STREAM(sw.body_rate_setpoint);// body rate setpoint
        
        // calc time variable trajectory
        // calc_time_var_path(
        //     pos_speed,//sw.vel_setpoint,
        //     angle_speed,//sw.body_rate_setpoint,
        //     pos_data,
        //     euler_data,
        //     position_output_ls,
        //     angle_output_ls,
        //     end_position,
        //     end_angle,
        //     delta_time
        // );
//    使用矩形轨迹
    position_output = calc_rectangular(
        rect_width,          // 宽度
        rect_height,         // 高度
        rect_x_center,       // 中心x
        rect_y_center,       // 中心y
        rect_z_height,       // 高度
        rect_speed,          // 速度
        rect_progress,       // 进度
        rect_last_progress,  // 上一次进度
        delta_time           // 时间步长
    );
        // position_output=calc_circle(radius, x_center, y_center, omega_ang, origin_ang,angle_ls, fly_h,delta_time);



        // creat six dimension 
        // six_d_time_var_traj(radius, x_center, y_center,
        // omega_ang, origin_ang,angle_ls, fly_h,delta_time);
        // position_output.x =0.5;
        // position_output.y =0.5;
        // position_output.z =0.5;

        // angle_output.x = 0;
        // angle_output.y = 0;
        // angle_output.z = 0;

        nominal_position_pub.publish(position_output);
        nominal_eular_angles_pub.publish(angle_output);
        nominal_velocity_pub.publish(velocity_out);
        nominal_body_rate_pub.publish(body_rate_out);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    // this node 
    // input speed and then output position and euler angle 
    // also need to define the end position and euler angle
    ros::spin();

    return 0;
}