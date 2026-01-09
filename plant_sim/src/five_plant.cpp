//kaidi wang
// 2023.11.28
//define a class and declearation of class member
#include <plant_sim/four_plant.h>

//basic function list
//init publisher function
void plant_sim::init_publisher()
{
// real radio signal section
	pos_pub   = nh.advertise<geometry_msgs::Point>("/main_position",10,this);
	euler_pub = nh.advertise<geometry_msgs::Point>("/main_euler_angles",10,this);
	body_rate_pub = nh.advertise<geometry_msgs::Point>("/main_body_rates",10,this);
	velocity_pub  = nh.advertise<geometry_msgs::Point>("/main_velocity",10,this);

    // simulation radio signal section only for h-infinity controller
    // pos_pub = nh.advertise<geometry_msgs::Point>("/main_position",10,this);
	// euler_pub = nh.advertise<geometry_msgs::Point>("/main_euler_angles",10,this);
	// body_rate_pub = nh.advertise<geometry_msgs::Point>("/main_body_rates",10,this);
	// velocity_pub = nh.advertise<geometry_msgs::Point>("/main_velocity",10,this);

    thrust_pub = nh.advertise<geometry_msgs::Point>("/thrust_pub",10,this); 
    torque_pub = nh.advertise<geometry_msgs::Point>("/torque_pub",10,this);
}

//init subscriber function
void plant_sim::init_subscriber()
{
    add_thrust_sub = nh.subscribe<geometry_msgs::Point>
    ("/add_thrust",1,&plant_sim::add_thrust_sub_cb,this);
    add_torque_sub = nh.subscribe<geometry_msgs::Point>
    ("/add_torque",1,&plant_sim::add_torque_sub_cb,this);

    angle1_sub = nh.subscribe<geometry_msgs::Point>
	("/angle1",1,&plant_sim::angle1_sub_cb,this);
    angle2_sub = nh.subscribe<geometry_msgs::Point>
	("/angle2",1,&plant_sim::angle2_sub_cb,this);
    angle3_sub = nh.subscribe<geometry_msgs::Point>
	("/angle3",1,&plant_sim::angle3_sub_cb,this);
    angle4_sub = nh.subscribe<geometry_msgs::Point>
	("/angle4",1,&plant_sim::angle4_sub_cb,this);

    thrust1_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust1",1,&plant_sim::thrust1_sub_cb,this);
    thrust2_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust2",1,&plant_sim::thrust2_sub_cb,this);
    thrust3_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust3",1,&plant_sim::thrust3_sub_cb,this);
    thrust4_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust4",1,&plant_sim::thrust4_sub_cb,this);

	init_euler_angles_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_euler_angles_cmd",1,&plant_sim::init_euler_angles_cmd_sub_cb,this);
	init_pos_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_position_cmd",1,&plant_sim::init_pos_cmd_sub_cb,this);
	init_body_rates_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_body_rates_cmd",1,&plant_sim::init_body_rates_cmd_sub_cb,this);
	init_velocity_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_velocity_cmd",1,&plant_sim::init_velocity_cmd_sub_cb,this);

	control_start_sub_att = nh.subscribe<std_msgs::Bool>
	("/start_pub_att",1,&plant_sim::control_start_sub_att_cb,this);
    start_sub = nh.subscribe<std_msgs::Bool>
    ("/mode_switch", 10, &plant_sim::mode_switch_sub_cb,this);

}
void plant_sim::output_publish()
{
    //updata message data
    position.x = x0_w(0);
    position.y = x0_w(1);
    position.z = x0_w(2);

    if(euler_angles(0)>PI)
    {
        // euler_angles(0) = euler_angles(0)-2*PI;
        euler_angles(0) = fmod(euler_angles(0),2*PI);
    }
    if(euler_angles(0)<-PI)
    {
        // euler_angles(0) = euler_angles(0)+2*PI;
        euler_angles(0) = fmod(euler_angles(0),2*PI);
    }
    euler_angle.x = euler_angles(0);


    if(euler_angles(1)>PI)
    {
        // euler_angles(1) = euler_angles(1)-2*PI;
        euler_angles(1) = fmod(euler_angles(1),2*PI);
    }
    if(euler_angles(1)<-PI)
    {
        // euler_angles(1) = euler_angles(1)+2*PI;
        euler_angles(1) = fmod(euler_angles(1),2*PI);
    }
    euler_angle.y = euler_angles(1);


    if(euler_angles(2)>PI)
    {
        euler_angles(2) = euler_angles(2)-2*PI;
        // euler_angles(2) = fmod(euler_angles(2),2*PI);
    }
    if(euler_angles(2)<-PI)
    {
        euler_angles(2) = euler_angles(2)+2*PI;
        // euler_angles(2) = fmod(euler_angles(2),2*PI);
    }
    // double yaw = (euler_angles(2))%(2*PI);
    euler_angle.z = euler_angles(2);

    //output body rate
    body_rate.x = body_rates(0);
    body_rate.y = body_rates(1);
    body_rate.z = body_rates(2);
    //output velocity
    velocity.x = v0(0);
    velocity.y = v0(1);
    velocity.z = v0(2);
    //publish message ouput
	pos_pub.publish(position) ;
	euler_pub.publish(euler_angle) ;
	body_rate_pub.publish(body_rate) ;
	velocity_pub.publish(velocity);

    // publish thrust 
    thrust_pub.publish(thrust_p); 
    torque_pub.publish(torque_p);
}


void plant_sim::add_thrust_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    add_thrust = *msg;

}
void plant_sim::add_torque_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    add_torque = *msg;
}

//angle1 sub function
void plant_sim::angle1_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    ang1 = *msg;
}

//angle2 sub function
void plant_sim::angle2_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    ang2 = *msg;
}

//angle3 sub function
void plant_sim::angle3_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    ang3 = *msg;
}

void plant_sim::angle4_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    ang4 = *msg;
}

//thrust1 sub function
void plant_sim::thrust1_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
    thu1 = *msg;
}

//thrust2 sub function
void plant_sim::thrust2_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
    thu2 = *msg;
}

//thrust3 sub function
void plant_sim::thrust3_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
    thu3 = *msg;
}

void plant_sim::thrust4_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
    thu4 = *msg;
}

//init euler cmd function
void plant_sim::init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    init_euler_angles = *msg;
}

//init postion cmd function
void plant_sim::init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    init_position = *msg;
}

//init body rates cmd function
void plant_sim::init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    init_body_rate = *msg;
}

//init velocity cmd function
void plant_sim::init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    init_velocity = *msg;
}  

//control start chk
void plant_sim::control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg)
{
	start_pub_att = *msg;
}

//mode switch subcribe callback function
void plant_sim::mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
    // ROS_INFO_STREAM("........");
    mode_switch = *msg;
    if (mode_switch.data)
    {
        // ROS_INFO_STREAM("plant start...");
    }
    else
    {
        // ROS_INFO_STREAM("plant not start...");
    }
}


// compostion function
void plant_sim::composition()
{
    //calc attitude rotation
    Eigen::Matrix3f r_mat0, rt_mat0, rmat1,rmat2,rmat3,rmat4;//main_euler_angle rotation mat
    r_mat0 = euler_to_rotation_mat(euler_angle);
    rt_mat0 = r_mat0.inverse();//inverse function of rotation matrix
    //calc child vehicle rotation 
    rmat1 = euler_to_rotation_mat(ang1);
    rmat2 = euler_to_rotation_mat(ang2);
    rmat3 = euler_to_rotation_mat(ang3);
    rmat4 = euler_to_rotation_mat(ang4);

    //get vector of thrust1~3
    Eigen::Vector3f vec_thu1,vec_thu2,vec_thu3,vec_thu4;
    vec_thu1(0)=vec_thu1(1)=0;
    vec_thu1(2)=-thu1.data;

    vec_thu2(0)=vec_thu2(1)=0;
    vec_thu2(2)=-thu2.data;

    vec_thu3(0)=vec_thu3(1)=0;
    vec_thu3(2)=-thu3.data;

    vec_thu4(0)=vec_thu4(1)=0;
    vec_thu4(2)=-thu4.data;

    //
    Eigen::Vector3f fw1,fw2,fw3,fw4,f01,f02,f03,f04;
    fw1 = rmat1*vec_thu1;
    f01 = rt_mat0*fw1;

    fw2 = rmat2*vec_thu2;
    f02 = rt_mat0*fw2;

    fw3 = rmat3*vec_thu3;
    f03 = rt_mat0*fw3;

    fw4 = rmat4*vec_thu4;
    f04 = rt_mat0*fw4;
    //define identity matrix3f
    Eigen::Matrix3f I;
	I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;
    //define position s1~3
	pos_s1=pos_mat(param.fly1_pos.x,param.fly1_pos.y,param.fly1_pos.z);
	pos_s2=pos_mat(param.fly2_pos.x,param.fly2_pos.y,param.fly2_pos.z);
	pos_s3=pos_mat(param.fly3_pos.x,param.fly3_pos.y,param.fly3_pos.z);
	pos_s4=pos_mat(param.fly4_pos.x,param.fly4_pos.y,param.fly4_pos.z);

    //define B matrix
    Eigen::MatrixXf B(6, 12);
	B.block<3,3>(0,0)=I;
	B.block<3,3>(0,3)=I;
	B.block<3,3>(0,6)=I;
	B.block<3,3>(0,9)=I;

	B.block<3,3>(3,0)=pos_s1;
	B.block<3,3>(3,3)=pos_s2;
	B.block<3,3>(3,6)=pos_s3;
	B.block<3,3>(3,9)=pos_s4;

    //make an vector(9)
    Eigen::MatrixXf f01234(12, 1);
    f01234.block<3,1>(0,0)=f01;
	f01234.block<3,1>(3,0)=f02;
	f01234.block<3,1>(6,0)=f03;
    f01234.block<3,1>(9,0)=f04;
    //define a 6x1 vector U
    Eigen::MatrixXf U(6, 1);
    U = B*f01234;
    //get thrust and torques
    thrust_u(0) = U(0);
    thrust_u(1) = U(1);
    thrust_u(2) = U(2);
    torques_u(0) = U(3);
    torques_u(1) = U(4);
    torques_u(2) = U(5);
    thrust_p.x = thrust_u(0);
    thrust_p.y = thrust_u(1);
    thrust_p.z = thrust_u(2);
    torque_p.x = torques_u(0);
    torque_p.y = torques_u(1);
    torque_p.z = torques_u(2);

}

// system plant function
void plant_sim::system_plant()
{
    rotation_kinematics();
    rotation_dynamics();
    translational_kinematics();
    translational_dynamics();
}

//second level function list
void plant_sim::rotation_kinematics()
{
    //calc p q r
    p = integrator(p_dot,0,&p_dot_last,detla_time);
    q = integrator(q_dot,0,&q_dot_last,detla_time);
    r = integrator(r_dot,0,&r_dot_last,detla_time);
    //output body rates
    body_rates(0) = p;
    body_rates(1) = q;
    body_rates(2) = r;

    //kinematics sections
    phi_dot   = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot   = q * sin(phi) * (1/cos(theta)) + r * cos(phi) * (1/cos(theta));

    //integrator attitude angle dot to get euler angles
    phi = integrator(phi_dot, init_euler_angles.x, &phi_dot_last, detla_time);
    theta = integrator(theta_dot, init_euler_angles.y, &theta_dot_last, detla_time);
    psi = integrator(psi_dot, init_euler_angles.z, &psi_dot_last, detla_time);

    //make vectorx3f form
    euler_angles(0) = phi;
    euler_angles(1) = theta;
    euler_angles(2) = psi;
} 

//translational_kinematics function
void plant_sim::translational_kinematics()
{
    //define rotation matrix and inverse of rotation matrix
    Eigen::Matrix3f r_mat0, rt_mat0;//main_euler_angle rotation mat
    r_mat0  = euler_to_rotation_mat(euler_angle);

    //get v1~3_dot value
    v1_dot = v0_dot(0);
    v2_dot = v0_dot(1);
    v3_dot = v0_dot(2);

    //integrator v1~3_dot value to v1~3 value
    v1 = integrator(v1_dot, 0, &v1_dot_last, detla_time);
    v2 = integrator(v2_dot, 0, &v2_dot_last, detla_time);
    v3 = integrator(v3_dot, 0, &v3_dot_last, detla_time);

    //output vector v0
    v0(0) = v1;
    v0(1) = v2;
    v0(2) = v3;
    //calc x_dot
    Eigen::Vector3f x_dot;
    x_dot = r_mat0*v0;

    //calc x0_w vector
    w_x_dot = x_dot(0);
    w_y_dot = x_dot(1);
    w_z_dot = x_dot(2);

    w_x = integrator(w_x_dot, init_position.x, &w_x_dot_last, detla_time);
    w_y = integrator(w_y_dot, init_position.y, &w_y_dot_last, detla_time);
    w_z = integrator(w_z_dot, init_position.z, &w_z_dot_last, detla_time);

    x0_w(0) = w_x;
    x0_w(1) = w_y;
    x0_w(2) = w_z;
}

//rotation dynamics function
void plant_sim::rotation_dynamics()
{
    //get body rates to a loacl var
    Eigen::Vector3f omega;
    omega = body_rates;

    Eigen::Vector3f N;

    te(0) = add_torque.x;
    te(1) = add_torque.y;
    te(2) = add_torque.z;
    ROS_INFO_STREAM("te: "<<te);    
    if(ideal_model)
    {
        N = te+torques_u+(J*omega).cross(omega);
    }
    else
    {
        N = te+torques_u+(J*omega).cross(omega);
    }

    //calc p,q,r
    p_dot = N(0)/param.I_sys.x;
    q_dot = N(1)/param.I_sys.y;
    r_dot = N(2)/param.I_sys.z;
}

// translational_dynamics function
void plant_sim::translational_dynamics()
{
    //calc rotation matrix and inverse of rotation matrix
    Eigen::Matrix3f r_mat0, rt_mat0;//main_euler_angle rotation mat
    r_mat0  = euler_to_rotation_mat(euler_angle);
    rt_mat0 = r_mat0.inverse();
    //get omega vector
    Eigen::Vector3f omega;
    omega = body_rates;
    //get vel vector
    Eigen::Vector3f v;
    v = v0;

    fe(0) = add_thrust.x;
    fe(1) = add_thrust.y;
    fe(2) = add_thrust.z;
    ROS_INFO_STREAM("fe: "<<fe);
    //calc vo_dot value
    if(ideal_model)
    {
        v0_dot = (param.S3Q_mass*G*rt_mat0*e3+fe- param.S3Q_mass*(omega.cross(v))+thrust_u)/param.S3Q_mass;
    }
    else
    {
        v0_dot = (param.S3Q_mass*G*rt_mat0*e3+fe- param.S3Q_mass*(omega.cross(v))+thrust_u)/param.S3Q_mass;
    }

}
//third level function list
// void plant_sim::kinematics()
// {

// }
// void plant_sim::v2x_dot()
// {

// }

//generate pos matrix
Eigen::Matrix3f plant_sim::pos_mat(double x,double y,double z)
{
	Eigen::Matrix3f s;
	s(0,0)=s(1,1)=s(2,2)=0;
	s(0,1)=-z;
	s(0,2)= y;
	s(1,0)= z;
	s(1,2)=-x;
	s(2,0)=-y;
	s(2,1)= x;
	return s;
}

//euler to rotation matrix function
Eigen::Matrix3f plant_sim::euler_to_rotation_mat(geometry_msgs::Point angle)
{
	Eigen::Matrix3f mat;
	mat(0,0)=cos(angle.y)*cos(angle.z);
	mat(0,1)=sin(angle.x)*sin(angle.y)*cos(angle.z)-cos(angle.x)*sin(angle.z);
	mat(0,2)=cos(angle.z)*sin(angle.y)*cos(angle.x)+sin(angle.z)*sin(angle.x);
	mat(1,0)=cos(angle.y)*sin(angle.z);
	mat(1,1)=sin(angle.z)*sin(angle.y)*sin(angle.x)+cos(angle.z)*cos(angle.x);
	mat(1,2)=sin(angle.z)*sin(angle.y)*cos(angle.x)-cos(angle.z)*sin(angle.x);
	mat(2,0)=-sin(angle.y);
	mat(2,1)=sin(angle.x)*cos(angle.y);
	mat(2,2)=cos(angle.x)*cos(angle.y);
	//ROS_INFO_STREAM(mat);
	return mat;
}
//integrator function
double plant_sim::integrator(double input,double orign,double *ls,double t)//new integrator function
{
	double output=0;
	*ls = *ls+input*t;
    // ROS_INFO_STREAM("last_sum:"<<*ls);
	//last_sum = input;
	output = *ls+orign;
	return output;
}

//get rand data
double plant_sim::getRandData(double min,double max)
{
	return min +rand() / double(RAND_MAX/(max -min ));
}
//construction function
plant_sim::plant_sim(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    detla_time = T;
	//define 9.8 as gravity
	g=G;
    //unit vector of z axis
    e3(0) = 0;
    e3(1) = 0;
    e3(2) = 1;

    // ROS_INFO_STREAM("gaoling");
    J(0,0)=param.I_sys.x;
	J(1,1)=param.I_sys.y;
	J(2,2)=param.I_sys.z;
	J(0,1)=J(0,2)=J(1,0)=J(1,2)=J(2,0)=J(2,1)=0;

    p_dot=0;
    q_dot=0;
    r_dot=0;
    p_dot_last=0;
    q_dot_last=0;
    r_dot_last=0;
    p=0,q=0,r=0;

    phi=0,theta=0,psi=0;
    phi_dot=0,theta_dot=0,psi_dot=0;
    phi_dot_last=0,theta_dot_last=0,psi_dot_last=0;

    v1=0,v2=0,v3=0;
    v1_dot=0,v2_dot=0,v3_dot=0;
    v1_dot_last=0,v2_dot_last=0,v3_dot_last=0;

    w_x=0        ,w_y=0        ,w_z=0;
    w_x_dot=0     ,w_y_dot=0     ,w_z_dot=0;
    w_x_dot_last=0,w_y_dot_last=0,w_z_dot_last=0;

    // kaidi wang add this section on 2022.1.19
    // input and output var initialize section
    position.x=0;
    position.y=0;
    position.z=0;
    euler_angle.x=0;
    euler_angle.y=0;
    euler_angle.z=0;
    body_rate.x=0;
    body_rate.y=0;
    body_rate.z=0;
    velocity.x=0;
    velocity.y=0;
    velocity.z=0;
    ang1.x=0;
    ang1.y=0;
    ang1.z=0;
    ang2.x=0;
    ang2.y=0;
    ang2.z=0;
    ang3.x=0;
    ang3.y=0;
    ang3.z=0;
    thu1.data = 0;
    thu2.data = 0;
    thu3.data = 0;
    init_euler_angles.x=0;
    init_euler_angles.y=0;
    init_euler_angles.z=0;
    init_position.x=0;
    init_position.y=0;
    init_position.z=0;
    init_body_rate.x=0;
    init_body_rate.y=0;
    init_body_rate.z=0;
    init_velocity.x=0;
    init_velocity.y=0;
    init_velocity.z=0;

    init_publisher();
    init_subscriber();

    //Add noise and define the range of noise
    ROS_INFO_STREAM("Do you need to add noise:Y/n");
	std::string str,t_x,t_y,t_z,tao_x,tao_y,tao_z;
    //define some double variable to store noise
    double thux,thuy,thuz,taox,taoy,taoz;
	std::getline(std::cin,str);
	if (str == "Y")
	{
        ideal_model = false;
		ROS_INFO_STREAM("Please enter the range of noise.");
		ROS_INFO_STREAM("thrust x direction(N): ");std::getline(std::cin,t_x);
        ROS_INFO_STREAM("thrust y direction(N): ");std::getline(std::cin,t_y);
        ROS_INFO_STREAM("thrust z direction(N): ");std::getline(std::cin,t_z);

        ROS_INFO_STREAM("torque x direction(Nm) ");std::getline(std::cin,tao_x);
        ROS_INFO_STREAM("torque y direction(Nm) ");std::getline(std::cin,tao_y);
        ROS_INFO_STREAM("torque z direction(Nm) ");std::getline(std::cin,tao_z);

        thux=stod(t_x);thuy=stod(t_y);thuz=stod(t_z);
        taox=stod(tao_x);taoy=stod(tao_y);taoz=stod(tao_z);

        fe(0) = getRandData(-thux,thux);
        fe(1) = getRandData(-thuy,thuy);
        fe(2) = getRandData(-thuz,thuz);
        te(0) = getRandData(-taox,taox);
        te(1) = getRandData(-taoy,taoy);
        te(2) = getRandData(-taoz,taoz);

	}
	else
	{
        ideal_model = true;
		ROS_INFO_STREAM("Run the ideal model");
	}


    calc_timer = nh.createTimer(ros::Duration(T), &plant_sim::calc_cb, this);
}

//timer callback function 
void plant_sim::calc_cb(const ros::TimerEvent&)
{
    ROS_INFO_STREAM("four plant...");
    if(start_pub_att.data)
    {
        // if(mode_switch.data)
        {

            //kaidi wang code this section on
            if((thu1.data!=0)&&(thu2.data!=0)&&(thu3.data!=0))
            {
                composition();
                system_plant();
                //output message 
                output_publish();
                ROS_INFO_STREAM("into plant");
            }
            else
            {
                pos_pub.publish(position) ;
                euler_pub.publish(euler_angle) ;
                body_rate_pub.publish(body_rate) ;
                velocity_pub.publish(velocity);
                // ROS_INFO_STREAM("init section..");
                position = init_position;
                euler_angle = init_euler_angles;
                body_rate = init_body_rate;
                velocity = init_velocity;
                // ROS_INFO_STREAM("position: "<<position.x<<" "<<position.y<<" "<<position.z);
                // ROS_INFO_STREAM("euler_angle: "<<euler_angle.x<<" "<<euler_angle.y<<" "<<euler_angle.z);
                // ROS_INFO_STREAM("body_rate: "<<body_rate.x<<" "<<body_rate.y<<" "<<body_rate.z);
                // ROS_INFO_STREAM("velocity: "<<velocity.x<<" "<<velocity.y<<" "<<velocity.z);
            }
        }
        // else
        {
            // ROS_INFO_STREAM("init section..");
            // position = init_position;
            // euler_angle = init_euler_angles;
            // body_rate = init_body_rate;
            // velocity = init_velocity;

            // ROS_INFO_STREAM("position: "<<position.x<<" "<<position.y<<" "<<position.z);
            // ROS_INFO_STREAM("euler_angle: "<<euler_angle.x<<" "<<euler_angle.y<<" "<<euler_angle.z);
            // ROS_INFO_STREAM("body_rate: "<<body_rate.x<<" "<<body_rate.y<<" "<<body_rate.z);
            // ROS_INFO_STREAM("velocity: "<<velocity.x<<" "<<velocity.y<<" "<<velocity.z);

            // pos_pub.publish(init_position);
            // euler_pub.publish(init_euler_angles);
            // body_rate_pub.publish(init_body_rate);
            // velocity_pub.publish(init_velocity);
        }
    }
    
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc,argv,"plant_Sim");
    ros::NodeHandle nh;
    plant_sim plant_sim_node(&nh);

    ros::spin();
    return 0;
}