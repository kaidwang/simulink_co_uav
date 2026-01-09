// kaidi wang, 2023.3.28
// define a class and declearation of class member
// linear plant simulation

#include <plant_sim/linear_plant.h>


namespace special_mat
{
    // define special matrix 
    Eigen::MatrixXf M(6,6); // mass matrix 
    Eigen::MatrixXf B(6,9); // B matrix related to sub-aricrafts position
    Eigen::MatrixXf C(6,6); // Coriolis matrix 
    // Eigen::VectorXf G(6); // gravity effect
    Eigen::MatrixXf omega_a(5,6); // omega_a matrix
    Eigen::MatrixXf omega_at(6,5); 
    Eigen::MatrixXf omega_ut(6, 1);

    Eigen::MatrixXf delta_a(6,5); // delta_a matrix
    Eigen::MatrixXf omega_u(1,6); // omega_u matrix
    Eigen::MatrixXf delta_u(6,1); // delta_u matrix
    Eigen::VectorXf v_a(5);   // va 
    Eigen::VectorXf v_u(1);   // vu

    Eigen::MatrixXf Ma(5,5);  // Ma=delta_a.T*M*delta_a
    Eigen::MatrixXf Mu(1,1);

    Eigen::MatrixXf Ca(5,5);  // Ca
    Eigen::MatrixXf Cu(1,1); 
    Eigen::MatrixXf Cau(5,1); // Cau
    Eigen::MatrixXf Cua(1,1);
    

    Eigen::MatrixXf delta(6,6); // delta matrix 6x6
    Eigen::MatrixXf omega(6,6); // omega matrix 6x6

    Eigen::VectorXf state_v(6); // define vc and w0 vector as state_v
    Eigen::VectorXf u_a(5); // ua
    Eigen::VectorXf u_u(1); // uu

    Eigen::VectorXf f_a(5); // fa
    Eigen::VectorXf f_u(1); // fu

    Eigen::VectorXf g_a(5);  // ga
    Eigen::VectorXf g_u(1);  // gu

    Eigen::VectorXf dot_va(5); // dot va
    Eigen::VectorXf dot_vu(1);
}

//basic function list
//init publisher function
void plant_sim::init_publisher()
{
// real radio signal section
	pos_pub   = nh.advertise<geometry_msgs::Point>("/main_position_sim",10,this);
	euler_pub = nh.advertise<geometry_msgs::Point>("/main_euler_angles_sim",10,this);
	body_rate_pub = nh.advertise<geometry_msgs::Point>("/main_body_rates_sim",10,this);
	velocity_pub  = nh.advertise<geometry_msgs::Point>("/main_velocity_sim",10,this);

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
    angle1_sub = nh.subscribe<geometry_msgs::Point>
	("/angle1",1,&plant_sim::angle1_sub_cb,this);
    angle2_sub = nh.subscribe<geometry_msgs::Point>
	("/angle2",1,&plant_sim::angle2_sub_cb,this);
    angle3_sub = nh.subscribe<geometry_msgs::Point>
	("/angle3",1,&plant_sim::angle3_sub_cb,this);

    thrust1_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust1",1,&plant_sim::thrust1_sub_cb,this);
    thrust2_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust2",1,&plant_sim::thrust2_sub_cb,this);
    thrust3_sub = nh.subscribe<std_msgs::Float64>
    ("/thrust3",1,&plant_sim::thrust3_sub_cb,this);

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


void plant_sim::init_mat()// calc delta_a matrix
{
    zero(0,0)=zero(1,1)=zero(2,2)=0;
	zero(0,1)=zero(0,2)=zero(1,0)=zero(1,2)=zero(2,0)=zero(2,1)=0;

    I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;
    Eigen::Matrix3f pos_s1;      //position matrix
    Eigen::Matrix3f pos_s2;
    Eigen::Matrix3f pos_s3;

	pos_s1=pos_mat(param.fly1_pos.x,param.fly1_pos.y,param.fly1_pos.z);
	pos_s2=pos_mat(param.fly2_pos.x,param.fly2_pos.y,param.fly2_pos.z);
	pos_s3=pos_mat(param.fly3_pos.x,param.fly3_pos.y,param.fly3_pos.z);
    // define M matrix
    special_mat::M.block<3,3>(0,0) = param.S3Q_mass*I;
	special_mat::M.block<3,3>(0,3) = zero;
    special_mat::M.block<3,3>(3,0) = zero;
	special_mat::M.block<3,3>(3,3) = J;
    //special_mat::M.block<3,3>(3,3) = J_bar;

    // define B matrix
    
    special_mat::B.block<3,3>(0,0)=I;
	special_mat::B.block<3,3>(0,3)=I;
	special_mat::B.block<3,3>(0,6)=I;
	special_mat::B.block<3,3>(3,0)=pos_s1;
	special_mat::B.block<3,3>(3,3)=pos_s2;
	special_mat::B.block<3,3>(3,6)=pos_s3;

    // define omega_a transpose
    // special_mat::omega_a.transpose() = special_mat::B.block(0,0,6,5);
    special_mat::omega_a.transpose().block<6,1>(0,0) = special_mat::B.block(0,0,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,1) = special_mat::B.block(0,1,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,2) = special_mat::B.block(0,2,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,3) = special_mat::B.block(0,4,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,4) = special_mat::B.block(0,5,6,1);
    // define delta_u 
    special_mat::delta_u(0) = 0;
    special_mat::delta_u(1) = param.fly1_pos.z;
    special_mat::delta_u(2) = 0;
    special_mat::delta_u(3) = 1;
    special_mat::delta_u(4) = 0;
    special_mat::delta_u(5) = 0;
    // calc delta_a 
    Eigen::MatrixXf inter_a(5, 5);
    inter_a = special_mat::omega_a*special_mat::M.inverse()*special_mat::omega_a.transpose();
    special_mat::delta_a = special_mat::M.inverse()*special_mat::omega_a.transpose()*inter_a.inverse();
    //special_mat::delta_a = special_mat::M.inverse()*special_mat::omega_a.transpose()*((special_mat::omega_a*special_mat::M.inverse()*special_mat::omega_a.transpose()).inverse());
    // define Ma Ca Ga
    special_mat::Ma = special_mat::delta_a.transpose()*special_mat::M*special_mat::delta_a;
    special_mat::Mu = special_mat::delta_u.transpose()*special_mat::M*special_mat::delta_u;
    // calc omega_u transpose
    special_mat::omega_u.transpose() = special_mat::M*special_mat::delta_u*((special_mat::delta_u.transpose()*special_mat::M*special_mat::delta_u).inverse());
    // define delta matrix 6x6
    special_mat::delta.block<6,5>(0,0) = special_mat::delta_a; // 0,0 start, 6x5
    special_mat::delta.block<6,1>(0,5) = special_mat::delta_u; // 0,5 start, 6x1
    // define omega matrix 6x6

	//calc omega_at matrix
	special_mat::omega_at.block<6,1>(0,0) = special_mat::B.block(0,0,6,1);
    special_mat::omega_at.block<6,1>(0,1) = special_mat::B.block(0,1,6,1);
    special_mat::omega_at.block<6,1>(0,2) = special_mat::B.block(0,2,6,1);
    special_mat::omega_at.block<6,1>(0,3) = special_mat::B.block(0,4,6,1);
    special_mat::omega_at.block<6,1>(0,4) = special_mat::B.block(0,5,6,1);

    Eigen::MatrixXf inter_u(5, 5);
	inter_u = (special_mat::delta_u.transpose()*special_mat::M*special_mat::delta_u).inverse();// check
	special_mat::omega_ut = special_mat::M*special_mat::delta_u*inter_u; // check

    // Omega.block<6,5>(0,0) = omega_at;
	// Omega.block<6,1>(0,5) = omega_ut;
    // special_mat::omega.block<6,5>(0,0) = special_mat::omega_a.transpose(); // 0,0 start, 6x5
    // special_mat::omega.block<6,1>(0,5) = special_mat::omega_u.transpose(); // 0,5 start, 6x1

    special_mat::omega.block<6,5>(0,0) = special_mat::omega_at; // 0,0 start, 6x5
    special_mat::omega.block<6,1>(0,5) = special_mat::omega_ut; // 0,5 start, 6x1
    
}
// compostion function
void plant_sim::composition()
{
    ROS_INFO_STREAM("special_mat::M: "<<special_mat::M);
    ROS_INFO_STREAM("special_mat::delta_u: "<<special_mat::delta_u);
    ROS_INFO_STREAM("special_mat::omega_at: "<<special_mat::omega_at);
    ROS_INFO_STREAM("special_mat::omega_ut: "<<special_mat::omega_ut);

    //calc attitude rotation
    Eigen::Matrix3f r_mat0, rt_mat0, rmat1,rmat2,rmat3;//main_euler_angle rotation mat
    r_mat0 = euler_to_rotation_mat(euler_angle);
    rt_mat0 = r_mat0.inverse();//inverse function of rotation matrix
    //calc child vehicle rotation 
    rmat1 = euler_to_rotation_mat(ang1);
    rmat2 = euler_to_rotation_mat(ang2);
    rmat3 = euler_to_rotation_mat(ang3);
    //get vector of thrust1~3
    Eigen::Vector3f vec_thu1,vec_thu2,vec_thu3;
    vec_thu1(0)=vec_thu1(1)=0;
    vec_thu1(2)=-thu1.data;

    vec_thu2(0)=vec_thu2(1)=0;
    vec_thu2(2)=-thu2.data;

    vec_thu3(0)=vec_thu3(1)=0;
    vec_thu3(2)=-thu3.data;

    // ROS_INFO_STREAM("plant node body_euler: " << body_euler);
    ROS_INFO_STREAM("plant node sub1_ang: " << ang1);
    ROS_INFO_STREAM("plant node sub1_thu: " << ang2);
    ROS_INFO_STREAM("plant node sub2_ang: " << ang3);
    ROS_INFO_STREAM("plant node sub2_thu: " << thu1);    
    ROS_INFO_STREAM("plant node sub3_ang: " << thu2);
    ROS_INFO_STREAM("plant node sub3_thu: " << thu3);

    //
    Eigen::Vector3f fw1,fw2,fw3,f01,f02,f03;
    fw1 = rmat1*vec_thu1;
    f01 = rt_mat0*fw1;

    fw2 = rmat2*vec_thu2;
    f02 = rt_mat0*fw2;

    fw3 = rmat3*vec_thu3;
    f03 = rt_mat0*fw3;

    //define identity matrix3f
    Eigen::Matrix3f I;
	I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;
    //define position s1~3
	pos_s1=pos_mat(param.fly1_pos.x,param.fly1_pos.y,param.fly1_pos.z);
	pos_s2=pos_mat(param.fly2_pos.x,param.fly2_pos.y,param.fly2_pos.z);
	pos_s3=pos_mat(param.fly3_pos.x,param.fly3_pos.y,param.fly3_pos.z);
    //define B matrix
    Eigen::MatrixXf B(6, 9);
	B.block<3,3>(0,0)=I;
	B.block<3,3>(0,3)=I;
	B.block<3,3>(0,6)=I;
	B.block<3,3>(3,0)=pos_s1;
	B.block<3,3>(3,3)=pos_s2;
	B.block<3,3>(3,6)=pos_s3;

    // full rank B matrix
	Eigen::MatrixXf B_full(5,9);
	B_full.block<1,9>(0,0) = B.block(0,0,1,9);
	B_full.block<1,9>(1,0) = B.block(1,0,1,9);
	B_full.block<1,9>(2,0) = B.block(2,0,1,9);
	B_full.block<1,9>(3,0) = B.block(4,0,1,9);
	B_full.block<1,9>(4,0) = B.block(5,0,1,9);


    ROS_INFO_STREAM("B_full: "<<B_full);
    ROS_INFO_STREAM("special_mat::omega: "<<special_mat::omega);

    //make an vector(9)
    Eigen::MatrixXf f0123(9, 1);
    f0123.block<3,1>(0,0)=f01;
	f0123.block<3,1>(3,0)=f02;
	f0123.block<3,1>(6,0)=f03;
    //define a 6x1 vector U
    Eigen::MatrixXf U(6, 1);
    // U = B*f0123;

    special_mat::u_a =  B_full*f0123;
    Eigen::VectorXf U_au(6); // ua and uu vector
    U_au.head(5) = special_mat::u_a;
    U_au(5) =  0; //special_mat::u_u(0);

    U = special_mat::omega * U_au;

    ROS_INFO_STREAM("f0123: "<<f0123);
	ROS_INFO_STREAM("plant ua: "<<special_mat::u_a);
    ROS_INFO_STREAM("U: "<< U);
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
    if(ideal_model)
    {
        N = torques_u+(J*omega).cross(omega);
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

    //calc vo_dot value
    if(ideal_model)
    {
        v0_dot = (param.S3Q_mass*G*rt_mat0*e3- param.S3Q_mass*(omega.cross(v))+thrust_u)/param.S3Q_mass;
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

    init_mat();//init param matrix

    calc_timer = nh.createTimer(ros::Duration(T), &plant_sim::calc_cb, this);
}

//timer callback function 
void plant_sim::calc_cb(const ros::TimerEvent&)
{
    // ROS_INFO_STREAM("gaoling...");
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