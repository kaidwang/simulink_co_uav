//controller main funciton
//four sub-aircraft platform
#include <controller/imp_four_controller_node.h>
// #include <controller/config_param.h>

//i need to control the controller when to run
using Eigen::MatrixXd;

//construct function list
controller_class::controller_class(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
	//ROS_INFO_STREAM("gaoling love ...");
	detla_time = TIME;
	//define 9.8 as gravity
	run_times = 0;
	g=GRAVITY;

	// define a I matrix as a local variable
	Eigen::MatrixXf I33(3,3);
	I33.setIdentity(3,3) ;
	// define a 0 matrix as a local variable
	Eigen::MatrixXf zero_33(3,3);
	zero_33.setZero(3,3); 
	// define a 0 vector as a local variable
	Eigen::Vector3f zero_v3;
	zero_v3(0)=zero_v3(1)=zero_v3(2)=0;

	r_mat=I33;
	r_mat_d=I33;
	err_xyz = zero_v3;
	position_err = zero_v3;
	grad_v = zero_v3;
	att_err = zero_v3;
	grad_w = zero_v3;
	ev = zero_v3;
	ew = zero_v3;
	thrust_u = zero_v3;
	torques_u = zero_v3;
	lamda1 = lamda2= lamda3= zero_v3;
	omega_d = zero_v3;
	omege = zero_v3;
	vd=zero_v3;
	epsilon_d_v=zero_v3;
	epsilon_d_v_dot=zero_v3;
	epsilon_d_w=zero_v3;
	epsilon_d_w_dot=zero_v3;

	// controller output init section
	ang1.x=ang1.y=ang1.z=0;
	ang2.x=ang2.y=ang2.z=0;
	ang3.x=ang3.y=ang3.z=0;
	thu1.data = 0;
	thu2.data = 0;
	thu3.data = 0;
	nominal_position = init_position;
	nominal_euler_angles = init_euler_angles;


	dot_u_integra_val_1 = 0;
    dot_u_integra_val_2 = 0;
    dot_v_integra_val_1 = 0;
    dot_v_integra_val_2 = 0;
    dot_w_integra_val_1 = 0;
    dot_w_integra_val_2 = 0;

    dot_p_integra_val_1 = 0;
    dot_p_integra_val_2 = 0;
    dot_q_integra_val_1 = 0;
    dot_q_integra_val_2 = 0;
    dot_r_integra_val_1 = 0;
    dot_r_integra_val_2 = 0;

    dot_phi_integra_val_1 = 0;
    dot_phi_integra_val_2 = 0;
    dot_theta_integra_val_1 = 0;
	dot_theta_integra_val_2 = 0;
    dot_psi_integra_val_1 = 0;
    dot_psi_integra_val_2 = 0;

    dot_xd_integra_val_1 = 0;
    dot_xd_integra_val_2 = 0;
    dot_yd_integra_val_1 = 0;
    dot_yd_integra_val_2 = 0;
    dot_zd_integra_val_1 = 0;
    dot_zd_integra_val_2 = 0;

    ang_err_init_x_integar_last = 0;
    ang_err_init_y_integar_last = 0;
    ang_err_init_z_integar_last = 0;

    pos_err_init_x_integar_last = 0;
    pos_err_init_y_integar_last = 0;
    pos_err_init_z_integar_last = 0;


    epsilon_d_v_dot_u_last=0;
    epsilon_d_v_dot_v_last=0;
    epsilon_d_v_dot_w_last=0;

    epsilon_d_w_dot_p_last=0;
    epsilon_d_w_dot_q_last=0;
	epsilon_d_w_dot_r_last=0;

    phi_bar_dot_last=0;
    theta_bar_dot_last=0;
    psi_bar_dot_last=0;

    xd_dot_last=0;
    yd_dot_last=0;
    zd_dot_last=0;
	
	//init psi Runge Kutta method x,y,z
	phi_bar_dot_x=0;
    phi_bar_dot_y=init_euler_angles.x;
    phi_bar_dot_z=0;
	phi_bar_dot  =0;
    //init theta Runge Kutta method related val
	theta_bar_dot_x=0;
    theta_bar_dot_y=init_euler_angles.y;
    theta_bar_dot_z=0;
    theta_bar_dot  =0;
	//init psi Runge Kutta method related val
    psi_bar_dot_x=0;
    psi_bar_dot_y=init_euler_angles.z;
    psi_bar_dot_z=0;
    psi_bar_dot  =0;
	//init xd runge kutta method related val
	xd_dot_x=0;
    xd_dot_y=init_position.x;
    xd_dot_z=0;
	xd_dot=0;
	//init yd runge kutta method related val
	yd_dot_x=0;
    yd_dot_y=init_position.y;
    yd_dot_z=0;
    yd_dot=0;
    //init zd Runge Kutta method related val
    zd_dot_x=0;
    zd_dot_y=init_position.z;
    zd_dot_z=0;
    zd_dot=0;
    //init epsilon_d_v_dot_u Runge Kutta method related val
    epsilon_d_v_dot_u_x=0;
    epsilon_d_v_dot_u_y=0;
    epsilon_d_v_dot_u_z=0;
    epsilon_d_v_dot_u=0;
    //init epsilon_d_v_dot_v Runge Kutta method related val
    epsilon_d_v_dot_v_x=0;
    epsilon_d_v_dot_v_y=0;
    epsilon_d_v_dot_v_z=0;
    epsilon_d_v_dot_v=0;
    //init epsilon_d_v_dot_w Runge Kutta method related val
    epsilon_d_v_dot_w_x=0;
    epsilon_d_v_dot_w_y=0;
    epsilon_d_v_dot_w_z=0;
    epsilon_d_v_dot_w=0;
    //init epsilon_d_v_dot_p Runge Kutta method related val
    epsilon_d_w_dot_p_x=0;
    epsilon_d_w_dot_p_y=0;
    epsilon_d_w_dot_p_z=0;
    epsilon_d_w_dot_p=0;
    //init epsilon_d_v_dot_v Runge Kutta method related val
    epsilon_d_w_dot_q_x=0;
    epsilon_d_w_dot_q_y=0;
    epsilon_d_w_dot_q_z=0;
    epsilon_d_w_dot_q=0;
    //init epsilon_d_v_dot_w Runge Kutta method related val
    epsilon_d_w_dot_r_x=0;
    epsilon_d_w_dot_r_y=0;
    epsilon_d_w_dot_r_z=0;
    epsilon_d_w_dot_r=0;

	kp_pos(0,0)=param.kp_x;
	kp_pos(1,1)=param.kp_y;
	kp_pos(2,2)=param.kp_z;
	kp_pos(0,1)=kp_pos(0,2)=kp_pos(1,0)=
	kp_pos(1,2)=kp_pos(2,0)=kp_pos(2,1)=0;
	//ROS_INFO_STREAM(kp_pos);

	ki_pos(0,0)=param.ki_x;
	ki_pos(1,1)=param.ki_y;
	ki_pos(2,2)=param.ki_z;
	ki_pos(0,1)=ki_pos(0,2)=ki_pos(1,0)=
	ki_pos(1,2)=ki_pos(2,0)=ki_pos(2,1)=0;

	kd_pos(0,0)=param.kd_x;
	kd_pos(1,1)=param.kd_y;
	kd_pos(2,2)=param.kd_z;
	kd_pos(0,1)=kd_pos(0,2)=kd_pos(1,0)=
	kd_pos(1,2)=kd_pos(2,0)=kd_pos(2,1)=0;

/***********************autopilot control PID theory************************/
	kp(0,0)=param.kpx;
	kp(1,1)=param.kpy;
	kp(2,2)=param.kpz;
	kp(0,1)=kp(0,2)=kp(1,0)=kp(1,2)=kp(2,0)=kp(2,1)=0;
	// init err_v integral last number
	err_v_integral_x_ls = 0;
    err_v_integral_y_ls = 0;
    err_v_integral_z_ls = 0;
/***************************************************************************/

	kp_att(0,0)=param.kp_phi;
	kp_att(1,1)=param.kp_theta;
	kp_att(2,2)=param.kp_psi;
	kp_att(0,1)=kp_att(0,2)=kp_att(1,0)=
	kp_att(1,2)=kp_att(2,0)=kp_att(2,1)=0;

	ki_att(0,0)=param.ki_phi;
	ki_att(1,1)=param.ki_theta;
	ki_att(2,2)=param.ki_psi;
	ki_att(0,1)=ki_att(0,2)=ki_att(1,0)=
	ki_att(1,2)=ki_att(2,0)=ki_att(2,1)=0;

	kd_att(0,0)=param.kd_phi;
	kd_att(1,1)=param.kd_theta;
	kd_att(2,2)=param.kd_psi;
	kd_att(0,1)=kd_att(0,2)=kd_att(1,0)=
	kd_att(1,2)=kd_att(2,0)=kd_att(2,1)=0;

	kp_f(0,0)=1;//param.kf_p_x;
	kp_f(1,1)=1;//param.kf_p_y;
	kp_f(2,2)=1;//param.kf_p_z;
	kp_f(0,1)=kp_f(0,2)=kp_f(1,0)=
	kp_f(1,2)=kp_f(2,0)=kp_f(2,1)=0;

	ki_f(0,0)=0.1;//param.kf_i_x;
	ki_f(1,1)=0.1;//param.kf_i_y;
	ki_f(2,2)=0.1;//param.kf_i_z;
	ki_f(0,1)=ki_f(0,2)=ki_f(1,0)=
	ki_f(1,2)=ki_f(2,0)=ki_f(2,1)=0;

	kd_f(0,0)=2;//param.kf_d_x;
	kd_f(1,1)=2;//param.kf_d_y;
	kd_f(2,2)=2;//param.kf_d_z;
	kd_f(0,1)=kd_f(0,2)=kd_f(1,0)=
	kd_f(1,2)=kd_f(2,0)=kd_f(2,1)=0;

	kp_t(0,0)=1;//param.kt_p_roll;
	kp_t(1,1)=1;//param.kt_p_pitch;
	kp_t(2,2)=1;//param.kt_p_yaw;
	kp_t(0,1)=kp_t(0,2)=kp_t(1,0)=
	kp_t(1,2)=kp_t(2,0)=kp_t(2,1)=0;

	ki_t(0,0)=0.1;//param.kt_i_roll;
	ki_t(1,1)=0.1;//param.kt_i_pitch;
	ki_t(2,2)=0.1;//param.kt_i_yaw;
	ki_t(0,1)=ki_t(0,2)=ki_t(1,0)=
	ki_t(1,2)=ki_t(2,0)=ki_t(2,1)=0;

	kd_t(0,0)=2;//param.kt_d_roll;
	kd_t(1,1)=2;//param.kt_d_pitch;
	kd_t(2,2)=2;//param.kt_d_yaw;
	kd_t(0,1)=kd_t(0,2)=kd_t(1,0)=
	kd_t(1,2)=kd_t(2,0)=kd_t(2,1)=0;

	J(0,0)=param.I_sys.x;
	J(1,1)=param.I_sys.y;
	J(2,2)=param.I_sys.z;
	J(0,1)=J(0,2)=J(1,0)=J(1,2)=J(2,0)=J(2,1)=0;

	omege(0)=main_body_rates.x;
	omege(1)=main_body_rates.y;
	omege(2)=main_body_rates.z;

	init_subscriber();//init subscriber list handle
	init_publisher(); //init publisher list handle
	//define a timer 
    calc_timer = nh.createTimer(ros::Duration(detla_time), &controller_class::calc_cb, this);  //timer used to publish state, should be at least for some minimal frequency
	
	// define param list printf
	ROS_INFO_STREAM("fly1_pos:  "<<param.fly1_pos.x<<" "<<param.fly1_pos.y);
	ROS_INFO_STREAM("fly2_pos:  "<<param.fly2_pos.x<<" "<<param.fly2_pos.y);
	ROS_INFO_STREAM("fly3_pos:  "<<param.fly3_pos.x<<" "<<param.fly3_pos.y);
	ROS_INFO_STREAM("fly4_pos:  "<<param.fly4_pos.x<<" "<<param.fly4_pos.y);

	ROS_INFO_STREAM("center_mass:"<<param.center_mass);
	ROS_INFO_STREAM("fly1_mass:  "<<param.fly1_mass);
	ROS_INFO_STREAM("fly2_mass:  "<<param.fly2_mass);
	ROS_INFO_STREAM("fly3_mass:  "<<param.fly3_mass);
	ROS_INFO_STREAM("fly4_mass:  "<<param.fly4_mass);

	ROS_INFO_STREAM("S3Q_mass:   "<<param.S3Q_mass);
	ROS_INFO_STREAM("param diplay");
	ROS_INFO_STREAM("I_sys.x:"<<param.I_sys.x);
	ROS_INFO_STREAM("I_sys.y:"<<param.I_sys.y);
	ROS_INFO_STREAM("I_sys.z:"<<param.I_sys.z);
	ROS_INFO_STREAM("S3Q_mass: "<<param.S3Q_mass);

	// ROS_INFO_STREAM("kp_x:  "<<param.kp_x);
	// ROS_INFO_STREAM("kp_y:  "<<param.kp_y);
	// ROS_INFO_STREAM("kp_z:  "<<param.kp_z);
	// ROS_INFO_STREAM("ki_x:  "<<param.ki_x);
	// ROS_INFO_STREAM("ki_y:  "<<param.ki_y);
	// ROS_INFO_STREAM("ki_z:  "<<param.ki_z);
	// ROS_INFO_STREAM("kd_x:  "<<param.kd_x);
	// ROS_INFO_STREAM("kd_y:  "<<param.kd_y);
	// ROS_INFO_STREAM("kd_z:  "<<param.kd_z);	

	// ROS_INFO_STREAM("kp_phi:  "<<param.kp_phi);
	// ROS_INFO_STREAM("kp_theta:  "<<param.kp_theta);
	// ROS_INFO_STREAM("kp_psi:  "<<param.kp_psi);
	// ROS_INFO_STREAM("ki_phi:  "<<param.ki_phi);
	// ROS_INFO_STREAM("ki_theta:  "<<param.ki_theta);
	// ROS_INFO_STREAM("ki_psi:  "<<param.ki_psi);	
	// ROS_INFO_STREAM("kd_phi:  "<<param.kd_phi);
	// ROS_INFO_STREAM("kd_theta:  "<<param.kd_theta);
	// ROS_INFO_STREAM("kd_psi:  "<<param.kd_psi);	

	// ROS_INFO_STREAM("omega_phi:  "<<param.omega_phi);
	// ROS_INFO_STREAM("epsilon_phi:  "<<param.epsilon_phi);	
	// ROS_INFO_STREAM("omega_theta:  "<<param.omega_theta);
	// ROS_INFO_STREAM("epsilon_theta:  "<<param.epsilon_theta);
	// ROS_INFO_STREAM("omega_psi:  "<<param.omega_psi);
	// ROS_INFO_STREAM("epsilon_psi:  "<<param.epsilon_psi);

	// ROS_INFO_STREAM("omega_u:  "<<param.omega_u);
	// ROS_INFO_STREAM("epsilon_u:  "<<param.epsilon_u);	
	// ROS_INFO_STREAM("omega_v:  "<<param.omega_v);
	// ROS_INFO_STREAM("epsilon_v:  "<<param.epsilon_v);
	// ROS_INFO_STREAM("omega_w:  "<<param.omega_w);
	// ROS_INFO_STREAM("epsilon_w:  "<<param.epsilon_w);

	// ROS_INFO_STREAM("omega_x:  "<<param.omega_x);
	// ROS_INFO_STREAM("epsilon_x:  "<<param.epsilon_x);	
	// ROS_INFO_STREAM("omega_y:  "<<param.omega_y);
	// ROS_INFO_STREAM("epsilon_y:  "<<param.epsilon_y);
	// ROS_INFO_STREAM("omega_z:  "<<param.omega_z);
	// ROS_INFO_STREAM("epsilon_z:  "<<param.epsilon_z);	

	// ROS_INFO_STREAM("omega_p:  "<<param.omega_p);
	// ROS_INFO_STREAM("epsilon_p:  "<<param.epsilon_p);	
	// ROS_INFO_STREAM("omega_q:  "<<param.omega_q);
	// ROS_INFO_STREAM("epsilon_q:  "<<param.epsilon_q);
	// ROS_INFO_STREAM("omega_r:  "<<param.omega_r);
	// ROS_INFO_STREAM("epsilon_r:  "<<param.epsilon_r);	
}

//publisher function,all publish is in NED frame
void controller_class::init_publisher()
{
	//init this publisher pub, and then i can use these handle in every private member function of this class
	angle1_pub = nh.advertise<geometry_msgs::Point>("/angle1",10,this);
	angle2_pub = nh.advertise<geometry_msgs::Point>("/angle2",10,this);
	angle3_pub = nh.advertise<geometry_msgs::Point>("/angle3",10,this);
	angle4_pub = nh.advertise<geometry_msgs::Point>("/angle4",10,this);

	thrust1_pub = nh.advertise<std_msgs::Float64>("/thrust1",10,this);
	thrust2_pub = nh.advertise<std_msgs::Float64>("/thrust2",10,this);
	thrust3_pub = nh.advertise<std_msgs::Float64>("/thrust3",10,this);
	thrust4_pub = nh.advertise<std_msgs::Float64>("/thrust4",10,this);

	thrust_value_pub = nh.advertise<geometry_msgs::Point>("/thrust_value",10,this);
    torque_value_pub = nh.advertise<geometry_msgs::Point>("/torque_value",10,this);
}

//subscriber function
void controller_class::init_subscriber()
{
	//subscriber handle list
	/*
	ros::Subscriber main_eular_angles_sub ;	//controller input of main_eular_angles
	ros::Subscriber main_body_rates_sub ;	//controller input of main_body_rates
	ros::Subscriber init_euler_angles_cmd_sub ;	//init cmd publish: euler angles
	ros::Subscriber init_pos_cmd_sub ;	//init cmd publish: position
	ros::Subscriber init_body_rates_cmd_sub ;	//init cmd publish: body_rates
	ros::Subscriber init_velocity_cmd_sub ;	//init cmd publish: velocity

	ros::Subscriber control_start_sub_att ;	//control switch of when publish attitude to child node, besides control when start the controller  
   
    //subscribe the bias of psi angle that each child flight needed
    //std_msgs::Float64
	ros::Subscriber psi_bias_1_sub;
	ros::Subscriber psi_bias_2_sub;
	ros::Subscriber psi_bias_3_sub;
	*/
	nominal_position_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_position",1,&controller_class::nominal_position_sub_cb,this);
	nominal_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_euler_angles",1,&controller_class::nominal_eular_angles_sub_cb, this);
	nominal_velocity_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_velocity",1,&controller_class::nominal_velocity_sub_cb, this);
	nominal_body_rate_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_body_rate",1,&controller_class::nominal_body_rate_sub_cb, this);


	ref_thrust_sub = nh.subscribe<geometry_msgs::Point>
	("/ref_thrust",1,&controller_class::ref_thrust_sub_cb,this);
	ref_torque_sub = nh.subscribe<geometry_msgs::Point>
	("/ref_torque",1,&controller_class::ref_torque_sub_cb, this);
	sensor_thrust_sub = nh.subscribe<geometry_msgs::Point>
	("/ext_thrust",1,&controller_class::sensor_thrust_sub_cb, this);
	sensor_torque_sub = nh.subscribe<geometry_msgs::Point>
	("/ext_torque",1,&controller_class::sensor_torque_sub_cb, this);


	// main_position_sub = nh.subscribe<geometry_msgs::Point>
	// ("/main_position_sim",1,&controller_class::main_position_sub_cb,this);
	// main_velocity_sub = nh.subscribe<geometry_msgs::Point>
	// ("/main_velocity_sim",1,&controller_class::main_velocity_sub_cb,this);
	// main_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	// ("/main_euler_angles_sim",1,&controller_class::main_eular_angles_sub_cb,this);
	// main_body_rates_sub = nh.subscribe<geometry_msgs::Point>
	// ("/main_body_rates_sim",1,&controller_class::main_body_rates_sub_cb,this);
	
	main_position_sub = nh.subscribe<geometry_msgs::Point>
	("/main_position",1,&controller_class::main_position_sub_cb,this);
	main_velocity_sub = nh.subscribe<geometry_msgs::Point>
	("/main_velocity",1,&controller_class::main_velocity_sub_cb,this);
	main_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	("/main_euler_angles",1,&controller_class::main_eular_angles_sub_cb,this);
	main_body_rates_sub = nh.subscribe<geometry_msgs::Point>
	("/main_body_rates",1,&controller_class::main_body_rates_sub_cb,this);

	init_euler_angles_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_euler_angles_cmd",1,&controller_class::init_euler_angles_cmd_sub_cb,this);
	init_pos_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_position_cmd",1,&controller_class::init_pos_cmd_sub_cb,this);
	init_body_rates_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_body_rates_cmd",1,&controller_class::init_body_rates_cmd_sub_cb,this);
	init_velocity_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_velocity_cmd",1,&controller_class::init_velocity_cmd_sub_cb,this);
	control_start_sub_att = nh.subscribe<std_msgs::Bool>
	("/start_pub_att",1,&controller_class::control_start_sub_att_cb,this);
	
	//kaidi wang, 2021.10.13, mode_switch_sub define
	mode_switch_sub       = nh.subscribe<std_msgs::Bool>
	("/mode_switch",1,&controller_class::mode_switch_sub_cb,this);

	psi_bias_1_sub = nh.subscribe<std_msgs::Float64>("/psi_bias_1",1,&controller_class::psi_bias_1_sub_cb,this);
	psi_bias_2_sub = nh.subscribe<std_msgs::Float64>("/psi_bias_2",1,&controller_class::psi_bias_2_sub_cb,this);
	psi_bias_3_sub = nh.subscribe<std_msgs::Float64>("/psi_bias_3",1,&controller_class::psi_bias_3_sub_cb,this);

	//output_publish(angle1,angle2,angle3,thrust1,thrust2,thrust3);
}



//top level function
void controller_class::controller_body()//controller body
{
	calc_grad();       //sencond level function
	calc_epsilon_d();  //
	// calc_thrust_autopilot_pid();
	// calc_thrust_new();

	calc_thrust();
	calc_torques();
}

//controller body
void controller_class::control_allocation()
{
	ctrl_allocation();
}


//second level function
void controller_class::calc_grad()
{
	calc_grad_v();	//calc_gard_v function
	calc_grad_w();  //calc_gard_w function
}
void controller_class::calc_epsilon_d()
{
	rotation_kine_inerse();//kinematic inverse of rotation
	trans_kine_inverse();  //translational kinematic inverse
	calc_epsilon_v();      //v epsilon 
	calc_epsilon_w();      //w epsilon

	//output param
	double u_bar   = epsilon_d_v(0);
	double v_bar   = epsilon_d_v(1);
	double w_bar   = epsilon_d_v(2);

	//first calc u as the input
	double u_u = gain_1(u_bar,param.omega_u);
	double v_u = gain_1(v_bar,param.omega_v);
	double w_u = gain_1(w_bar,param.omega_w);

	//second use RK4 solve ODE2
	epsilon_d_v_dot_u_last = epsilon_d_v_dot_u_y;
	RK4_02(u_u,param.omega_u,param.epsilon_u,
        &epsilon_d_v_dot_u_x, &epsilon_d_v_dot_u_y, &epsilon_d_v_dot_u_z, TIME);
	
	epsilon_d_v_dot_v_last = epsilon_d_v_dot_v_y;
	RK4_02(v_u,param.omega_v,param.epsilon_v,
        &epsilon_d_v_dot_v_x, &epsilon_d_v_dot_v_y, &epsilon_d_v_dot_v_z, TIME);

	epsilon_d_v_dot_w_last = epsilon_d_v_dot_w_y;
	RK4_02(w_u,param.omega_w,param.epsilon_w,
        &epsilon_d_v_dot_w_x, &epsilon_d_v_dot_w_y, &epsilon_d_v_dot_w_z, TIME);
	

	//third calc the different value of y
	differentiator_lastest(&epsilon_d_v_dot_u_last,&epsilon_d_v_dot_u_y,&epsilon_d_v_dot_u,TIME);
	differentiator_lastest(&epsilon_d_v_dot_v_last,&epsilon_d_v_dot_v_y,&epsilon_d_v_dot_v,TIME);
	differentiator_lastest(&epsilon_d_v_dot_w_last,&epsilon_d_v_dot_w_y,&epsilon_d_v_dot_w,TIME);
	//kaidi wang comment these lines on 2021.7.12
	// double epsilon_d_v_dot_u = differentiator(
	// 	epsilon_d_v(0),
	// 	0,
	// 	param.omega_u,
	// 	param.epsilon_u,
	// 	&epsilon_d_v_dot_u_last,
	// 	&dot_u_integra_val_1,
	// 	&dot_u_integra_val_2);

	// double epsilon_d_v_dot_v = differentiator(
	// 	epsilon_d_v(1),
	// 	0,
	// 	param.omega_v,
	// 	param.epsilon_v,
	// 	&epsilon_d_v_dot_v_last,
	// 	&dot_v_integra_val_1,
	// 	&dot_v_integra_val_2);

	// double epsilon_d_v_dot_w = differentiator(
	// 	epsilon_d_v(2),
	// 	0,
	// 	param.omega_w,
	// 	param.epsilon_w,
	// 	&epsilon_d_v_dot_w_last,
	// 	&dot_w_integra_val_1,
	// 	&dot_w_integra_val_2);

	epsilon_d_v_dot(0)=epsilon_d_v_dot_u;
	epsilon_d_v_dot(1)=epsilon_d_v_dot_v;
	epsilon_d_v_dot(2)=epsilon_d_v_dot_w;


	//output param
	double p_bar   = epsilon_d_w(0);
	double q_bar   = epsilon_d_w(1);
	double r_bar   = epsilon_d_w(2);

	//first calc u as the input
	double p_u = gain_1(p_bar,param.omega_p);
	double q_u = gain_1(q_bar,param.omega_q);
	double r_u = gain_1(r_bar,param.omega_r);

	//second use RK4 solve ODE2
	epsilon_d_w_dot_p_last = epsilon_d_w_dot_p_y;
	RK4_02(p_u,param.omega_p,param.epsilon_p,
        &epsilon_d_w_dot_p_x, &epsilon_d_w_dot_p_y, &epsilon_d_w_dot_p_z, TIME);
	
	epsilon_d_w_dot_q_last = epsilon_d_w_dot_q_y;
	RK4_02(q_u,param.omega_q,param.epsilon_q,
        &epsilon_d_w_dot_q_x, &epsilon_d_w_dot_q_y, &epsilon_d_w_dot_q_z, TIME);

	epsilon_d_w_dot_r_last = epsilon_d_w_dot_r_z;
	RK4_02(r_u,param.omega_r,param.epsilon_r,
        &epsilon_d_w_dot_r_x, &epsilon_d_w_dot_r_y, &epsilon_d_w_dot_r_z, TIME);
	

	//third calc the different value of y
	differentiator_lastest(&epsilon_d_w_dot_p_last,&epsilon_d_w_dot_p_y,&epsilon_d_w_dot_p,TIME);
	differentiator_lastest(&epsilon_d_w_dot_q_last,&epsilon_d_w_dot_q_y,&epsilon_d_w_dot_q,TIME);
	differentiator_lastest(&epsilon_d_w_dot_r_last,&epsilon_d_w_dot_r_y,&epsilon_d_w_dot_r,TIME);

	// kaidi wang comment on 2021.7.12
	// double epsilon_d_w_dot_p = differentiator(
	// 	epsilon_d_w(0),
	// 	0,
	// 	param.omega_p,
	// 	param.epsilon_p,
	// 	&epsilon_d_w_dot_p_last,
	// 	&dot_p_integra_val_1,
	// 	&dot_p_integra_val_2);

	// double epsilon_d_w_dot_q = differentiator(
	// 	epsilon_d_w(1),
	// 	0,
	// 	param.omega_q,
	// 	param.epsilon_q,
	// 	&epsilon_d_w_dot_q_last,
	// 	&dot_q_integra_val_1,
	// 	&dot_q_integra_val_2);

	// double epsilon_d_w_dot_r = differentiator(
	// 	epsilon_d_w(2),
	// 	0,
	// 	param.omega_r,
	// 	param.epsilon_r,
	// 	&epsilon_d_w_dot_r_last,
	// 	&dot_r_integra_val_1,
	// 	&dot_r_integra_val_2);
		
	epsilon_d_w_dot(0)=epsilon_d_w_dot_p;
	epsilon_d_w_dot(1)=epsilon_d_w_dot_q;
	epsilon_d_w_dot(2)=epsilon_d_w_dot_r;

	//debug lines
	// ROS_INFO_STREAM("epsilon_d_v_dot: "<<epsilon_d_v_dot);
	// ROS_INFO_STREAM("epsilon_d_w_dot: "<<epsilon_d_w_dot);
}

void controller_class::calc_thrust()
{
	Eigen::Vector3f vec_1;
	vec_1(0)=vec_1(1)=0;
	vec_1(2)=1;
	//calc ev
	Eigen::Vector3f vc;
	vc(0)=main_velocity.x;
	vc(1)=main_velocity.y;
	vc(2)=main_velocity.z;
	ev=vc-epsilon_d_v;

	//calc position_err_init
	Eigen::Vector3f position_err_init;

// 	double controller::integrator(double input,double orign,double *ls,double t)
// {
// 	double output;
// 	*ls = *ls+input*t;
// 	//last_sum = input;
// 	output = *ls+orign;
// 	return output;
// }
	double position_err_init_x = integrator(position_err(0),init_position.x,&pos_err_init_x_integar_last,detla_time);
	double position_err_init_y = integrator(position_err(1),init_position.y,&pos_err_init_y_integar_last,detla_time);
	double position_err_init_z = integrator(position_err(2),init_position.z,&pos_err_init_z_integar_last,detla_time);
	
	// kaidi wang, 2022.5.30 integrator without init data
	// double position_err_init_x = integrator(position_err(0),0,&pos_err_init_x_integar_last,detla_time);
	// double position_err_init_y = integrator(position_err(1),0,&pos_err_init_y_integar_last,detla_time);
	// double position_err_init_z = integrator(position_err(2),0,&pos_err_init_z_integar_last,detla_time);
	// ROS_INFO_STREAM(init_position);
	//debug lines
	// ROS_INFO_STREAM("position_err_init_x: "<<position_err_init_x);
	// ROS_INFO_STREAM("position_err_init_y: "<<position_err_init_y);
	// ROS_INFO_STREAM("position_err_init_z: "<<position_err_init_z);

	position_err_init(0)=position_err_init_x;
	position_err_init(1)=position_err_init_y;
	position_err_init(2)=position_err_init_z;

	d_ef = ef-ef_ls;
	Eigen::Vector3f impedance_thrust = thrust_impedance_controller(ref_thrust, sensor_thrust)+kd_f*d_ef;
	//calc thrust
	ef_ls = ef;
	if(mode_switch.data)
	{
		thrust_u = 
		param.S3Q_mass*epsilon_d_v_dot+
		(param.S3Q_mass*omege).cross(epsilon_d_v)-
		kd_pos*ev-
		grad_v-
		ki_pos*position_err_init+
		(-param.S3Q_mass*g*r_mat.transpose()*vec_1)
		+impedance_thrust;
		ROS_INFO_STREAM("into thrust: "<< impedance_thrust);

		thrust_value.x = thrust_u(0);
		thrust_value.y = thrust_u(1);
		thrust_value.z = thrust_u(2);

	}
	else
	{
		
		thrust_u = 
		param.S3Q_mass*epsilon_d_v_dot+
		(param.S3Q_mass*omege).cross(epsilon_d_v)-
		kd_pos*ev-
		grad_v-
		ki_pos*position_err_init+
		(-param.S3Q_mass*g*r_mat.transpose()*vec_1);


		thrust_value.x = thrust_u(0);
		thrust_value.y = thrust_u(1);
		thrust_value.z = thrust_u(2);
		
	}
	//calc thrust
	// thrust_u = 
	// 	param.S3Q_mass*epsilon_d_v_dot+
	// 	(param.S3Q_mass*omege).cross(epsilon_d_v)-
	// 	kd_pos*ev-
	// 	grad_v-
	// 	ki_pos*position_err_init+
	// 	(-param.S3Q_mass*g*r_mat.transpose()*vec_1);
	/*******************************************************************/

	/*******************************************************************/
	// ROS_INFO_STREAM("**************************************************");
	// ROS_INFO_STREAM("1: "<<param.S3Q_mass*epsilon_d_v_dot);
	// ROS_INFO_STREAM("2: "<<(param.S3Q_mass*omege).cross(epsilon_d_v));
	// ROS_INFO_STREAM("3: "<<kd_pos*ev);
	// ROS_INFO_STREAM("4: "<<grad_v);
	// ROS_INFO_STREAM("5: "<<ki_pos*position_err_init);
	// ROS_INFO_STREAM("6: "<<(-param.S3Q_mass*g*r_mat.transpose()*vec_1));	
	// ROS_INFO_STREAM("thrust: "<<thrust_u);

}

// attitude control
void controller_class::calc_torques()
{
	Eigen::Vector3f vec_1;
	vec_1(0)=vec_1(1)=0;
	vec_1(2)=1;
	//calc ew
	Eigen::Vector3f body_rate;
	body_rate(0)=main_body_rates.x;
	body_rate(1)=main_body_rates.y;
	body_rate(2)=main_body_rates.z;
	ew=body_rate-epsilon_d_w;
	//calc angle_err_init
	Eigen::Vector3f angle_err_init;
	double angle_err_init_x = integrator(att_err(0),init_euler_angles.x,&ang_err_init_x_integar_last,detla_time);
	double angle_err_init_y = integrator(att_err(1),init_euler_angles.y,&ang_err_init_y_integar_last,detla_time);
	double angle_err_init_z = integrator(att_err(2),init_euler_angles.z,&ang_err_init_z_integar_last,detla_time);
	
	// kaidi wang, 2022.5.30 integrator without init data
	// double angle_err_init_x = integrator(att_err(0),0,&ang_err_init_x_integar_last,detla_time);
	// double angle_err_init_y = integrator(att_err(1),0,&ang_err_init_y_integar_last,detla_time);
	// double angle_err_init_z = integrator(att_err(2),0,&ang_err_init_z_integar_last,detla_time);
	// ROS_INFO_STREAM(init_euler_angles);
	
	// debug lines
	// ROS_INFO_STREAM("angle_err_init_x: "<<angle_err_init_x);
	// ROS_INFO_STREAM("angle_err_init_y: "<<angle_err_init_y);
	// ROS_INFO_STREAM("angle_err_init_z: "<<angle_err_init_z);
	
	angle_err_init(0) = angle_err_init_x;
	angle_err_init(1) = angle_err_init_y;
	angle_err_init(2) = angle_err_init_z;

	d_et = et-et_ls;
	Eigen::Vector3f impedance_torque = torque_impedance_controller(ref_torque, sensor_torque)+kd_t*d_et;
	et_ls = et;
if(mode_switch.data)
	{
		torques_u = 
			J*epsilon_d_w_dot-
			(J*omege).cross(epsilon_d_w)-
			kd_att*ew-
			ki_att*angle_err_init-
			grad_w
			+impedance_torque;
			ROS_INFO_STREAM("into torque: "<< impedance_torque);

		torque_value.x = torques_u(0);
		torque_value.y = torques_u(1);
		torque_value.z = torques_u(2);
	}
	else
	{
	
		torques_u = 
			J*epsilon_d_w_dot-
			(J*omege).cross(epsilon_d_w)-
			kd_att*ew-
			ki_att*angle_err_init-
			grad_w;
		
		torque_value.x = torques_u(0);
		torque_value.y = torques_u(1);
		torque_value.z = torques_u(2);
	
	}
	// torques_u = 
	// 	J*epsilon_d_w_dot-
	// 	(J*omege).cross(epsilon_d_w)-
	// 	kd_att*ew-
	// 	ki_att*angle_err_init-
	// 	grad_w;
	//debug line
	// ROS_INFO_STREAM("torques: "<<torques_u);
}

// control allocation
void controller_class::ctrl_allocation()
{
	Eigen::Matrix3f I;
	I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;
	//get pos matrix
	pos_s1=pos_mat(param.fly1_pos.x,param.fly1_pos.y,param.fly1_pos.z);
	pos_s2=pos_mat(param.fly2_pos.x,param.fly2_pos.y,param.fly2_pos.z);
	pos_s3=pos_mat(param.fly3_pos.x,param.fly3_pos.y,param.fly3_pos.z);
	pos_s4=pos_mat(param.fly4_pos.x,param.fly4_pos.y,param.fly4_pos.z);
	//debug line
	// ROS_INFO_STREAM("pos_s1: "<<pos_s1);
	// ROS_INFO_STREAM("pos_s2: "<<pos_s2);
	// ROS_INFO_STREAM("pos_s3: "<<pos_s3);
	ROS_INFO_STREAM("pos_s4: "<<pos_s4);

	Eigen::MatrixXf B(6, 12);
	B.block<3,3>(0,0)=I;
	B.block<3,3>(0,3)=I;
	B.block<3,3>(0,6)=I;
	B.block<3,3>(0,9)=I;
	B.block<3,3>(3,0)=pos_s1;
	B.block<3,3>(3,3)=pos_s2;
	B.block<3,3>(3,6)=pos_s3;
	B.block<3,3>(3,9)=pos_s4;
	Eigen::MatrixXf B_calc(6, 6);
	B_calc = B*(B.transpose());

	//debug line
	// ROS_INFO_STREAM("B: "<<B);
	// ROS_INFO_STREAM("B_inverse: "<<B.transpose());
	// ROS_INFO_STREAM("B_calc: "<<B_calc);

	//get vector of U
	Eigen::VectorXf U(6);
	U.head(3)=thrust_u;
	U.tail<3>()=torques_u;

	Eigen::MatrixXf inter1(3, 6);
	inter1.block<3,3>(0,0)=I;
	inter1.block<3,3>(0,3)=pos_s1.transpose();

	Eigen::MatrixXf inter2(3, 6);
	inter2.block<3,3>(0,0)=I;
	inter2.block<3,3>(0,3)=pos_s2.transpose();

	Eigen::MatrixXf inter3(3, 6);
	inter3.block<3,3>(0,0)=I;
	inter3.block<3,3>(0,3)=pos_s3.transpose();

	Eigen::MatrixXf inter4(3, 6);
	inter4.block<3,3>(0,0)=I;
	inter4.block<3,3>(0,3)=pos_s4.transpose();

	lamda1=inter1*(B_calc.inverse()*U);
	lamda2=inter2*(B_calc.inverse()*U);
	lamda3=inter3*(B_calc.inverse()*U);
	lamda4=inter4*(B_calc.inverse()*U);
	//debug line
	// ROS_INFO_STREAM("lamda1: "<<lamda1);
	// ROS_INFO_STREAM("lamda2: "<<lamda2);
	// ROS_INFO_STREAM("lamda3: "<<lamda3);
	float angle = 45*PI/180;
	//allocation function
	double psi_cmd1=nominal_euler_angles.z+angle;
	double psi_cmd2=nominal_euler_angles.z+angle+PI/2;
	double psi_cmd3=nominal_euler_angles.z-angle-PI/2;
	double psi_cmd4=nominal_euler_angles.z-angle;

	Eigen::Vector4f thu_att1=alloc(lamda1,psi_cmd1);
	Eigen::Vector4f thu_att2=alloc(lamda2,psi_cmd2);
	Eigen::Vector4f thu_att3=alloc(lamda3,psi_cmd3);
	Eigen::Vector4f thu_att4=alloc(lamda4,psi_cmd4);

	//push data to message

	thu1.data = thu_att1(0);//thr_full;//thr_full=sub_m*g/hover_thr_p*1.1 60  =1.8*3+1.5+1.6=8.5kg 85N
	ang1.x    = thu_att1(1);
	ang1.y    = thu_att1(2);
	ang1.z    = thu_att1(3);

	thu2.data = thu_att2(0);
	ang2.x    = thu_att2(1);
	ang2.y    = thu_att2(2);
	ang2.z    = thu_att2(3);

	thu3.data = thu_att3(0);
	ang3.x    = thu_att3(1);
	ang3.y    = thu_att3(2);
	ang3.z    = thu_att3(3);

	thu4.data = thu_att4(0);
	ang4.x    = thu_att4(1);
	ang4.y    = thu_att4(2);
	ang4.z    = thu_att4(3);

	//debug line
	// ROS_INFO_STREAM("out1: "<<thu_att1);
	// ROS_INFO_STREAM("out2: "<<thu_att2);
	// ROS_INFO_STREAM("out3: "<<thu_att3);
}

Eigen::Vector4f controller_class::alloc(Eigen::Vector3f f,double psi_cmd)
{
	Eigen::Vector4f thu_att;
	Eigen::Vector3f fw;
	fw=r_mat*f;

	thu_att(0)=sqrt(fw(0)*fw(0)+fw(1)*fw(1)+fw(2)*fw(2));
	thu_att(1)=asin((fw(1)*cos(psi_cmd)-fw(0)*sin(psi_cmd))/thu_att(0));
    thu_att(2)=asin(-(fw(0)*cos(psi_cmd)+fw(1)*sin(psi_cmd))/sqrt((fw(0)*cos(psi_cmd)+fw(1)*sin(psi_cmd))*(fw(0)*cos(psi_cmd)+fw(1)*sin(psi_cmd))+ fw(2)*fw(2)));
	thu_att(3)=psi_cmd;
	return thu_att;
}

//third level function defination, chk
void controller_class::calc_grad_v()
{
	//ROS_INFO_STREAM("enter calc grad function.");
	//calc the rotation matrix
	r_mat=euler_to_rotation_mat(main_eular_angles);

	//ROS_INFO_STREAM("")
	//calc position error vector
	err_xyz(0)=main_position.x-nominal_position.x;
	err_xyz(1)=main_position.y-nominal_position.y;
	err_xyz(2)=main_position.z-nominal_position.z;
	position_err = r_mat.transpose()*err_xyz;

	//calc grad
	grad_v = kp_pos*position_err;
	// ROS_INFO_STREAM("grag_v err_xyz: "<<err_xyz);
	// ROS_INFO_STREAM("grad_v: "<<grad_v);
	// ROS_INFO_STREAM("pos_err: "<<position_err);
}

//decleration of calc_grad_w function, chk
void controller_class::calc_grad_w()
{
	//ROS_INFO_STREAM("enter calc_grad_w function.");

	//get tool position in body frame
	de_0(0) = param.tool_pos.x;
	de_0(1) = param.tool_pos.y;
	de_0(2) = param.tool_pos.z;
	r_mat=euler_to_rotation_mat(main_eular_angles);
	r_mat_d=euler_to_rotation_mat(nominal_euler_angles);

	Eigen::Matrix3f rotation_error = r_mat_d.transpose()*r_mat - r_mat.transpose()*r_mat_d;
	att_err(0)=rotation_error(2,1);
	att_err(1)=rotation_error(0,2);
	att_err(2)=rotation_error(1,0);

	Eigen::Matrix3f eta_matrix = kp_att*rotation_error;
	Eigen::Vector3f eta_vector ;
	eta_vector(0)=eta_matrix(2,1);
	eta_vector(1)=eta_matrix(0,2);
	eta_vector(2)=eta_matrix(1,0);
	//cross() axb vector
	grad_w = -de_0.cross(kp_pos*r_mat.transpose()*err_xyz)+eta_vector; 
	//this line has a problem, kp_pos -> kp_att
	
	// ROS_INFO_STREAM("grag_w err_xyz: "<<err_xyz);
	// ROS_INFO_STREAM("grad_w: "<<grad_w);
	// ROS_INFO_STREAM("att_err: "<<att_err);
}    

void controller_class::rotation_kine_inerse()//kinematic inverse of rotation
{
	double phi_bar   = nominal_euler_angles.x;
	double theta_bar = nominal_euler_angles.y;
	double psi_bar   = nominal_euler_angles.z;

	//ROS_INFO_STREAM("phi_bar: "<<phi_bar<<" theta_bar: "<<theta_bar<<" psi_bar: "<<psi_bar);

	//first calc u as the input
	double phi_u = gain_1(phi_bar,param.omega_phi);
	double theta_u = gain_1(theta_bar,param.omega_theta);
	double psi_u = gain_1(psi_bar,param.omega_psi);

	//second use RK4 solve ODE2
	phi_bar_dot_last = phi_bar_dot_y;
	RK4_02(phi_u,param.omega_phi,param.epsilon_phi,
        &phi_bar_dot_x, &phi_bar_dot_y, &phi_bar_dot_z, TIME);
	
	theta_bar_dot_last = theta_bar_dot_y;
	RK4_02(theta_u,param.omega_theta,param.epsilon_theta,
        &theta_bar_dot_x, &theta_bar_dot_y, &theta_bar_dot_z, TIME);
	
	psi_bar_dot_last = psi_bar_dot_y;
	RK4_02(psi_u,param.omega_psi,param.epsilon_psi,
        &psi_bar_dot_x, &psi_bar_dot_y, &psi_bar_dot_z, TIME);
	

	//third calc the different value of y
	differentiator_lastest(&phi_bar_dot_last,&phi_bar_dot_y,&phi_bar_dot,TIME);
	differentiator_lastest(&theta_bar_dot_last,&theta_bar_dot_y,&theta_bar_dot,TIME);
	differentiator_lastest(&psi_bar_dot_last,&psi_bar_dot_y,&psi_bar_dot,TIME);
	
	// ROS_INFO_STREAM("phi last: "<<phi_bar_dot_last<<" phi now: "<<phi_bar_dot_y<<" phi dot: "<<phi_bar_dot);
	// ROS_INFO_STREAM("theta last: "<<theta_bar_dot_last<<" theat now: "<<theta_bar_dot_y<<" theta dot: "<<theta_bar_dot);
	// ROS_INFO_STREAM("psi last: "<<psi_bar_dot_last<<" psi now: "<<psi_bar_dot_y<<" psi dot: "<<psi_bar_dot);

	//kaidi wang comment it on 2021.7.12
	// double phi_bar_dot  =differentiator(
	// 	phi_bar,
	// 	init_euler_angles.x,
	// 	param.omega_phi,
	// 	param.epsilon_phi,
	// 	&phi_bar_dot_last,
	// 	&dot_phi_integra_val_1,
	// 	&dot_phi_integra_val_2);
	// double theta_bar_dot=differentiator(
	// 	theta_bar,
	// 	init_euler_angles.y,
	// 	param.omega_theta,
	// 	param.epsilon_theta,
	// 	&theta_bar_dot_last,
	// 	&dot_theta_integra_val_1,
	// 	&dot_theta_integra_val_2);
	// double psi_bar_dot  =differentiator(
	// 	psi_bar,
	// 	init_euler_angles.z,
	// 	param.omega_psi,
	// 	param.epsilon_psi,
	// 	&psi_bar_dot_last,
	// 	&dot_psi_integra_val_1,
	// 	&dot_psi_integra_val_2);
	
	kinematics_pseudoinverse(phi_bar,phi_bar_dot,theta_bar,theta_bar_dot,psi_bar,psi_bar_dot);
}
void controller_class::trans_kine_inverse()//translational kinematic inverse
{
	//calc dot 
	double xd_bar   = nominal_position.x;
	double yd_bar   = nominal_position.y;
	double zd_bar   = nominal_position.z;

	double xd_u = gain_1(xd_bar,param.omega_x);
	double yd_u = gain_1(yd_bar,param.omega_y);
	double zd_u = gain_1(zd_bar,param.omega_z);

	//second use RK4 solve ODE2
	xd_dot_last = xd_dot_y;
	RK4_02(xd_u,param.omega_x,param.epsilon_x,
        &xd_dot_x, &xd_dot_y, &xd_dot_z, TIME);
	
	yd_dot_last = yd_dot_y;
	RK4_02(yd_u,param.omega_y,param.epsilon_y,
        &yd_dot_x, &yd_dot_y, &yd_dot_z, TIME);
	
	zd_dot_last = zd_dot_y;
	RK4_02(zd_u,param.omega_z,param.epsilon_z,
        &zd_dot_x, &zd_dot_y, &zd_dot_z, TIME);
	
	//third calc the different value of y
	differentiator_lastest(&xd_dot_last,&xd_dot_y,&xd_dot,TIME);
	differentiator_lastest(&yd_dot_last,&yd_dot_y,&yd_dot,TIME);
	differentiator_lastest(&zd_dot_last,&zd_dot_y,&zd_dot,TIME);

	// ROS_INFO_STREAM("xd last: "<<xd_dot_last<<" xd now: "<<xd_dot_y<<" xd dot: "<<xd_dot);
	// ROS_INFO_STREAM("yd last: "<<yd_dot_last<<" yd now: "<<yd_dot_y<<" yd dot: "<<yd_dot);
	// ROS_INFO_STREAM("zd last: "<<zd_dot_last<<" zd now: "<<zd_dot_y<<" zd dot: "<<zd_dot);
	// ROS_INFO_STREAM("xd_dot:  "<<xd_dot);
	// ROS_INFO_STREAM("yd_dot:  "<<yd_dot);
	// ROS_INFO_STREAM("zd_dot:  "<<zd_dot);

	//kaidi wang comment these line on 2021.7.12
	// double xd_dot = differentiator(
	// 	nominal_positon.x,
	// 	init_position.x,
	// 	param.omega_x,
	// 	param.epsilon_x,
	// 	&xd_dot_last,
	// 	&dot_xd_integra_val_1,
	// 	&dot_xd_integra_val_2);
	// double yd_dot = differentiator(
	// 	nominal_positon.y,
	// 	init_position.y,
	// 	param.omega_y,
	// 	param.epsilon_y,
	// 	&yd_dot_last,
	// 	&dot_yd_integra_val_1,
	// 	&dot_yd_integra_val_2);
	// double zd_dot = differentiator(
	// 	nominal_positon.z,
	// 	init_position.z,
	// 	param.omega_z,
	// 	param.epsilon_z,
	// 	&zd_dot_last,
	// 	&dot_zd_integra_val_1,
	// 	&dot_zd_integra_val_2);
	calc_vd(xd_dot,yd_dot,zd_dot);
}

//calc_epsilon_v function chk
void controller_class::calc_epsilon_v()
{
	//calc epsilon v function
	Eigen::Matrix3f de_matrix;
	de_matrix(0,0)=de_matrix(1,1)=de_matrix(2,2)=0;
	de_matrix(0,1)=-de_0(2);
	de_matrix(0,2)=de_0(1);
	de_matrix(1,0)=de_0(2);
	de_matrix(1,2)=-de_0(0);
	de_matrix(2,0)=-de_0(1);
	de_matrix(2,1)=de_0(0);

	Eigen::Vector3f wd;
	Eigen::Vector3f vdd;

	wd=omega_d;
	vdd=vd;

	//output param
	epsilon_d_v = de_matrix*r_mat.transpose()*r_mat_d*wd + r_mat.transpose()*r_mat_d*vd;

	// ROS_INFO_STREAM("epsilon_d_v: "<<epsilon_d_v);
}
void controller_class::calc_epsilon_w()
{
	//calc epsilon w function
	Eigen::Vector3f wd;
	wd = omega_d;	
	epsilon_d_w = r_mat.transpose()*r_mat_d*wd;
	// ROS_INFO_STREAM("epsilon_d_w: "<<epsilon_d_w);
}

//fourth level function
//this function is not good
double controller_class::differentiator
(
	double u, 
	double initial_angle,
	double omega,
	double epsilon,
	double *output_last,
	double *integral_val_1,
	double *integral_val_2)
{
	double u_dot = 0;
	
	double y1 = 0;
	double y2 = 0;
	double y3 = 0;
	y1 = gain_1(u,omega);

	//calc input1 var
	double input1 = 0;
	input1=y1-y2-y3;

	double int_output1 = 0;
	int_output1=integrator(input1,0,integral_val_1,detla_time);

	double int_output2 = 0;
	int_output2=integrator(int_output1,initial_angle,integral_val_2,detla_time);

	//y1=gain_1(u,omega);
	y2=gain_2(int_output1,omega,epsilon);
	y3=gain_1(int_output2,omega);

	//differentiator
	u_dot = (int_output2-*output_last)/detla_time;
	*output_last = int_output2;
	//this line is not good
	return u_dot;
}

// Runge-Kutta method to solve the ODE
void controller_class::RK4_02(double u,double omega,double epsilon,double *x,double *y,double *z,double h)
{
	double k1,k2,k3,k4,l1,l2,l3,l4;
    k1 = df(*x,*y,*z);
    l1 = d2f(u,omega,epsilon,*x,*y,*z);

    k2 = df(*x+h/2,*y+h*k1*0.5,*z+h*l1*0.5);
    l2 = d2f(u,omega,epsilon,*x+h/2,*y+h*k1*0.5,*z+h*l1*0.5);

    k3 = df(*x+h/2,*y+h*k2*0.5,*z+h*l2*0.5);
    l3 = d2f(u,omega,epsilon,*x+h/2,*y+h*k2*0.5,*z+h*l2*0.5);

    k4 = df(*x+h,*y+h*k3,*z+h*l3);
    l4 = d2f(u,omega,epsilon,*x+h,*y+h*k3,*z+h*l3);

    *x = *x+h;
    *y = *y+h*(k1+2*k2+2*k3+k4)/6;
    *z = *z+h*(l1+2*l2+2*l3+l4)/6;
}

//differential equation
void controller_class::differentiator_lastest(double *last,double *now,double *re,double h)
{
    *re = (*now-*last)/h;
}

void controller_class::kinematics_pseudoinverse
(double phi_bar, double phi_bar_dot,
double theta_bar, double theta_bar_dot,
double psi_bar, double psi_bar_dot)
{
	double p_bar = 0;
	double q_bar = 0;
	double r_bar = 0;

	p_bar = phi_bar_dot-psi_bar_dot*sin(theta_bar);
	q_bar = theta_bar_dot*cos(phi_bar)+psi_bar_dot*sin(phi_bar)*cos(theta_bar);
	r_bar = -theta_bar_dot * sin(phi_bar) + psi_bar_dot * cos(phi_bar) * cos(theta_bar);
	
	omega_d(0)=p_bar;
	omega_d(1)=q_bar;
	omega_d(2)=r_bar;
	// ROS_INFO_STREAM("omega_d: "<<omega_d);
}

//calc_vd chk
void controller_class::calc_vd(double xd_dot, double yd_dot, double zd_dot)
{
	Eigen::Vector3f xd_w_dot;
	xd_w_dot(0)=xd_dot;
	xd_w_dot(1)=yd_dot;
	xd_w_dot(2)=zd_dot;
    // xd_w_dot= calc_vd_p_controller(kp,nominal_position,main_position);
	vd = r_mat_d.transpose()*xd_w_dot;
	// ROS_INFO_STREAM("vd: "<<vd);
}

void controller_class::calc_thrust_autopilot_pid()
{
	Eigen::Vector3f vec_1;
	vec_1(0)=vec_1(1)=0;
	vec_1(2)=1;

	Eigen::Vector3f v_nominal;
	Eigen::Vector3f err_v;
	Eigen::Vector3f real_velocity;
	v_nominal = r_mat.inverse()*calc_vd_p_controller(kp,nominal_position,main_position);
	real_velocity(0) = main_velocity.x;
	real_velocity(1) = main_velocity.y;
	real_velocity(2) = main_velocity.z;
	// calc err_v
	err_v = real_velocity - v_nominal;
	// calc integral_err_v
	Eigen::Vector3f err_v_integral;
    double err_v_integral_x = integrator(err_v(0),0,&err_v_integral_x_ls,detla_time);
	double err_v_integral_y = integrator(err_v(1),0,&err_v_integral_y_ls,detla_time);
	double err_v_integral_z = integrator(err_v(2),0,&err_v_integral_z_ls,detla_time);
    err_v_integral(0) = err_v_integral_x;
    err_v_integral(1) = err_v_integral_y;
    err_v_integral(2) = err_v_integral_z;

	// param.S3Q_mass*epsilon_d_v_dot
	thrust_u = 
	-kd_pos*err_v-ki_pos*err_v_integral+
		(-param.S3Q_mass*g*r_mat.transpose()*vec_1);
	// ROS_INFO_STREAM("gaoling thrust: "<<thrust_u);
}

void controller_class::calc_thrust_new()
{
	Eigen::Vector3f vec_1;
	vec_1(0)=vec_1(1)=0;
	vec_1(2)=1;
	//calc ev
	Eigen::Vector3f vc;
	Eigen::Vector3f vd,vd_w;
	Eigen::Vector3f a_r;
	vc(0)=main_velocity.x;
	vc(1)=main_velocity.y;
	vc(2)=main_velocity.z;

	vd_w(0)=nominal_velocity.x;
	vd_w(1)=nominal_velocity.y;
	vd_w(2)=nominal_velocity.z;

	vd = r_mat_d*vd_w;
	ev=vc-vd;
	// calc the accelector reference
	a_r = kp*ev;
	Eigen::Vector3f position_err_init;

	double position_err_init_x = integrator(position_err(0),init_position.x,&pos_err_init_x_integar_last,detla_time);
	double position_err_init_y = integrator(position_err(1),init_position.y,&pos_err_init_y_integar_last,detla_time);
	double position_err_init_z = integrator(position_err(2),init_position.z,&pos_err_init_z_integar_last,detla_time);
	
	// kaidi wang, 2022.5.30 integrator without init data
	// double position_err_init_x = integrator(position_err(0),0,&pos_err_init_x_integar_last,detla_time);
	// double position_err_init_y = integrator(position_err(1),0,&pos_err_init_y_integar_last,detla_time);
	// double position_err_init_z = integrator(position_err(2),0,&pos_err_init_z_integar_last,detla_time);
	// ROS_INFO_STREAM(init_position);
	//debug lines
	// ROS_INFO_STREAM("position_err_init_x: "<<position_err_init_x);
	// ROS_INFO_STREAM("position_err_init_y: "<<position_err_init_y);
	// ROS_INFO_STREAM("position_err_init_z: "<<position_err_init_z);

	position_err_init(0)=position_err_init_x;
	position_err_init(1)=position_err_init_y;
	position_err_init(2)=position_err_init_z;

	thrust_u = 
		param.S3Q_mass*a_r+
		(param.S3Q_mass*omege).cross(vd)-
		kd_pos*ev-
		grad_v-
		ki_pos*position_err_init+
		(-param.S3Q_mass*g*r_mat.transpose()*vec_1);

	ROS_INFO_STREAM("**************************************************");
	ROS_INFO_STREAM("1: "<<param.S3Q_mass*epsilon_d_v_dot);
	ROS_INFO_STREAM("2: "<<(param.S3Q_mass*omege).cross(epsilon_d_v));
	ROS_INFO_STREAM("d: "<<kd_pos*ev);
	ROS_INFO_STREAM("p: "<<grad_v);
	ROS_INFO_STREAM("i: "<<ki_pos*position_err_init);
	ROS_INFO_STREAM("6: "<<(-param.S3Q_mass*g*r_mat.transpose()*vec_1));	
	ROS_INFO_STREAM("thrust: "<<thrust_u);

}


//gain_1 function chk
double controller_class::gain_1(double u,double omega)//chk
{
	double y;
	y=omega*omega*u;
	return y;
}

//gain_2 function chk
double controller_class::gain_2(double u,double omega,double epsilon)
{
	double y;
	y=2*omega*epsilon*u;
	return y;
}

double controller_class::integrator(double input,double orign,double *ls,double t)
{
	double output;
	*ls = *ls+input*t;
    // ROS_INFO_STREAM("last_sum:"<<*ls);
	//last_sum = input;
	output = *ls+orign;
	return output;
}

//calc rotation matrix from euler angle, chk
Eigen::Matrix3f controller_class::euler_to_rotation_mat(geometry_msgs::Point angle)
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

// Runge-Kutta method child function
double controller_class::d2f(double u,double omega,double epsilon,double x,double y,double z)
{
	double f2;
	f2=u-2*omega*epsilon*z-omega*omega*y;
	return f2;
}
double controller_class::df(double x,double y,double z)
{
	double f1;
	f1=z;
	return f1;
}

//other function
Eigen::Matrix3f controller_class::pos_mat(double x,double y,double z)
{
	Eigen::Matrix3f s;
	s(0,0)=s(1,1)=s(2,2)=0;
	s(0,1)=-z;
	s(0,2)=y;
	s(1,0)=z;
	s(1,2)=-x;
	s(2,0)=-y;
	s(2,1)=x;
	return s;
}

//callback funtion define list
//setpoint position, chk 
void controller_class::nominal_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_position = *msg;
	//ROS_INFO_STREAM("nominal_position: ");
	//ROS_INFO_STREAM(nominal_position);
}
//setpoint attitude, eular angles, chk
void controller_class::nominal_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_euler_angles = *msg;
	// ROS_INFO_STREAM("nominal_euler_angles: ");
	// ROS_INFO_STREAM(nominal_euler_angles);
}

void controller_class::nominal_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg )
{
	nominal_velocity = *msg;
}
void controller_class::nominal_body_rate_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_body_rate = *msg;
}


//position back, chk
void controller_class::main_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_position = *msg;
	// ROS_INFO_STREAM("main_position: ");
	// ROS_INFO_STREAM(main_position);
}
//velocity back, chk
void controller_class::main_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_velocity = *msg;
	// ROS_INFO_STREAM("main_velocity: ");
	// ROS_INFO_STREAM(main_velocity);
}
//attitude back, chk
void controller_class::main_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_eular_angles = *msg;
	// ROS_INFO_STREAM("main_eular_angles: ");
	// ROS_INFO_STREAM(main_eular_angles);
}
//body rates back,chk
void controller_class::main_body_rates_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_body_rates = *msg;
	// ROS_INFO_STREAM("main_body_rates: ");
	// ROS_INFO_STREAM(main_body_rates);
}
//init attitude,chk
void controller_class::init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_euler_angles = *msg;
	// ROS_INFO_STREAM("init_euler_angles: ");
	// ROS_INFO_STREAM(init_euler_angles);
}
//init position,chk
void controller_class::init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_position = *msg;
	// ROS_INFO_STREAM("init_position: ");
	// ROS_INFO_STREAM(init_position);
}
//init body rate,chk
void controller_class::init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_body_rate = *msg;
	// ROS_INFO_STREAM("init_body_rate: ");
	// ROS_INFO_STREAM(init_body_rate);
}
//init velocity 
void controller_class::init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_velocity = *msg;
	// ROS_INFO_STREAM("init_velocity: ");
	// ROS_INFO_STREAM(init_velocity);
}
//mode switch callback function
void controller_class::mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
	mode_switch = *msg;
}

//control start chk
void controller_class::control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg)
{
	start_pub_att = *msg;
}
//psi_bias 1 chk
void controller_class::psi_bias_1_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node1_yaw = *msg;
}
//psi_bias 2 chk
void controller_class::psi_bias_2_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node2_yaw = *msg;
}
//psi_bias 3 chk
void controller_class::psi_bias_3_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node3_yaw = *msg;
}

// get reference thrust value, this is set by user
void controller_class::ref_thrust_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	ref_thrust = *msg;
}

// get reference torque value, this is set by user
void controller_class::ref_torque_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	ref_torque = *msg;
}

void controller_class::sensor_thrust_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sensor_thrust = *msg;
}

void controller_class::sensor_torque_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sensor_torque = *msg;
}


//member function list
void controller_class::output_publish(
	geometry_msgs::Point ang1,
	geometry_msgs::Point ang2,
	geometry_msgs::Point ang3,
	geometry_msgs::Point ang4,
	std_msgs::Float64 thu1,
	std_msgs::Float64 thu2,
	std_msgs::Float64 thu3,
	std_msgs::Float64 thu4)
{
	//publish message output, this fucntion need to revise more
	// if()
	//geometry_msgs::Point ang1_pub = range_judge(ang1,0.2);
	//geometry_msgs::Point ang2_pub = range_judge(ang2,0.2);
	//geometry_msgs::Point ang3_pub = range_judge(ang3,0.2);
	//geometry_msgs::Point ang4_pub = range_judge(ang4,0.2);
	
	//ang1_pub=ang1;
	//ang2_pub=ang2;
	//ang3_pub=ang3;
	//ang4_pub=ang4;

	angle1_pub.publish(ang1);
	angle2_pub.publish(ang2);
	angle3_pub.publish(ang3);
	angle4_pub.publish(ang4);

	thrust1_pub.publish(thu1);
	thrust2_pub.publish(thu2);
	thrust3_pub.publish(thu3);
	thrust4_pub.publish(thu4);

	ROS_INFO_STREAM("ang1,x,y,z :"<<ang1.x<<" "<<ang1.y<<" "<<ang1.z);
	ROS_INFO_STREAM("ang2,x,y,z :"<<ang2.x<<" "<<ang2.y<<" "<<ang2.z);
	ROS_INFO_STREAM("ang3,x,y,z :"<<ang3.x<<" "<<ang3.y<<" "<<ang3.z);
	ROS_INFO_STREAM("ang4,x,y,z :"<<ang4.x<<" "<<ang4.y<<" "<<ang4.z);

	ROS_INFO_STREAM("thu1 :"<<thu1.data);
	ROS_INFO_STREAM("thu2 :"<<thu2.data);
	ROS_INFO_STREAM("thu3 :"<<thu3.data);
	ROS_INFO_STREAM("thu4 :"<<thu4.data);

}

geometry_msgs::Point controller_class::range_judge(geometry_msgs::Point origin, double range)
{
	geometry_msgs::Point ret;
	if(abs(origin.x) >range)
	{
		ret.x = 0;
	}
	else
	{
		ret.x = origin.x;
	}

	if(abs(origin.y) >range)
	{
		ret.y = 0;
	}
	else
	{
		ret.y = origin.y;
	}
	ret.z = origin.z;
	return ret;
}

// add a speed control loop which calc the vd function
Eigen::Vector3f controller_class::calc_vd_p_controller(Eigen::Matrix3f p, 
	geometry_msgs::Point nominal_pos,
	geometry_msgs::Point main_pos)
{
	Eigen::Vector3f res;
	Eigen::Vector3f err_pos;
	err_pos(0)=main_pos.x - nominal_pos.x;
	err_pos(1)=main_pos.y - nominal_pos.y;
	err_pos(2)=main_pos.z - nominal_pos.z;
	res = p*err_pos;
	return res;
}


//calc run times, this is a global variable
void controller_class::time_run()
{
	//count cycle times
	run_times++;
}

// reconfigure variable function 
void controller_class::reconfigure_param(config_param var_arg)
{
	param.kp_x=var_arg.kp_x;
	param.kp_y=var_arg.kp_y;
	param.kp_z=var_arg.kp_z;
	param.ki_x=var_arg.ki_x;
	param.ki_y=var_arg.ki_y;
	param.ki_z=var_arg.ki_z;
	param.kd_x=var_arg.kd_x;
	param.kd_y=var_arg.kd_y;
	param.kd_z=var_arg.kd_z;

	param.kp_phi=var_arg.kp_phi;
	param.kp_theta=var_arg.kp_theta;
	param.kp_psi=var_arg.kp_psi;
	param.ki_phi=var_arg.ki_phi;
	param.ki_theta=var_arg.ki_theta;
	param.ki_psi=var_arg.ki_psi;
	param.kd_phi=var_arg.kd_phi;
	param.kd_theta=var_arg.kd_theta;
	param.kd_psi=var_arg.kd_psi;
}

//timer callback function
void controller_class::calc_cb(const ros::TimerEvent&)
{
	// enter timer loop
	// ROS_INFO_STREAM("reconfigure parameter");
	// reconfigure_param(var);
	// ROS_INFO("input loop: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",             
	// 	param.kp_x, 
    //   	param.kp_y,             
	// 	param.kp_z,
    //   	param.ki_x,
    //   	param.ki_y,
    //   	param.ki_z,
	// 	param.kd_x, 
    //   	param.kd_y,             
	// 	param.kd_z,
    //   	param.kp_phi,
    //   	param.kp_theta,
    //   	param.kp_psi,
	// 	param.ki_phi,
    //   	param.ki_theta,
    //   	param.ki_psi,
	// 	param.kd_phi,
    //   	param.kd_theta,
    //   	param.kd_psi);
    ROS_INFO_STREAM("four air_crafts controller...");

	if(start_pub_att.data)
	{
		ROS_INFO_STREAM("impedance controller loop...");

		//if(init_position.x!=0 && init_position.y !=0 && init_position.z!=0 && init_euler_angles.z!=0)
		//if(mode_switch.data)
		{
			omege(0)=main_body_rates.x;
			omege(1)=main_body_rates.y;
			omege(2)=main_body_rates.z;
			time_run();
			//debug line
			ROS_INFO_STREAM("run times: "<< run_times);
			controller_body();//controller function
			control_allocation();//allocation function
			output_publish(ang1,ang2,ang3,ang4,thu1,thu2,thu3,thu4);
		}
	}
	else{
		//controller();
		main_position = init_position;
		main_velocity = init_velocity;
		main_eular_angles = init_euler_angles;
		main_body_rates = init_body_rate;
		nominal_position = init_position;      //setpoint position
		nominal_euler_angles = init_euler_angles; 
	}
	//angle1_pub.publish(ang1);
}

// void callback(controller::paramConfig &config, uint32_t level)
// {
//   	ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",             
// 		config.kp_x, 
//       	config.kp_y,             
// 		config.kp_z,
//       	config.ki_x,
//       	config.ki_y,
//       	config.ki_z,
// 		config.kd_x, 
//       	config.kd_y,             
// 		config.kd_z,
//       	config.kp_phi,
//       	config.kp_theta,
//       	config.kp_psi,
// 		config.ki_phi,
//       	config.ki_theta,
//       	config.ki_psi,
// 		config.kd_phi,
//       	config.kd_theta,
//       	config.kd_psi);
// 	var.kp_x = config.kp_x;
// 	var.kp_y = config.kp_y;
// 	var.kp_z = config.kp_z;
// 	var.ki_x = config.ki_x;
// 	var.ki_y = config.ki_y;
// 	var.ki_z = config.ki_z;
// 	var.kd_x = config.kd_x;
// 	var.kd_y = config.kd_y;
// 	var.kd_z = config.kd_z;
// 	var.kp_phi = config.kp_phi;
// 	var.kp_theta= config.kp_theta;
// 	var.kp_psi = config.kp_psi;
// 	var.ki_phi = config.ki_phi;
// 	var.ki_theta = config.ki_theta;
// 	var.ki_psi = config.ki_psi;
// 	var.kd_phi = config.kd_phi;
// 	var.kd_theta = config.kd_theta;
// 	var.kd_psi = config.kd_psi;
// 	// check wether the param is changed
// 	ROS_INFO("var change: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",             
// 		var.kp_x, 
//       	var.kp_y,             
// 		var.kp_z,
//       	var.ki_x,
//       	var.ki_y,
//       	var.ki_z,
// 		var.kd_x, 
//       	var.kd_y,             
// 		var.kd_z,
//       	var.kp_phi,
//       	var.kp_theta,
//       	var.kp_psi,
// 		var.ki_phi,
//       	var.ki_theta,
//       	var.ki_psi,
// 		var.kd_phi,
//       	var.kd_theta,
//       	var.kd_psi);
// }

void controller_class::calc_f_dir()
{
	// get F_ref_B
	Eigen::Vector3f F_ref_B;
	F_ref_B(0) = ref_thrust.x;
	F_ref_B(1) = ref_thrust.y;
	F_ref_B(2) = ref_thrust.z;
	
	ef(0) = sensor_thrust.x - ref_thrust.x;
	ef(1) = sensor_thrust.y - ref_thrust.y;
	ef(2) = sensor_thrust.z - ref_thrust.z;

	f_dir = (1/param.S3Q_mass)*(-F_ref_B+Kfp*ef+Kfi*integrator_vector3f(ef,ef_init,&ef_ls,detla_time));

	tao_dir_wavy(0) = f_dir(0);
	tao_dir_wavy(1) = f_dir(1);
	tao_dir_wavy(2) = f_dir(2);
	tao_dir_wavy(3) = tao_dir_wavy(4) = tao_dir_wavy(5) =0;
}

void controller_class::calc_tao_imp_wavy()
{
	ep_wavy(0) = position_err(0);
	ep_wavy(1) = position_err(1);
	ep_wavy(2) = position_err(2);
	ep_wavy(3) = att_err(0);
	ep_wavy(4) = att_err(1);
	ep_wavy(5) = att_err(2);

	ev_wavy(0) = ev(0);
	ev_wavy(1) = ev(1);
	ev_wavy(2) = ev(2);
	ev_wavy(0) = ew(0);
	ev_wavy(1) = ew(1);
	ev_wavy(2) = ew(2);

	Eigen::Matrix3f identityMatrix = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f zeroMatrix = Eigen::Matrix3f::Zero();
	R.setZero();
	R.block<3,3>(0,0) = identityMatrix;
	R.block<3,3>(3,3) = r_mat;

	II6.setZero();
	II6.block<3,3>(0,0) = identityMatrix;
	II6.block<3,3>(3,3) = identityMatrix;

	calc_tao_ext_wavy_hat();

	tao_imp_wavy = (R.transpose()*Mv.inverse()*R-II6)*tao_ext_wavy_hat- Dv*ev_wavy - Kv*ep_wavy;
}

void controller_class::calc_tao_cmd_wavy()
{
	tao_cmd_wavy_star = tao_dir_wavy+tao_imp_wavy+C*v_wavy+g_vector6;

	tao_com_wavy(0) = 0 ;
	tao_com_wavy(1) = 0 ;
	tao_com_wavy(2) = 0 ;

	tao_cmd_wavy=tao_cmd_wavy_star+tao_com_wavy;
}


void controller_class::calc_tao_ext_wavy_hat()
{
	inter_tao_ext = tao_cmd_wavy - C*v_wavy-g_vector6+tao_ext_wavy_hat;
	tao_ext_wavy_hat.head(3) = (M*v_wavy).head(3)-integrator_vector3f(inter_tao_ext.head(3),inter_tao_ext_init.head(3),&inter_tao_ext_ls_h3,detla_time);
	tao_ext_wavy_hat.tail(3) = (M*v_wavy).tail(3)-integrator_vector3f(inter_tao_ext.tail(3),inter_tao_ext_init.tail(3),&inter_tao_ext_ls_t3,detla_time);
}

void controller_class::impedance_controller()
{
	C.setZero();
	Eigen::Matrix3f omega0_s;
	Eigen::Matrix3f zero ;//J;
	zero.setZero();

	// J.setZero();
	// J(0,0) = param.I_sys.x;
	// J(1,1) = param.I_sys.y;
	// J(2,2) = param.I_sys.z;

	omega0_s = pos_mat(main_body_rates.x, main_body_rates.y, main_body_rates.z); 
	C.block<3,3>(0,0) = param.S3Q_mass*omega0_s;
	C.block<3,3>(0,3) = zero;
	C.block<3,3>(3,0) = zero;
	C.block<3,3>(3,3) = -J*omega0_s;


	g_vector6.head(3) = -param.S3Q_mass*g*r_mat.transpose()*e3;
    g_vector6.tail(3) = zero_vec;

	calc_tao_ext_wavy_hat();
	calc_f_dir();
	calc_tao_imp_wavy();
	calc_tao_cmd_wavy();

}

// thrust 
Eigen::Vector3f controller_class::thrust_impedance_controller(geometry_msgs::Point f_e, geometry_msgs::Point f)
{
	Eigen::Vector3f ret;

	Eigen::Vector3f ref_f;
	Eigen::Vector3f sen_f;
	ref_f(0) = f_e.x;
	ref_f(1) = f_e.y;
	ref_f(2) = f_e.z;

	sen_f(0) = f.x;
	sen_f(1) = f.y;
	sen_f(2) = f.z;
	// update err_f
	Eigen::Vector3f err_force;
	err_force = sen_f - ref_f;

	ret = kp_f*err_force + ki_f*integrator_vector3f(err_force,ef_init,&ef_ls,detla_time);
	return ret;
}

// torque
Eigen::Vector3f controller_class::torque_impedance_controller(geometry_msgs::Point t_e, geometry_msgs::Point t)
{
	Eigen::Vector3f ret;
	Eigen::Vector3f ref_t;
	Eigen::Vector3f sen_t;
	ref_t(0) = t_e.x;
	ref_t(1) = t_e.y;
	ref_t(2) = t_e.z;

	sen_t(0) = t.x;
	sen_t(1) = t.y;
	sen_t(2) = t.z;
	// update err_f
	Eigen::Vector3f err_torque;
	err_torque = sen_t - ref_t;

	ROS_INFO_STREAM(err_torque);
	ROS_INFO_STREAM(integrator_vector3f(err_torque,et_init,&et_ls,detla_time));
	ret = kp_t*err_torque + ki_t*integrator_vector3f(err_torque,et_init,&et_ls,detla_time);
	return ret;
}

// inegrator vector3f function
Eigen::Vector3f controller_class::integrator_vector3f(Eigen::Vector3f input,Eigen::Vector3f orign,Eigen::Vector3f *ls,double t)
{
	Eigen::Vector3f output;
	*ls= *ls+input*t;
    // ROS_INFO_STREAM("last_sum:"<<*ls);
	//last_sum = input;
	output = *ls+orign;
	return output;
}

void controller_class::differentiator_lastest_vector3f(Eigen::Vector3f *last,Eigen::Vector3f *now,Eigen::Vector3f *re,double h)
{
    *re = (*now-*last)/h;
}

//main function
int main(int argc, char **argv)
{
	ros::init(argc,argv,"controller");
	ros::NodeHandle nh;//create a node handle

	// dynamic_reconfigure::Server<controller::paramConfig> server;
	// dynamic_reconfigure::Server<controller::paramConfig>::CallbackType f; 

	// f = boost::bind(&callback, _1, _2);
	// server.setCallback(f);

	controller_class controller_node(&nh);//init some param and then start the controller 
	ros::spin();
	return 0;
}
