// kaidi wang, zean zhen 2023.3.1
// linear ava controller
#include <controller/linear_ava_controller.h>
#include <controller/uav_normal_function.h>

// Interface functions 
// init publisher list
void linear_ava_controller::init_publisher()
{
    angle1_pub = nh.advertise<geometry_msgs::Point>("/angle1",10,this);
	angle2_pub = nh.advertise<geometry_msgs::Point>("/angle2",10,this);
	angle3_pub = nh.advertise<geometry_msgs::Point>("/angle3",10,this);

	thrust1_pub = nh.advertise<std_msgs::Float64>("/thrust1",10,this);
	thrust2_pub = nh.advertise<std_msgs::Float64>("/thrust2",10,this);
	thrust3_pub = nh.advertise<std_msgs::Float64>("/thrust3",10,this);

	// only for simulation publish state
	pos_sim_pub = nh.advertise<geometry_msgs::Point>("/main_position_sim",10,this);
	euler_sim_pub = nh.advertise<geometry_msgs::Point>("/main_euler_angles_sim",10,this);
	body_rate_sim_pub = nh.advertise<geometry_msgs::Point>("/main_body_rates_sim",10,this);
	velocity_sim_pub = nh.advertise<geometry_msgs::Point>("/main_velocity_sim",10,this);

	// publish thrust and torque data
	thrust_pub = nh.advertise<geometry_msgs::Point>("/thrust_data",10,this);
    torque_pub = nh.advertise<geometry_msgs::Point>("/torque_data",10,this);
}
// init subscriber list
void linear_ava_controller::init_subscriber()
{
    nominal_position_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_position",1,&linear_ava_controller::nominal_position_sub_cb,this);
	nominal_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_euler_angles",1,&linear_ava_controller::nominal_eular_angles_sub_cb, this);
	nominal_velocity_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_velocity",1,&linear_ava_controller::nominal_velocity_sub_cb, this);
	nominal_body_rate_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_body_rate",1,&linear_ava_controller::nominal_body_rate_sub_cb, this);

	main_position_sub = nh.subscribe<geometry_msgs::Point>
	("/main_position",1,&linear_ava_controller::main_position_sub_cb,this);
	main_velocity_sub = nh.subscribe<geometry_msgs::Point>
	("/main_velocity",1,&linear_ava_controller::main_velocity_sub_cb,this);
	main_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	("/main_euler_angles",1,&linear_ava_controller::main_eular_angles_sub_cb,this);
	main_body_rates_sub = nh.subscribe<geometry_msgs::Point>
	("/main_body_rates",1,&linear_ava_controller::main_body_rates_sub_cb,this);

	init_euler_angles_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_euler_angles_cmd",1,&linear_ava_controller::init_euler_angles_cmd_sub_cb,this);
	init_pos_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_position_cmd",1,&linear_ava_controller::init_pos_cmd_sub_cb,this);
	init_body_rates_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_body_rates_cmd",1,&linear_ava_controller::init_body_rates_cmd_sub_cb,this);
	init_velocity_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_velocity_cmd",1,&linear_ava_controller::init_velocity_cmd_sub_cb,this);
	
	control_start_sub_att = nh.subscribe<std_msgs::Bool>
	("/start_pub_att",1,&linear_ava_controller::control_start_sub_att_cb,this);
	mode_switch_sub       = nh.subscribe<std_msgs::Bool>
	("/mode_switch",1,&linear_ava_controller::mode_switch_sub_cb,this);
	psi_bias_1_sub = nh.subscribe<std_msgs::Float64>
	("/psi_bias_1",1,&linear_ava_controller::psi_bias_1_sub_cb,this);
	psi_bias_2_sub = nh.subscribe<std_msgs::Float64>
	("/psi_bias_2",1,&linear_ava_controller::psi_bias_2_sub_cb,this);
	psi_bias_3_sub = nh.subscribe<std_msgs::Float64>
	("/psi_bias_3",1,&linear_ava_controller::psi_bias_3_sub_cb,this);

}

// output publish list
void linear_ava_controller::output_publish(geometry_msgs::Point ang1,geometry_msgs::Point ang2,geometry_msgs::Point ang3,std_msgs::Float64 thu1,std_msgs::Float64 thu2,std_msgs::Float64 thu3)
{
    // geometry_msgs::Point ang1_pub = range_judge(ang1,0.2);
	// geometry_msgs::Point ang2_pub = range_judge(ang2,0.2);
	// geometry_msgs::Point ang3_pub = range_judge(ang3,0.2);
	geometry_msgs::Point ang1_pub;
	geometry_msgs::Point ang2_pub;
	geometry_msgs::Point ang3_pub;

	ang1_pub = ang1;
	ang2_pub = ang2;
	ang3_pub = ang3;

	angle1_pub.publish(ang1_pub);
	angle2_pub.publish(ang2_pub);
	angle3_pub.publish(ang3_pub);
	thrust1_pub.publish(thu1);
	thrust2_pub.publish(thu2);
	thrust3_pub.publish(thu3);

	plant_input plant_input;
	// plant_input.angle1 = range_judge(ang1,0.2);
	// plant_input.angle2 = range_judge(ang2,0.2);
	// plant_input.angle3 = range_judge(ang3,0.2);
	plant_input.thrust1 = thu1;
	plant_input.thrust2 = thu2;
	plant_input.thrust3 = thu3;
	// plant_input.init_pos = init_position;
	// plant_input.init_att = init_euler_angles;
	// plant_input.init_vel = init_velocity;
	// plant_input.init_body_rate = init_body_rate;
}


void linear_ava_controller::init()
{
	//define 9.8 as gravity
	g=GRAVITY;

	// define a I33 matrix as a local variable
	// Eigen::MatrixXf I33(3,3);
	I33.setIdentity(3,3) ;
	// define a I55 matrix as a local variable
	Eigen::MatrixXf I55(5,5);
	I55.setIdentity(5,5) ;
	// define a I51 matrix as a local variable
	Eigen::MatrixXf I51(5,1);
	I51.setIdentity(5,1) ;
	Eigen::MatrixXf I15(1,5);
	I15.setIdentity(1,5) ;
	// define a I66 matrix as a local variable
	Eigen::MatrixXf I66(6,6);
	I66.setIdentity(6,6) ;
	// define a I61 matrix as a local variable
	Eigen::MatrixXf I61(6,1);
	I61.setIdentity(6,1) ;
	// define a I65 matrix as a local variable
	Eigen::MatrixXf I65(6,5);
	I65.setIdentity(6,5) ;
	// define a 0 matrix as a local variable
	// Eigen::MatrixXf zero_33(3,3);
	zero_33.setZero(3,3); 
	// define a 0 vector as a local variable
	Eigen::Vector3f zero_v3;
	zero_v3(0)=zero_v3(1)=zero_v3(2)=0;
	Eigen::Vector3f e3;
	e3(0)=0; e3(1)=0; e3(2)=1;

	update_mian = 0; 

	pos_s1 = I33;
	pos_s2 = I33;
	pos_s3 = I33;
	r_mat=I33;
	r_mat_d=I33;

	S_bar = I66;
	S_bar_t = I66;

	J = I33;
	J_bar = I33;

	//dynamics used
	M = I66;	
	C = I66;
	G = I61;

	Ma = I55;
	Ca = I55;
	Cau = I51;
	ga = I51;

	grad_phi = I15;
	grad_v = zero_v3;
	grad_w = zero_v3;

	delta_a = I65;
	delta_u  = I61;
	Omega = I66;
	omega_at = I65;
	omega_ut = I61;
	V = I61;
	V_d = I61;
	V_d_dot = I61;
	Va = I51;
	ua = I51;
	Va_d = I51;
	ua_d = I51;
	Va_d_dot = I51;
	ua_d_dot = I51;
	U = I61;
	
	uu = 0;
	Vu = 0;
	Vu_d = 0;
	Vu_d_dot = 0;

	r_e0 = zero_v3;
	r_ew = zero_v3;

	err_xyz = zero_v3;
	de_0 = zero_v3;
	position_err = zero_v3;
	att_err = zero_v3;	
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
	main_position = init_position;
	main_velocity = init_velocity;
	main_eular_angles = init_euler_angles;
	main_body_rates = init_body_rate;

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

	kp_pos(0,0)=param_linear.kp_x;
	kp_pos(1,1)=param_linear.kp_y;
	kp_pos(2,2)=param_linear.kp_z;
	kp_pos(0,1)=kp_pos(0,2)=kp_pos(1,0)=
	kp_pos(1,2)=kp_pos(2,0)=kp_pos(2,1)=0;
	//ROS_INFO_STREAM(kp_pos);

	ki_pos(0,0)=param_linear.ki_x;
	ki_pos(1,1)=param_linear.ki_y;
	ki_pos(2,2)=param_linear.ki_z;
	ki_pos(0,1)=ki_pos(0,2)=ki_pos(1,0)=
	ki_pos(1,2)=ki_pos(2,0)=ki_pos(2,1)=0;

	kd_pos(0,0)=param_linear.kd_x;
	kd_pos(1,1)=param_linear.kd_y;
	kd_pos(2,2)=param_linear.kd_z;
	kd_pos(0,1)=kd_pos(0,2)=kd_pos(1,0)=
	kd_pos(1,2)=kd_pos(2,0)=kd_pos(2,1)=0;

/***********************autopilot control PID theory************************/
	kp(0,0)=param_linear.kpx;
	kp(1,1)=param_linear.kpy;
	kp(2,2)=param_linear.kpz;
	kp(0,1)=kp(0,2)=kp(1,0)=kp(1,2)=kp(2,0)=kp(2,1)=0;
	// init err_v integral last number
	err_v_integral_x_ls = 0;
    err_v_integral_y_ls = 0;
    err_v_integral_z_ls = 0;
/***************************************************************************/

	kp_att(0,0)=param_linear.kp_phi;
	kp_att(1,1)=param_linear.kp_theta;
	kp_att(2,2)=param_linear.kp_psi;
	kp_att(0,1)=kp_att(0,2)=kp_att(1,0)=
	kp_att(1,2)=kp_att(2,0)=kp_att(2,1)=0;

	ki_att(0,0)=param_linear.ki_phi;
	ki_att(1,1)=param_linear.ki_theta;
	ki_att(2,2)=param_linear.ki_psi;
	ki_att(0,1)=ki_att(0,2)=ki_att(1,0)=
	ki_att(1,2)=ki_att(2,0)=ki_att(2,1)=0;

	kd_att(0,0)=param_linear.kd_phi;
	kd_att(1,1)=param_linear.kd_theta;
	kd_att(2,2)=param_linear.kd_psi;
	kd_att(0,1)=kd_att(0,2)=kd_att(1,0)=
	kd_att(1,2)=kd_att(2,0)=kd_att(2,1)=0;

	// kv = delta_a.transpose()*kd*delta_a;
	kv(0,0) = param_linear.kd_x ;
	kv(1,1) = param_linear.kd_y;
	kv(2,2) = param_linear.kd_z ;
	kv(3,3) = param_linear.kd_theta ;
	kv(4,4) = param_linear.kd_psi ;
	kv(0,1) = kv(0,2) = kv(0,3) = kv(0,4) = 0;
	kv(1,0) = kv(1,2) = kv(1,3) = kv(1,4) = 0;
	kv(2,0) = kv(2,1) = kv(2,3) = kv(2,4) = 0;
	kv(3,0) = kv(3,1) = kv(3,2) = kv(3,4) = 0;
	kv(4,0) = kv(4,1) = kv(4,2) = kv(4,3) = 0;


	// kv = delta_a.transpose()*kd*delta_a;
	ki(0,0) = param_linear.ki_x ;
	ki(1,1) = param_linear.ki_y;
	ki(2,2) = param_linear.ki_z ;
	ki(3,3) = param_linear.ki_theta ;
	ki(4,4) = param_linear.ki_psi ;
	ki(0,1) = ki(0,2) = ki(0,3) = ki(0,4) = 0;
	ki(1,0) = ki(1,2) = ki(1,3) = ki(1,4) = 0;
	ki(2,0) = ki(2,1) = ki(2,3) = ki(2,4) = 0;
	ki(3,0) = ki(3,1) = ki(3,2) = ki(3,4) = 0;
	ki(4,0) = ki(4,1) = ki(4,2) = ki(4,3) = 0;

	kd.block<3,3>(0,0) =  kd_pos;
	kd.block<3,3>(3,3) =  kd_att;
	kd.block<3,3>(0,3) = kd.block<3,3>(3,0) = zero_33;

	
	J(0,0)=param_linear.I_sys.x;
	J(1,1)=param_linear.I_sys.y;
	J(2,2)=param_linear.I_sys.z;
	J(0,1)=J(0,2)=J(1,0)=J(1,2)=J(2,0)=J(2,1)=0;

	omege(0)=main_body_rates.x;
	omege(1)=main_body_rates.y;
	omege(2)=main_body_rates.z;


	// define M matrix
	M.block<3,3>(0,0) = param_linear.sys_mass*I33;
	M.block<3,3>(0,3) = zero_33;
	M.block<3,3>(3,0) = zero_33;
	// M.block<3,3>(3,3) = J_bar;
	M.block<3,3>(3,3) = J;

	vector3_zero(0)=vector3_zero(1)=vector3_zero(2) =0;
    G.block<3,1>(0,0) = e3;
	G.block<3,1>(3,0) = vector3_zero;

	sum_ea_front_ls = vector3_zero;
    sum_ea_end_ls = vector3_zero;

	sum_grad_phi_t_front_ls = vector3_zero;
    sum_grad_phi_t_end_ls = vector3_zero;
	//get pos matrix
	pos_s1=pos_mat(param_linear.fly1_pos.x,param_linear.fly1_pos.y,param_linear.fly1_pos.z);
	pos_s2=pos_mat(param_linear.fly2_pos.x,param_linear.fly2_pos.y,param_linear.fly2_pos.z);
	pos_s3=pos_mat(param_linear.fly3_pos.x,param_linear.fly3_pos.y,param_linear.fly3_pos.z);
    //calc B matrix
	B.block<3,3>(0,0)=I;
	B.block<3,3>(0,3)=I;
	B.block<3,3>(0,6)=I;
	B.block<3,3>(3,0)=pos_s1;
	B.block<3,3>(3,3)=pos_s2;
	B.block<3,3>(3,6)=pos_s3;

	// B_full matrix for calc lamda
	B_full.block<1,9>(0,0) = B.block(0,0,1,9);
	B_full.block<1,9>(1,0) = B.block(1,0,1,9);
	B_full.block<1,9>(2,0) = B.block(2,0,1,9);
	B_full.block<1,9>(3,0) = B.block(4,0,1,9);
	B_full.block<1,9>(4,0) = B.block(5,0,1,9);

	//calc omega_at matrix
	omega_at.block<6,1>(0,0) = B.block(0,0,6,1);
    omega_at.block<6,1>(0,1) = B.block(0,1,6,1);
    omega_at.block<6,1>(0,2) = B.block(0,2,6,1);
    omega_at.block<6,1>(0,3) = B.block(0,4,6,1);
    omega_at.block<6,1>(0,4) = B.block(0,5,6,1);
	
	//calc delta_u matrix
   	delta_u(0) = 0;
    delta_u(1) = -0.0413;
    delta_u(2) = 0;
    delta_u(3) = 1;
    delta_u(4) = 0;
    delta_u(5) = 0;

	//calc delta_a matrix
	Eigen::MatrixXf inter_a(5, 5);
	inter_a = (omega_at.transpose()*M.inverse()*omega_at).inverse();// check
	delta_a =  M.inverse()*omega_at*inter_a; // check

	//calc omega_ut matrix
	Eigen::MatrixXf inter_u(5, 5);
	inter_u = (delta_u.transpose()*M*delta_u).inverse();// check
	omega_ut = M*delta_u*inter_u; // check

	// calc Ma Ca Cau ga
	Ma = delta_a.transpose()*M*delta_a; // check

	Omega.block<6,5>(0,0) = omega_at;
	Omega.block<6,1>(0,5) = omega_ut;
	// ROS_INFO_STREAM("Omega: " <<Omega);

	delta.block<6,5>(0,0) = delta_a;
	delta.block<6,1>(0,5) = delta_u;

	// calc vector of unit d1-d2
	pos1(0) = param_linear.fly1_pos.x;
	pos1(1) = param_linear.fly1_pos.y;
	pos1(2) = param_linear.fly1_pos.z;

	pos2(0) = param_linear.fly2_pos.x;
	pos2(1) = param_linear.fly2_pos.y;
	pos2(2) = param_linear.fly2_pos.z;

	//calc r_e0 r_ew r_d_omega
	// unit_inter1 = pos2 - pos1;
	unit_inter1 = pos1 - pos2;
	float unit_inter2;
	unit_inter2= sqrt(unit_inter1(0)*unit_inter1(0)+unit_inter1(1)*unit_inter1(1)+unit_inter1(2)*unit_inter1(2));
	r_e0  = unit_inter1/unit_inter2;

	de_0(0) = param_linear.tool_pos.x;
	de_0(1) = param_linear.tool_pos.y;
	de_0(2) = param_linear.tool_pos.z;
	de_s = pos_mat(de_0(0),de_0(1),de_0(2));
}

// range judge function
geometry_msgs::Point linear_ava_controller::range_judge(geometry_msgs::Point origin, double range)
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
// controller body
void linear_ava_controller::controller()
{	
	// update state
	
	// ROS_INFO_STREAM("*********************update_main_pos******************");
	// ROS_INFO_STREAM("ctrl node main_eular: " <<main_eular_angles);

	full_actuacted_dynamics_eqution();//full actuacted variables,Ma,Ca,ga... 
	controller_body();//controller function
	control_allocation();//allocation function
	output_publish(ang1,ang2,ang3,thu1,thu2,thu3);
	//debug line
	// ROS_INFO_STREAM("***************************************");
	// ROS_INFO_STREAM(init_euler_angles);
	// ROS_INFO_STREAM(init_position);
}

// plant body
void linear_ava_controller::plant()
{
	// if (param_linear.flag_ask_noise == 0) 
    // {
    // 	param_linear.ask_noise();
    // 	param_linear.flag_ask_noise =1;
    // }
    //param_linear.init_pos(init_position,init_velocity,init_euler_angles,init_body_rate);
    //debug line
    //ROS_INFO_STREAM("main_eular_angles: " << main_eular_angles);
	Eigen::Vector3f u_f3;
	u_f3(0) = U(0,0);
	u_f3(1) = U(1,0);
	u_f3(2) = U(2,0);
	Eigen::Vector3f u_e3;
	u_e3(0) = U(3,0);
	u_e3(1) = U(4,0);
	u_e3(2) = U(5,0);
	
    if((thu1.data!=0)&&(thu2.data!=0)&&(thu3.data!=0)&&(!isnan(thu1.data))&&(!isnan(thu2.data))&&(!isnan(thu3.data)))
    {
    	param_linear.composition(main_eular_angles,ang1,ang2,ang3,thu1,thu2,thu3,u_f3,u_e3);
		// param_linear.dynamic_model();
		param_linear.system_plant();
		//update_main_pos();
		publish_main_pos();
    }
    else
    {
        ROS_INFO_STREAM("controller output is nan");
        // ROS_INFO_STREAM("init section..");
        // main_position = init_position;
        // main_eular_angles = init_euler_angles;
        // main_body_rates = init_body_rate;
        // main_velocity = init_velocity;

        //publish message ouput
		pos_sim_pub.publish(main_position) ;
		euler_sim_pub.publish(main_eular_angles) ;
		body_rate_sim_pub.publish(main_body_rates) ;
		velocity_sim_pub.publish(main_velocity);

    }
    
	
}

void linear_ava_controller::controller_body()
{
	
	calc_grad_phi();//calculate grad phi
	calc_epsilon_d(); //calculate epsilon_d
	calc_U();//calculate ua,U
}

// control_allocation function
void linear_ava_controller::control_allocation()
{
	// ctrl_allocation();
	ctrl_allocation_new();
}

void linear_ava_controller::dynamics_eqution()
{
	//r_mat=euler_to_rotation_mat(main_eular_angles);
	//get pos matrix
	// pos_s1=pos_mat(param_linear.fly1_pos.x,param_linear.fly1_pos.y,param_linear.fly1_pos.z);
	// pos_s2=pos_mat(param_linear.fly2_pos.x,param_linear.fly2_pos.y,param_linear.fly2_pos.z);
	// pos_s3=pos_mat(param_linear.fly3_pos.x,param_linear.fly3_pos.y,param_linear.fly3_pos.z);

	// Eigen::Matrix3f I33;
	// I33(0,0)=I33(1,1)=I33(2,2)=1;
	// I33(0,1)=I33(0,2)=I33(1,0)=I33(1,2)=I33(2,0)=I33(2,1)=0;

	// Eigen::MatrixXf zero_33(3,3);

	// Eigen::Matrix3f J0;
	// J0(0,1)=J0(0,2)=J0(1,0)=J0(1,2)=J0(2,0)=J0(2,1)=0;
	// J0(0,0) = param_linear.I_platform.x;
	// J0(1,1) = param_linear.I_platform.y;
	// J0(2,2) = param_linear.I_platform.z;
	
	// define C matrix check

	//debug line
	// ROS_INFO_STREAM("*********************dynamics_eqution******************");
	// ROS_INFO_STREAM("ctrl node M matrix: " <<M);
	// ROS_INFO_STREAM("ctrl node C matrix: " <<C);
	// ROS_INFO_STREAM("ctrl node G matrix: " <<G);
}

void linear_ava_controller::full_actuacted_dynamics_eqution()
{
	
	// // define M matrix
	// M.block<3,3>(0,0) = param_linear.sys_mass*I33;
	// M.block<3,3>(0,3) = zero_33;
	// M.block<3,3>(3,0) = zero_33;
	// // M.block<3,3>(3,3) = J_bar;
	// M.block<3,3>(3,3) = J;

	Eigen::Matrix3f omega0_s;
	omega0_s = pos_mat(main_body_rates.x, main_body_rates.y, main_body_rates.z); 
	C.block<3,3>(0,0) = param_linear.sys_mass*omega0_s;
	C.block<3,3>(0,3) = zero_33;
	C.block<3,3>(3,0) = zero_33;
	C.block<3,3>(3,3) = -J*omega0_s;

	Ca = delta_a.transpose()*C*delta_a; // check
	Cau = delta_a.transpose()*C*delta_u; 
	//ga = delta_a.transpose()*G;
	//debug line
	// ROS_INFO_STREAM("*********************full_actuacted_dynamics_eqution******************");
	// ROS_INFO_STREAM("ctrl node M matrix: " <<M);
	// ROS_INFO_STREAM("ctrl node B matrix: " <<B);
	// ROS_INFO_STREAM("ctrl node omega_at matrix: " <<omega_at);
	// ROS_INFO_STREAM("ctrl node omega_ut matrix: " <<omega_ut);
	// ROS_INFO_STREAM("ctrl node delta_u matrix: " <<delta_u);
	// ROS_INFO_STREAM("ctrl node delta_a matrix: " <<delta_a);
	// ROS_INFO_STREAM("ctrl node omega matrix: " <<Omega);
	// ROS_INFO_STREAM("ctrl node omega_at.transpose() matrix: " <<omega_at.transpose());
	// ROS_INFO_STREAM("ctrl node M.inverse() matrix: " <<M.inverse());
	// ROS_INFO_STREAM("ctrl node inter_a matrix: " <<inter_a);
	// ROS_INFO_STREAM("ctrl node delta_a matrix: " <<delta_a);
	// ROS_INFO_STREAM("ctrl node Ca matrix: " <<Ca);
	// ROS_INFO_STREAM("ctrl node Cau matrix: " <<Cau);
	// ROS_INFO_STREAM("ctrl node ga matrix: " <<ga);
}

void linear_ava_controller::calc_grad_phi()
{
	r_mat=euler_to_rotation_mat(main_eular_angles);//main_euler_angle rotation mat
	r_mat_d=euler_to_rotation_mat(nominal_euler_angles);//nominal_euler_angles rotation mat

	//calc position error vector
	err_xyz(0)=main_position.x-nominal_position.x;
	err_xyz(1)=main_position.y-nominal_position.y;
	err_xyz(2)=main_position.z-nominal_position.z;
	position_err = r_mat.transpose()*err_xyz;
	// position_err = err_xyz; //r_mat.transpose()*

	r_ew = r_mat*r_e0;
	r_d_omega = r_mat_d*r_e0;

	r_e0_s = pos_mat(r_e0(0),r_e0(1),r_e0(2));
	r_ew_s = pos_mat(r_ew(0),r_ew(1),r_ew(2));
	//calc S_bar
	S_bar.block<3,3>(0,0) = I33;
	S_bar.block<3,3>(0,3) = -de_s;
	S_bar.block<3,3>(3,0) = zero_33;
	S_bar.block<3,3>(3,3) = r_e0_s;//-r_e0_s

	//calc grad_phi
	Eigen::Matrix<float, 6, 1> grad_phi_inter;
	grad_phi_inter.block<3,1>(0,0) = kp_pos*position_err;// check
	grad_phi_inter.block<3,1>(3,0) = kp_att*r_mat.transpose()*r_d_omega; // check
	// grad_phi_inter.block<3,1>(3,0) = kp_att*r_mat*r_d_omega; // check
	grad_phi = grad_phi_inter.transpose()*(S_bar)*(delta_a); // check

	// Eigen::Matrix<float, 6, 1> grad_phi_inter;
	// ROS_INFO_STREAM(" r_e0 "<< r_e0);
	// ROS_INFO_STREAM(" r_mat_d "<< r_mat_d);
	// ROS_INFO_STREAM(" r_ew "<< r_ew);

	// ROS_INFO_STREAM(" r_d_omega "<< r_d_omega);
	// ROS_INFO_STREAM(" position_err "<< position_err);
	// ROS_INFO_STREAM(" r_mat.transpose()*r_d_omega "<< r_mat.transpose()*r_d_omega);
	// ROS_INFO_STREAM(" grad_phi_inter "<< grad_phi_inter);
}

double linear_ava_controller::gain_1(double u,double omega)
{
	double y;
	y=omega*omega*u; 
	return y;
}

void linear_ava_controller::calc_epsilon_d()
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
	double u_u = gain_1(u_bar,param_linear.omega_u);
	double v_u = gain_1(v_bar,param_linear.omega_v);
	double w_u = gain_1(w_bar,param_linear.omega_w);

	//second use RK4 solve ODE2
	epsilon_d_v_dot_u_last = epsilon_d_v_dot_u_y;
	RK4_02(u_u,param_linear.omega_u,param_linear.epsilon_u,
        &epsilon_d_v_dot_u_x, &epsilon_d_v_dot_u_y, &epsilon_d_v_dot_u_z, TIME);
	
	epsilon_d_v_dot_v_last = epsilon_d_v_dot_v_y;
	RK4_02(v_u,param_linear.omega_v,param_linear.epsilon_v,
        &epsilon_d_v_dot_v_x, &epsilon_d_v_dot_v_y, &epsilon_d_v_dot_v_z, TIME);

	epsilon_d_v_dot_w_last = epsilon_d_v_dot_w_y;
	RK4_02(w_u,param_linear.omega_w,param_linear.epsilon_w,
        &epsilon_d_v_dot_w_x, &epsilon_d_v_dot_w_y, &epsilon_d_v_dot_w_z, TIME);

	
	//third calc the different value of y
	differentiator_lastest(&epsilon_d_v_dot_u_last,&epsilon_d_v_dot_u_y,&epsilon_d_v_dot_u,TIME);
	differentiator_lastest(&epsilon_d_v_dot_v_last,&epsilon_d_v_dot_v_y,&epsilon_d_v_dot_v,TIME);
	differentiator_lastest(&epsilon_d_v_dot_w_last,&epsilon_d_v_dot_w_y,&epsilon_d_v_dot_w,TIME);

	//debug lines
	// ROS_INFO_STREAM("u_u: "<<u_u);
	// ROS_INFO_STREAM("TIME: "<<TIME);
	// ROS_INFO_STREAM("epsilon_d_v_dot_u_y: "<<epsilon_d_v_dot_u_y);
	// ROS_INFO_STREAM("epsilon_d_v_dot_u: "<<epsilon_d_v_dot_u);
	epsilon_d_v_dot(0)=epsilon_d_v_dot_u;
	epsilon_d_v_dot(1)=epsilon_d_v_dot_v;
	epsilon_d_v_dot(2)=epsilon_d_v_dot_w;

	//output param
	double p_bar   = epsilon_d_w(0);
	double q_bar   = epsilon_d_w(1);
	double r_bar   = epsilon_d_w(2);

	//first calc u as the input
	double p_u = gain_1(p_bar,param_linear.omega_p);
	double q_u = gain_1(q_bar,param_linear.omega_q);
	double r_u = gain_1(r_bar,param_linear.omega_r);

	//second use RK4 solve ODE2
	epsilon_d_w_dot_p_last = epsilon_d_w_dot_p_y;
	RK4_02(p_u,param_linear.omega_p,param_linear.epsilon_p,
        &epsilon_d_w_dot_p_x, &epsilon_d_w_dot_p_y, &epsilon_d_w_dot_p_z, TIME);
	
	epsilon_d_w_dot_q_last = epsilon_d_w_dot_q_y;
	RK4_02(q_u,param_linear.omega_q,param_linear.epsilon_q,
        &epsilon_d_w_dot_q_x, &epsilon_d_w_dot_q_y, &epsilon_d_w_dot_q_z, TIME);

	epsilon_d_w_dot_r_last = epsilon_d_w_dot_r_y;
	RK4_02(r_u,param_linear.omega_r,param_linear.epsilon_r,
        &epsilon_d_w_dot_r_x, &epsilon_d_w_dot_r_y, &epsilon_d_w_dot_r_z, TIME);
	
	//third calc the different value of y
	differentiator_lastest(&epsilon_d_w_dot_p_last,&epsilon_d_w_dot_p_y,&epsilon_d_w_dot_p,TIME);
	differentiator_lastest(&epsilon_d_w_dot_q_last,&epsilon_d_w_dot_q_y,&epsilon_d_w_dot_q,TIME);
	differentiator_lastest(&epsilon_d_w_dot_r_last,&epsilon_d_w_dot_r_y,&epsilon_d_w_dot_r,TIME);

	epsilon_d_w_dot(0)=epsilon_d_w_dot_p;
	epsilon_d_w_dot(1)=epsilon_d_w_dot_q;
	epsilon_d_w_dot(2)=epsilon_d_w_dot_r;

	delta.block<6,5>(0,0) = delta_a;
	delta.block<6,1>(0,5) = delta_u;

	// ROS_INFO_STREAM("ctrl node delta matrix: " <<delta);
	// ROS_INFO_STREAM("omega x delta matrix: " <<Omega.transpose()*delta);
	// ROS_INFO_STREAM("zero: " <<delta_a.transpose()*M*delta_u);

	Eigen::Vector3f vel;
	vel(0) = main_velocity.x;
	vel(1) = main_velocity.y;
	vel(2) = main_velocity.z;

	Eigen::Vector3f body_rates;
	body_rates(0) = main_body_rates.x;
	body_rates(1) = main_body_rates.y;
	body_rates(2) = main_body_rates.z;

	// body_rates(0) = main_body_rates.z;
	// body_rates(1) = main_body_rates.y;
	// body_rates(2) = main_body_rates.x;


	epsilon.block<3,1>(0,0) = vel;
	epsilon.block<3,1>(3,0) = body_rates;
	V = delta.inverse()*epsilon;
	Va(0) = V(0);//va
	Va(1) = V(1);
	Va(2) = V(2);
	Va(3) = V(3);
	Va(4) = V(4);
	Vu = V(5);//vu
	
	// ROS_INFO_STREAM("epsilon: "<< epsilon);
	// ROS_INFO_STREAM("Va: "<< Va);
	// ROS_INFO_STREAM("Vu: "<< Vu);

	// nominal velocity and nominal body rate
	// nominal_velocity
	// nominal_body_rate

	// epsilon_d_v(0) = nominal_velocity.x;
	// epsilon_d_v(1) = nominal_velocity.y;
	// epsilon_d_v(2) = nominal_velocity.z;
	// epsilon_d_w(0) = nominal_body_rate.x;
	// epsilon_d_w(1) = nominal_body_rate.y;
	// epsilon_d_w(2) = nominal_body_rate.z;
	epsilon_d.block<3,1>(0,0) = epsilon_d_v;
	epsilon_d.block<3,1>(3,0) = epsilon_d_w;

	V_d = delta.inverse()*epsilon_d;
	Va_d(0) = V_d(0);
	Va_d(1) = V_d(1);
	Va_d(2) = V_d(2);
	Va_d(3) = V_d(3);
	Va_d(4) = V_d(4);
	Vu_d = V_d(5);


	// add a integral item to control
	ea = Va - Va_d;
	ea_front(0) = ea(0);
	ea_front(1) = ea(1); 
	ea_front(2) = ea(2); 
	ea_end(0) = ea(3);
	ea_end(1) = ea(4); 
	ea_end(2) = 0; 
	// ROS_INFO_STREAM("V_d: "<< V_d);
	// ROS_INFO_STREAM("Va_d: "<< Va_d);

	epsilon_d_dot.block<3,1>(0,0) = epsilon_d_v_dot;
	epsilon_d_dot.block<3,1>(3,0) = epsilon_d_w_dot;

	//debug lines
	// ROS_INFO_STREAM("epsilon_d_v_dot: "<<epsilon_d_v_dot);
	// ROS_INFO_STREAM("epsilon_d_w_dot: "<<epsilon_d_w_dot);
	// ROS_INFO_STREAM("delta.inverse(): "<<delta.inverse());

	V_d_dot = delta.inverse()*epsilon_d_dot;
	Va_d_dot(0) = V_d_dot(0);
	Va_d_dot(1) = V_d_dot(1);
	Va_d_dot(2) = V_d_dot(2);
	Va_d_dot(3) = V_d_dot(3);
	Va_d_dot(4) = V_d_dot(4);
	Vu_d_dot = V_d_dot(5);

	// ROS_INFO_STREAM("epsilon_d_dot: "<< epsilon_d_dot);
	// ROS_INFO_STREAM("V_d_dot: "<< V_d_dot);

}

void linear_ava_controller::calc_U()
{

	// ROS_INFO_STREAM("kd: " <<kd);
	// ROS_INFO_STREAM("kv: " <<kv);

	//update G ga
	Eigen::Vector3f e3;
	e3(0)=0; e3(1)=0; e3(2)=1;
	G.block<3,1>(0,0) = -param_linear.sys_mass*g*r_mat.transpose()*e3;
	// ga = (delta.inverse()*G).head(5);
	ga = delta_a.transpose()*G;
	// position error calc 
	Eigen::Vector3f position_err_init;
	double position_err_init_x = integrator(position_err(0),init_position.x,&pos_err_init_x_integar_last,TIME);
	double position_err_init_y = integrator(position_err(1),init_position.y,&pos_err_init_y_integar_last,TIME);
	double position_err_init_z = integrator(position_err(2),init_position.z,&pos_err_init_z_integar_last,TIME);
	position_err_init(0)=position_err_init_x;
	position_err_init(1)=position_err_init_y;
	position_err_init(2)=position_err_init_z;

	// position loop integra item 
	// Eigen::VectorXf integra_pos_xy(5);
	// Eigen::VectorXf integra_pos_err(6);
	// integra_pos_err.head(3) = ki_pos*position_err_init;
	// integra_pos_err(2) = 0;
	// integra_pos_err(3) = integra_pos_err(4) = integra_pos_err(5) = 0;
	// integra_pos_xy = integra_pos_err.transpose()*(S_bar)*(delta_a);
	// position error calc end
	sum_ea_front = vec_integrator(ea_front,vector3_zero,&sum_ea_front_ls,TIME); // sum of ea first three items
    sum_ea_end = vec_integrator(ea_end,vector3_zero,&sum_ea_end_ls,TIME);   // sum of ea end three items
	sum_ea(0) = sum_ea_front(0);
	sum_ea(1) = sum_ea_front(1);
	sum_ea(2) = sum_ea_front(2);
	sum_ea(3) = sum_ea_end(0);
	sum_ea(4) = sum_ea_end(1);
 
    // i for grad_phi.transpose() grad_phi_t

	grad_phi_t_front(0) = grad_phi.transpose()(0);
	grad_phi_t_front(1) = grad_phi.transpose()(1); 
	grad_phi_t_front(2) = grad_phi.transpose()(2); 
	grad_phi_t_end(0) = grad_phi.transpose()(3);
	grad_phi_t_end(1) = grad_phi.transpose()(4); 
	grad_phi_t_end(2) = 0; 

	sum_grad_phi_t_front = vec_integrator(grad_phi_t_front,vector3_zero,&sum_grad_phi_t_front_ls,TIME); // sum of ea first three items
    sum_grad_phi_t_end = vec_integrator(grad_phi_t_end,vector3_zero,&sum_grad_phi_t_end_ls,TIME);   // sum of ea end three items
	sum_grad_phi_t(0) = sum_grad_phi_t_front(0);
	sum_grad_phi_t(1) = sum_grad_phi_t_front(1);
	sum_grad_phi_t(2) = sum_grad_phi_t_front(2);
	sum_grad_phi_t(3) = sum_grad_phi_t_end(0);
	sum_grad_phi_t(4) = sum_grad_phi_t_end(1);


	// ua =  ga - kv*(Va) - grad_phi.transpose() - ki*sum_grad_phi_t; //check
	// ua =  ga - kv*(Va) - grad_phi.transpose(); //check
	// ua = Cau*Vu + ga + Ma*Va_d_dot +Ca*Va_d - kv*(Va-Va_d) - grad_phi.transpose(); //check
	ua = Cau*Vu + ga + Ma*Va_d_dot +Ca*Va_d - kv*(Va-Va_d) - grad_phi.transpose()- ki*sum_ea;
	uu = 0; // check

	Eigen::VectorXf U_u(6);
	U_u.head(5)=ua;
	U_u(5)=0;

	U = Omega*U_u;// -integra_pos_err
	geometry_msgs::Point thu;
	geometry_msgs::Point tor;
	thu.x = U(0);
	thu.y = U(1);
	thu.z = U(2);
	tor.x = U(3);
	tor.y = U(4);
	tor.z = U(5);

	thrust_pub.publish(thu);
	torque_pub.publish(tor);
	ROS_INFO_STREAM("Omega: "<<Omega);
	// ROS_INFO_STREAM("front 4 items: " <<Cau*Vu + ga + Ma*Va_d_dot +Ca*Va_d);
	// ROS_INFO_STREAM(" 5 items: " <<Va-Va_d);
	// ROS_INFO_STREAM(" 6 items: " <<grad_phi.transpose());
	// U(3) = 0;
	//debug line
	// ROS_INFO_STREAM("ctrl node G matrix: " <<G);
	// ROS_INFO_STREAM("ctrl node ga matrix: " <<ga);
	// ROS_INFO_STREAM("ctrl node Va: " <<Va);
	// ROS_INFO_STREAM("ctrl node Va_d: " <<Va_d);
	// ROS_INFO_STREAM("ctrl node Va_d_dot: " <<Va_d_dot);
	// ROS_INFO_STREAM("ctrl node grad_phi: " <<grad_phi);
	// ROS_INFO_STREAM("ctrl node ua: " <<ua);
	// ROS_INFO_STREAM("C: " <<C);
	// ROS_INFO_STREAM("Ca: " <<Ca);
	// ROS_INFO_STREAM("ctrl node U: " <<U);
}

// allocation function 
void linear_ava_controller::ctrl_allocation_new()
{
	// ROS_INFO_STREAM("B_full: "<<B_full);
	// ROS_INFO_STREAM("B: "<<B);
	Eigen::VectorXf U_calc_lamda(5);
	U_calc_lamda(0) = U(0); 
	U_calc_lamda(1) = U(1); 
	U_calc_lamda(2) = U(2); 
	U_calc_lamda(3) = U(4); 
	U_calc_lamda(4) = U(5); 


	Eigen::VectorXf lamda(9);
	// lamda = B_full.transpose()*(B_full*B_full.transpose()).inverse()*ua;
	lamda = B_full.transpose()*(B_full*B_full.transpose()).inverse()*U_calc_lamda;

	// ROS_INFO_STREAM("lamda: "<<lamda);
	// ROS_INFO_STREAM("controller ua: "<<ua);

	lamda1(0) = lamda(0);
	lamda1(1) = lamda(1);
	lamda1(2) = lamda(2);
	lamda2(0) = lamda(3);
	lamda2(1) = lamda(4);
	lamda2(2) = lamda(5);
	lamda3(0) = lamda(6);
	lamda3(1) = lamda(7);
	lamda3(2) = lamda(8);

	//allocation function
	double psi_cmd1=nominal_euler_angles.z+param_linear.node1_yaw;
	double psi_cmd2=nominal_euler_angles.z+param_linear.node2_yaw;
	double psi_cmd3=nominal_euler_angles.z+param_linear.node3_yaw;

	Eigen::Vector4f thu_att1=alloc(lamda1,psi_cmd1);
	Eigen::Vector4f thu_att2=alloc(lamda2,psi_cmd2);
	Eigen::Vector4f thu_att3=alloc(lamda3,psi_cmd3);

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

	//debug line
	// ROS_INFO_STREAM("out1: "<<thu_att1);
	// ROS_INFO_STREAM("out2: "<<thu_att2);
	// ROS_INFO_STREAM("out3: "<<thu_att3);

}

// control allocation
void linear_ava_controller::ctrl_allocation()
{
	Eigen::MatrixXf B_calc(6, 6);
	B_calc = B*(B.transpose());
	// debug line
	// ROS_INFO_STREAM("B_calc: "<<B_calc);
	// ROS_INFO_STREAM("B_calc: "<<B_calc.inverse());

	// debug line
	// ROS_INFO_STREAM("B: "<<B);
	// ROS_INFO_STREAM("B_inverse: "<<B.transpose());
	// ROS_INFO_STREAM("B_calc: "<<B_calc);

	// this section need to be changed
	Eigen::MatrixXf inter1(3, 6);
	inter1.block<3,3>(0,0)=I;
	inter1.block<3,3>(0,3)=pos_s1.transpose();

	Eigen::MatrixXf inter2(3, 6);
	inter2.block<3,3>(0,0)=I;
	inter2.block<3,3>(0,3)=pos_s2.transpose();

	Eigen::MatrixXf inter3(3, 6);
	inter3.block<3,3>(0,0)=I;
	inter3.block<3,3>(0,3)=pos_s3.transpose();

	Eigen::MatrixXf B_calc_inv(6, 6);
	B_calc_inv << 
	0.3356,         0,         0,         0,    0.0556,         0,
         0,    0.3323,         0,    0.0133,         0,         0,
         0,         0,    0.3333,         0,         0,         0,
         0,    0.0133,         0,    0.0005,         0,         0,
   	0.0556,         0,         0,         0,    1.3889,         0,
         0,         0,         0,         0,         0,    1.3889;

	lamda1=inter1*(B_calc_inv*U);
	lamda2=inter2*(B_calc_inv*U);
	lamda3=inter3*(B_calc_inv*U);
	// lamda1=inter1*(B_calc.inverse()*U);
	// lamda2=inter2*(B_calc.inverse()*U);
	// lamda3=inter3*(B_calc.inverse()*U);
	
	// ROS_INFO_STREAM("lamda1: "<<lamda1);
	// ROS_INFO_STREAM("lamda2: "<<lamda2);
	// ROS_INFO_STREAM("lamda3: "<<lamda3);

	//allocation function
	double psi_cmd1=nominal_euler_angles.z+param_linear.node1_yaw;
	double psi_cmd2=nominal_euler_angles.z+param_linear.node2_yaw;
	double psi_cmd3=nominal_euler_angles.z+param_linear.node3_yaw;

	Eigen::Vector4f thu_att1=alloc(lamda1,psi_cmd1);
	Eigen::Vector4f thu_att2=alloc(lamda2,psi_cmd2);
	Eigen::Vector4f thu_att3=alloc(lamda3,psi_cmd3);

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

	//debug line
	// ROS_INFO_STREAM("out1: "<<thu_att1);
	// ROS_INFO_STREAM("out2: "<<thu_att2);
	// ROS_INFO_STREAM("out3: "<<thu_att3);
}

Eigen::Vector4f linear_ava_controller::alloc(Eigen::Vector3f f,double psi_cmd)
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

// Runge-Kutta method to solve the ODE
void linear_ava_controller::RK4_02(double u,double omega,double epsilon,double *x,double *y,double *z,double h)
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

void linear_ava_controller::kinematics_pseudoinverse
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
void linear_ava_controller::calc_vd(double xd_dot, double yd_dot, double zd_dot)
{
	Eigen::Vector3f xd_w_dot;
	xd_w_dot(0)=xd_dot;
	xd_w_dot(1)=yd_dot;
	xd_w_dot(2)=zd_dot;
    // xd_w_dot= calc_vd_p_controller(kp,nominal_position,main_position);
	vd = r_mat_d.transpose()*xd_w_dot;
	// ROS_INFO_STREAM("vd: "<<vd);
}

//kinematic inverse of rotation
void linear_ava_controller::rotation_kine_inerse()
{
	double phi_bar   = nominal_euler_angles.x;
	double theta_bar = nominal_euler_angles.y;
	double psi_bar   = nominal_euler_angles.z;

	//ROS_INFO_STREAM("phi_bar: "<<phi_bar<<" theta_bar: "<<theta_bar<<" psi_bar: "<<psi_bar);

	//first calc u as the input
	double phi_u = gain_1(phi_bar,param_linear.omega_phi);
	double theta_u = gain_1(theta_bar,param_linear.omega_theta);
	double psi_u = gain_1(psi_bar,param_linear.omega_psi);
	// ROS_INFO_STREAM("psi_u: "<<psi_u);

	//second use RK4 solve ODE2
	phi_bar_dot_last = phi_bar_dot_y;
	RK4_02(phi_u,param_linear.omega_phi,param_linear.epsilon_phi,
        &phi_bar_dot_x, &phi_bar_dot_y, &phi_bar_dot_z, TIME);
	// ROS_INFO_STREAM("phi_bar_dot_y: "<<phi_bar_dot_y);

	theta_bar_dot_last = theta_bar_dot_y;
	RK4_02(theta_u,param_linear.omega_theta,param_linear.epsilon_theta,
        &theta_bar_dot_x, &theta_bar_dot_y, &theta_bar_dot_z, TIME);
	// ROS_INFO_STREAM("theta_bar_dot_yr: "<<theta_bar_dot_y);

	psi_bar_dot_last = psi_bar_dot_y;
	RK4_02(psi_u,param_linear.omega_psi,param_linear.epsilon_psi,
        &psi_bar_dot_x, &psi_bar_dot_y, &psi_bar_dot_z, TIME);
	// ROS_INFO_STREAM("psi_bar_dot_y: "<<psi_bar_dot_y);

	//third calc the different value of y
	differentiator_lastest(&phi_bar_dot_last,&phi_bar_dot_y,&phi_bar_dot,TIME);
	differentiator_lastest(&theta_bar_dot_last,&theta_bar_dot_y,&theta_bar_dot,TIME);
	differentiator_lastest(&psi_bar_dot_last,&psi_bar_dot_y,&psi_bar_dot,TIME);
	
	phi_bar_dot = nominal_body_rate.x;
	theta_bar_dot = nominal_body_rate.y;
	psi_bar_dot = nominal_body_rate.z;
	kinematics_pseudoinverse(phi_bar,phi_bar_dot,theta_bar,theta_bar_dot,psi_bar,psi_bar_dot);
}

void linear_ava_controller::trans_kine_inverse()//translational kinematic inverse
{
	//calc dot 
	double xd_bar   = nominal_position.x;
	double yd_bar   = nominal_position.y;
	double zd_bar   = nominal_position.z;

	double xd_u = gain_1(xd_bar,param_linear.omega_x);
	double yd_u = gain_1(yd_bar,param_linear.omega_y);
	double zd_u = gain_1(zd_bar,param_linear.omega_z);

	//second use RK4 solve ODE2
	xd_dot_last = xd_dot_y;
	RK4_02(xd_u,param_linear.omega_x,param_linear.epsilon_x,
        &xd_dot_x, &xd_dot_y, &xd_dot_z, TIME);
	
	yd_dot_last = yd_dot_y;
	RK4_02(yd_u,param_linear.omega_y,param_linear.epsilon_y,
        &yd_dot_x, &yd_dot_y, &yd_dot_z, TIME);
	
	zd_dot_last = zd_dot_y;
	RK4_02(zd_u,param_linear.omega_z,param_linear.epsilon_z,
        &zd_dot_x, &zd_dot_y, &zd_dot_z, TIME);
	
	//third calc the different value of y
	differentiator_lastest(&xd_dot_last,&xd_dot_y,&xd_dot,TIME);
	differentiator_lastest(&yd_dot_last,&yd_dot_y,&yd_dot,TIME);
	differentiator_lastest(&zd_dot_last,&zd_dot_y,&zd_dot,TIME);

	// replace xd_dot, yd_dot, zd_dot to nominal velocity
	xd_dot = nominal_velocity.x ;
	yd_dot = nominal_velocity.y ;
	zd_dot = nominal_velocity.z ;
	calc_vd(xd_dot,yd_dot,zd_dot);
}

//calc_epsilon_v function chk
void linear_ava_controller::calc_epsilon_v()
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

void linear_ava_controller::calc_epsilon_w()
{
	//calc epsilon w function
	Eigen::Vector3f wd;
	wd = omega_d;	
	epsilon_d_w = r_mat.transpose()*r_mat_d*wd;
	// ROS_INFO_STREAM("epsilon_d_w: "<<epsilon_d_w);
}

// update ava plant output 
void linear_ava_controller::update_main_pos()
{
    // ROS_INFO_STREAM("ctrl node thu1: " << thu1);
    // ROS_INFO_STREAM("ctrl node thu2: " << thu2);
    // ROS_INFO_STREAM("ctrl node thu3: " << thu3);
    if((thu1.data!=0)&&(thu2.data!=0)&&(thu3.data!=0)&&(!isnan(thu1.data))&&(!isnan(thu2.data))&&(!isnan(thu3.data)))
    {
    	// update velocity 
	    main_velocity.x = param_linear.output.main_velocity.x;
	    main_velocity.y = param_linear.output.main_velocity.y;
	    main_velocity.z = param_linear.output.main_velocity.z;
	    // update position 
	    main_position.x = param_linear.output.main_position.x;
	    main_position.y = param_linear.output.main_position.y;
	    main_position.z = param_linear.output.main_position.z;
		// update body rate 
	    main_body_rates.x = param_linear.output.main_body_rate.x;
	    main_body_rates.y = param_linear.output.main_body_rate.y;
	    main_body_rates.z = param_linear.output.main_body_rate.z;
		// update position 
	    main_eular_angles.x = param_linear.output.main_attitude.x;
	    main_eular_angles.y = param_linear.output.main_attitude.y;
	    main_eular_angles.z = param_linear.output.main_attitude.z;


		//update_mian = 1;
    }
    else 
    {
	    main_position = init_position;
		main_velocity = init_velocity;
		main_eular_angles = init_euler_angles;
		main_body_rates = init_body_rate;
		// ROS_INFO_STREAM("ctrl node main_eular_angles: " << main_eular_angles);
		nominal_position = init_position;      //setpoint position
		nominal_euler_angles = init_euler_angles;
    }
    
    //debug line
// 	ROS_INFO_STREAM("*********************update_main_pos******************");
// 	ROS_INFO_STREAM("ctrl node main_velocity: " << main_velocity);
// 	ROS_INFO_STREAM("ctrl node main_position: " << main_position);
// 	ROS_INFO_STREAM("ctrl node main_body_rates: " << main_body_rates);
// 	ROS_INFO_STREAM("ctrl node main_eular_angles: " << main_eular_angles);
 }

void linear_ava_controller::publish_main_pos()
{

    if(main_eular_angles.x>PI)
    {
        main_eular_angles.x = fmod(main_eular_angles.x,2*PI);
    }
    if(main_eular_angles.x<-PI)
    {
        main_eular_angles.x = fmod(main_eular_angles.x,2*PI);
    }

	// give pitch angle a limit range
	// if(main_eular_angles.y>0.523599)
    // {
    //     main_eular_angles.y = 0.523599;
    // }
    // if(main_eular_angles.y<-0.523599)
    // {
    //     main_eular_angles.y = -0.523599;
    // }

    if(main_eular_angles.y>PI/2)
    {
        main_eular_angles.y = fmod(main_eular_angles.y,2*PI);
    }
    if(main_eular_angles.y<-PI/2)
    {
        main_eular_angles.y = fmod(main_eular_angles.y,2*PI);
    }


    if(main_eular_angles.z>PI)
    {
        // main_eular_angles.z = fmod(main_eular_angles.z,2*PI);
		main_eular_angles.z = main_eular_angles.z-2*PI;
    }
    if(main_eular_angles.z<-PI)
    {
        // main_eular_angles.z = fmod(main_eular_angles.z,2*PI);
		main_eular_angles.z = main_eular_angles.z+2*PI;
    }

    //publish message ouput
	pos_sim_pub.publish(main_position) ;
	euler_sim_pub.publish(main_eular_angles) ;
	body_rate_sim_pub.publish(main_body_rates) ;
	velocity_sim_pub.publish(main_velocity);
}

// calc run times, this is a global variable
void linear_ava_controller::time_run()
{
	//count cycle times
	run_times++;
}

// timer callback function
void linear_ava_controller::calc_cb(const ros::TimerEvent&)
{
	
    // ROS_INFO_STREAM("kaidi...");
    if(start_pub_att.data)
    {

    	time_run();
        // ROS_INFO_STREAM("run times: "<< run_times);
		
        if (flag_sim_real == 1)
        {
			if(run_times == 1)
			{
				ROS_INFO_STREAM("init state");
				param_linear.init_state(init_position,init_velocity,init_euler_angles,init_body_rate);
				main_position = init_position;
				main_velocity = init_velocity;
				main_eular_angles = init_euler_angles;
				main_body_rates = init_body_rate;
				nominal_position = init_position;      //setpoint position
				nominal_euler_angles = init_euler_angles;
			}
			ROS_INFO_STREAM("three linear ava");
			// count_loop++;
            // simulate controller
			// if(count_loop>11 || count_loop==0)
			{
				update_main_pos();
				// ROS_INFO_STREAM("count_loop: "<<count_loop);
				controller();//
				//count_loop=1;
			}
            
            plant();
            // ROS_INFO_STREAM("plant...");

        }
        else if (flag_sim_real == 0)
        {
            // real fly
            // ROS_INFO_STREAM("ctrl node main_position: " << main_position); 
            controller();
        }
    }
    else
    {
		// 
        main_position = init_position;
		main_velocity = init_velocity;
		main_eular_angles = init_euler_angles;
		main_body_rates = init_body_rate;
		nominal_position = init_position;      //setpoint position
		nominal_euler_angles = init_euler_angles;
		//debug line
		// ROS_INFO_STREAM("*********************init******************");
		// ROS_INFO_STREAM("ctrl node init main_position: " << main_position);
		// ROS_INFO_STREAM("ctrl node init nominal_position: " << nominal_position);

    }
}


// main function
int main(int argc, char **argv)
{
    ros::init(argc,argv,"linear_ava_controller");
	ros::NodeHandle nh;//create a node handle

    // define a class
    linear_ava_controller linear_ava_controller_node(&nh);
    ros::spin();

    return 0;
}

