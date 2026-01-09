// kaidi wang 
// 2023.3.1
// linear ava platform plant cpp file
#include <controller/linear_ava_plant.h>
#include <controller/uav_normal_function.h>

namespace special_mat
{
    // define special matrix 
    Eigen::MatrixXf M(6,6); // mass matrix 
    Eigen::MatrixXf B(6,9); // B matrix related to sub-aricrafts position
    Eigen::MatrixXf C(6,6); // Coriolis matrix 
    Eigen::VectorXf G(6); // gravity effect
    Eigen::MatrixXf omega_a(5,6); // omega_a matrix
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

// define xyz class
xyz::xyz(/* args */)
{
    x = 0;
    y = 0;
    z = 0;
}

xyz::~xyz()
{
}

// plant input 
plant_input::plant_input(/* args */)
{
}

plant_input::~plant_input()
{
}

// plant output
plant_output::plant_output(/* args */)
{
}

plant_output::~plant_output()
{
}

linear_ava_plant::linear_ava_plant()
{
    // init extern force 
    fe = zero_vec;// first of all define extern force is zero 
    te = zero_vec;
/*
    // fly1_position
    fly1_pos.x = 0;
    fly1_pos.y = 0;
    fly1_pos.z = -0.04135;
    //fly2_positon
    fly2_pos.x = 0.94;
    fly2_pos.y = 0;
    fly2_pos.z = -0.04135;
    //fly3_positon
    fly3_pos.x = -0.94;
    fly3_pos.y = 0;
    fly3_pos.z = -0.04135;
*/
//  fly1_position
    fly1_pos.x = 0;
    fly1_pos.y = 0;
    fly1_pos.z = -0.04135;
    //fly2_positon
    fly2_pos.x = 0.94;
    fly2_pos.y = 0;
    fly2_pos.z = -0.04135;
    //fly3_positon
    fly3_pos.x = -0.94;
    fly3_pos.y = 0;
    fly3_pos.z = -0.04135;
    // inertia of linear platform
    I_platform.x = 0.0011; // inertia of linear platform
    I_platform.y = 0.6929;
    I_platform.z = 0.8232;

    tool_pos.x = 0;
    tool_pos.y = 0;
    tool_pos.z = -0.04135;

    // mass param
    platform_mass = 2.3;//1.67; // mass of linear platform
    fly1_mass = 1.7;
    fly2_mass = 1.7;
    fly3_mass = 1.7;

    node1_yaw = -PI/2;
    node2_yaw = PI;
    node3_yaw = 0;

    // k_taux param
    k_taux = 0.1;
    //positon_PID
    kp_x = 18;//5.5;//1;//6.0; //6.0;0.01
    kp_y = 0.2;//5;//1.5;//5.0; //5.0;0.02
    kp_z = 5.5; //4.5

    kd_x = 20;  //20;20//50
    kd_y = 2;  //18;51//50
    kd_z = 3.2;  //26.0;40//40
    kp_phi   = 16.5;//x;
    kp_theta = 0.5;//y;
    kp_psi   = 1.5; //z;

    kd_phi   = 10;//x;
    kd_theta = 5;//y;
    kd_psi   = 1;//z;30


    ki_phi   = 0.05;//x;
    ki_theta = 0.05;//y;
    ki_psi   = 0.05;//z;

    ki_x = 0.05; //6.6
    ki_y = 0.05; //6.6
    ki_z = 0.05; //6.2
    //omege and epsilon param
    omega_phi   = 1.5;
    epsilon_phi = 4;

    omega_theta   = 2.5;    
    epsilon_theta = 4;

    omega_psi   = 1;//1
    epsilon_psi = 3;  //30  

    omega_u   = 1.5;
    epsilon_u = 4;

    omega_v   = 5.5;
    epsilon_v = 4;

    omega_w   = 3.5;
    epsilon_w = 4;

    omega_x   = 2.5;//2.5//4
    epsilon_x = 2;//2

    omega_y   = 2.5;//2.5//4
    epsilon_y = 2;//2

    omega_z   = 2.5;//2.5
    epsilon_z = 4;//4

    omega_p   = 8.5;
    epsilon_p = 4;

    omega_q   = 5.5;
    epsilon_q = 4;

    omega_r   = 8.5;
    epsilon_r = 6;
    g = 9.8;

    //calc mass of S3Q platform
    sys_mass = fly1_mass+fly2_mass+fly3_mass+platform_mass;
    I_sys.x =  I_platform.x + 
               fly1_mass*(fly1_pos.y*fly1_pos.y+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.y*fly2_pos.y+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.y*fly3_pos.y+fly3_pos.z*fly3_pos.z);

    // I_sys.x = 3;

    I_sys.y =  I_platform.y + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.z*fly3_pos.z);

    I_sys.z =  I_platform.z + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.y*fly1_pos.y)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.y*fly2_pos.y)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.y*fly3_pos.y);
    
    I = define_I_matrix();
    zero = define_zero_matrix();
    J = define_J_matrix(I_sys.x ,I_sys.y ,I_sys.z );
    e3 = define_e3_vector();
    zero_vec = define_zero_vector();
    
    // init integrator temp var
    vu_dot_last = 0;
    va0_dot_last = va1_dot_last = va2_dot_last = va3_dot_last = va4_dot_last = 0;
    position_x_last = position_y_last = position_z_last = 0;
    attitude_x_last = attitude_y_last = attitude_z_last = 0;
    init_mat();//init param matrix


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

	// init taux_last value
	taux_last = 0;
    // define init C matrix 
    // special_mat::C.block<3,3>(0,0) = I;
	// special_mat::C.block<3,3>(0,3) = zero;
    // special_mat::C.block<3,3>(3,0) = zero;
	// special_mat::C.block<3,3>(3,3) = I;
    // ROS_INFO_STREAM("M matrix: " <<special_mat::M);
    // ROS_INFO_STREAM("C matrix: " <<special_mat::C);
    // ROS_INFO_STREAM("G matrix: " <<special_mat::G);
    // ROS_INFO_STREAM("B matrix: " <<special_mat::B);
    // ROS_INFO_STREAM("omega_at: " <<special_mat::omega_a.transpose());
    // ROS_INFO_STREAM("delta_a: " <<special_mat::delta_a);
    // ROS_INFO_STREAM("inter1 delta_a: " << special_mat::omega_a*special_mat::M.inverse()*special_mat::omega_a.transpose());
    // ROS_INFO_STREAM("inter2 delta_a: " << (special_mat::omega_a*special_mat::M.inverse()*special_mat::omega_a.transpose()).inverse());

}

linear_ava_plant::~linear_ava_plant()
{
}

//get init pos from controller
void linear_ava_plant::init_state(
        geometry_msgs::Point init_position,
        geometry_msgs::Point init_velocity,
        geometry_msgs::Point init_euler_angles,
        geometry_msgs::Point init_body_rate)
{
    output.main_velocity = init_velocity;
    output.main_position = init_position;
    output.main_body_rate = init_body_rate;
    output.main_attitude = init_euler_angles;
    input.init_pos = init_position;
    input.init_att = init_euler_angles;
    input.init_vel = init_velocity;
    input.init_body_rate = init_body_rate;
    //debug line
    // ROS_INFO_STREAM("plant node init main_velocity: " << input.init_vel);
    // ROS_INFO_STREAM("plant node init main_position: " << input.init_pos);
    // ROS_INFO_STREAM("plant node init main_body_rates: " << input.init_body_rate);
    // ROS_INFO_STREAM("plant node init main_eular_angles: " << input.init_att);

    // ROS_INFO_STREAM("plant node init main_velocity: " << output.main_velocity);
    // ROS_INFO_STREAM("plant node init main_position: " << output.main_position);
    // ROS_INFO_STREAM("plant node init main_body_rates: " << output.main_body_rate);
    // ROS_INFO_STREAM("plant node init main_eular_angles: " << output.main_attitude);
}


// compose angle, thrust to a vector for calc
void linear_ava_plant::composition( 
    geometry_msgs::Point body_euler, 
    geometry_msgs::Point sub1_ang, 
    geometry_msgs::Point sub2_ang, 
    geometry_msgs::Point sub3_ang, 
    std_msgs::Float64 sub1_thu,
    std_msgs::Float64 sub2_thu,
    std_msgs::Float64 sub3_thu,
    Eigen::Vector3f U_front,
    Eigen::Vector3f U_end)
{
    Eigen::Matrix3f r_mat0, rt_mat0, rmat1,rmat2,rmat3; //main_euler_angle rotation mat
    // r_mat0 = euler_to_rotation_mat(body_euler);
    r_mat0 = euler_to_rotation_mat(output.main_attitude);
    rt_mat0 = r_mat0.inverse();//inverse function of rotation matrix
    rmat1 = euler_to_rotation_mat(sub1_ang);
    rmat2 = euler_to_rotation_mat(sub2_ang);
    rmat3 = euler_to_rotation_mat(sub3_ang);
    //debug line
    //ROS_INFO_STREAM("*********************composition******************");
    // // ROS_INFO_STREAM("plant node body_euler: " << body_euler);
    // ROS_INFO_STREAM("plant node sub1_ang: " << sub1_ang);
    // ROS_INFO_STREAM("plant node sub1_thu: " << sub1_thu);
    // ROS_INFO_STREAM("plant node sub2_ang: " << sub2_ang);
    // ROS_INFO_STREAM("plant node sub2_thu: " << sub2_thu);    
    // ROS_INFO_STREAM("plant node sub3_ang: " << sub3_ang);
    // ROS_INFO_STREAM("plant node sub3_thu: " << sub3_thu);

    Eigen::Vector3f vec_thu1,vec_thu2,vec_thu3;
    vec_thu1(0)=vec_thu1(1)=0;
    vec_thu1(2)=-sub1_thu.data;
    vec_thu2(0)=vec_thu2(1)=0;
    vec_thu2(2)=-sub2_thu.data;
    vec_thu3(0)=vec_thu3(1)=0;
    vec_thu3(2)=-sub3_thu.data;

    Eigen::Vector3f fw1,fw2,fw3,f01,f02,f03;
    fw1 = rmat1*vec_thu1;
    f01 = rt_mat0*fw1;
    fw2 = rmat2*vec_thu2;
    f02 = rt_mat0*fw2;
    fw3 = rmat3*vec_thu3;
    f03 = rt_mat0*fw3;

    // repeat code 
    Eigen::Matrix3f pos_s1,pos_s2,pos_s3;
    pos_s1=pos_mat(fly1_pos.x,fly1_pos.y,fly1_pos.z);
	pos_s2=pos_mat(fly2_pos.x,fly2_pos.y,fly2_pos.z);
	pos_s3=pos_mat(fly3_pos.x,fly3_pos.y,fly3_pos.z);

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


    Eigen::MatrixXf f0123(9, 1);
    f0123.block<3,1>(0,0)=f01;
	f0123.block<3,1>(3,0)=f02;
	f0123.block<3,1>(6,0)=f03;
    // ROS_INFO_STREAM("f01: "<< f01);
    // ROS_INFO_STREAM("f02: "<< f02);
    // ROS_INFO_STREAM("f03: "<< f03);
    // ROS_INFO_STREAM("M matrix: " <<special_mat::M);
    // ROS_INFO_STREAM("omega matrix: " <<special_mat::omega);
    // ROS_INFO_STREAM("B: "<< B);
    Eigen::MatrixXf U(6, 1);
    // U = B*f0123;

    // special_mat::u_a =  B_full*f0123;
    // Eigen::VectorXf U_au(6); // ua and uu vector
    // U_au.head(5) = special_mat::u_a;
    // U_au(5) =  special_mat::u_u(0);

    // U = special_mat::omega * U_au;

    U = B*f0123;
    // Damping section
    U(3) = U(3) - k_taux * output.main_body_rate.x;

    // redesign
    // U(0,0) = U_front(0);
    // U(1,0) = U_front(1);
    // U(2,0) = U_front(2);
    // U(3,0) = U_end(0);
    // U(4,0) = U_end(1);
    // U(5,0) = U_end(2);

    // U(3) = taux_last - k_taux * output.main_body_rate.x;
	// taux_last = U(3);

    // redesign tau_x


	// taux_last = U(3);
    // U(3) = 0;

	// ROS_INFO_STREAM("plant ua: "<<special_mat::u_a);
    // ROS_INFO_STREAM("U: "<< U);
    thrust_u(0) = U(0);
    thrust_u(1) = U(1);
    thrust_u(2) = U(2);
    torques_u(0) = U(3);
    torques_u(1) = U(4);
    torques_u(2) = U(5);
    output.thrust.x = thrust_u(0);
    output.thrust.y = thrust_u(1);
    output.thrust.z = thrust_u(2);
    output.torque.x =  torques_u(0);
    output.torque.y =  torques_u(1);
    output.torque.z =  torques_u(2);
}

void linear_ava_plant::init_mat()// calc delta_a matrix
{
    Eigen::Matrix3f pos_s1;      //position matrix
    Eigen::Matrix3f pos_s2;
    Eigen::Matrix3f pos_s3;
    pos_s1=pos_mat(fly1_pos.x,fly1_pos.y,fly1_pos.z);
    pos_s2=pos_mat(fly2_pos.x,fly2_pos.y,fly2_pos.z);
    pos_s3=pos_mat(fly3_pos.x,fly3_pos.y,fly3_pos.z);

    // define M matrix
    special_mat::M.block<3,3>(0,0) = sys_mass*I;
	special_mat::M.block<3,3>(0,3) = zero;
    special_mat::M.block<3,3>(3,0) = zero;
	special_mat::M.block<3,3>(3,3) = J;
    //special_mat::M.block<3,3>(3,3) = J_bar;

    // ROS_INFO_STREAM("M: "<< special_mat::M);
    // define B matrix
    
    special_mat::B.block<3,3>(0,0)=I;
	special_mat::B.block<3,3>(0,3)=I;
	special_mat::B.block<3,3>(0,6)=I;
	special_mat::B.block<3,3>(3,0)=pos_s1;
	special_mat::B.block<3,3>(3,3)=pos_s2;
	special_mat::B.block<3,3>(3,6)=pos_s3;

    // define init G vector s   
    special_mat::G.block<3,1>(0,0)=e3;
	special_mat::G.block<3,1>(3,0)=zero_vec;

    // define omega_a transpose
    // special_mat::omega_a.transpose() = special_mat::B.block(0,0,6,5);
    special_mat::omega_a.transpose().block<6,1>(0,0) = special_mat::B.block(0,0,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,1) = special_mat::B.block(0,1,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,2) = special_mat::B.block(0,2,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,3) = special_mat::B.block(0,4,6,1);
    special_mat::omega_a.transpose().block<6,1>(0,4) = special_mat::B.block(0,5,6,1);
    // define delta_u 
    special_mat::delta_u(0) = 0;
    special_mat::delta_u(1) = -0.0413;
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
    special_mat::omega.block<6,5>(0,0) = special_mat::omega_a.transpose(); // 0,0 start, 6x5
    special_mat::omega.block<6,1>(0,5) = special_mat::omega_u.transpose(); // 0,5 start, 6x1
    
    // get noise vector 
    Eigen::VectorXf noise(6);
    noise(0) = fe(0);
    noise(1) = fe(1);
    noise(2) = fe(2);
    noise(3) = te(0);
    noise(4) = te(1);
    noise(5) = te(2);
    Eigen::VectorXf f_au(6);
    f_au = special_mat::omega.inverse()* noise;
    special_mat::f_a = f_au.head(5); // get f_a value 
    special_mat::f_u = f_au.tail(1);
}

void linear_ava_plant::update_mat(
    geometry_msgs::Point v, 
    geometry_msgs::Point body_rate,
    geometry_msgs::Point thrust,
    geometry_msgs::Point torque,
    geometry_msgs::Point main_euler)
{
    // calc update matrix, first of all put data into state_v vector
    special_mat::state_v(0) = v.x;
    special_mat::state_v(1) = v.y;
    special_mat::state_v(2) = v.z;
    special_mat::state_v(3) = body_rate.x;
    special_mat::state_v(4) = body_rate.y;
    special_mat::state_v(5) = body_rate.z;

    // define a temp vector to store va and vu
    Eigen::VectorXf v_au(6);
    v_au = special_mat::delta.inverse()*special_mat::state_v;
    // get va and vu value and update 
    special_mat::v_a(0) = v_au(0);   // va 
    special_mat::v_a(1) = v_au(1);
    special_mat::v_a(2) = v_au(2);
    special_mat::v_a(3) = v_au(3);
    special_mat::v_a(4) = v_au(4);
    special_mat::v_u(0) = v_au(5);   // vu

    // get U_make vector 
    Eigen::VectorXf U_make(6); // same to U vector
    U_make(0) = thrust.x;
    U_make(1) = thrust.y;
    U_make(2) = thrust.z;
    U_make(3) = torque.x;
    U_make(4) = torque.y;
    U_make(5) = torque.z;
    Eigen::VectorXf U_au(6); // ua and uu vector
    U_au = special_mat::omega.inverse()* U_make;
    // get ua and uu value
    special_mat::u_a = U_au.head(5);
    special_mat::u_u(0) = 0;

    rotation_matrix = euler_to_rotation_mat(main_euler); // update rotation matrix
    special_mat::G.head(3) = -sys_mass*g*rotation_matrix.transpose()*e3;
    special_mat::G.tail(3) = zero_vec;
    Eigen::VectorXf g_au(6);
    g_au = special_mat::delta.inverse()* special_mat::G;
    special_mat::g_a = g_au.head(5); // get g_a value
    special_mat::g_u = g_au.tail(1);

    	// define C matrix
	Eigen::Matrix3f omega0_s;
	omega0_s = pos_mat(body_rate.x, body_rate.y, body_rate.z); 
	special_mat::C.block<3,3>(0,0) = sys_mass*omega0_s;
	special_mat::C.block<3,3>(0,3) = zero;
	special_mat::C.block<3,3>(3,0) = zero;
	special_mat::C.block<3,3>(3,3) = -J*omega0_s;

    special_mat::Ca = special_mat::delta_a.transpose()*special_mat::C*special_mat::delta_a;
    special_mat::Cu = special_mat::delta_u.transpose()*special_mat::C*special_mat::delta_u;
    special_mat::Cau= special_mat::delta_a.transpose()*special_mat::C*special_mat::delta_u; // Cau
    special_mat::Cua= special_mat::delta_u.transpose()*special_mat::C*special_mat::delta_a;
}

// ask noise function 
void linear_ava_plant::ask_noise()
{
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
}

//get rand data
double linear_ava_plant::getRandData(double min,double max)
{
	return min +rand() / double(RAND_MAX/(max -min ));
}


// dynamic model function
void linear_ava_plant::dynamic_model()
{
    // first of all update matrix 
    update_mat(
        output.main_velocity, 
        output.main_body_rate,
        output.thrust,
        output.torque,
        output.main_attitude);
    //debug line
    // ROS_INFO_STREAM("*********************update_mat******************");
    // ROS_INFO_STREAM("plant node output.main_velocity: " << output.main_velocity);
    // ROS_INFO_STREAM("plant node output.main_body_rate: " << output.main_body_rate);
    // ROS_INFO_STREAM("plant node output.thrust: " << output.thrust);
    // ROS_INFO_STREAM("plant node output.torque: " << output.torque);
    // ROS_INFO_STREAM("plant node output.main_attitude: " << output.main_attitude);
    // calc dot_va
    special_mat::dot_va = special_mat::Ma.inverse()*
        (-special_mat::Ca*special_mat::v_a
        -special_mat::Cau*special_mat::v_u
        -special_mat::g_a
        +special_mat::u_a
        +special_mat::f_a);

    special_mat::dot_vu = special_mat::Mu.inverse()*
        (-special_mat::Cu*special_mat::v_u
        -special_mat::Cua*special_mat::v_a
        -special_mat::g_u
        +special_mat::u_u
        +special_mat::f_u);
    //debug line 
    // ROS_INFO_STREAM("*********************dot_va******************");
    // ROS_INFO_STREAM("plant node special_mat::dot_va: " << special_mat::dot_va);
    // ROS_INFO_STREAM("plant node special_mat::v_a: " << special_mat::v_a);
    // ROS_INFO_STREAM("plant node special_mat::v_u: " << special_mat::v_u);
    // ROS_INFO_STREAM("plant node special_mat::g_a: " << special_mat::g_a);
    // ROS_INFO_STREAM("plant node special_mat::u_a: " << special_mat::u_a);
    // ROS_INFO_STREAM("plant node special_mat::f_a: " << special_mat::f_a);

    // update va 
    update_va();
    //debug line
    // ROS_INFO_STREAM("*********************update_va******************");
    // ROS_INFO_STREAM("plant node special_mat::state_v: " << special_mat::state_v);
    // update main_velocity and main position
    update_velocity_position();
    // update main_body_rate and main_attitude
    update_body_rate_attitude();
}

// update va value
void linear_ava_plant::update_va()
{
    special_mat::v_a(0) = integrator(special_mat::dot_va(0),0,&va0_dot_last,TIME);
    special_mat::v_a(1) = integrator(special_mat::dot_va(1),0,&va1_dot_last,TIME);
    special_mat::v_a(2) = integrator(special_mat::dot_va(2),0,&va2_dot_last,TIME);
    special_mat::v_a(3) = integrator(special_mat::dot_va(3),0,&va3_dot_last,TIME);
    special_mat::v_a(4) = integrator(special_mat::dot_va(4),0,&va4_dot_last,TIME);


    special_mat::v_u(0) = integrator(special_mat::dot_vu(0),0,&vu_dot_last,TIME);

    // update state_v
    Eigen::VectorXf v_au(6);
    v_au.head(5) = special_mat::v_a;
    v_au(5) = special_mat::v_u(0); // 
    special_mat::state_v = special_mat::delta * v_au;
}

// update velocity and position 
void linear_ava_plant::update_velocity_position()
{
    // get velocity 
    output.main_velocity.x = special_mat::state_v(0);
    output.main_velocity.y = special_mat::state_v(1);
    output.main_velocity.z = special_mat::state_v(2);
    // get position 
    output.main_position.x = integrator(output.main_velocity.x,input.init_pos.x,&position_x_last,TIME);
    output.main_position.y = integrator(output.main_velocity.y,input.init_pos.y,&position_y_last,TIME);
    output.main_position.z = integrator(output.main_velocity.z,input.init_pos.z,&position_z_last,TIME);
    //debug line
    // ROS_INFO_STREAM("*********************update_velocity_position******************");
    // ROS_INFO_STREAM("plant node main_velocity: " << output.main_velocity);
    // ROS_INFO_STREAM("plant node main_position: " << output.main_position);
}

// update body_rate and attitude 
void linear_ava_plant::update_body_rate_attitude()
{
    // get velocity 
    output.main_body_rate.x = special_mat::state_v(3);
    output.main_body_rate.y = special_mat::state_v(4);
    output.main_body_rate.z = special_mat::state_v(5);
    // get position 
    output.main_attitude.x = integrator(output.main_body_rate.x,input.init_att.x,&attitude_x_last,TIME);
    output.main_attitude.y = integrator(output.main_body_rate.y,input.init_att.y,&attitude_y_last,TIME);
    output.main_attitude.z = integrator(output.main_body_rate.z,input.init_att.z,&attitude_z_last,TIME);
    //debug line
    // ROS_INFO_STREAM("*********************update_body_rate_attitude******************");
    // ROS_INFO_STREAM("plant node main_body_rates: " << output.main_body_rate);
    // ROS_INFO_STREAM("plant node main_eular_angles: " << output.main_attitude);
}

// system plant function
void linear_ava_plant::system_plant()
{
    update_mat(
        output.main_velocity, 
        output.main_body_rate,
        output.thrust,
        output.torque,
        output.main_attitude);
    rotation_kinematics();
    rotation_dynamics();
    translational_kinematics();
    translational_dynamics();
    output_publish();
}

//second level function list
void linear_ava_plant::rotation_kinematics()
{
        //calc p q r
    p = integrator(p_dot,0,&p_dot_last,TIME);
    q = integrator(q_dot,0,&q_dot_last,TIME);
    r = integrator(r_dot,0,&r_dot_last,TIME);
    //output body ratesvoid linear_ava_plant::rotation_kinematics()

    body_rates(0) = p;
    body_rates(1) = q;
    body_rates(2) = r;

    //kinematics sections
    phi_dot   = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot   = q * sin(phi) * (1/cos(theta)) + r * cos(phi) * (1/cos(theta));

    //integrator attitude angle dot to get euler angles
    phi = integrator(phi_dot, input.init_att.x, &phi_dot_last, TIME);
    theta = integrator(theta_dot, input.init_att.y, &theta_dot_last, TIME);
    psi = integrator(psi_dot, input.init_att.z, &psi_dot_last, TIME);

    //make vectorx3f form
    euler_angles(0) = phi;
    euler_angles(1) = theta;
    euler_angles(2) = psi;
}

void linear_ava_plant::translational_kinematics()
{
    //define rotation matrix and inverse of rotation matrix
    Eigen::Matrix3f r_mat0, rt_mat0;//main_euler_angle rotation mat
    r_mat0  = euler_to_rotation_mat(euler_angle);

    //get v1~3_dot value
    v1_dot = v0_dot(0);
    v2_dot = v0_dot(1);
    v3_dot = v0_dot(2);

    //integrator v1~3_dot value to v1~3 value
    v1 = integrator(v1_dot, 0, &v1_dot_last, TIME);
    v2 = integrator(v2_dot, 0, &v2_dot_last, TIME);
    v3 = integrator(v3_dot, 0, &v3_dot_last, TIME);

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

    w_x = integrator(w_x_dot, input.init_pos.x, &w_x_dot_last, TIME);
    w_y = integrator(w_y_dot, input.init_pos.y, &w_y_dot_last, TIME);
    w_z = integrator(w_z_dot, input.init_pos.z, &w_z_dot_last, TIME);

    x0_w(0) = w_x;
    x0_w(1) = w_y;
    x0_w(2) = w_z;
}

void linear_ava_plant::rotation_dynamics()
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
    p_dot = N(0)/I_sys.x;
    q_dot = N(1)/I_sys.y;
    r_dot = N(2)/I_sys.z;
}


void linear_ava_plant::translational_dynamics()
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
        v0_dot = (sys_mass*g*rt_mat0*e3- sys_mass*(omega.cross(v))+thrust_u)/sys_mass;
    }
    else
    {
        v0_dot = (sys_mass*g*rt_mat0*e3+fe- sys_mass*(omega.cross(v))+thrust_u)/sys_mass;
    }
}


void linear_ava_plant::output_publish()
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

    output.main_velocity =velocity ; 
    // get position 
    output.main_position = position;
     //debug line
    // ROS_INFO_STREAM("*********************update_velocity_position******************");
    // ROS_INFO_STREAM("plant node main_velocity: " << output.main_velocity);
    // ROS_INFO_STREAM("plant node main_position: " << output.main_position);

    // get velocity 
    output.main_body_rate = body_rate;
    
    // get position 
    output.main_attitude = euler_angle;

    // ROS_INFO_STREAM("plant node main_body_rates: " << output.main_body_rate);
    // ROS_INFO_STREAM("plant node main_eular_angles: " << output.main_attitude);
}
