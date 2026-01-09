#include <controller/impedance_control_param.h>

some_xyz::some_xyz(/* args */)
{
    x=0;
    y=0;
    z=0;
}

some_xyz::~some_xyz()
{
}

controller_base::controller_base(/* args */)
{

    //fly1 position

    // zhenzean param
    // 1:(0.58227,0) 2:(0,0.50951) 3:(0,-0.50951)

    // fly1_pos.x = 0.50882;
    // fly1_pos.y = 0;
    // fly1_pos.z = -0.04135;

    // //fly2_positon
    // fly2_pos.x = -0.26956;
    // fly2_pos.y = 0.43641;
    // fly2_pos.z = -0.04135;

    // //fly3_positon
    // fly3_pos.x = -0.26956;
    // fly3_pos.y = -0.43641;
    // fly3_pos.z = -0.04135;

    fly1_pos.x = 0.6195;
    fly1_pos.y = 0;
    fly1_pos.z = -0.04135;

    //fly2_positon
    fly2_pos.x = -0.3097;
    fly2_pos.y = 0.51;
    fly2_pos.z = -0.04135;

    //fly3_positon
    fly3_pos.x = -0.3097;
    fly3_pos.y = -0.51;
    fly3_pos.z = -0.04135;

/*
    fly1_pos.x = 0.45314;
    fly1_pos.y = 0;
    fly1_pos.z = -0.04135;

    //fly2_positon
    fly2_pos.x = -0.22657;
    fly2_pos.y = 0.380;
    fly2_pos.z = -0.04135;

    //fly3_positon
    fly3_pos.x = -0.22657;
    fly3_pos.y = -0.380;
    fly3_pos.z = -0.04135;

    //tool position
    tool_pos.x = 0;
    tool_pos.y = 0;
    tool_pos.z = 0;

    //mass param
    // fly1_mass = 1.597;//1.597,1.95;//
    // fly2_mass = 1.604;//1.604,1.97;//
    // fly3_mass = 1.583;//1.583,1.94;//
    // center_mass = 1.607;//+1.283;

    fly1_mass = 1.7;//1.597,1.95;//
    fly2_mass = 1.7;//1.604,1.97;//
    fly3_mass = 1.7;//1.583,1.94;//
    center_mass = 2.112;//+1.283;

    //I center param
    I_center.x = 0.3786;
    I_center.y = 0.4136;
    I_center.z = 0.7801;
*/
    // fly1_pos.x = -0.05;
    // fly1_pos.y = -0.26;
    // fly1_pos.z = -0.046;

    // //fly2_positon
    // fly2_pos.x = 0.59;
    // fly2_pos.y = 0.85;
    // fly2_pos.z = -0.046;

    // //fly3_positon
    // fly3_pos.x = -0.68;
    // fly3_pos.y = -0.63;
    // fly3_pos.z = -0.046;

    // //tool position
    // tool_pos.x = 0;
    // tool_pos.y = 0;
    // tool_pos.z = 0;

    //mass param
    // fly1_mass = 1.597;//1.597,1.95;//
    // fly2_mass = 1.604;//1.604,1.97;//
    // fly3_mass = 1.583;//1.583,1.94;//
    // center_mass = 1.607;//+1.283;

    fly1_mass = 1.7;//1.597,1.95;//
    fly2_mass = 1.7;//1.604,1.97;//
    fly3_mass = 1.7;//1.583,1.94;//
    center_mass = 1.2;//+1.283;

    //I center param
    I_center.x = 0.056804546;
    I_center.y = 0.057436539;
    I_center.z = 0.108610087;

    
    //positon_PID
    kp_x = 7;//6.0; //6.0
    kp_y = 6;//5.0; //5.0
    kp_z = 8.5; //4.5

    ki_x = 0.02;//1.6; //6.6
    ki_y = 0.02;//1.6; //6.6
    ki_z = 0.8;// 1.2; //6.2

    kd_x = 20;//32;  //20;20
    kd_y = 15;//51;  //18;51
    kd_z = 20;  //26.0;40
    /***********************autopilot control PID theory************************/
    kpx = 1.5;
    kpy = 1.5;
    kpz = 1.6;
  
    /***************************************************************************/
    kp_phi   = 8.5;// 6.5;//x;
    kp_theta = 8.0;//6.5;//y;
    kp_psi   = 7.5;//10; //z;

    ki_phi   = 1.65;//x;
    ki_theta = 1.7;//y;
    ki_psi   = 1.5;//1.5;//z;

    kd_phi   =6.2;// 30;//x;
    kd_theta =6.5;// 30;//y;
    kd_psi   =7.5;// 40;//z;30


    kf_p_x = 1;
    kf_p_y = 1;
    kf_p_z = 1;
    
    kf_i_x = 0.1;
    kf_i_y = 0.1;
    kf_i_z = 0.1;

    kf_d_x = 2;
    kf_d_y = 2;
    kf_d_z = 2;


    kt_p_roll  = 1;
    kt_p_pitch = 1;
    kt_p_yaw   = 1;

    kt_i_roll  = 0.1;
    kt_i_pitch = 0.1;
    kt_i_yaw   = 0.1;

    kt_d_roll  = 2;
    kt_d_pitch = 2;
    kt_d_yaw   = 2;

    
    //omege and epsilon param
    omega_phi   = 2.5;
    epsilon_phi = 4;

    omega_theta   = 2.5;
    epsilon_theta = 4;

    omega_psi   = 1;
    epsilon_psi = 30;

    omega_u   = 1.5;
    epsilon_u = 4;

    omega_v   = 5.5;
    epsilon_v = 4;

    omega_w   = 3.5;
    epsilon_w = 4;

    omega_x   = 2.5;
    epsilon_x = 4;

    omega_y   = 2.5;
    epsilon_y = 4;

    omega_z   = 2.5;
    epsilon_z = 4;

    omega_p   = 8.5;
    epsilon_p = 4;

    omega_q   = 5.5;
    epsilon_q = 4;

    omega_r   = 8.5;
    epsilon_r = 6;

    //calc mass of S3Q platform
    S3Q_mass = fly1_mass+fly2_mass+fly3_mass+center_mass;
    I_sys.x =  I_center.x + 
               fly1_mass*(fly1_pos.y*fly1_pos.y+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.y*fly2_pos.y+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.y*fly3_pos.y+fly3_pos.z*fly3_pos.z);

    I_sys.y =  I_center.y + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.z*fly3_pos.z);

    I_sys.z =  I_center.z + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.y*fly1_pos.y)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.y*fly2_pos.y)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.y*fly3_pos.y);
}

controller_base::~controller_base()
{
    
}
