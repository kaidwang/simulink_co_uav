#include <controller/five_param.h>


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

    float base_len = 0.452;
float angle = 45*PI/180;
float step  = 72*PI/180;   // 360/5 = 72 deg

float vertical_dis  = 0;
float horizontal_dis = 0;

// fly1_position（
vertical_dis   = base_len*cos(angle);
horizontal_dis = base_len*cos(angle);
fly1_pos.x = vertical_dis;
fly1_pos.y = horizontal_dis;
fly1_pos.z = -0.046;

// fly2_position
angle = angle + step;
vertical_dis   = base_len*cos(angle);
horizontal_dis = base_len*sin(angle);
fly2_pos.x = vertical_dis;
fly2_pos.y = horizontal_dis;
fly2_pos.z = -0.046;

// fly3_position
angle = angle + step;
vertical_dis   = base_len*cos(angle);
horizontal_dis = base_len*sin(angle);
fly3_pos.x = vertical_dis;
fly3_pos.y = horizontal_dis;
fly3_pos.z = -0.046;

// fly4_position
angle = angle + step;
vertical_dis   = base_len*cos(angle);
horizontal_dis = base_len*sin(angle);
fly4_pos.x = vertical_dis;
fly4_pos.y = horizontal_dis;
fly4_pos.z = -0.046;

// fly5_position
angle = angle + step;
vertical_dis   = base_len*cos(angle);
horizontal_dis = base_len*sin(angle);
fly5_pos.x = vertical_dis;
fly5_pos.y = horizontal_dis;
fly5_pos.z = -0.046;

    //tool position
    tool_pos.x = 0;
    tool_pos.y = 0;
    tool_pos.z = 0;

    //mass param
    // fly1_mass = 1.597;//1.597,1.95;//
    // fly2_mass = 1.604;//1.604,1.97;//
    // fly3_mass = 1.583;//1.583,1.94;//
    // center_mass = 1.607;//+1.283;

    fly1_mass = 1.3;//1.597,1.95;//
    fly2_mass = 1.3;//1.604,1.97;//
    fly3_mass = 1.3;//1.583,1.94;//
    fly4_mass = 1.3;
    fly5_mass = 1.3;

    center_mass = 1.2+1.8;//+1.283;
    //I center param
    I_center.x = 0.056804546;
    I_center.y = 0.057436539;
    I_center.z = 0.108610089;

    
    //positon_PID
    kp_x = 14;//6.0; //6.0
    kp_y = 14;//5.0; //5.0
    kp_z = 8.5; //4.5

    ki_x = 0.1;//1.6; //6.6
    ki_y = 0.1;//1.6; //6.6
    ki_z = 0.5;// 1.2; //6.2

    kd_x = 16;//32;  //20;20
    kd_y = 16;//51;  //18;51
    kd_z = 8.5;  //26.0;40
    /***********************autopilot control PID theory************************/
    kpx = 1.5;
    kpy = 1.5;
    kpz = 1.6;
  
    /***************************************************************************/
    kp_phi   = 6.5;// 6.5;//x;
    kp_theta = 6.5;//6.5;//y;
    kp_psi   = 4;//10; //z;

    ki_phi   = 1.6;//x;
    ki_theta = 1.6;//y;
    ki_psi   = 1.4;//1.5;//z;

    kd_phi   =6;// 30;//x;
    kd_theta =6.5;// 30;//y;
    kd_psi   =8.5;// 40;//z;30

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
    S3Q_mass = fly1_mass+fly2_mass+fly3_mass+fly4_mass+fly5_mass+center_mass;
    I_sys.x =  I_center.x + 
               fly1_mass*(fly1_pos.y*fly1_pos.y+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.y*fly2_pos.y+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.y*fly3_pos.y+fly3_pos.z*fly3_pos.z)+
               fly4_mass*(fly4_pos.y*fly4_pos.y+fly4_pos.z*fly4_pos.z)+
               fly5_mass*(fly5_pos.y*fly5_pos.y + fly5_pos.z*fly5_pos.z);

    I_sys.y =  I_center.y + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.z*fly3_pos.z)+
               fly4_mass*(fly4_pos.x*fly4_pos.x+fly4_pos.z*fly4_pos.z)+
               fly5_mass*(fly5_pos.x*fly5_pos.x + fly5_pos.z*fly5_pos.z);

    I_sys.z =  I_center.z + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.y*fly1_pos.y)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.y*fly2_pos.y)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.y*fly3_pos.y)+
               fly4_mass*(fly4_pos.x*fly4_pos.x+fly4_pos.y*fly4_pos.y)+
               fly5_mass*(fly5_pos.x*fly5_pos.x + fly5_pos.y*fly5_pos.y);
}

controller_base::~controller_base()
{
    
}
