//kaidi wang code 2021.5.31
//define a class of controller basic param
#ifndef CONTROLLER_STAR_AVA_PARAM_H_
#define CONTROLLER_STAR_AVA_PARAM_H_

#include <Eigen/Dense>

// #include <controller/paramConfig.h>
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

class controller_base
{
private:

public:
    //will be used in controller_node.cpp
    /* define data list */
    some_xyz fly1_pos;
    some_xyz fly2_pos;
    some_xyz fly3_pos;
    
    some_xyz I_center;

    double center_mass;
    double fly1_mass;
    double fly2_mass;
    double fly3_mass;

    some_xyz I_sys;
    double S3Q_mass;

    some_xyz tool_pos;
    //controller param
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

    controller_base(/* args */);
    ~controller_base();
};
#endif
