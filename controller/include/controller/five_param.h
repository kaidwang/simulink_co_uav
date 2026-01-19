// kaidi wang code 2021.5.31
// define a class of controller basic param
#ifndef FIVE_PARAM_H_
#define FIVE_PARAM_H_

#include <Eigen/Dense>
#define PI 3.14
// #include <controller/paramConfig.h>

class some_xyz
{
private:
    // none private param
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
    // will be used in controller_node.cpp
    /* define data list */
    some_xyz fly1_pos;
    some_xyz fly2_pos;
    some_xyz fly3_pos;
    some_xyz fly4_pos;
    some_xyz fly5_pos;   // <<< 新增：第5架位置

    some_xyz I_center;

    float center_mass;
    float fly1_mass;
    float fly2_mass;
    float fly3_mass;
    float fly4_mass;
    float fly5_mass;     // <<< 新增：第5架质量

    some_xyz I_sys;
    float S3Q_mass;

    some_xyz tool_pos;

    // controller param
    float kp_x;
    float kp_y;
    float kp_z;

    float ki_x;
    float ki_y;
    float ki_z;

    float kd_x;
    float kd_y;
    float kd_z;

    /***********************autopilot control PID theory************************/
    float kpx;
    float kpy;
    float kpz;
    /***************************************************************************/
    float kp_phi;// x;
    float kp_theta;// y;
    float kp_psi;// z;

    float ki_phi;// x;
    float ki_theta;// y;
    float ki_psi;// z;

    float kd_phi;// x;
    float kd_theta;// y;
    float kd_psi;// z;

    // omege and epsilon param
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

