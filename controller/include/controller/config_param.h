// kaidi wang 2023.1.28
// global variables

class config_param
{
private:
    /* data */
public:
    double kp_x;
    double kp_y;
    double kp_z;

    double ki_x;
    double ki_y;
    double ki_z;

    double kd_x;
    double kd_y;
    double kd_z;

    double kp_phi;//x;
    double kp_theta;//y;
    double kp_psi;//z;

    double ki_phi;//x;
    double ki_theta;//y;
    double ki_psi;//z;

    double kd_phi;//x;
    double kd_theta;//y;
    double kd_psi;//z;

    config_param(/* args */);
    ~config_param();
};

config_param::config_param(/* args */)
{
    // position controller parameter
    kp_x = 6.0; //6.0
    kp_y = 5.0; //5.0
    kp_z = 7.5; //4.5

    ki_x = 6.6; //6.6
    ki_y = 6.6; //6.6
    ki_z = 4.2; //6.2

    kd_x = 32;  //20;20
    kd_y = 51;  //18;51
    kd_z = 40;  //26.0;40

    // attitude controller parameter
    kp_phi   = 10;//x;
    kp_theta = 10;//y;
    kp_psi   = 10;//z;

    ki_phi   = 1.6;//x;
    ki_theta = 1.6;//y;
    ki_psi   = 0.6;//z;

    kd_phi   = 10;//x;
    kd_theta = 15;//y;
    kd_psi   = 40;//z;30
}

config_param::~config_param()
{
}

config_param var;