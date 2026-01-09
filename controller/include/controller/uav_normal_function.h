// kaidi wang, zean zhen 
// 2023.3.2
// uav normal function
#ifndef UAV_NORMAL_FUNCTION_H_
#define UAV_NORMAL_FUNCTION_H_
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#define TIME 0.01
#define GRAVITY 9.8
#define PI 3.1415926

// euler to rotatoin matrix function 
Eigen::Matrix3f euler_to_rotation_mat(geometry_msgs::Point angle);
// define I matrix function 
Eigen::Matrix3f define_I_matrix();
// define zero matrix function 
Eigen::Matrix3f define_zero_matrix();
// define e3 vector 
Eigen::Vector3f define_e3_vector();
// define zero vector 
Eigen::Vector3f define_zero_vector();
// define inteira matrix function
Eigen::Matrix3f define_J_matrix(double ixx,double iyy,double izz);
// solve the antisymmetry matrix
Eigen::Matrix3f pos_mat(double x,double y,double z);

// double gain_1(double u,double omega);
double gain_2(double u,double omega,double epsilon);
double integrator(double input,double orign,double *ls,double t);
void differentiator_lastest(double *last,double *now,double *re,double h);
double d2f(double u,double omega,double epsilon,double x,double y,double z);
double df(double x,double y,double z);

// vector3f integrator function 
Eigen::Vector3f vec_integrator(Eigen::Vector3f input,Eigen::Vector3f orign,Eigen::Vector3f *ls,double t);

#endif

