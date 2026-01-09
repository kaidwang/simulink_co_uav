// kaidi wang, zean zhen
// 2023.3.2
// uav normal function
#include <controller/uav_normal_function.h>

//calc rotation matrix from euler angle, chk
Eigen::Matrix3f euler_to_rotation_mat(geometry_msgs::Point angle)
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

Eigen::Matrix3f define_I_matrix()
{
    Eigen::Matrix3f I;
    I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;

    return I;
}

Eigen::Matrix3f define_zero_matrix()
{
	Eigen::Matrix3f zero;
    zero(0,0)=zero(1,1)=zero(2,2)=0;
	zero(0,1)=zero(0,2)=zero(1,0)=zero(1,2)=zero(2,0)=zero(2,1)=0;

    return zero;
}

Eigen::Matrix3f define_J_matrix(double ixx,double iyy,double izz)
{
	Eigen::Matrix3f J;
	J(0,0)=ixx;
	J(1,1)=iyy;
	J(2,2)=izz;
	J(0,1)=J(0,2)=J(1,0)=J(1,2)=J(2,0)=J(2,1)=0;
	return J;
}

Eigen::Vector3f define_e3_vector()
{
	Eigen::Vector3f ret;
	ret(0) = ret(1) = 0;
	ret(2) = 1;
	return ret;
}

Eigen::Vector3f define_zero_vector()
{
	Eigen::Vector3f ret;
	ret(0) = ret(1) = ret(2) = 0;
	return ret;
}

Eigen::Matrix3f pos_mat(double x,double y,double z)
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

//gain_1 function chk
// double gain_1(double u,double omega)
// {
// 	double y;
// 	y=omega*omega*u; 
// 	return y;
// }

//gain_2 function chk
double gain_2(double u,double omega,double epsilon)
{
	double y;
	y=2*omega*epsilon*u;
	return y;
}

double integrator(double input,double orign,double *ls,double t)
{
	double output;
	*ls = *ls+input*t;
    // ROS_INFO_STREAM("last_sum:"<<*ls);
	//last_sum = input;
	output = *ls+orign;
	return output;
}
//differential equation
void differentiator_lastest(double *last,double *now,double *re,double h)
{
    *re = (*now-*last)/h;
}

// Runge-Kutta method child function
double d2f(double u,double omega,double epsilon,double x,double y,double z)
{
	double f2;
	f2=u-2*omega*epsilon*z-omega*omega*y;
	return f2;
}
double df(double x,double y,double z)
{
	double f1;
	f1=z;
	return f1;
}

// vector 3x1 integral function
Eigen::Vector3f vec_integrator(Eigen::Vector3f input,Eigen::Vector3f orign,Eigen::Vector3f *ls,double t)
{
	Eigen::Vector3f ret;
	*ls = *ls+input*t;
    // ROS_INFO_STREAM("last_sum:"<<*ls);
	//last_sum = input;
	ret = *ls+orign;
	return ret;
}