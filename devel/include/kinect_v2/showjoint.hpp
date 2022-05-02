


#include <eigen3/Eigen/Dense>
#include <string>
#include <kinect_v2/BodyJoints.h>
using namespace std;
#pragma once
class Joint 
{

public:

Joint(const kinect_v2::BodyJoints &user_i);

Joint();
~Joint();
void init_BodyVectors(void);
void init_OrthagonalVectors(void);
void convert_Lleg_to_Lnao_leg(void);
void convert_Rleg_to_Rnao_leg(void);
double get_Lelbow_joint(void);
double get_Relbow_joint(void);
double get_Lshoulder_roll(void);
double get_Rshoulder_roll(void);
double get_Lknee_joint(void);
double get_Rknee_joint(void);
double get_Lshoulder_pitch(void);
double get_Rshoulder_pitch(void);
double get_Lhip_roll(void);
double get_Rhip_roll(void);
double get_Lhip_pitch(void);
double get_Rhip_pitch(void);
double get_Lleg_yaw(void);
double get_Rleg_yaw(void);
double length_of_v(const Eigen::Vector3d &v);
double find_angle(const Eigen::Vector3d &v, const Eigen::Vector3d &v1);
Eigen::Vector3d find_Lnao_hip_v(void);
Eigen::Vector3d find_Rnao_hip_v(void);
Eigen::Vector3d find_leg_normal(int tail, int head, int h, int t);
Eigen::Vector3d find_vector(int head, int tail);
Eigen::Vector3d get_2D_orthagonal_v(Eigen::Vector3d v1,Eigen::Vector3d v2);
Eigen::Vector3d find_best_normal(int hip, int knee, int ankle, int foot);

private:

	double Lyaw, Ryaw;
	double temp, key, key2;
	double v1m, v2m;//length of vectors
	int alt_hip, i, j, indexcounter;

	Eigen::Vector3d v,v1,v2,v3;//temp vectors

	Eigen::Vector3d Lhip_v,		Rhip_v,
					LupperLeg_v,RupperLeg_v,
					LlowerLeg_v,RlowerLeg_v,
					LupperArm_v,RupperArm_v,
					LlowerArm_v,RlowerArm_v, 	
					Uback_v,	Dback_v, 		
					Lshoulder_v,Rshoulder_v,		
					Lfoot_v,	Rfoot_v,
					Lnaohip_v,	Rnaohip_v;

    Eigen::Vector3d LorthShoulderBack_v,	RorthShoulderBack_v,
					LhipXNaohip_v,			RhipXNaohip_v,
					LlegNorm_v,				RlegNorm_v,
					LlegNormXNaohip_v,		RlegNormXNaohip_v,
					LNaoLeg_v,				RNaoLeg_v,
					LHipXNaohipXNaoHip_v,	RHipXNaohipXNaoHip_v;

	Eigen::VectorXd test_normal, test_angle, test_length;
	Eigen::VectorXi indextracker;
	Eigen::Vector3d best_norm,reference_normal, temp_v;

	Eigen::Matrix3d WtoLNaohipFrame,WtoRNaohipFrame, x_Lrot, x_Rrot;

	const kinect_v2::BodyJoints user_i;

};