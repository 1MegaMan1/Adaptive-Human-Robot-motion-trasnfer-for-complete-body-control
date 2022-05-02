#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinect_v2/BodyJoints.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <kinect_v2/showjoint.hpp>

#define _USE_MATH_DEFINES
using namespace std;
#define  base_link 0
#define  Lshoulder 1
#define  Lelbow 2
#define  Lhand 3
#define  Rshoulder 4
#define  Relbow 5
#define  Rhand 6
#define  Lhip 7
#define  Lknee 8
#define  Lankle 9
#define  Lfoot 10
#define  Rhip 11
#define  Rknee 12
#define  Rankle 13
#define  Rfoot 14
#define  shoulder_spine 15
#define  LEFT -1.0
#define  RIGHT 1.0
#define  NEGATIVE -1.0



Joint::Joint(const kinect_v2::BodyJoints &user_i)
: 	user_i(user_i),	
	Lhip_v(),				Rhip_v(),
	LupperLeg_v(),			RupperLeg_v(),
	LlowerLeg_v(),			RlowerLeg_v(),
	LupperArm_v(),			RupperArm_v(),
	LlowerArm_v(),			RlowerArm_v(), 	
	Uback_v(),				Dback_v(), 		
	Lshoulder_v(),			Rshoulder_v(),		
	Lfoot_v(),				Rfoot_v(),
	Lnaohip_v(),			Rnaohip_v(),
	LorthShoulderBack_v(),	RorthShoulderBack_v(),
	LHipXNaohipXNaoHip_v(), RHipXNaohipXNaoHip_v(),
	LlegNorm_v(),			RlegNorm_v(),
	LlegNormXNaohip_v(),	RlegNormXNaohip_v(),
	LhipXNaohip_v(),		RhipXNaohip_v(),
	LNaoLeg_v(),			RNaoLeg_v()
	{}

Joint::Joint() {}
Joint::~Joint() {}
void Joint::init_BodyVectors()
{
	Lhip_v 			= find_vector(Lhip,Rhip);					Rhip_v 			= find_vector(Rhip,Lhip); 
	LupperLeg_v		= find_vector(Lknee,Lhip); 					RupperLeg_v 	= find_vector(Rknee,Rhip);
	LlowerLeg_v		= find_vector(Lankle,Lknee); 				RlowerLeg_v 	= find_vector(Rankle,Rknee);
	LupperArm_v		= find_vector(Lelbow,Lshoulder); 			RupperArm_v 	= find_vector(Relbow,Rshoulder);
	LlowerArm_v 	= find_vector(Lhand,Lelbow); 				RlowerArm_v 	= find_vector(Rhand,Relbow); 
	Uback_v			= find_vector(shoulder_spine,base_link);	Dback_v 		= find_vector(base_link,shoulder_spine); 
	Lshoulder_v		= find_vector(Lshoulder,shoulder_spine);	Rshoulder_v		= find_vector(Rshoulder,shoulder_spine);
	Lfoot_v			= find_vector(Lfoot,Lankle);				Rfoot_v			= find_vector(Rfoot,Rankle);
	Lnaohip_v 		= find_Lnao_hip_v(); 						Rnaohip_v 		= find_Rnao_hip_v(); 
}
void Joint::init_OrthagonalVectors()
{
	LorthShoulderBack_v = get_2D_orthagonal_v(Lshoulder_v, LupperArm_v);RorthShoulderBack_v = get_2D_orthagonal_v(Rshoulder_v, RupperArm_v);
	LhipXNaohip_v 		= Lnaohip_v.cross(Rhip_v.normalized()); 		RhipXNaohip_v 	 	= Rnaohip_v.cross(Lhip_v.normalized());
	LlegNorm_v 			= find_best_normal(Lhip, Lknee, Lankle, Lfoot);	RlegNorm_v 			= find_best_normal(Rhip, Rknee, Rankle, Rfoot);
	LlegNormXNaohip_v 	= Rnaohip_v.cross(LlegNorm_v);					RlegNormXNaohip_v 	= Lnaohip_v.cross(RlegNorm_v);//rototation 
	LHipXNaohipXNaoHip_v = Lnaohip_v.cross(LhipXNaohip_v);				RHipXNaohipXNaoHip_v = Rnaohip_v.cross(RhipXNaohip_v);//reference
}
double Joint::get_Lelbow_joint()
{
	return LEFT*find_angle(LupperArm_v,LlowerArm_v); 
}
double Joint::get_Relbow_joint()
{
	return RIGHT*find_angle(RupperArm_v,RlowerArm_v);
}
double Joint::get_Lshoulder_roll()
{
    return LEFT*(find_angle(LupperArm_v,Lshoulder_v)-M_PI_2); 
}
double Joint::get_Rshoulder_roll()
{
    return RIGHT*(find_angle(RupperArm_v,Rshoulder_v)-M_PI_2);
}
double Joint::get_Lknee_joint()
{
	return find_angle(LupperLeg_v,LlowerLeg_v);
}
double Joint::get_Rknee_joint()
{
	return find_angle(RupperLeg_v,RlowerLeg_v);
}
double Joint::get_Lshoulder_pitch()
{ 
     return NEGATIVE*(find_angle(Dback_v,LorthShoulderBack_v)-M_PI_2);
}
double Joint::get_Rshoulder_pitch()
{ 
     return NEGATIVE*(find_angle(Dback_v,RorthShoulderBack_v)-M_PI_2);
}
double Joint::get_Lhip_roll()
{    	
     return LEFT*find_angle(LNaoLeg_v, Lhip_v)+M_PI_2;
}
double Joint::get_Rhip_roll()
{    
     return RIGHT*find_angle(RNaoLeg_v, Rhip_v)-M_PI_2;
}
double Joint::get_Lhip_pitch()
{
  	 return find_angle(LNaoLeg_v, LhipXNaohip_v)-M_PI_2;
}
double Joint::get_Rhip_pitch()
{
  	 return NEGATIVE*find_angle(RNaoLeg_v, RhipXNaohip_v)+M_PI_2;
}
double Joint::get_Lleg_yaw()
{
	return NEGATIVE*find_angle(get_2D_orthagonal_v(Lnaohip_v,Lhip_v), get_2D_orthagonal_v(Lnaohip_v,LlegNorm_v));
}
double Joint::get_Rleg_yaw()
{
	return NEGATIVE*find_angle(get_2D_orthagonal_v(Rnaohip_v,Lhip_v), get_2D_orthagonal_v(Rnaohip_v,RlegNorm_v));
}
Eigen::Vector3d Joint::find_Lnao_hip_v()
{
	return (get_2D_orthagonal_v(Lhip_v,Uback_v).normalized() - Lhip_v.normalized());
}
Eigen::Vector3d Joint::find_Rnao_hip_v()
{
	return (get_2D_orthagonal_v(Rhip_v,Uback_v).normalized() - Rhip_v.normalized());
}
Eigen::Vector3d Joint::get_2D_orthagonal_v(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
	return (v2 - ( (v1.normalized().dot(v2)) * v1.normalized() ) );
}
void Joint::convert_Lleg_to_Lnao_leg()
{
	Lyaw = get_Lleg_yaw()*NEGATIVE;

 	WtoLNaohipFrame	<< 	Lnaohip_v(0), LHipXNaohipXNaoHip_v(0), LhipXNaohip_v(0),
			   	   		Lnaohip_v(1), LHipXNaohipXNaoHip_v(1), LhipXNaohip_v(1),
			       		Lnaohip_v(2), LHipXNaohipXNaoHip_v(2), LhipXNaohip_v(2);


    x_Lrot  <<  1 ,       0 ,        0 ,
			    0 ,cos(Lyaw),-sin(Lyaw), 
			    0 ,sin(Lyaw), cos(Lyaw); 
		 		    
	LNaoLeg_v = WtoLNaohipFrame * x_Lrot * WtoLNaohipFrame.transpose() * LupperLeg_v;// H*R*H^T*V;
}
void Joint::convert_Rleg_to_Rnao_leg()
{
	Ryaw = get_Rleg_yaw();

 	WtoRNaohipFrame	<< 	Rnaohip_v(0), RHipXNaohipXNaoHip_v(0), RhipXNaohip_v(0),
			   	   		Rnaohip_v(1), RHipXNaohipXNaoHip_v(1), RhipXNaohip_v(1),
			       		Rnaohip_v(2), RHipXNaohipXNaoHip_v(2), RhipXNaohip_v(2);
    x_Rrot  <<  1 ,       0 ,        0 ,
			    0 ,cos(Ryaw),-sin(Ryaw), 
				0 ,sin(Ryaw), cos(Ryaw); 
		 		    
	RNaoLeg_v = WtoRNaohipFrame * x_Rrot * WtoRNaohipFrame.transpose() * RupperLeg_v;// H*R*H^T*V;

}
double Joint::find_angle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
 	v1m 	= length_of_v(v1); 
 	v2m 	= length_of_v(v2);
     
     if((v1m!=0.0) && (v2m!=0.0))
     {
		return ( acos( (v1.dot(v2)) / (v1m * v2m) ) );
     }
     else
     {
	//	ROS_ERROR("Illegal frames");
		return 0.0;
     }
}

Eigen::Vector3d Joint::find_leg_normal(int tail, int head, int h, int t)
{
	  v1 = find_vector(head, tail);
	  v2 = find_vector(h, t);//t,h
	  v3 = v1.cross(v2); 
	return v3;
}
Eigen::Vector3d Joint::find_vector(int head, int tail)
{
     v(0) = user_i.joints[head].position.x - user_i.joints[tail].position.x;
     v(1) = user_i.joints[head].position.y - user_i.joints[tail].position.y;
     v(2) = user_i.joints[head].position.z - user_i.joints[tail].position.z;

  return v;
}
double Joint::length_of_v(const Eigen::Vector3d &v1)
{
  return( sqrt(v1(0)*v1(0)+v1(1)*v1(1)+v1(2)*v1(2)) );
}

Eigen::Vector3d Joint::find_best_normal(int hip, int knee, int ankle, int foot)
{
	Eigen::VectorXd test_normal(15), test_angle(5), test_length(5);
	Eigen::VectorXi indextracker(5);
	indexcounter = 0;

    	test_normal << find_leg_normal(ankle, knee, foot,  ankle),find_leg_normal(ankle, hip,  foot,  ankle),
    				   find_leg_normal(hip,  knee,  ankle, knee), find_leg_normal(foot,  hip,  foot, ankle),
    				   find_leg_normal(foot,  knee, foot, ankle);

		reference_normal = Lhip_v;

		for( i = 0; i < 5; i++)//if test_average is not pointing the appromiate dir as reference then disregard it
		{
			temp_v = test_normal.segment<3>(i*3);
			test_angle(i)   = find_angle(temp_v, reference_normal);
			test_length(i)  = length_of_v(temp_v);
			indextracker(i) = indexcounter;
			if ( test_angle(i) < M_PI_2  || test_length(i) < .00001)
			{
				test_angle(i) = 0.0;
			}
			indexcounter+=3;
		}
		for( i = 1; i < 5; i++) //sort based on the test angles and keep track of the index so we use test average
		{ 
			key =  test_length (i); 
        	key2 = indextracker(i);
			j = i - 1; 
			while (j >= 0 && test_length(j) > key) 
			{ 
				test_length  (j + 1) = test_length (j); 
	          	indextracker(j + 1) = indextracker(j);
				j = j - 1; 
			} 
			test_length (j + 1) = key; 
     	 	indextracker(j + 1) = key2;
		} 

		if( test_length(4) > 0.00001 )//incase  all vectors are pointing the wrong way then set to ref
		{
			best_norm(0) = test_normal(indextracker(4));
			best_norm(1) = test_normal(indextracker(4)+1);
			best_norm(2) = test_normal(indextracker(4)+2);
			return best_norm.normalized(); 
		}
		else
		{
			return reference_normal; 
		}
}


