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
#include <kinect_v2/kalmanfilter.hpp>
#include <kinect_v2/showjoint.hpp>
//#include <kinect_v2/NaoInterface.hpp>
//#include <controller_manager/controller_manager.h>

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
#define  NUM_OF_JOINTS 14
ros::Publisher joint_pub;
static sensor_msgs::JointState cmd_joint_states;
static sensor_msgs::JointState read_joint_state;

void publishTransform(const kinect_v2::BodyJoints &user_i, int user_id, int j);
//void readCurrentJointState(const sensor_msgs::JointState::ConstPtr& joint_state);
const string get_key_name_tf(int n);
const string get_key_name_model(int n);
//ros::Subscriber nodeTracked(const kinect_v2::BodyJoints user_i);

void callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
//  readCurrentJointState(const sensor_msgs::JointState::ConstPtr& joint_state);
}
void callback_0(const kinect_v2::BodyJoints &user_0)
{
  if (user_0.tracked == "YES")
    {
      publishTransform(user_0, 0, 0);
    }
}

void callback_1(const kinect_v2::BodyJoints &user_1)
{
  if (user_1.tracked == "YES")
    {
      publishTransform(user_1, 1, 0);
    }
}

void callback_2(const kinect_v2::BodyJoints &user_2)
{
  if (user_2.tracked == "YES")
    {
      publishTransform(user_2, 2, 0);
    }
}

void callback_3(const kinect_v2::BodyJoints &user_3)
{
  if (user_3.tracked == "YES")
    {
        publishTransform(user_3, 3, 0);
    }
}

void callback_4(const kinect_v2::BodyJoints &user_4)
{
  if (user_4.tracked == "YES")
  {
      publishTransform(user_4, 4, 0);
  }
}

void callback_5(const kinect_v2::BodyJoints &user_5)
{
  if (user_5.tracked == "YES")
  {
      publishTransform(user_5, 5, 0);
  }
}


/*  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
    { // Change the level to fit your needs
      ros::console::notifyLoggerLevelsChanged();
    }
*/

    int states = 3; // Number of states - angle position, velocity, acceleration
    int measurements = 1; // Number of measurements

    double dt;//Time Step

    Eigen::MatrixXd A(states, states); // System dynamics matrix(intergrates time step with state)
    Eigen::MatrixXd C(measurements, states); // Output matrix
    Eigen::MatrixXd Q(states, states); // Process uncertainty covariance
    Eigen::MatrixXd R(measurements, measurements); // Measurement noise covariance
    Eigen::MatrixXd P(states, states); // Estimate uncertainty covariance

    KalmanFilter RightelbowKF(dt,A, C, Q, R, P);
    KalmanFilter RightShoulderPitchKF(dt,A, C, Q, R, P);
    KalmanFilter RightShoulderRollKF(dt,A, C, Q, R, P);
    KalmanFilter RightKneeKF(dt,A, C, Q, R, P);
    KalmanFilter RightHipPitchKF(dt,A, C, Q, R, P);
    KalmanFilter RightHipRollKF(dt,A, C, Q, R, P);
    KalmanFilter RightNaoHipKF(dt,A, C, Q, R, P);
    KalmanFilter LeftelbowKF(dt,A, C, Q, R, P);
    KalmanFilter LeftShoulderPitchKF(dt,A, C, Q, R, P);
    KalmanFilter LeftShoulderRollKF(dt,A, C, Q, R, P);
    KalmanFilter LeftKneeKF(dt,A, C, Q, R, P);
    KalmanFilter LeftHipPitchKF(dt,A, C, Q, R, P);
    KalmanFilter LeftHipRollKF(dt,A, C, Q, R, P);
    KalmanFilter LeftNaoHipKF(dt,A, C, Q, R, P);



int main(int argc, char **argv)
{

	  RightelbowKF.init();
    RightShoulderPitchKF.init();
    RightShoulderRollKF.init();
    RightKneeKF.init();
    RightHipPitchKF.init();
    RightHipRollKF.init();
    RightNaoHipKF.init();
    LeftelbowKF.init();
    LeftShoulderPitchKF.init();
    LeftShoulderRollKF.init();
    LeftKneeKF.init();
    LeftHipPitchKF.init();
    LeftHipRollKF.init();
    LeftNaoHipKF.init();



	ros::init(argc, argv, "skeleton_tracking");
	ros::NodeHandle n;
  
  //NaoInterface NaoRobot;
  //controller_manager::ControllerManager cm(&NaoRobot);
 // ros::Time prev_time = ros::Time::now();
  ros::Subscriber sub = n.subscribe("joint_states", 10, &callback);
	ros::Subscriber sub_0 = n.subscribe("user_0", 1, &callback_0);
	ros::Subscriber sub_1 = n.subscribe("user_1", 1, &callback_1);
	ros::Subscriber sub_2 = n.subscribe("user_2", 1, &callback_2);
	ros::Subscriber sub_3 = n.subscribe("user_3", 1, &callback_3);
	ros::Subscriber sub_4 = n.subscribe("user_4", 1, &callback_4);
  ros::Subscriber sub_5 = n.subscribe("user_5", 1, &callback_5);

//  const ros::Time time = ros::Time::now();
 // const ros::Duration period = time - prev_time;

 // NaoRobot.init(n, nh);

//  NaoRobot.read(time, period);
 // cm.update(time, period);
 // NaoRobot.write(time, period);

  joint_pub = n.advertise<sensor_msgs::JointState>("kin_joint_states", 10);

	ros::spin();
	return 0;
}


void publishTransform(const kinect_v2::BodyJoints &user_i,int user_id, int joint_id)
{
    geometry_msgs::TransformStamped transformStamped;
    static tf2_ros::TransformBroadcaster br;

		for (int joint_id = 0; joint_id < 16; ++joint_id)
		   {
		      transformStamped.header.stamp = ros::Time::now();
			    transformStamped.header.frame_id = "torso";
			    transformStamped.child_frame_id = get_key_name_tf(joint_id); //	  + "_" + to_string(user_id);

			    transformStamped.transform.translation.x = user_i.joints[joint_id].position.x;
			    transformStamped.transform.translation.y = user_i.joints[joint_id].position.y;
			    transformStamped.transform.translation.z = user_i.joints[joint_id].position.z;
			
			    transformStamped.transform.rotation.w = user_i.joints[joint_id].orientation.w;
			    transformStamped.transform.rotation.x = user_i.joints[joint_id].orientation.x;
			    transformStamped.transform.rotation.y = user_i.joints[joint_id].orientation.y;
			    transformStamped.transform.rotation.z = user_i.joints[joint_id].orientation.z;

		       	br.sendTransform(transformStamped);
		   }

   Joint joint(user_i);
   joint.init_BodyVectors();
   joint.init_OrthagonalVectors();
   joint.convert_Lleg_to_Lnao_leg();
   joint.convert_Rleg_to_Rnao_leg();

   cmd_joint_states.name.resize(NUM_OF_JOINTS);
   cmd_joint_states.position.resize(NUM_OF_JOINTS);
   cmd_joint_states.header.stamp = ros::Time::now();
   cmd_joint_states.header.frame_id = "torso";
     dt = ros::Time::now().toSec(); // Time step
     A << 1, dt, 0, 
           0, 1, dt, 
           0, 0, 1;

   RightelbowKF.update(joint.get_Relbow_joint(),dt,A);
   LeftelbowKF.update(joint.get_Lelbow_joint(),dt,A);
   RightShoulderRollKF.update(joint.get_Rshoulder_roll(),dt,A); 
   LeftShoulderRollKF.update(joint.get_Lshoulder_roll(),dt,A);
   LeftShoulderPitchKF.update(joint.get_Lshoulder_pitch(),dt,A);
   RightShoulderPitchKF.update(joint.get_Rshoulder_pitch(),dt,A);  
   LeftKneeKF.update(joint.get_Lknee_joint(),dt,A);
   RightKneeKF.update(joint.get_Rknee_joint(),dt,A);
   LeftHipRollKF.update(joint.get_Lhip_roll(),dt,A);
   RightHipRollKF.update(joint.get_Rhip_roll(),dt,A);
   LeftHipPitchKF.update(joint.get_Lhip_pitch(),dt,A);
   RightHipPitchKF.update(joint.get_Rhip_pitch(),dt,A);
   LeftNaoHipKF.update(joint.get_Lleg_yaw(),dt,A);
   RightNaoHipKF.update(joint.get_Rleg_yaw(),dt,A);

   cmd_joint_states.name[0] = get_key_name_model(23);//RElbowRoll
   cmd_joint_states.position[0] = RightelbowKF.position();

   cmd_joint_states.name[1] = get_key_name_model(17);//LElbowRoll
   cmd_joint_states.position[1] = LeftelbowKF.position();

   cmd_joint_states.name[2] = get_key_name_model(21);//RShoulderRoll
   cmd_joint_states.position[2] = RightShoulderRollKF.position();;

   cmd_joint_states.name[3] = get_key_name_model(15);//LShoulderRoll
   cmd_joint_states.position[3] = LeftShoulderRollKF.position();

   cmd_joint_states.name[4] = get_key_name_model(14);//LshoulderPitch
   cmd_joint_states.position[4] = LeftShoulderPitchKF.position();

   cmd_joint_states.name[5] = get_key_name_model(20);//RshoulderPitch
   cmd_joint_states.position[5] = RightShoulderPitchKF.position();   

   cmd_joint_states.name[6] = get_key_name_model(5);//Lknee
   cmd_joint_states.position[6] = LeftKneeKF.position();

   cmd_joint_states.name[7] = get_key_name_model(11);//RhipPitch
   cmd_joint_states.position[7] = RightKneeKF.position();

   cmd_joint_states.name[8] = get_key_name_model(3);//LhipRoll
   cmd_joint_states.position[8] = LeftHipRollKF.position();

   cmd_joint_states.name[9] = get_key_name_model(9);//RhipRoll
   cmd_joint_states.position[9] = RightHipRollKF.position();

   cmd_joint_states.name[10] = get_key_name_model(4);//LhipPitch
   cmd_joint_states.position[10] = LeftHipPitchKF.position();;

   cmd_joint_states.name[11] = get_key_name_model(10);//RhipPitch
   cmd_joint_states.position[11] = RightHipPitchKF.position();

   cmd_joint_states.name[12] = get_key_name_model(2);//LYawPitchhip
   cmd_joint_states.position[12] = LeftNaoHipKF.position();

   cmd_joint_states.name[13] = get_key_name_model(8);//RYawPitchhip
   cmd_joint_states.position[13] = RightNaoHipKF.position();
/*
   cmd_joint_states.name[14] = get_key_name_model(7);//LAnklePitch
   cmd_joint_states.position[14] = get_joint_roll(user_i, Lknee, Lankle, Lfoot);

   cmd_joint_states.name[15] = get_key_name_model(13);//RAnklePitch
   cmd_joint_states.position[15] = get_joint_roll(user_i, Rknee, Rankle, Rfoot);

   cmd_joint_states.name[16] = get_key_name_model(6);//RAnklePitch
   cmd_joint_states.position[16] = get_joint_roll(user_i, Lknee, Lankle, Lfoot);

   cmd_joint_states.name[17] = get_key_name_model(12);//RAnklePitch
   cmd_joint_states.position[17] = get_joint_roll(user_i, Rknee, Rankle, Rfoot);
   */ 

  

   joint_pub.publish(cmd_joint_states);
}

void readCurrentJointState(const sensor_msgs::JointState &read_joint_state)
{
  /*
   read_joint_state.name.resize(NUM_OF_JOINTS);
   read_joint_state.position.resize(NUM_OF_JOINTS);
   read_joint_state.header.stamp = ros::Time::now();
   read_joint_state.header.frame_id = "torso";

   read_joint_state.name[0] = get_key_name_model(23);//RElbowRoll
   read_joint_state.position[0] = joint_state[];

   read_joint_state.name[1] = get_key_name_model(17);//LElbowRoll
   read_joint_state.position[1] = LeftelbowKF.position();

   read_joint_state.name[2] = get_key_name_model(21);//RShoulderRoll
   read_joint_state.position[2] = RightShoulderRollKF.position();;

   read_joint_state.name[3] = get_key_name_model(15);//LShoulderRoll
   read_joint_state.position[3] = LeftShoulderRollKF.position();

   read_joint_state.name[4] = get_key_name_model(14);//LshoulderPitch
   read_joint_state.position[4] = LeftShoulderPitchKF.position();

   read_joint_state.name[5] = get_key_name_model(20);//RshoulderPitch
   read_joint_state.position[5] = RightShoulderPitchKF.position();   

   read_joint_state.name[6] = get_key_name_model(5);//Lknee
   read_joint_state.position[6] = LeftKneeKF.position();

   read_joint_state.name[7] = get_key_name_model(11);//RhipPitch
   read_joint_state.position[7] = RightKneeKF.position();

   read_joint_state.name[8] = get_key_name_model(3);//LhipRoll
   read_joint_state.position[8] = LeftHipRollKF.position();

   read_joint_state.name[9] = get_key_name_model(9);//RhipRoll
   read_joint_state.position[9] = RightHipRollKF.position();

   read_joint_state.name[10] = get_key_name_model(4);//LhipPitch
   read_joint_state.position[10] = LeftHipPitchKF.position();;

   read_joint_state.name[11] = get_key_name_model(10);//RhipPitch
   read_joint_state.position[11] = RightHipPitchKF.position();

   read_joint_state.name[12] = get_key_name_model(2);//LYawPitchhip
   read_joint_state.position[12] = LeftNaoHipKF.position();

   read_joint_state.name[13] = get_key_name_model(8);//RYawPitchhip
   read_joint_state.position[13] = RightNaoHipKF.position();
   */
}
const string get_key_name_tf(int n)
{
  switch (n)
  {
    case  base_link:return         "k_base_link"; break;
        //  case  1:return  "k_neck"; break;
        //  case  2:return  "k_Head"; break;
    case  Lshoulder:return         "k_LShoulder"; break;
    case  Lelbow:return            "k_Lelbow";    break;
    case  Lhand:return             "k_l_gripper"; break;
    case  Rshoulder:return         "k_RShoulder"; break;
    case  Relbow:return            "k_Relbow";    break;
    case  Rhand:return             "k_r_gripper"; break;
    case  Lhip:return              "k_LHip";      break;
    case  Lfoot:return             "k_L_foot";    break;         
    case  Lknee:return             "k_left_knee"; break;
    case  Lankle:return            "k_l_ankle";   break;
    case  Rhip:return              "k_RHip";      break;
    case  Rknee:return             "k_right_knee";break;
    case  Rankle:return            "k_r_ankle";   break;
    case  Rfoot:return             "k_R_foot";    break;    
    case  shoulder_spine:return    "k_shoulder_spine"; break;
  }
}
const string get_key_name_model(int n)
{
  switch (n)
  {
    case  0:return  "HeadYaw";        break;
    case  1:return  "HeadPitch";      break;
    case  2:return  "LHipYawPitch";   break;
    case  3:return  "LHipRoll";       break;
    case  4:return  "LHipPitch";      break;
    case  5:return  "LKneePitch";     break;
    case  6:return  "LAnklePitch";    break;
    case  7:return  "LAnkleRoll";     break;
    case  8:return  "RHipYawPitch";   break;
    case  9:return  "RHipRoll";       break;
    case  10:return "RHipPitch";      break;
    case  11:return "RKneePitch";     break;
    case  12:return "RAnklePitch";    break;
    case  13:return "RAnkleRoll";     break;
    case  14:return "LShoulderPitch"; break;
    case  15:return "LShoulderRoll";  break;
    case  16:return "LElbowYaw";      break;
    case  17:return "LElbowRoll";     break;
    case  18:return "LWristYaw";      break;
    case  19:return "LHand";          break;
    case  20:return "RShoulderPitch"; break;
    case  21:return "RShoulderRoll";  break;
    case  22:return "RElbowYaw";      break;
    case  23:return "RElbowRoll";     break;
    case  24:return "RWristYaw";      break;
    case  25:return "RHand";          break;
    case  26:return "RFinger23";      break;
    case  27:return "RFinger13";      break;
    case  28:return "RFinger12";      break;
    case  29:return "LFinger21";      break;
    case  30:return "LFinger13";      break;
    case  31:return "LFinger11";      break;
    case  32:return "RFinger22";      break;
    case  33:return "LFinger22";      break;
    case  34:return "RFinger21";      break;
    case  35:return "LFinger12";      break;
    case  36:return "RFinger11";      break;
    case  37:return "LFinger23";      break;
    case  38:return "LThumb1";        break;
    case  39:return "RThumb1";        break;
    case  40:return "RThumb2";        break;
    case  41:return "LThumb2";        break;
  }
}
/*
ros::Subscriber nodeTracked(const kinect_v2::BodyJoints &user_i)
{
  if(user_0.tracked == "YES") return sub_0;
  else if(user_1.tracked =="YES")return sub_1;
  else if(user_2.tracked =="YES")return sub_2;
  else if(user_3.tracked =="YES")return sub_3;
  else if(user_4.tracked =="YES")return sub_4;
  else return sub_5;
}
*/