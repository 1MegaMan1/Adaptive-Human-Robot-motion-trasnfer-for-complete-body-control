#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <kinect_v2/NaoInterface.hpp>

#define   HeadPitch 0
#define   HeadYaw 1
#define   LAnklePitch 2
#define   LAnkleRoll 3
#define   LElbowRoll 4
#define   LElbowYaw 5
#define   LHand 6
#define   LHipPitch 7
#define   LHipRoll 8
#define   LHipYawPitch 9
#define   LKneePitch 10
#define   LShoulderPitch 11
#define   LShoulderRoll 12
#define   LWristYaw 13
#define   RAnklePitch 14
#define   RAnkleRoll 15
#define   RElbowRoll 16
#define   RElbowYaw 17
#define   RHand 18
#define   RHipPitch 19
#define   RHipRoll 20
#define   RHipYawPitch 21
#define   RKneePitch 22
#define   RShoulderPitch 23
#define   RShoulderRoll 24
#define   RWristYaw 25

 //bool NaoInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
NaoInterface::NaoInterface(): jnt_state_interface(jnt_state_interface), jnt_pos_interface(jnt_pos_interface)
 { 
  //reads   

   joint_states.name.resize(26);
   joint_states.position.resize(26);
   joint_states.velocity.resize(26);
   joint_states.effort.resize(26);
   cmd_joint_states.name.resize(14);
   cmd_joint_states.position.resize(14);

    hardware_interface::JointStateHandle state_handle_LElbowYaw          (joint_states.name[LElbowYaw],     &joint_states.position[LElbowYaw],     &joint_states.velocity[LElbowYaw],     &joint_states.effort[LElbowYaw]);
     hardware_interface::JointStateHandle state_handle_LHand              (joint_states.name[LHand],         &joint_states.position[LHand],         &joint_states.velocity[LHand],         &joint_states.effort[LHand]);
     hardware_interface::JointStateHandle state_handle_LWristYaw          (joint_states.name[LWristYaw],     &joint_states.position[LWristYaw],     &joint_states.velocity[LWristYaw],     &joint_states.effort[LWristYaw]);
     hardware_interface::JointStateHandle state_handle_RElbowYaw          (joint_states.name[RElbowYaw],     &joint_states.position[RElbowYaw],     &joint_states.velocity[RElbowYaw],     &joint_states.effort[RElbowYaw]);

     hardware_interface::JointStateHandle state_handle_Rightelbow         (joint_states.name[RElbowRoll],    &joint_states.position[RElbowRoll],    &joint_states.velocity[RElbowRoll],    &joint_states.effort[RElbowRoll]);
     hardware_interface::JointStateHandle state_handle_Leftelbow          (joint_states.name[LElbowRoll],    &joint_states.position[LElbowRoll],    &joint_states.velocity[LElbowRoll],    &joint_states.effort[LElbowRoll]);
     hardware_interface::JointStateHandle state_handle_RShoulderRoll      (joint_states.name[RShoulderRoll], &joint_states.position[RShoulderRoll], &joint_states.velocity[RShoulderRoll], &joint_states.effort[RShoulderRoll]);
     hardware_interface::JointStateHandle state_handle_LShoulderRoll      (joint_states.name[LShoulderRoll], &joint_states.position[LShoulderRoll], &joint_states.velocity[LShoulderRoll], &joint_states.effort[LShoulderRoll]);
     hardware_interface::JointStateHandle state_handle_LShoulderPitch     (joint_states.name[LShoulderPitch],&joint_states.position[LShoulderPitch],&joint_states.velocity[LShoulderPitch],&joint_states.effort[LShoulderPitch]);
     hardware_interface::JointStateHandle state_handle_RShoulderPitch     (joint_states.name[RShoulderPitch],&joint_states.position[RShoulderPitch],&joint_states.velocity[RShoulderPitch],&joint_states.effort[RShoulderPitch]);
     hardware_interface::JointStateHandle state_handle_LeftKnee           (joint_states.name[LKneePitch],    &joint_states.position[LKneePitch],    &joint_states.velocity[LKneePitch],    &joint_states.effort[LKneePitch]);
     hardware_interface::JointStateHandle state_handle_RightKnee          (joint_states.name[RKneePitch],    &joint_states.position[RKneePitch],    &joint_states.velocity[RKneePitch],    &joint_states.effort[RKneePitch]);
     hardware_interface::JointStateHandle state_handle_LeftHipRoll        (joint_states.name[LHipRoll],      &joint_states.position[LHipRoll],      &joint_states.velocity[LHipRoll],      &joint_states.effort[LHipRoll]);
     hardware_interface::JointStateHandle state_handle_RightHipRoll       (joint_states.name[RHipRoll],      &joint_states.position[RHipRoll],      &joint_states.velocity[RHipRoll],      &joint_states.effort[RHipRoll]);
     hardware_interface::JointStateHandle state_handle_LeftHipPitch       (joint_states.name[LHipPitch],     &joint_states.position[LHipPitch],     &joint_states.velocity[LHipPitch],     &joint_states.effort[LHipPitch]);
     hardware_interface::JointStateHandle state_handle_RightHipPitch      (joint_states.name[RHipPitch],     &joint_states.position[RHipPitch],     &joint_states.velocity[RHipPitch],     &joint_states.effort[RHipPitch]);
     hardware_interface::JointStateHandle state_handle_LeftNaoHip         (joint_states.name[LHipYawPitch],  &joint_states.position[LHipYawPitch],  &joint_states.velocity[LHipYawPitch],  &joint_states.effort[LHipYawPitch]);
     hardware_interface::JointStateHandle state_handle_RightNaoHip        (joint_states.name[RHipYawPitch],  &joint_states.position[RHipYawPitch],  &joint_states.velocity[RHipYawPitch],  &joint_states.effort[RHipYawPitch]);
   
     hardware_interface::JointStateHandle state_handle_RAnklePitch        (joint_states.name[RAnklePitch],   &joint_states.position[RAnklePitch],   &joint_states.velocity[RAnklePitch],   &joint_states.effort[RAnklePitch]);
     hardware_interface::JointStateHandle state_handle_RAnkleRoll         (joint_states.name[RAnkleRoll],    &joint_states.position[RAnkleRoll],    &joint_states.velocity[RAnkleRoll],    &joint_states.effort[RAnkleRoll]);
     hardware_interface::JointStateHandle state_handle_RHand              (joint_states.name[RHand],         &joint_states.position[RHand],         &joint_states.velocity[RHand],         &joint_states.effort[RHand]);
     hardware_interface::JointStateHandle state_handle_RWristYaw          (joint_states.name[RWristYaw],     &joint_states.position[RWristYaw],     &joint_states.velocity[RWristYaw],     &joint_states.effort[RWristYaw]);
     hardware_interface::JointStateHandle state_handle_HeadYaw            (joint_states.name[HeadYaw],       &joint_states.position[HeadYaw],       &joint_states.velocity[HeadYaw],       &joint_states.effort[HeadYaw]);
     hardware_interface::JointStateHandle state_handle_HeadPitch          (joint_states.name[HeadPitch],     &joint_states.position[HeadPitch],     &joint_states.velocity[HeadPitch],     &joint_states.effort[HeadPitch]);
     hardware_interface::JointStateHandle state_handle_LAnklePitch        (joint_states.name[LAnklePitch],   &joint_states.position[LAnklePitch],   &joint_states.velocity[LAnklePitch],   &joint_states.effort[LAnklePitch]);
     hardware_interface::JointStateHandle state_handle_LAnkleRoll         (joint_states.name[LAnkleRoll],    &joint_states.position[LAnkleRoll],    &joint_states.velocity[LAnkleRoll],    &joint_states.effort[LAnkleRoll]);
    


     jnt_state_interface.registerHandle(state_handle_Rightelbow);
     jnt_state_interface.registerHandle(state_handle_Leftelbow);
     jnt_state_interface.registerHandle(state_handle_RShoulderRoll);
     jnt_state_interface.registerHandle(state_handle_LShoulderRoll);
     jnt_state_interface.registerHandle(state_handle_LShoulderPitch);
     jnt_state_interface.registerHandle(state_handle_RShoulderPitch); 
     jnt_state_interface.registerHandle(state_handle_LeftKnee);
     jnt_state_interface.registerHandle(state_handle_RightKnee);
     jnt_state_interface.registerHandle(state_handle_LeftHipRoll);
     jnt_state_interface.registerHandle(state_handle_RightHipRoll);
     jnt_state_interface.registerHandle(state_handle_LeftHipPitch);
     jnt_state_interface.registerHandle(state_handle_RightHipPitch);
     jnt_state_interface.registerHandle(state_handle_LeftNaoHip);    
     jnt_state_interface.registerHandle(state_handle_RightNaoHip);

      jnt_state_interface.registerHandle(state_handle_LElbowYaw);
      jnt_state_interface.registerHandle(state_handle_LHand);
      jnt_state_interface.registerHandle(state_handle_LWristYaw);
     jnt_state_interface.registerHandle(state_handle_RElbowYaw);
     jnt_state_interface.registerHandle(state_handle_HeadYaw);
     jnt_state_interface.registerHandle(state_handle_HeadPitch);
     jnt_state_interface.registerHandle(state_handle_RAnklePitch);
     jnt_state_interface.registerHandle(state_handle_LAnklePitch);
     jnt_state_interface.registerHandle(state_handle_LAnkleRoll); 
     jnt_state_interface.registerHandle(state_handle_RAnkleRoll);
     jnt_state_interface.registerHandle(state_handle_RHand);
     jnt_state_interface.registerHandle(state_handle_RWristYaw);

   registerInterface(&jnt_state_interface);
   //commands
     hardware_interface::JointHandle pos_handle_RightelbowKF        (jnt_state_interface.getHandle(cmd_joint_states.name[0]),&cmd_joint_states.position[0]);
     hardware_interface::JointHandle pos_handle_LeftelbowKF         (jnt_state_interface.getHandle(cmd_joint_states.name[1]),&cmd_joint_states.position[1]);
     hardware_interface::JointHandle pos_handle_RShoulderRollKF     (jnt_state_interface.getHandle(cmd_joint_states.name[2]),&cmd_joint_states.position[2]);
     hardware_interface::JointHandle pos_handle_LShoulderRollKF     (jnt_state_interface.getHandle(cmd_joint_states.name[3]),&cmd_joint_states.position[3]);
     hardware_interface::JointHandle pos_handle_LShoulderPitchKF    (jnt_state_interface.getHandle(cmd_joint_states.name[4]),&cmd_joint_states.position[4]);
     hardware_interface::JointHandle pos_handle_RShoulderPitchKF    (jnt_state_interface.getHandle(cmd_joint_states.name[5]),&cmd_joint_states.position[5]);
     hardware_interface::JointHandle pos_handle_LeftKneeKF          (jnt_state_interface.getHandle(cmd_joint_states.name[6]),&cmd_joint_states.position[6]);
     hardware_interface::JointHandle pos_handle_RightKneeKF         (jnt_state_interface.getHandle(cmd_joint_states.name[7]),&cmd_joint_states.position[7]);
     hardware_interface::JointHandle pos_handle_LeftHipRollKF       (jnt_state_interface.getHandle(cmd_joint_states.name[8]),&cmd_joint_states.position[8]);
     hardware_interface::JointHandle pos_handle_RightHipRollKF      (jnt_state_interface.getHandle(cmd_joint_states.name[9]),&cmd_joint_states.position[9]);
     hardware_interface::JointHandle pos_handle_LeftHipPitchKF      (jnt_state_interface.getHandle(cmd_joint_states.name[10]),&cmd_joint_states.position[10]);
     hardware_interface::JointHandle pos_handle_RightHipPitchKF     (jnt_state_interface.getHandle(cmd_joint_states.name[11]),&cmd_joint_states.position[11]);
     hardware_interface::JointHandle pos_handle_LeftNaoHipKF        (jnt_state_interface.getHandle(cmd_joint_states.name[12]),&cmd_joint_states.position[12]);
     hardware_interface::JointHandle pos_handle_RightNaoHipKF       (jnt_state_interface.getHandle(cmd_joint_states.name[13]),&cmd_joint_states.position[13]);

     jnt_pos_interface.registerHandle(pos_handle_RightelbowKF);
     jnt_pos_interface.registerHandle(pos_handle_LeftelbowKF);
     jnt_pos_interface.registerHandle(pos_handle_RShoulderRollKF);
     jnt_pos_interface.registerHandle(pos_handle_LShoulderRollKF);
     jnt_pos_interface.registerHandle(pos_handle_LShoulderPitchKF);
     jnt_pos_interface.registerHandle(pos_handle_RShoulderPitchKF);
     jnt_pos_interface.registerHandle(pos_handle_LeftKneeKF);
     jnt_pos_interface.registerHandle(pos_handle_RightKneeKF);
     jnt_pos_interface.registerHandle(pos_handle_LeftHipRollKF);
     jnt_pos_interface.registerHandle(pos_handle_RightHipRollKF);
     jnt_pos_interface.registerHandle(pos_handle_LeftHipPitchKF);
     jnt_pos_interface.registerHandle(pos_handle_RightHipPitchKF);
     jnt_pos_interface.registerHandle(pos_handle_LeftNaoHipKF);
     jnt_pos_interface.registerHandle(pos_handle_RightNaoHipKF);

   registerInterface(&jnt_pos_interface);

   ROS_ERROR("Read: %f",joint_states.position[0]);
   ROS_ERROR("CMD: %f",cmd_joint_states.position[0]);
 //  return true;
   return;
  }
NaoInterface:: ~NaoInterface(){}
