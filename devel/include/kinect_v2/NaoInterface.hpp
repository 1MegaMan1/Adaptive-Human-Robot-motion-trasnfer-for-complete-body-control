
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <string>
#pragma once

class NaoInterface : public hardware_interface::RobotHW
{
public:
  NaoInterface();
  ~NaoInterface();
//bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
private:
hardware_interface::JointStateInterface jnt_state_interface;
hardware_interface::PositionJointInterface jnt_pos_interface;
sensor_msgs::JointState cmd_joint_states;
sensor_msgs::JointState joint_states;
};