// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
Author: Prabin Kumar Rath
*******************************************************************************/

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <ros/callback_queue.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;
 
// AX-18A Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_PRESENT_POSITION 36
#define ADDR_GOAL_POSITION    30
#define ADDR_MOVING_SPEED     32

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE              1000000          // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

class RoboArm : public hardware_interface::RobotHW {
  public:
  RoboArm(){
    int status = init();
    if(status<0){
      ROS_ERROR("Failed to init the robot");
    }
    
    hardware_interface::JointStateHandle state_handle_arm_shoulder_pan_joint("arm_shoulder_pan_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_arm_shoulder_pan_joint);
    hardware_interface::JointStateHandle state_handle_arm_shoulder_lift_joint("arm_shoulder_lift_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_arm_shoulder_lift_joint);
    hardware_interface::JointStateHandle state_handle_arm_elbow_flex_joint("arm_elbow_flex_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_arm_elbow_flex_joint);
    hardware_interface::JointStateHandle state_handle_arm_wrist_flex_joint("arm_wrist_flex_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_arm_wrist_flex_joint);
    hardware_interface::JointStateHandle state_handle_gripper_joint("gripper_joint", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_gripper_joint);
    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle_arm_shoulder_pan_joint(jnt_state_interface.getHandle("arm_shoulder_pan_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_arm_shoulder_pan_joint);
    hardware_interface::JointHandle pos_handle_arm_shoulder_lift_joint(jnt_state_interface.getHandle("arm_shoulder_lift_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_arm_shoulder_lift_joint);
    hardware_interface::JointHandle pos_handle_arm_elbow_flex_joint(jnt_state_interface.getHandle("arm_elbow_flex_joint"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_arm_elbow_flex_joint);
    hardware_interface::JointHandle pos_handle_arm_wrist_flex_joint(jnt_state_interface.getHandle("arm_wrist_flex_joint"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_arm_wrist_flex_joint);
    hardware_interface::JointHandle pos_handle_gripper_joint(jnt_state_interface.getHandle("gripper_joint"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_gripper_joint);
    registerInterface(&jnt_pos_interface);
  }

  ~RoboArm(){
    portHandler->closePort();
    delete groupSyncWrite;
    delete portHandler;
    delete packetHandler;
  }

  void read(const ros::Time& time, const ros::Duration& period) override {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t dxl_var = -1;
    for(size_t i=0;i<ids.size();i++){
      dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler, ids[i], ADDR_PRESENT_POSITION, &dxl_var, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to get position for Dynamixel ID %d", ids[i]);
        return;
      }
      pos[i] = -(150.0-(300.0/1023.0)*(double)dxl_var)*(3.14/180.0);
      if(i==4){ // Presmatic output for gripper
        pos[i] = 0.024 - 2*(dxl_var*0.0334-2.7511)/1000;
      }
    }
  }

  void write(const ros::Time& time, const ros::Duration& period) override {
    short position_cmd[ids.size()];
    for(size_t i=0;i<ids.size();i++){
      position_cmd[i] = (1023.0/300.0)*(150.0-180.0/3.14*(-cmd[i]));
      if(i==4){ // Presmatic input fixed for open/close
        position_cmd[i] = cmd[i]>0.001 ? 320 : 511;
      }
      ROS_INFO("Command: %lf, Pos_Cmd: %d", cmd[i], position_cmd[i]);
    }

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;
    uint8_t param_goal_position[2];

    // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    for(size_t i=0;i<ids.size();i++){
      uint16_t position = (unsigned int)position_cmd[i]; // Convert int16 -> uint16
      param_goal_position[0] = DXL_LOBYTE(position);
      param_goal_position[1] = DXL_HIBYTE(position);

      dxl_addparam_result = groupSyncWrite->addParam((uint8_t)ids[i], param_goal_position);
      if (dxl_addparam_result != true) {
        ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", ids[i]);
      }
    }

    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
      for(size_t i=0;i<ids.size();i++){
        ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", ids[i], position_cmd[i]);
      }
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }

    groupSyncWrite->clearParam();
  }

  private:
    int init(){
      groupSyncWrite = new GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 2);
      uint8_t dxl_error = 0;
      int dxl_comm_result = COMM_TX_FAIL;

      if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port!");
        return -1;
      }

      if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
      }

      std::string dynamixel_ids;
      if (ros::param::get("/cube_stack_arm/dynamixel_ids", dynamixel_ids)){
        int id = 0;
        for(size_t i=0;i<dynamixel_ids.size();i++){
          if(dynamixel_ids[i]==44){
            ids.push_back(id);
            id = 0;
            continue;
          }
          id = 10*id + (int)dynamixel_ids[i] - 48;
        }

        // Enable Torque
        for(size_t i=0;i<ids.size();i++){
          dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, ids[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", ids[i]);
            return -1;
          }
        }

        // Set Motor Max Speed
        for(size_t i=0;i<ids.size();i++){
          dxl_comm_result = packetHandler->write2ByteTxRx(
            portHandler, ids[i], ADDR_MOVING_SPEED, 50, &dxl_error); // 901 corresponds to 100 rev/min
          if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", ids[i]);
            return -1;
          }
        }

        ROS_INFO("Robot init success!");
      }

      ROS_INFO("Found %d Dynamixel IDs", (int)ids.size());
      if (ids.size()==0){
        ROS_ERROR("No Dynamixel IDs found in parameter server");
        return -1;
      }

      return 0;
    }

    PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    GroupSyncWrite *groupSyncWrite;

    std::vector<int> ids;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[5];
    double pos[5];
    double vel[5];
    double eff[5];
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "roboarm_hardware_interface");
  ros::NodeHandle nh;
  // https://answers.ros.org/question/119316/using-controller-manager-and-getting-it-to-work/
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  RoboArm robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Duration(3).sleep(); // wait for controllers to start
  ROS_INFO("Ready to take Commands");

  ros::Time ts_last = ros::Time::now();
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::Time ts_now = ros::Time::now();
    ros::Duration dur = ts_now - ts_last;
    robot.read(ts_now, dur);
    cm.update(ts_now, dur);
    robot.write(ts_now, dur);
    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}
