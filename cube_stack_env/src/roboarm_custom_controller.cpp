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
#include <mutex>

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
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

class RoboArm {
  public:
  RoboArm(const ros::NodeHandle nh): 
  nh_(nh),
  joint_names({"arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint", "gripper_joint"})
  {
    zero_vec.resize(ids.size(), 0);
    joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/cube_stack_arm/slow_joint_states", 10);
    int status = init();
    if(status<0){
      ROS_ERROR("Failed to init the robot");
    }
    command_sub = nh_.subscribe("/cube_stack_arm/command", 10, &RoboArm::command_callback, this);
  }

  ~RoboArm() {
    portHandler->closePort();
    delete groupSyncWrite;
    delete portHandler;
    delete packetHandler;
  }

  void read() {
    sensor_msgs::JointState msg;
    msg.position.resize(ids.size(), 0);
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
      msg.position[i] = -(150.0-(300.0/1023.0)*(double)dxl_var)*(3.14/180.0);
      if(i==4){ // Presmatic output for gripper
        msg.position[i] = 0.024 - 2*(dxl_var*0.0334-2.7511)/1000;
      }
      pos[i] = msg.position[i];
    }

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.name = joint_names;
    msg.velocity = zero_vec;
    msg.effort = zero_vec;
    joint_state_pub.publish(msg);
  }

  void write() {
    short position_cmd[ids.size()];
    mtx.lock();
    for(size_t i=0;i<ids.size();i++){
      position_cmd[i] = (1023.0/300.0)*(150.0-180.0/3.14*(-cmd[i]));
      if(i==4){ // Presmatic input fixed for open/close
        position_cmd[i] = cmd[i]>0.001 ? 300 : 511;
      }
      // ROS_INFO("Command: %lf, Pos_Cmd: %d", cmd[i], position_cmd[i]);
    }
    mtx.unlock();

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
        // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", ids[i], position_cmd[i]);
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

        read(); // init positions set as command
        for(size_t i=0;i<ids.size();i++){
          cmd[i] = pos[i];
        }
        write();

        ROS_INFO("Robot init success!");
      }

      ROS_INFO("Found %d Dynamixel IDs", (int)ids.size());
      if (ids.size()==0){
        ROS_ERROR("No Dynamixel IDs found in parameter server");
        return -1;
      }

      return 0;
    }

    void command_callback(const std_msgs::Float64MultiArray& msg) {
      mtx.lock();
      for(size_t i=0;i<ids.size();i++){
        cmd[i] = msg.data[i];
      }
      mtx.unlock();
    }

    PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    GroupSyncWrite *groupSyncWrite;

    ros::NodeHandle nh_;
    ros::Subscriber command_sub;
    ros::Publisher joint_state_pub;

    std::vector<int> ids;
    std::vector<double> zero_vec;
    std::vector<std::string> joint_names;
    std::mutex mtx;
    double cmd[5], pos[5];
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "roboarm_custom_controller");
  ros::NodeHandle nh;

  RoboArm robot(nh);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Duration(3).sleep(); // wait for controllers to start
  ROS_INFO("Ready to take Commands");

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    robot.read();
    robot.write();
    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}
