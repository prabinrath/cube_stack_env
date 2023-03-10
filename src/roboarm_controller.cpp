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
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun cube_stack_env sync_read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub /cube_stack_arm/sync_set_positions cube_stack_env/SyncSetPositions "{id:[1,2,3,4,5], position:[511,511,511,511,511]}" --once
 * $ rostopic echo /cube_stack_arm/joint_states
 *
 * Author: Jaehyun Shim, Prabin Kumar Rath
*******************************************************************************/

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>

#include "sensor_msgs/JointState.h"
#include "cube_stack_env/SyncSetPositions.h"
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

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 2);

void SyncSetPositionsCallback(const cube_stack_env::SyncSetPositions::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_position[2];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  for(size_t i=0;i<msg->id.size();i++){
    uint16_t position = (unsigned int)msg->position[i]; // Convert int16 -> uint16
    param_goal_position[0] = DXL_LOBYTE(position);
    param_goal_position[1] = DXL_HIBYTE(position);

    dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id[i], param_goal_position);
    if (dxl_addparam_result != true) {
      ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id[i]);
    }
  }

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    for(size_t i=0;i<msg->id.size();i++){
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id[i], msg->position[i]);
    }
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }

  groupSyncWrite.clearParam();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "sync_read_write_node");

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

  std::vector<int> ids;
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
  }

  ROS_INFO("Found %d Dynamixel IDs", (int)ids.size());
  if (ids.size()==0){
    ROS_ERROR("No Dynamixel IDs found in parameter server");
    return 0;
  }

  ros::NodeHandle nh;
  ros::Subscriber sync_set_position_sub = nh.subscribe("/cube_stack_arm/sync_set_positions", 10, SyncSetPositionsCallback);
  ros::Publisher position_feedback_pub = nh.advertise<sensor_msgs::JointState>("/cube_stack_arm/joint_states", 10);
  
  std::vector<std::string> joint_names = {"arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint", "gripper_joint"};
  std::vector<double> zero_vec(ids.size());
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t dxl_var = -1;

    // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
    std::vector<double> position(ids.size());

    for(size_t i=0;i<ids.size();i++){
      dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler, ids[i], ADDR_PRESENT_POSITION, &dxl_var, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to get position for Dynamixel ID %d", ids[i]);
        return false;
      }
      position[i] = (150.0-(300.0/1023.0)*(double)dxl_var)*(3.14/180.0);
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.name = joint_names;
    msg.position = position;
    msg.velocity = zero_vec;
    msg.effort = zero_vec;
    position_feedback_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ros::spin();

  portHandler->closePort();
  return 0;
}
