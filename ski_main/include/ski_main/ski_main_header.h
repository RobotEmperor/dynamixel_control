#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include "../include/test_dynamixel/dynamixel_MX106.h"
#include "ski_main/dynamixel_info.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

#define PI  3.14159265358979323

using namespace dynamixel_controller;
using namespace dynamixel;

namespace ski
{

class ski_main_class{

  private:


  bool torque;
  double count_time;
  int position[15];
  int speed[15];

  double ans;
  double ans_pre;
  double alpha;
  std_msgs::Float32 msg1;
  std_msgs::Float32 msg;

  public:
    ski_main_class();
    ros::NodeHandle nh;
    ros::Publisher ans_pub;
    ros::Publisher dynamixel_output;

    DynamixelController dy;
    void control_loop(const ros::TimerEvent& event);
    void pose_sub(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void command_sub(const std_msgs::Bool::ConstPtr& msg);




};

}
