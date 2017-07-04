#include "../include/ski_main/ski_main_header.h"
using namespace std;
using namespace ski;
int8_t dxl_id[all_DXL]={0,1,0,3,0,5,0,0,0,0,0,0,0,13,0};
double amplitude = 250;
double frequency = 1;


ski_main_class::ski_main_class()
{
  ans_pub= nh.advertise<std_msgs::Float32>("ans", 100);
  dynamixel_output = nh.advertise<std_msgs::Float32>("output", 100);

  dy.initDynamixel();

  torque = false;
  ans = 0.0;
  ans_pre = 0.0;
  alpha = 0.9;

  count_time = 0;

  for(int i=1;i<all_DXL;i++)
  {
    speed[i] = 360; // max velocity //
    position[i] = 0; //initial position //
  }
}

void ski_main_class::control_loop(const ros::TimerEvent& event)
{



  //////////////////////////////////////////////////////////////////////////////////////////////
  dy.bulk_rxpacket(dxl_id, 128, 8);

  dy.sync_Write_torque(dxl_id, torque);
  dy.sync_Write(dxl_id, position, speed);

  dy.bulk_txpacket(dxl_id, 128, 8);
  //////////////////////////////////////////////////////////////////////////////////////////////
  count_time ++;

  ans = ans_pre*alpha + (1-alpha)*ans;
  msg.data = ans;

  for(int i=1;i<all_DXL;i++)
  {
    position[i] = (int) ans;
  }

  ans = amplitude/2*sin(2*PI*frequency*count_time*0.008)+amplitude/2+10;

  ans_pre = ans;

  msg1.data = (double) dy.real_position[1];


  ans_pub.publish(msg);
  dynamixel_output.publish(msg1);


  //sleep(0.001);




}

void ski_main_class::pose_sub(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

  amplitude = msg->data[0];
  frequency = msg->data[1];


}
void ski_main_class::command_sub(const std_msgs::Bool::ConstPtr& msg)
{
  torque = msg->data;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dynamixel_node");
  ski_main_class node;
  ros::Timer control_timer = node.nh.createTimer(ros::Duration(0.008), &ski_main_class::control_loop, &node);
  ros::Subscriber dynamixel_angle = node.nh.subscribe("option",100, &ski_main_class::pose_sub, &node);
  ros::Subscriber dynamixel_command = node.nh.subscribe("dynamixel_command",100, &ski_main_class::command_sub, &node);

  ros::spin();

  return 0;
}
