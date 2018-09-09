#include "infrastructure/roboland_node.hh"

#include <ros/time.h>
#include <std_msgs/Int32.h>

#include <iostream>

using namespace std;

ROBOLAND_NODE()

ros::Publisher msg_publisher;

void publishMsg(const ros::TimerEvent &e)
{
  static uint32_t i = 0;

  std_msgs::Int32 msg;
  msg.data = i++;

  msg_publisher.publish(msg);
}

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nh;

  REQUIRED_NODE("node2", "/");

  msg_publisher = nh.advertise<std_msgs::Int32>("message_publisher", 1);

  CREATE_ROBOLAND_NODE("node1");

  ros::Timer publish_timer = nh.createTimer(ros::Duration(0.1), publishMsg);

  ros::Rate loop(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
