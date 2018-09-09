#include "infrastructure/roboland_node.hh"

#include <std_msgs/Int32.h>

#include <iostream>

using namespace std;

ROBOLAND_NODE()

void messageCallback(const std_msgs::Int32::ConstPtr &msg)
{
  cout << msg->data << endl;
}

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle nh;
  ros::Subscriber sub_msg;

  sub_msg = nh.subscribe("message_publisher", 1, messageCallback);

  CREATE_ROBOLAND_NODE("node2");

  ros::Rate loop(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
