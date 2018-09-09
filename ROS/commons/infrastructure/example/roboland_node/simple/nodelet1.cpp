#include <infrastructure/roboland_node.hh>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Int32.h>

class Nodelet1 : public roboland::Nodelet
{
public:
  Nodelet1():
    roboland::Nodelet("Nodelet1")
  {
    this->requiredNode("Nodelet2");
  }

  void onInitRobolandNodelet() override
  {
    ros::NodeHandle nh = this->getNodeHandle();

    this->msg_publisher = nh.advertise<std_msgs::Int32>("message_publisher", 1);
    this->publish_timer = nh.createTimer(ros::Duration(0.1), &Nodelet1::publishMsg, this);
  }

  void publishMsg(const ros::TimerEvent &e)
  {
    static uint32_t i = 0;

    std_msgs::Int32 msg;
    msg.data = i++;

    this->msg_publisher.publish(msg);
  }

private:
  ros::Publisher msg_publisher;
  ros::Timer publish_timer;
};

PLUGINLIB_DECLARE_CLASS(infrastructure, Nodelet1, Nodelet1, nodelet::Nodelet)
