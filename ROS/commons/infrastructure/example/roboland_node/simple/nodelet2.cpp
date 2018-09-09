#include <infrastructure/roboland_node.hh>

#include <std_msgs/Int32.h>
#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>

class Nodelet2 : public roboland::Nodelet
{
public:
  Nodelet2():
    roboland::Nodelet("Nodelet2")
  {}

  void onInitRobolandNodelet() override
  {
    ros::NodeHandle nh = this->getNodeHandle();

    this->sub_msg = nh.subscribe("message_publisher", 1, &Nodelet2::messageCallback, this);
  }

  void messageCallback(const std_msgs::Int32::ConstPtr &msg)
  {
    cout << msg->data << endl;
  }

private:
  ros::Subscriber sub_msg;
};

PLUGINLIB_DECLARE_CLASS(infrastructure, Nodelet2, Nodelet2, nodelet::Nodelet)
