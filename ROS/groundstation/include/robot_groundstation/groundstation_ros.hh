#ifndef _CAR_Groundstation_ROS_HH_
#define _CAR_Groundstation_ROS_HH_

#include "robot_groundstation/groundstation.hh"
#include "robot_groundstation/camera_network.hh"

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <image_transport/image_transport.h>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv/cv.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <config_server/parameter.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/builtin_bool.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace roboland {

class GroundstationRos {
public:
  GroundstationRos(ros::NodeHandle &nh,
          ros::NodeHandle &pnh, int argc, char *argv[],std::string id);
  
  ~GroundstationRos();

  boost::thread thread_main;

  void thrMain();

  Groundstation *ground_station;

  //Publisher
  ros::Publisher pub_motor1;
  ros::Publisher pub_motor2;
  ros::Publisher pub_motor3;
  ros::Publisher pub_torque;
  ros::Publisher pub_led;
  ros::Publisher pub_beep;

  ros::Publisher pub_marker;
  ros::Publisher pub_marker2;
  ros::Publisher pub_marker3;

  //Subscriber
  ros::Subscriber sub_speed1 ;
  ros::Subscriber sub_speed2;
  ros::Subscriber sub_position;
  ros::Subscriber sub_load;
  ros::Subscriber sub_ismoveing;
  ros::Subscriber sub_alarm;
  ros::Subscriber sub_voltage;
  ros::Subscriber sub_sensor;
  ros::Publisher  pub_laser;

  void callbackSpeed1(const std_msgs::Int32::ConstPtr &msg);
  void callbackSpeed2(const std_msgs::Int32::ConstPtr &msg);
  void callbackPosition(const std_msgs::Int32::ConstPtr &msg);
  void callbackLoad(const std_msgs::Int32::ConstPtr &msg);
  void callbackIsMoveing(const std_msgs::Int32::ConstPtr &msg);
  void callbackVoltage(const std_msgs::Int32::ConstPtr &msg);
  void callbackSensor(const std_msgs::Int32::ConstPtr &msg);
  void callbackAlarm(const std_msgs::Int32::ConstPtr &msg);

  int current_speed1;
  int current_speed2;
  int current_position;
  int current_load;
  int current_ismoveing;
  int current_voltage;
  int current_sensor;
  int current_alaram;
  bool making_plan;

  ros::Subscriber sub_laser;
  ros::Subscriber sub_slam_pose;
  ros::Subscriber sub_map;
  ros::ServiceClient client_makeplan;
  ros::ServiceClient client_resetcostmap;
  ros::Publisher pub_slam_origin;
  ros::Publisher pub_slam_reset;

  //Local and global paths
  nav_msgs::Path globalPath;
  int global_path_size;

  nav_msgs::Path localPath;
  int local_path_size;

  double Quat2Rad(double orientation[]);

  //position
  geometry_msgs::Pose2D current_slam_position;
  geometry_msgs::Pose2D current_goal_position;
  geometry_msgs::Pose2D current_temp_gaol_position;

  //map
  nav_msgs::OccupancyGrid current_map;

  //path
  nav_msgs::Path current_path;
  nav_msgs::Path current_temp_path;

  //laser
  sensor_msgs::LaserScan current_laser;

  void callbackLaser(const sensor_msgs::LaserScan::ConstPtr &msg);
  void GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackGroundstation(CarCommand cmd);
  void updateHectorOrigin(geometry_msgs::Pose2D p);
  void GetCostmap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void updateViz();

  nav_msgs::Path makePlan();
  void resetHectorSlam();

  bool mutex;
  bool app_exit;
  int counter;
  int send_counter;
  ros::NodeHandle ref_nh;

  config_server::Parameter<std::string> p_tcp_groundstation_remote_ip;


};


} 

#endif 
