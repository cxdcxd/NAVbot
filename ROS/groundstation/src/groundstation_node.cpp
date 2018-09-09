
#include <infrastructure/roboland_node.hh>
#include <ros/ros.h>
#include <robot_groundstation/groundstation_ros.hh>

ROBOLAND_NODE()

int
main(int argc, char *argv[])
{

  ros::init(argc, argv, "robot_groundstation");

#ifdef ROBOLAND_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  REQUIRED_NODE("config_server", "/");
  
  boost::shared_ptr<roboland::GroundstationRos> en(new roboland::GroundstationRos(nh,pnh,argc,argv,"1"));

  CREATE_ROBOLAND_NODE("robot_groundstation");

  ros::Rate loop(100);

  std::string kill_path =  ros::package::getPath("robot_groundstation") + "/shell/tcpkill.sh";

  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }

  int _result = system(kill_path.c_str());
  ROS_INFO_STREAM("Kill tcp thread with result : "<<_result);

  return 0;
}
