#ifndef ROBOLAND_INFRASTRUCTURE_ROBOLAND_NODE_HH
#define ROBOLAND_INFRASTRUCTURE_ROBOLAND_NODE_HH

#include "infrastructure/NodeInfo.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <string>

using namespace std;

namespace roboland
{

/**
 * @brief A class for helping add some fake nodes to a unit case test.
 *
 * Each test has some dependencies which may be provided using another
 * modules which are not related to this one, so to avoid testing all
 * of modules together, using this class other modules can constructed
 * fake. For more info RTD.
 */

class TestNode
{
public:
  TestNode(const string& name):
    name(name)
  {
  }

  /**
   * @brief The main service which uses for a node to declare itself
   *        exist.
   *
   * @param req
   * @param res
   *
   * @return
   */
  bool srvReady(infrastructure::NodeInfo::Request &req,
                infrastructure::NodeInfo::Response &res)
  {
    if (req.str.data == "check_for_ready")
      req.str.data == "ok";

    return true;
  }

  /**
   * @brief Using the above service and this function some class can
   *        show many different modules.
   *
   * @param name
   */
  void addFakeNode(const string& name)
  {
    ros::NodeHandle nh;

    ros::ServiceServer ready_server =
      nh.advertiseService(name+"/ready",
                          &TestNode::srvReady,
                          this);
    this->fake_node_services.push_back(ready_server);
  }

  virtual ~TestNode()
  {}

private:
  const string name;
  vector<ros::ServiceServer> fake_node_services;
};

/**
 * @brief A class which uses to register a nodelet as a regular
 *        roboland node. RTD for more info.
 *
 * @param name
 *
 * @return
 */
class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet(const string& name):
    name(name),
    last_requirement_index(0)
  {}

  bool srvReady(infrastructure::NodeInfo::Request &req,
                infrastructure::NodeInfo::Response &res)
  {
    if (req.str.data == "check_for_ready")
      res.str.data == "ok";
    return true;
  }

  void onInit() override
  {
    ros::NodeHandle nh = this->getMTNodeHandle();

    this->timer_ready = nh.createTimer(ros::Duration(0.1),
                                       &roboland::Nodelet::loopCheckRequirements,
                                       this);
  }

  /**
   * @brief Main loop which use requirements of a nodelet are ready,
   *        using calling NodeInfo of them.
   *
   * @param e
   */
  void loopCheckRequirements(const ros::TimerEvent &e)
  {
    boost::mutex::scoped_lock guard(this->requirement_nodes_mutex);

    if (this->last_requirement_index == this->requirement_nodes.size())
    {
      this->timer_ready.stop();

      ros::NodeHandle nh = this->getNodeHandle();

      this->srv_ready = nh.advertiseService(this->name + "/ready",
                                                &roboland::Nodelet::srvReady,
                                                this);
      this->onInitRobolandNodelet();
    }
    else
    {
      ros::NodeHandle nh = this->getNodeHandle();

      auto node_name = this->requirement_nodes[this->last_requirement_index];
      ROS_INFO_STREAM("Checking for dependency \"" << node_name << "\"");
      ros::ServiceClient c = nh.serviceClient<infrastructure::NodeInfo>(node_name + "/ready");

      infrastructure::NodeInfo srv;
      srv.request.str.data = "check_for_ready";

      int i = 0;
      ros::Rate loop(10);

      while (ros::ok() && !c.call(srv) && i < 1000)
      {
        loop.sleep();
        ++i;
      }

      if (i < 1000)
      {
        ROS_INFO_STREAM("Dependency \"" << node_name << "\" is active.");
      }
      else
      {
        ROS_INFO_STREAM("Could not find dependency \"" << node_name << "\"");
      }

      ++this->last_requirement_index;
    }

  }

  virtual void onInitRobolandNodelet() = 0;

  /**
   * @brief Add a node as a requirement for the current one.
   *
   * @param name
   */
  void requiredNode(const string& name)
  {
    this->requirement_nodes.push_back(name);
  }

private:
  ros::ServiceServer srv_ready;
  const string name;
  vector<string> requirement_nodes;
  ros::Timer timer_ready;
  unsigned int last_requirement_index;
  boost::mutex requirement_nodes_mutex;
};

} // namespace roboland

/**
 * @brief Declare a node as a roboland node. Roboland node add and
 *        registers some functionality to code.
 *
 */
#define ROBOLAND_NODE()                                                 \
  namespace roboland{                                                   \
  struct RobolandNode                                                   \
  {                                                                     \
    RobolandNode(ros::NodeHandle nh, const string& name)                \
    {                                                                   \
      this->srv_ready = nh.advertiseService(name + "/ready",            \
                                                &RobolandNode::srvReady, this); \
    }                                                                   \
    ros::ServiceServer srv_ready;                                       \
    bool srvReady(infrastructure::NodeInfo::Request &req,               \
                  infrastructure::NodeInfo::Response &res)              \
    {                                                                   \
      if (req.str.data == "check_for_ready")                            \
        res.str.data = "ok";                                            \
      return true;                                                      \
    }                                                                   \
  };                                                                    \
  }

/**
 * @brief Macro which create a node which registered before.
 *
 *
 */
#define CREATE_ROBOLAND_NODE(name)                              \
  boost::shared_ptr<roboland::RobolandNode>                     \
  roboland_node(new roboland::RobolandNode(ros::NodeHandle(), name))

/**
 * @brief A macro for add a dependency to another registered node as a
 *        roboland node, this basically just check existence of a
 *        service may be changed in the future.
 *
 */
#define REQUIRED_NODE(name, ns)                                         \
  {                                                                     \
    ros::ServiceClient c =                                              \
      ros::NodeHandle().serviceClient<infrastructure::NodeInfo>(string(ns)+"/"+string(name)+"/ready"); \
    infrastructure::NodeInfo srv;                                       \
    srv.request.str.data = "check_for_ready";                           \
    ROS_INFO_STREAM("Checking for dependency \"" << name << "\" in namespace \"" << ns << "\""); \
    ros::Rate loop(10);                                                 \
    int i = 0;                                                          \
    while (ros::ok() && !c.call(srv) && i < 1000)                       \
    {                                                                   \
      loop.sleep();                                                     \
      ros::spinOnce();                                                  \
      ++i;                                                              \
    }                                                                   \
                                                                        \
    if (ros::ok())                                                      \
    {                                                                   \
      if (i < 1000)                                                     \
      {                                                                 \
        ROS_INFO_STREAM("Dependency \"" << name << "\" is active.");    \
      }                                                                 \
      else                                                              \
      {                                                                 \
        ROS_INFO_STREAM("Dependency \"" << name << "\" could not active."); \
        exit(1);                                                        \
      }                                                                 \
    }                                                                   \
  }

#endif /* ROBOLAND_INFRASTRUCTURE_ROBOLAND_NODE_HH */
