#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sepanta_msgs/MasterAction.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>

using namespace std;

actionlib::SimpleActionClient<sepanta_msgs::MasterAction> * ac;

std::string target_location;

void thread_logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    //ac->cancelGoal ();

}

void read_file()
{
        std::string path_points =  ros::package::getPath("managment") + "/maps/client.txt";
        cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        text.open(path_points.c_str(), ios_base::in);

        if (text.is_open())
        {
        	
            getline(text,line);

            while (text.good())
            {
                cout<<"Target : "<<line<<endl;
                target_location = line;
                getline(text,line);
            }
            text.close();

        }
        else
        {
            std::cout<<"[Unable to open file]"<< std::endl;
        }

            std::cout<<"Read Done"<<std::endl;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_movebase");

  // create the action client
  // true causes the client to spin its own thread
 
  read_file();

  boost::thread _thread_(&thread_logic);

  ROS_INFO("Waiting for action server to start.");

  ac = new actionlib::SimpleActionClient<sepanta_msgs::MasterAction>("SepantaMoveBaseAction", true);
  // wait for the action server to start
  ac->waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sepanta_msgs::MasterGoal goal;
  goal.action = "exe";
  goal.id = target_location;

  ac->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac->waitForResult();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

    sepanta_msgs::MasterResult::ConstPtr _res = ac->getResult();

     ROS_INFO("Action result : %s",_res->result.c_str());

  }
  else
  {
     ac->cancelGoal ();
     ROS_INFO("Action did not finish before the time out.");
  }
   

   _thread_.interrupt();
   _thread_.join();

  //exit
  return 0;
}
