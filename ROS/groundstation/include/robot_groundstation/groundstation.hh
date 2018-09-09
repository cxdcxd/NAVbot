//2016

#ifndef CAR_GROUNDSTATION_HH_
#define CAR_GROUNDSTATION_HH_

//STD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <functional>

//BOOST
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <robot_groundstation/video_stream.hh>
#include <robot_groundstation/network.hh>

#include <Main.pb.h>
#include <functional>

using namespace std;

namespace roboland
{

class Groundstation
{

public:
  Groundstation (string groundstation_remote_ip, int argc, char *argv[]);
  ~Groundstation ();

  Network *net_interface;

  bool is_connected;
  int time_out;
  string robot_id;
  bool app_exit;
  boost::thread thread_main;

  void kill();
  void receiveCallback(char * data,int size);
  void send(CarRobot msg);
  void thrMain();
  void wait(int ms);
  void sendAlive();

  std::function<void (CarCommand cmd)> callBackCMD;
};

} 
#endif

