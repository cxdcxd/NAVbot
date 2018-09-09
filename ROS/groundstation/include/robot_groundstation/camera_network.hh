#ifndef HEX_NETWORK_HH_
#define HEX_NETWORK_HH_

//TCPIP
#include <tcp/tcpacceptor.h>
#include <tcp/tcpstream.h>
#include <tcp/tcpconnector.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>

//ros_msgs
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Vector3.h>

//Boost
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

//STD
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

class CameraNetwork
{

public:
  CameraNetwork(int port,string name);
  ~CameraNetwork();
  
  /**
   * @brief Network::wait Call boost thread wait
   * @param ms The time to wait (ms)
   */

  void wait(int ms);

  /**
   * @brief Network::thrTCPRead The fucntion to handle tcp blockable read (Main read loop)
   */

  void thrTCPRead();

  /**
   * @brief Network::tcpWrite The main network write to first connected socket
   * @param buffer The pointer of write buffer
   * @param size The size of data to send in bytes
   */

  void tcpWrite(char * buffer,int size);

  /**
   * @brief Network::kill Destroy all obkects related to this class
   */

  void kill();

  bool is_connected;
  bool app_exit;
  int tcp_port; 
  string tcp_name;

  boost::thread thread;

  TCPAcceptor* acceptor_tcp;
  TCPStream* stream_tcp;
};

#endif
