#ifndef ROBOLAND_NETWORK_HH_
#define ROBOLAND_NETWORK_HH_

//STD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <functional>

//Roboland
#include <Main.pb.h>
#include "zmq.hpp"

//BOOST
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace roboland
{

/**
 * @brief The Network class
 * This class is responsible for create a safe and reusable functionality to interface with other applications
 * such as Unity3d engine or Xamarin Mono via TCP/IP protocol over ZMQ library.
 *
 * It served the minimum functions to read and write data from ZMQ sockets.
 */
class Network
{

public:
  Network(std::string port, std::string subsribe_port, std::string subscribe_ip, std::string name);
  ~Network();

  /**
   * @brief Network::wait Call boost thread wait
   * @param ms The time to wait (ms)
   */
  void wait(int ms);

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

  /**
   * @brief Main network callback for receive envets
   * @param data Data bytes
   * @param size Data bytes size
   */
  void receiveCallback(char * data,int size);


  bool app_exit;
  std::string tcp_port;
  int index;
  std::string tcp_name;
  std::string subscribe_tcp_ip;
  std::string subscribe_tcp_port;

  boost::thread thread_ZMQ;

  void thrZmqSubscriber();
  std::string connectionp;
  std::string connections;

  zmq::socket_t *publisher;
  zmq::socket_t *subscriber;
  zmq::context_t *zmqcontext;

  std::function<void (char * data,int size)> callBackFunction;
};

} //roboland namespace
#endif
