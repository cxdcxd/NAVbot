
#include "robot_groundstation/camera_network.hh"

CameraNetwork::CameraNetwork(int port,string name) :

  is_connected(false),
  app_exit(false),
  tcp_port(port),
  tcp_name(name),
  thread(&CameraNetwork::thrTCPRead,this)
{

}

CameraNetwork::~CameraNetwork()
{

}

void CameraNetwork::kill()
{
  app_exit = true;
  this->thread.interrupt();
  this->thread.join();
  this->thread.~thread();

  this->acceptor_tcp->~TCPAcceptor();

  delete this->acceptor_tcp;
  delete this->stream_tcp;
}

void CameraNetwork::wait(int ms)
{
  //wait
  boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void CameraNetwork::thrTCPRead()
{
  ROS_INFO_STREAM("TCP server : " << this->tcp_name << " Starting...");
  geometry_msgs::Vector3 _msg;

  stream_tcp = NULL;
  acceptor_tcp = NULL;
  acceptor_tcp = new TCPAcceptor(this->tcp_port);

  ssize_t len;
  char line[4096];
  char vline[4096];

  int read = 0;
  int header = 0;
  int length = 0;

  if (acceptor_tcp->start() == 0)
  {
    while (!app_exit)
    {
      ROS_INFO_STREAM("Wait for client on port " << tcp_port);
      stream_tcp = acceptor_tcp->accept();
      if ( this->tcp_name == "unity")
        ROS_INFO_STREAM("Unity Connected");
      if ( this->tcp_name == "android")
        ROS_INFO_STREAM("Android Connected");

      is_connected = true;

      if (stream_tcp != NULL)
      {
        while ((len = stream_tcp->receive(line, sizeof(line))) > 0 && !app_exit)
        {
          for ( int i = 0 ; i < len ; i++)
          {
            if ( (uint8_t)line[i] == 204  &&  header == 0)  { header++; continue;} else if ( header == 0 ) header = 0;
            if ( (uint8_t)line[i] == 73   &&  header == 1)  { header++; continue;} else if ( header == 1 ) header = 0;
            if ( (uint8_t)line[i] == 90   &&  header == 2)  { header++; continue;} else if ( header == 2 ) header = 0;
            if ( (uint8_t)line[i] == 252  &&  header == 3)  { header++; continue;} else if ( header == 3 ) header = 0;
            if ( (uint8_t)line[i] == 22   &&  header == 4)  { header++; continue;} else if ( header == 4 ) header = 0;
            if ( header == 5) {length = (uint8_t)line[i] * 256 ;  header++; continue;}
            if ( header == 6) {length += (uint8_t)line[i]      ;  header++; continue;}
            if ( header == 7 ) { header++;}
            if ( header == 8 && read != length)
            {
              vline[read] = line[i];
              read++;
            }
            else if ( header == 8 && read == length)
            {

             if ( this->tcp_name == "android")
             {
               //statics->tcpAndroidReadEvent(vline,read);
             }
               
              read = 0;
              length = 0;
              header = 0;
            }
          }
        }

        delete stream_tcp;
        ROS_ERROR_STREAM("TCP server : " << this->tcp_name << " Disconnected.");
        is_connected = false;
      }
    }
  }
}

void CameraNetwork::tcpWrite(char * buffer,int size)
{
  if ( is_connected )
  {
    char message[size + 7];

    message[0] = 204;
    message[1] = 73;
    message[2] = 90;
    message[3] = 252;
    message[4] = 22;
    message[5] = size / 256;
    message[6] = size % 256;

    for ( int i = 0 ; i < size ; i++)
    {
      message[7 + i] = buffer[i];
    }

    stream_tcp->send(message, size + 7);
  }

}






