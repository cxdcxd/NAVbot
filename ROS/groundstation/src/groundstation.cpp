
#include "robot_groundstation/groundstation.hh"

using namespace std;

namespace roboland
{

Groundstation::Groundstation(std::string groundstation_remote_ip,int argc, char *argv[]) :
    app_exit(false),
    thread_main(&Groundstation::thrMain,this)
{
    is_connected = false;
    time_out = 5;

    net_interface = new Network("7888","7889",groundstation_remote_ip,"groundstation");
    net_interface->callBackFunction = std::bind(&Groundstation::receiveCallback, this, std::placeholders::_1, std::placeholders::_2);
}

Groundstation::~Groundstation()
{
   kill();
}

void Groundstation::receiveCallback(char * data,int size)
{
   bool show_info = true;

   CarCommand msg;
   msg.ParseFromArray(data,size);


    time_out = 5;
    is_connected = true;

    if ( callBackCMD )
         callBackCMD(msg);


}

void Groundstation::thrMain()
{
   wait(1000);
   while ( app_exit == false )
   {
       sendAlive();
       wait(1000);

       //if ( is_connected == false )
       //std::cout << "Is Connected : " << is_connected << std::endl;
   }
}

void Groundstation::sendAlive()
{
   CarRobot msg;
   msg.set_version(100);

   send(msg);

   time_out--;
   if ( time_out <= 0)
   {
       time_out = 5;
       is_connected = false;
   }
}

void Groundstation::wait(int ms)
{
    //wait
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void Groundstation::send(CarRobot msg)
{
   int size = msg.ByteSize();
   char buffer[size];
   msg.SerializeToArray(buffer,size);
   net_interface->tcpWrite(buffer,size);
}

void Groundstation::kill()
{
   delete net_interface;
   this->thread_main.interrupt();
   this->thread_main.~thread();
}


} 
