//Roboland International Inc. 2016
#include "robot_groundstation/network.hh"

using namespace std;

namespace roboland
{

Network::Network(std::string port, std::string subsribe_port, std::string subscribe_ip, std::string name) :
    app_exit(false),
    tcp_port(port),
    tcp_name(name),
    subscribe_tcp_ip(subscribe_ip),
    subscribe_tcp_port(subsribe_port),
    thread_ZMQ(&Network::thrZmqSubscriber,this)
{
    connectionp = "tcp://*:" + tcp_port;
    std::cout << "ZMQ Publisher Started " << connectionp << " for " << std::endl;

    zmqcontext = new zmq::context_t(1);
    publisher  = new zmq::socket_t((*zmqcontext), ZMQ_PUB);
    publisher->bind(connectionp);

    connections = "tcp://" + subscribe_tcp_ip + ":" + subscribe_tcp_port;
    std::cout << "ZMQ Subscriber Started " << connections << " for " << tcp_name << std::endl;

    subscriber = new zmq::socket_t((*zmqcontext), ZMQ_SUB);
    subscriber->connect(connections);

    const char *filter = "";
    subscriber->setsockopt(ZMQ_SUBSCRIBE, filter, strlen (filter));
}

Network::~Network()
{
    kill();
}

void Network::receiveCallback(char * data,int size)
{

}

void Network::kill()
{
    app_exit = true;
    this->thread_ZMQ.interrupt();
    this->thread_ZMQ.~thread();

    publisher->close();
    std::cout<<"close publisher"<<std::endl;

    subscriber->disconnect(connections);
    std::cout<<"close subscriber"<<std::endl;

    std::cout<<"kill done"<<std::endl;
}

void Network::wait(int ms)
{
    //wait
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void Network::thrZmqSubscriber()
{
    wait(1000);
    while ( !app_exit )
    {
        zmq::message_t update;
        subscriber->recv(&update);
        char* get = static_cast<char*>(update.data());
        if ( callBackFunction )
        callBackFunction(get,(int)update.size());
        receiveCallback(get,(int)update.size());
    }
}

void Network::tcpWrite(char * buffer,int size)
{
  publisher->send(buffer, size);
}

}





