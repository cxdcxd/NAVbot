#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>
#include <signal.h>
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "sepanta_msgs/arm.h"
#include "sepanta_msgs/omnidata.h"
#include "sepanta_msgs/head.h"
#include "sepanta_msgs/irsensor.h"
#include "sepanta_msgs/led.h"
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <iostream>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;

bool app_exit = false;

int mobileplatform_motors_write[6] = {128, 128, 128, 1, 0, 0};
int mobileplatform_motors_read[10] = {128, 128, 128, 128,128,128,128,128,128,128};

int serial_state = 1;

ros::Publisher pub_feedbacks[20];

int serial_read_hz = 0;
int serial_rw_count = 0;

int w1_current = 128;
int w2_current = 128;
int w3_current = 128;

int w1_desire = 128;
int w2_desire = 128;
int w3_desire = 128;

int feedbackSpeed1 = 128;
int feedbackSpeed2 = 128;
int feedbackPosition = 128;
int feedbackLoad = 128;
int feedbackIsMoving = 128;
int feedbackAlarm = 128;
int feedbackVoltage = 128;
int feedbackSensor = 128;

const string port_name  = "/dev/serial/by-id/usb-ROBOTIS_CO._LTD._ROBOTIS_Virtual_COM_Port-if00";
const int baudrate = 1000000;


void callbackMotor1(const std_msgs::Int32::ConstPtr &msg)
{
    int x = msg->data;

    if ( x < 28 ) x = 28;
    if ( x > 228 ) x = 228;

    w1_desire = x;
}

void callbackMotor2(const std_msgs::Int32::ConstPtr &msg)
{
    int x = msg->data;

    if ( x < 28 ) x = 28;
    if ( x > 228 ) x = 228;

    w2_desire = x;
}

void callbackMotor3(const std_msgs::Int32::ConstPtr &msg)
{
    int x = msg->data;

    if ( x < 5 ) x = 5;
    if ( x > 250 ) x = 250;

    w3_desire = x;
}

void callbackTorque(const std_msgs::Bool::ConstPtr &msg)
{
    if ( msg->data == true )
        mobileplatform_motors_write[3] = 1;
    else
        mobileplatform_motors_write[3] = 0;

}

void callbackLED(const std_msgs::Bool::ConstPtr &msg)
{

    if ( msg->data == true )
        mobileplatform_motors_write[4] = 1;
    else
        mobileplatform_motors_write[4] = 0;
}

void callbackBEEP(const std_msgs::Bool::ConstPtr &msg)
{
    if ( msg->data == true )
        mobileplatform_motors_write[5] = 1;
    else
        mobileplatform_motors_write[5] = 0;

}

void logic()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    ROS_INFO("CAR DCM Started Version : 1.0");
    while (app_exit == false  && ros::ok() )
    {
        serial_rw_count = 0;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        serial_read_hz =  serial_rw_count;
        //ROS_INFO("USB Serial Hz %d",serial_read_hz);
    }
}

void Omnidrive(int w1, int w2, int w3)
{
    if ( w1_current != w1)
    {
        if ( w1 > w1_current)
            w1_current++;
        else
            w1_current--;
    }

    if ( w2_current != w2)
    {
        if ( w2 > w2_current)
            w2_current++;
        else
            w2_current--;
    }

    if ( w3_current != w3)
    {
        if ( w3 > w3_current)
            w3_current++;
        else
            w3_current--;
    }

    if (  w1_current < 28  )  w1_current = 28;
    if (  w1_current > 228 )  w1_current = 228;

    if (  w2_current < 28  )  w2_current = 28;
    if (  w2_current > 228 )  w2_current = 228;

    if ( w3_current < 5   )  w3_current = 5;
    if ( w3_current > 250  )  w3_current = 250;

    mobileplatform_motors_write[0] = w1_current;
    mobileplatform_motors_write[1] = w2_current;
    mobileplatform_motors_write[2] = w3_current;
}

void smooth_drive()
{
    Omnidrive(128,128,128);
    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    while (app_exit == false  && ros::ok() )
    {
        Omnidrive(w1_desire,w2_desire,w3_desire);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}

void serial_logic()
{

    uint8_t result_write[30];
    uint8_t result_read[30];

    while (app_exit == false)
    {
        try
        {
            try
            {
                serial::Serial my_serial(port_name, baudrate, serial::Timeout::simpleTimeout(500));

                //Config Serial
                my_serial.close();
                my_serial.setBaudrate(baudrate);
                serial::parity_t val1 = serial::parity_none;
                my_serial.setParity(val1);
                serial::stopbits_t val2 = serial::stopbits_one;
                my_serial.setStopbits(val2);
                serial::bytesize_t val3 = serial::eightbits;
                my_serial.setBytesize(val3);
                my_serial.open();

                //=======================================================

                if (my_serial.isOpen())
                {
                    ROS_INFO("USB Serial Port OK? : YES");
                }
                else
                {
                    ROS_ERROR("USB Serial Port Not Found");
                    continue;
                }

                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

                while (app_exit == false)
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                    serial_rw_count++;
                    /////////////////////////////////////////////////////
                    //Write Packet

                    //HEADER
                    result_write[0] = 255;
                    result_write[1] = 190;
                    result_write[2] = 255;
                    result_write[3] = 100;
                    result_write[4] = 40;
                    
                    //DATA
                    result_write[5] = mobileplatform_motors_write[0];	//M1
                    result_write[6] = mobileplatform_motors_write[1];	//M2
                    result_write[7] = mobileplatform_motors_write[2];	//DXL position is 512+this vlaue(128=512, 255 = 512+128, 0 = 512-128)
                    //ROS_INFO_STREAM("Position : " << (int)result_write[7] );
                    result_write[8] = mobileplatform_motors_write[3];	//DXL Torque enable = Boolean
                    result_write[9] = mobileplatform_motors_write[4];	//DXL LED = Boolean
                    result_write[10] = mobileplatform_motors_write[5];	//beep times

                    my_serial.write(result_write,11);
                    my_serial.flush();

                    //cout<<"Write OK"<<endl;

                    boost::this_thread::sleep(boost::posix_time::milliseconds(25));

                    uint8_t read;
                    int x = 0;

                    while (app_exit == false && ros::ok() )
                    {
                        my_serial.read(&read, 1);
                        x = read;

                        if ( x == 255 )
                        {
                            my_serial.read(&read, 1);
                            x = read;

                            if ( x == 190 )
                            {
                                my_serial.read(&read, 1);
                                x = read;
                                
                                if ( x == 255)
                                {
                                    //cout<<"READ HEADER OK"<<endl;
                                    my_serial.read(result_read,10);

                                    mobileplatform_motors_read[0] = result_read[0];
                                    mobileplatform_motors_read[1] = result_read[1];
                                    mobileplatform_motors_read[2] = result_read[2];
                                    mobileplatform_motors_read[3] = result_read[3];
                                    mobileplatform_motors_read[4] = result_read[4];
                                    mobileplatform_motors_read[5] = result_read[5];
                                    mobileplatform_motors_read[6] = result_read[6];
                                    mobileplatform_motors_read[7] = result_read[7];
                                    mobileplatform_motors_read[8] = result_read[8];
                                    mobileplatform_motors_read[9] = result_read[9];

                                    //feedbackSpeed1 = mobileplatform_motors_read[0];
                                    feedbackSpeed1 = mobileplatform_motors_write[0];
                                    //feedbackSpeed2 = mobileplatform_motors_read[1];
                                    feedbackSpeed2 = mobileplatform_motors_write[1];

                                    //feedbackPosition = mobileplatform_motors_read[2] << 8 + mobileplatform_motors_read[3];
                                    feedbackPosition = mobileplatform_motors_write[2];
                                    feedbackLoad = mobileplatform_motors_read[4] << 8 + mobileplatform_motors_read[5];
                                    feedbackIsMoving =  mobileplatform_motors_read[6];
                                    feedbackAlarm = mobileplatform_motors_read[7];
                                    feedbackVoltage = mobileplatform_motors_read[8];
                                    feedbackSensor = mobileplatform_motors_read[9];

                                    break;
                                }
                            }
                        }

                        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                    }
                }

            }//try
            catch (serial::SerialException e)
            {
                ROS_ERROR("USB Serial Read Error");
            }

        }//try
        catch (serial::IOException e)
        {
            ROS_ERROR("USB Serial Port Error");
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }//while
}

void update()
{
    std_msgs::Int32 msg1;
    msg1.data = feedbackSpeed1;

    std_msgs::Int32 msg2;
    msg2.data = feedbackSpeed2;

    std_msgs::Int32 msg3;
    msg3.data = feedbackPosition;

    std_msgs::Int32 msg4;
    msg4.data = feedbackLoad;

    std_msgs::Int32 msg5;
    msg5.data = feedbackIsMoving;

    std_msgs::Int32 msg6;
    msg6.data = feedbackAlarm;

    std_msgs::Int32 msg7;
    msg7.data = feedbackVoltage;

    std_msgs::Int32 msg8;
    msg8.data = feedbackSensor;

    pub_feedbacks[0].publish(msg1);
    pub_feedbacks[1].publish(msg2);
    pub_feedbacks[2].publish(msg3);
    pub_feedbacks[3].publish(msg4);
    pub_feedbacks[4].publish(msg5);
    pub_feedbacks[5].publish(msg6);
    pub_feedbacks[6].publish(msg7);
    pub_feedbacks[7].publish(msg8);
}

int main(int argc, char **argv)
{
    boost::thread thread_logic(&logic);
    boost::thread thread_serial(&serial_logic);
    boost::thread thread_drive(&smooth_drive);

    ros::init(argc, argv, "lowerbodycore");

    ros::NodeHandle node_handle;
    ros::Subscriber sub_handles[15];

    sub_handles[1] = node_handle.subscribe("car/motor1", 1, callbackMotor1);
    sub_handles[2] = node_handle.subscribe("car/motor2", 1, callbackMotor2);
    sub_handles[3] = node_handle.subscribe("car/motor3", 1, callbackMotor3);
    sub_handles[4] = node_handle.subscribe("car/torque", 1, callbackTorque);
    sub_handles[5] = node_handle.subscribe("car/led", 1, callbackLED);
    sub_handles[6] = node_handle.subscribe("car/beep", 1, callbackBEEP);

    pub_feedbacks[0] = node_handle.advertise<std_msgs::Int32>("car/feedbackSpeed1",1);
    pub_feedbacks[1] = node_handle.advertise<std_msgs::Int32>("car/feedbackSpeed2",1);
    pub_feedbacks[2] = node_handle.advertise<std_msgs::Int32>("car/feedbackPosition",1);
    pub_feedbacks[3] = node_handle.advertise<std_msgs::Int32>("car/feedbackLoad",1);
    pub_feedbacks[4] = node_handle.advertise<std_msgs::Int32>("car/feedbackIsMoving",1);
    pub_feedbacks[5] = node_handle.advertise<std_msgs::Int32>("car/feedbackAlarm",1);
    pub_feedbacks[6] = node_handle.advertise<std_msgs::Int32>("car/feedbackVoltage",1);
    pub_feedbacks[7] = node_handle.advertise<std_msgs::Int32>("car/feedbackSensor",1);

    ros::Rate loop_rate(20); //20 Hz

    while (ros::ok() && app_exit == false)
    {
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    app_exit = true;

    thread_drive.interrupt();
    thread_drive.join();

    thread_logic.interrupt();
    thread_logic.join();

    thread_serial.interrupt();
    thread_serial.join();

    return 0;
}
