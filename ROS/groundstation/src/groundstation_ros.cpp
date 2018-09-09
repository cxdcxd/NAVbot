#include "robot_groundstation/groundstation_ros.hh"

namespace roboland 
{

int counter = 0;
int plan_counter = 0;

GroundstationRos::GroundstationRos(ros::NodeHandle &nh,
                                   ros::NodeHandle &pnh,
                                   int argc,
                                   char *argv[],
                                   std::string id) :
    thread_main(&GroundstationRos::thrMain,this),
    p_tcp_groundstation_remote_ip("/groundstation/remote_ip","192.168.1.2")
{

    ground_station = new Groundstation(p_tcp_groundstation_remote_ip(),argc,argv);
    ground_station->callBackCMD = std::bind(&GroundstationRos::callbackGroundstation, this, std::placeholders::_1);

   
    ref_nh = nh;
    mutex = false;
    app_exit = false;
    counter = 0;
    send_counter = 0;
    
    pub_motor1 = nh.advertise<std_msgs::Int32>("car/motor1", 1);
    pub_motor2 = nh.advertise<std_msgs::Int32>("car/motor2", 1);
    pub_motor3 = nh.advertise<std_msgs::Int32>("car/motor3", 1);
    pub_torque = nh.advertise<std_msgs::Bool>("car/torque", 1);
    pub_led = nh.advertise<std_msgs::Bool>("car/led", 1);
    pub_beep = nh.advertise<std_msgs::Bool>("car/beep", 1);

    sub_speed1 = nh.subscribe<std_msgs::Int32>("car/feedbackSpeed1",1,&GroundstationRos::callbackSpeed1,this);
    sub_speed2 = nh.subscribe<std_msgs::Int32>("car/feedbackSpeed2",1,&GroundstationRos::callbackSpeed2,this);
    sub_position = nh.subscribe<std_msgs::Int32>("car/feedbackPosition",1,&GroundstationRos::callbackPosition,this);
    sub_load =  nh.subscribe<std_msgs::Int32>("car/feedbackLoad",1,&GroundstationRos::callbackLoad,this);
    sub_ismoveing =  nh.subscribe<std_msgs::Int32>("car/feedbackIsMoving",1,&GroundstationRos::callbackIsMoveing,this);
    sub_alarm = nh.subscribe<std_msgs::Int32>("car/feedbackAlarm",1,&GroundstationRos::callbackAlarm,this);
    sub_voltage = nh.subscribe<std_msgs::Int32>("car/feedbackVoltage",1,&GroundstationRos::callbackVoltage,this);
    sub_sensor = nh.subscribe<std_msgs::Int32>("car/feedbackSensor",1,&GroundstationRos::callbackSensor,this);

    sub_laser = nh.subscribe("rawscan", 1, &GroundstationRos::callbackLaser,this);
    pub_laser = nh.advertise<sensor_msgs::LaserScan>("scan",1);

    sub_slam_pose = nh.subscribe("slam_out_pose", 1, &GroundstationRos::GetPos,this);
    sub_map = nh.subscribe("/map", 1, &GroundstationRos::GetCostmap,this);
    client_makeplan = nh.serviceClient<nav_msgs::GetPlanRequest>("move_base/make_plan");
    client_resetcostmap = nh.serviceClient<std_srvs::EmptyRequest>("move_base/clear_costmaps");
    pub_slam_origin = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    pub_slam_reset = nh.advertise<std_msgs::String>("syscommand", 1);

    pub_marker =   nh.advertise<visualization_msgs::Marker>("visualization_marker_steps", 1);
    pub_marker2 =  nh.advertise<visualization_msgs::Marker>("visualization_marker_goals", 1);
    pub_marker3 =  nh.advertise<visualization_msgs::Marker>("visualization_marker_goals_arrow", 1);
}

void GroundstationRos::updateViz()
{
    visualization_msgs::Marker points , points2 , points3;

    points.header.frame_id =  "map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points2.header.frame_id =  "map";
    points2.header.stamp = ros::Time::now();
    points2.action = visualization_msgs::Marker::ADD;
    points2.id = 1;
    points2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    points2.scale.x = 1;
    points2.scale.y = 1;
    points2.scale.z = 0.4;
    points2.color.b = 0.5;
    points2.color.r = 1;
    points2.color.a = 1;

    points3.header.frame_id =  "map";
    points3.header.stamp = ros::Time::now();
    points3.action = visualization_msgs::Marker::ADD;
    points3.id = 2;
    points3.type = visualization_msgs::Marker::ARROW;
    points3.scale.x = 0.5;
    points3.scale.y = 0.05;
    points3.scale.z = 0.05;
    points3.color.b = 0.5;
    points3.color.r = 1;
    points3.color.a = 1;


     // Create the vertices for the points and lines
    for (int i = 0; i < current_path.poses.size(); i += 30)
    {

      geometry_msgs::Point p;
      p.x = current_path.poses[i].pose.position.x;
      p.y = current_path.poses[i].pose.position.y;
      p.z = 0;

      points.points.push_back(p);

    }

     points2.pose.position.x =  current_goal_position.x;
     points2.pose.position.y =  current_goal_position.y;
     points2.pose.position.z = 0;
     points2.text = "Goal";
     points2.ns = "Goal";
     pub_marker2.publish(points2);

     points3.pose.position.x =  current_goal_position.x;
     points3.pose.position.y =  current_goal_position.y;
     points3.pose.position.z = 0;
     points3.pose.orientation = tf::createQuaternionMsgFromYaw(current_goal_position.theta);
     points3.ns = "Arrow";

     //pub_marker3.publish(points3);
     pub_marker.publish(points);
}

bool get_new = false;
void GroundstationRos::callbackGroundstation(CarCommand cmd)
{
    ROS_INFO_STREAM("Groundstation get data");
    get_new = true;
    counter = 0;

    bool beep = cmd.beep();
    std_msgs::Bool msg;
    msg.data = beep;
    pub_beep.publish(msg);

    bool led = cmd.led();
    msg;
    msg.data = led;
    pub_led.publish(msg);

    int speed1 = cmd.speed1();
    std_msgs::Int32 msg2;
    msg2.data = speed1;
    pub_motor1.publish(msg2);

    int speed2 = cmd.speed2();
    msg2.data = speed2;
    pub_motor2.publish(msg2);

    int position = cmd.position();
    msg2.data = position;
    pub_motor3.publish(msg2);

    if ( cmd.has_target_location() )
    {
        RVector3 target_location = cmd.target_location();

        current_goal_position.x = target_location.x();
        current_goal_position.y = target_location.y();
        current_goal_position.theta = target_location.theta();

	   
          if ( plan_counter == 0 )
          {
               plan_counter = 10;
               current_path = makePlan();
          }

      
    }

    if ( cmd.has_temp_target_location() )
    {
        RVector3 temp_target_location = cmd.temp_target_location();

        current_temp_gaol_position.x = temp_target_location.x();
        current_temp_gaol_position.y = temp_target_location.y();
        current_temp_gaol_position.theta = temp_target_location.theta();
    }
}

void GroundstationRos::callbackSpeed1(const std_msgs::Int32::ConstPtr &msg)
{
    current_speed1 = msg->data;
}

void GroundstationRos::callbackSpeed2(const std_msgs::Int32::ConstPtr &msg)
{
    current_speed2 = msg->data;
}

void GroundstationRos::callbackPosition(const std_msgs::Int32::ConstPtr &msg)
{
    current_position = msg->data;
}

void GroundstationRos::callbackLoad(const std_msgs::Int32::ConstPtr &msg)
{
    current_load = msg->data;
}

void GroundstationRos::callbackIsMoveing(const std_msgs::Int32::ConstPtr &msg)
{
    current_ismoveing = msg->data;
}

void GroundstationRos::callbackVoltage(const std_msgs::Int32::ConstPtr &msg)
{
    current_voltage = msg->data;
}

void GroundstationRos::callbackSensor(const std_msgs::Int32::ConstPtr &msg)
{
    current_sensor = msg->data;
}

void GroundstationRos::callbackAlarm(const std_msgs::Int32::ConstPtr &msg)
{
    current_alaram = msg->data;
}

void GroundstationRos::thrMain()
{
    CarRobot status;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    while(ros::ok() && app_exit == false)
    {
        counter++;
        if ( counter > 20 )
        {
          counter = 0;
         
          
	    std_msgs::Int32 msg2;
	    msg2.data = 128;
	    pub_motor1.publish(msg2);
	    
	    msg2.data = 128;
	    pub_motor2.publish(msg2);
	    
	    msg2.data = 128;
	    pub_motor3.publish(msg2);
          
        }

        if ( plan_counter > 0 ) plan_counter--;



        RVector3 *location = new RVector3();
        RVector3 *target_location = new RVector3();
        RVector3 *temp_target_location = new RVector3();
        Laser *laser_data = new Laser();


        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        //ROS_INFO_STREAM("Groundstation loop");

        if ( current_alaram == 1)
            status.set_alarm(true);
        else
            status.set_alarm(false);

        if ( current_load == 1)
            status.set_load(true);
        else
            status.set_load(false);

        status.set_battery(current_voltage);
        status.set_sensor(current_sensor);
        status.set_position(current_position);
        status.set_speed1(current_speed1);
        status.set_speed2(current_speed2);

        //Add path
        status.clear_path();
        for ( int i = 0 ; i < current_path.poses.size() ; i += 30)
        {
            status.add_path();
        }

        int k = 0;

        for ( int i = 0 ; i < current_path.poses.size() ; i += 30)
        {
            geometry_msgs::Pose pose = current_path.poses.at(i).pose;
            geometry_msgs::Pose2D pose_2d;

            pose_2d.x = pose.position.x;
            pose_2d.y = pose.position.y;

            double orientation[4];

            orientation[0] = pose.orientation.x;
            orientation[1] = pose.orientation.y;
            orientation[2] = pose.orientation.z;
            orientation[3] = pose.orientation.w;

            pose_2d.theta = Quat2Rad(orientation);

            status.mutable_path(k)->set_x(pose_2d.x);
            status.mutable_path(k)->set_y(pose_2d.y);
            status.mutable_path(k)->set_theta(pose_2d.theta);

            k++;
        }


        //if ( current_map.data.size() != 0 )
        //{
        //    status.set_map_size(current_map.data.size());
        //    status.set_map_data(&current_map.data[0],current_map.data.size());

        //}

        //Add temp path
        status.clear_temp_path();
        for ( int i = 0 ; i < current_temp_path.poses.size() ; i++)
        {
            status.add_temp_path();
        }

        for ( int i = 0 ; i < current_temp_path.poses.size() ; i++)
        {
            geometry_msgs::Pose pose = current_temp_path.poses.at(i).pose;
            geometry_msgs::Pose2D pose_2d;

            pose_2d.x = pose.position.x;
            pose_2d.y = pose.position.y;

            double orientation[4];

            orientation[0] = pose.orientation.x;
            orientation[1] = pose.orientation.y;
            orientation[2] = pose.orientation.z;
            orientation[3] = pose.orientation.w;

            pose_2d.theta = Quat2Rad(orientation);

            status.mutable_temp_path(i)->set_x(pose_2d.x);
            status.mutable_temp_path(i)->set_y(pose_2d.y);
            status.mutable_temp_path(i)->set_theta(pose_2d.theta);
        }

        location->set_x(current_slam_position.x);
        location->set_y(current_slam_position.y);
        location->set_theta(current_slam_position.theta);

        target_location->set_x(current_goal_position.x);
        target_location->set_y(current_goal_position.y);
        target_location->set_theta(current_goal_position.theta);

        temp_target_location->set_x(current_temp_gaol_position.x);
        temp_target_location->set_y(current_temp_gaol_position.y);
        temp_target_location->set_theta(current_temp_gaol_position.theta);

        laser_data->set_robot_id(0);
        laser_data->clear_ranges();

        //for ( int i = 0 ; i < current_laser.ranges.size() ; i++)
        //{
          //  laser_data->add_ranges(current_laser.ranges.at(i));
        //}

        status.set_allocated_location(location);
        status.set_allocated_target_location(target_location);
        status.set_allocated_temp_target_location(temp_target_location);
        status.set_allocated_laser(laser_data);

        ground_station->send(status);
        //ROS_INFO_STREAM("Send Done");

        updateViz();


        location = NULL;
        target_location = NULL;
        temp_target_location = NULL;
        laser_data = NULL;

        delete location;
        delete target_location;
        delete temp_target_location;
        delete laser_data;
    }


}

double GroundstationRos::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void GroundstationRos::updateHectorOrigin(geometry_msgs::Pose2D p)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
    msg.pose.pose.position.x = p.x;
    msg.pose.pose.position.y = p.y;
    pub_slam_origin.publish(msg);
}

void GroundstationRos::GetCostmap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    //ROS_INFO_STREAM("Map data size " << msg->data.size());
    //current_map = *msg;
}

void GroundstationRos::callbackLaser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    current_laser = *msg;

    for ( int i = 0; i < current_laser.ranges.size() ; i++)
    {
        if ( i > 135 && i < 225 )
       current_laser.ranges[i] = 0;
    }

    pub_laser.publish(current_laser);
}

void GroundstationRos::GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double orientation[4];
    current_slam_position.x = msg->pose.position.x;
    current_slam_position.y = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    current_slam_position.theta = Quat2Rad(orientation);
}

nav_msgs::Path GroundstationRos::makePlan()
{
    
    ROS_INFO("Make Plan");

    nav_msgs::GetPlan srv;

    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = current_slam_position.x;
    srv.request.start.pose.position.y = current_slam_position.y;
    srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(current_slam_position.theta);

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = current_goal_position.x;
    srv.request.goal.pose.position.y = current_goal_position.y;
    srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(current_goal_position.theta);

    srv.request.tolerance = 0.1;
    client_makeplan.call(srv);

    ROS_INFO_STREAM("Make Plan result : " << srv.response.plan.poses.size());
    return srv.response.plan;
}

void GroundstationRos::resetHectorSlam()
{
    std_msgs::String msg;
    msg.data = "reset";
    pub_slam_reset.publish(msg);
}

GroundstationRos::~GroundstationRos()
{
    app_exit = true;
    thread_main.interrupt();
    thread_main.join();
}

} 
