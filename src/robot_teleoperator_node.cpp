#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

geometry_msgs::Twist _teleoperator_msg;
bool _teleoperator_msg_received = false;

void subscriberCallback(const geometry_msgs::Twist& msg) 
{
    ROS_INFO("Got message to subscriber");
    _teleoperator_msg_received = true;
    _teleoperator_msg = msg;
}

/*void lidarCallback(const 
{

}

bool checkLidar() 
{

}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_teleoperator_default"/*, ros::init_options::AnonymousName*/);

    ros::NodeHandle nh;

    //publish twist message to cmd_vel
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Subscriber vel_sub = nh.subscribe("des_vel", 1000, &subscriberCallback);
    //ros::Subscriber lidar_sub = nh.subscribe("...", 1000, &lidarCallback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
    
        if (_teleoperator_msg_received)
        {
            //if (checkLidar(_teleoperator_msg)) 
            //{
                pub.publish(_teleoperator_msg);
                _teleoperator_msg_received = false;
            //}
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
