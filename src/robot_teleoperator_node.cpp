#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

bool _teleoperator_twist_msg_received;
bool _teleoperator_lidar_msg_received;
geometry_msgs::Twist _teleoperator_twist_msg;

void subscriberCallback(const geometry_msgs::Twist& msg) 
{
    float l1 = msg.linear.x;
    float l2 = msg.linear.y;
    float l3 = msg.linear.z;

    float a1 = msg.angular.x;
    float a2 = msg.angular.y;
    float a3 = msg.angular.z;

    ROS_INFO("Got message from velocity %f %f %f %f %f %f", l1, l2, l3, a1, a2, a3);

    _teleoperator_twist_msg_received = true;
    _teleoperator_twist_msg = msg;
}

void lidarCallback(const sensor_msgs::LaserScan& msg)
{
    //store the max/min values as opposites so can correctly identify the current
    //minimum and maximum range as reported by the scan
    double min = msg.range_max;
    double max = msg.range_min;
    double min_index;
    double max_index;
    int value_range = (int)(abs(msg.angle_max - msg.angle_min)/msg.angle_increment);
    int iterator;
    ROS_DEBUG("Angle min: %f, Angle max %f, Angle incr.: %f", msg.angle_min, msg.angle_max, msg.angle_increment);
    ROS_DEBUG("Calculated %i indexes", value_range);
    for (iterator = 0; iterator <= value_range; iterator++)
    {
        if (msg.ranges[iterator] >= msg.range_min && msg.ranges[iterator] < min)
        {
            min = msg.ranges[iterator];
            min_index = iterator;
        }
        if (msg.ranges[iterator] <= msg.range_max && msg.ranges[iterator] > max)
        {
            max = msg.ranges[iterator];
            max_index = iterator;
        }
    }

    double min_degree = msg.angle_min + min_index*msg.angle_increment;

    ROS_DEBUG("Scan had %i values from %f to %f", value_range, msg.range_min, msg.range_max);
    ROS_DEBUG("Got message from lidar: min: %f at %f degrees, max: %f", min, min_degree, max);
    bool _teleoperator_lidar_msg_received = true;
}

int main(int argc, char **argv)
{
    int looping_rate = 10;
    ros::init(argc, argv, "robot_teleoperator_default"/*, ros::init_options::AnonymousName*/);

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);

    ros::Subscriber vel_sub = nh.subscribe("des_vel", 1000, &subscriberCallback);
    ros::Subscriber lidar_sub = nh.subscribe("robot0/laser_1", 1000, &lidarCallback);

    ros::Rate loop_rate(looping_rate);

    int twist_msg_not_received_count = 0;
    int lidar_msg_not_received_count = 0;
    while(ros::ok())
    {
        if(_teleoperator_lidar_msg_received)
        {
            //error check lidar scan vs velocity
            _teleoperator_twist_msg.linear.x = 0.0;
            _teleoperator_twist_msg.linear.y = 0.0;
            _teleoperator_twist_msg.linear.z = 0.0;

            _teleoperator_twist_msg.angular.x = 0.0;
            _teleoperator_twist_msg.angular.y = 0.0;
            _teleoperator_twist_msg.angular.z = 1.0;

            ROS_INFO("Publishing message %f %f %f %f %f %f", _teleoperator_twist_msg.linear.x, _teleoperator_twist_msg.linear.y, _teleoperator_twist_msg.linear.z, _teleoperator_twist_msg.angular.x, _teleoperator_twist_msg.angular.y, _teleoperator_twist_msg.angular.z);
            pub.publish(_teleoperator_twist_msg);
        }
	//error checking for missing twist messages
        if (_teleoperator_twist_msg_received)
        {
            _teleoperator_twist_msg_received = false;
            twist_msg_not_received_count = 0;
        }
        else
        {
            twist_msg_not_received_count++;
        }
        //print error every 1 minutes
        if (twist_msg_not_received_count > 60)
        {
            ROS_DEBUG_THROTTLE(60, "Haven't heard from publisher in a minute: %i cycles", twist_msg_not_received_count);
        }

	//error checking for missing lidar messages
	if (_teleoperator_lidar_msg_received)
        {
            _teleoperator_lidar_msg_received = false;
            lidar_msg_not_received_count = 0;
        }
        else
        {
            lidar_msg_not_received_count++;
        }
        //print error every 1 minutes
        if (lidar_msg_not_received_count > 60)
        {
            ROS_DEBUG_THROTTLE(60, "Haven't heard from lidar in a minute: %i cycles", lidar_msg_not_received_count);
        }

        ros::spinOnce();
    }
}
