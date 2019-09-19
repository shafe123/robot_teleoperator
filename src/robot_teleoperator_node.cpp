#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

bool g_teleoperator_twist_msg_received;
bool g_teleoperator_lidar_msg_received;
bool g_stop_linear_motion;
geometry_msgs::Twist g_teleoperator_twist_msg;

void subscriberCallback(const geometry_msgs::Twist& msg) 
{
    float l1 = msg.linear.x;
    float l2 = msg.linear.y;
    float l3 = msg.linear.z;

    float a1 = msg.angular.x;
    float a2 = msg.angular.y;
    float a3 = msg.angular.z;

    ROS_DEBUG("Got message from velocity %f %f %f %f %f %f", l1, l2, l3, a1, a2, a3);

    g_teleoperator_twist_msg_received = true;
    g_teleoperator_twist_msg = msg;
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
        // check if lidar measurement will run us into wall
	    //if y component is less than 1.0 and x component is less than +2.0
	    if ( abs(msg.ranges[iterator]*sin(msg.angle_min+msg.angle_increment*iterator)) < 1.0 
            && msg.ranges[iterator]*cos(msg.angle_min+msg.angle_increment*iterator) < 2)
        {
            ROS_INFO_THROTTLE(1, "Wall too close, stopping linear motion.");
            g_stop_linear_motion = true;
        }
    }
    g_teleoperator_lidar_msg_received = true;
}

int main(int argc, char **argv)
{
    bool critical_fail = false;
    const int LOOPING_RATE = 10;
    const std::string TOPIC_NAME = argv[1];
    const int FAIL_TIME = std::stoi(argv[2]);
    const std::string NAMESPACE = argv[3];
    const int ERROR_NOTIFICATION_TIME = 5;

    geometry_msgs::Vector3 ZERO_VECTOR;
    ZERO_VECTOR.x = 0.0;
    ZERO_VECTOR.y = 0.0;
    ZERO_VECTOR.z = 0.0;

    geometry_msgs::Twist STOP_TWIST;
    STOP_TWIST.linear = ZERO_VECTOR;
    STOP_TWIST.angular = ZERO_VECTOR;

    geometry_msgs::Twist no_forward_twist;
    no_forward_twist.linear = ZERO_VECTOR;

    ros::init(argc, argv, "robot_teleoperator_default");
    ros::NodeHandle robot_namespace(NAMESPACE);
    ros::NodeHandle global_namespace;
    ros::Publisher pub = robot_namespace.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber lidar_sub = robot_namespace.subscribe("laser_1", 1000, &lidarCallback);
    ros::Subscriber vel_sub = global_namespace.subscribe(TOPIC_NAME, 1000, &subscriberCallback);
    ros::Rate loop_rate(LOOPING_RATE);
    
    ROS_INFO("Publishing to %s, subscribing to %s and %s", pub.getTopic().c_str(), vel_sub.getTopic().c_str(), lidar_sub.getTopic().c_str());

    int twist_msg_not_received_count = 0;
    int lidar_msg_not_received_count = 0;
    while(ros::ok())
    {
        //message handling
        if(g_teleoperator_lidar_msg_received)
        {
            g_teleoperator_lidar_msg_received = false;
            lidar_msg_not_received_count = 0;
            critical_fail = true;
        }
        else
        {
            pub.publish(STOP_TWIST);
            lidar_msg_not_received_count++;
        }
        if(g_teleoperator_twist_msg_received && !critical_fail)
        {
            if(!g_stop_linear_motion)
            {
                ROS_INFO("Publishing velocity message %f %f %f %f %f %f",
                         g_teleoperator_twist_msg.linear.x, g_teleoperator_twist_msg.linear.y,
                         g_teleoperator_twist_msg.linear.z, g_teleoperator_twist_msg.angular.x,
                         g_teleoperator_twist_msg.angular.y, g_teleoperator_twist_msg.angular.z);
                pub.publish(g_teleoperator_twist_msg);
            }
            else
            {
                no_forward_twist.angular = g_teleoperator_twist_msg.angular;
                ROS_INFO("Publishing velocity message %f %f %f %f %f %f",
                         no_forward_twist.linear.x, no_forward_twist.linear.y,
                         no_forward_twist.linear.z, no_forward_twist.angular.x,
                         no_forward_twist.angular.y, no_forward_twist.angular.z);
                pub.publish(no_forward_twist);
            }
            g_teleoperator_twist_msg_received = false;
            twist_msg_not_received_count = 0;
        }
        else
        {
            twist_msg_not_received_count++;
        }
        //stop message handling

        //print errors
        if (twist_msg_not_received_count > FAIL_TIME)
        {
            ROS_WARN_THROTTLE(ERROR_NOTIFICATION_TIME, "Haven't heard from publisher in a minute: %i cycles", twist_msg_not_received_count);
        }
        if (lidar_msg_not_received_count > FAIL_TIME)
        {
            ROS_ERROR_THROTTLE(ERROR_NOTIFICATION_TIME, "Haven't heard from lidar in a minute: %i cycles, publishing stop message", lidar_msg_not_received_count);
            pub.publish(STOP_TWIST);
            critical_fail = false;
        }
        //end error printing

        ros::spinOnce();
        loop_rate.sleep();
    }
}
