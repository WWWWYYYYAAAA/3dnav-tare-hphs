#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
// #include "tf2_ros/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数


using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
string robot_name = "a1";
nav_msgs::Odometry Odom;
double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;


void callback_BASE(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    int index = 0;
    for (auto &linkName : msg->name) {
        if (linkName == robot_name+"_gazebo::base")
            break;
        ++index;
    }
    ros::Rate rate(500);//延迟至100hz发布，避免重复发布

    Odom.header.stamp = ros::Time::now();
    Odom.header.frame_id = "world";
    Odom.child_frame_id = "base";

    // set the position
    Odom.pose.pose.position.x = msg->pose[index].position.x;
    Odom.pose.pose.position.y = msg->pose[index].position.y;
    Odom.pose.pose.position.z = msg->pose[index].position.z;

    Odom.pose.pose.orientation.w = msg->pose[index].orientation.w;
    Odom.pose.pose.orientation.x = msg->pose[index].orientation.x;
    Odom.pose.pose.orientation.y = msg->pose[index].orientation.y;
    Odom.pose.pose.orientation.z = msg->pose[index].orientation.z;

    // set the velocity
    Odom.twist.twist.linear.x= msg->twist[index].linear.x;
    Odom.twist.twist.linear.y= msg->twist[index].linear.y;
    Odom.twist.twist.linear.z= msg->twist[index].linear.z;

    Odom.twist.twist.angular.x = msg->twist[index].angular.x;
    Odom.twist.twist.angular.y = msg->twist[index].angular.y;
    Odom.twist.twist.angular.z = msg->twist[index].angular.z;

    robotVelocity_BASE_frame_pub.publish(Odom);
    rate.sleep();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_from_gazebo");
    ros::NodeHandle nh("~");
    ros::NodeHandle node;
    ros::Subscriber tfState_BASE_sub;
  
    nh.param<std::string>("robot_name", robot_name, string("a1"));
    tfState_BASE_sub = node.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, callback_BASE);
    robotVelocity_BASE_frame_pub = node.advertise<nav_msgs::Odometry>("/Odometry_gazebo", 1);

    ros::spin();
    return 0;
}



