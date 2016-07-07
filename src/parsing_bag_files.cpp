/*
 * File: parsing_bag_files.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include "tac_glove/glove.h"


void gloveSensorsCB(const tac_glove::gloveConstPtr msg)
{
    ROS_INFO("gloveSensorsCB\n");
}

void imuTrackerCB(const geometry_msgs::PoseArrayConstPtr msg)
{
    ROS_INFO("imuTrackerCB\n");
}

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "parsing_bag_files");
    ros::NodeHandle nh;

    ROS_INFO("parsing grasping bag files");

    ros::Subscriber glovev_sub = nh.subscribe("glove_sensors", 1, gloveSensorsCB);
    ros::Subscriber imu_sub = nh.subscribe("tac_glove_imutracker_raw", 1, imuTrackerCB);

    ros::spin();

    // shutdown ROS
    ros::shutdown();

    return 0;
}

