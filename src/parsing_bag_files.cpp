/*
 * File: parsing_bag_files.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseArray.h>
#include "tac_glove/glove.h"

#define GLOVE_OUTPUT_FILE "glove_output.txt"
#define IMU_OUTPUT_FILE "imu_output.txt"

using namespace std;

ofstream glove_output;
ofstream imu_output;

unsigned int glove_sensor_cnt = 0;
unsigned int imu_sensor_cnt = 0;

void gloveSensorsCB(const tac_glove::gloveConstPtr msg)
{
    // In current, this topic does not have time stamp,
    // so use ros simulated time instead
    // NOTE: Don't forget to play bag files with --clock option
    // also, use_sim_time should be true (rosparam set use_sim_time true)
  
    // to use ros simulated time instread of time stamp here
    // for synchronization
    glove_output << glove_sensor_cnt << "\t" << ros::Time::now() << "\t"
               << msg->segments[0].force[0] << " " << msg->segments[0].force[1] << " "
               << msg->segments[1].force[0] << " " << msg->segments[0].force[1] << " "
               << msg->segments[2].force[0] << " " << msg->segments[0].force[1] << " "
               << msg->segments[3].force[0] << " " << msg->segments[0].force[1] << " "
               << endl;

    //glove_output << msg->header.stamp << endl;
    
    glove_sensor_cnt++;
}

void imuTrackerCB(const geometry_msgs::PoseArrayConstPtr msg)
{
    // In current, this topic does not have time stamp,
    // so use ros simulated time instead
    // NOTE: Don't forget to play bag files with --clock option
    // also, use_sim_time should be true (rosparam set use_sim_time true)

    // palm, thumb(base,tip), index(base,med,tip), mide(base,med,tip)
    // ring(base,med,tip), pinkie(base,med,tip)
    imu_output << imu_sensor_cnt << "\t" << ros::Time::now() << ":\n";
               // IMU: 0 ~ 14 (total 15 sensors)
               for (int i = 0; i < 15; i++) {
                   imu_output << "\t" << msg->poses[i].position.x << " " << msg->poses[i].position.y << " " << msg->poses[i].position.z << " ";
                   imu_output << msg->poses[i].orientation.x << " " << msg->poses[i].orientation.y << " " << msg->poses[i].orientation.z <<  msg->poses[i].orientation.w << endl;
               }

    //imu_output << msg->header.stamp << endl;

    imu_sensor_cnt++;
}

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "parsing_bag_files");
    ros::NodeHandle nh;

    ROS_INFO("parsing grasping bag files");

    glove_output.open(GLOVE_OUTPUT_FILE);
    imu_output.open(IMU_OUTPUT_FILE);

    ros::Subscriber glovev_sub = nh.subscribe("glove_sensors", 1, gloveSensorsCB);
    ros::Subscriber imu_sub = nh.subscribe("tac_glove_imutracker_raw", 1, imuTrackerCB);

    ros::spin();

    // shutdown ROS
    ros::shutdown();

    glove_output.close();
    imu_output.close();

    return 0;
}

