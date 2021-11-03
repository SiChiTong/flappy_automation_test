#ifndef FLAPPY_LASER_TOPOINTS_H_
#define FLAPPY_LASER_TOPOINTS_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

//Ros nodehandle
ros::NodeHandle* nh_= NULL;
ros::NodeHandle* nh= NULL;
//Publisher for passing point
ros::Publisher pub_points;
//Subscriber for velocity
ros::Subscriber sub_vel;
//Subscriber for laser scan
ros::Subscriber sub_laser_scan;


void initNode();
void velCallback(const geometry_msgs::Vector3::ConstPtr &msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

// Function to determine to what column each point belongs
void column_detector();

// Function to split a column into smaller blocks, in order to find a hole
void space_splitter();

// Rank the found holes and pick the best
void split_ranker();

//Find a path between the holes and return the points of this said path
void path_finder();

#endif
