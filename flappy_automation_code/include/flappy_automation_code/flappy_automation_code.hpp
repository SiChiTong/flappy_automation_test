#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

//Ros nodehandle
ros::NodeHandle* nh_= NULL;
//Publisher for acceleration command
ros::Publisher pub_acc_cmd;
//Subscriber for velocity
ros::Subscriber sub_vel;
//Subscriber for path points
ros::Subscriber pub_points;

void initNode();
void velCallback(const geometry_msgs::Vector3::ConstPtr &msg);
void PointsReceivedCallback(const geometry_msgs::Point::ConstPtr &msg);

//Function to find the minum distance in order to brake for a certain speed and deacceleration
float minimum_braking_distance(float V, float A);

//Function to find and publish the accelerations
float path_planner();


#endif


