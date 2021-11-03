#include <iostream>
#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include <cmath>

using namespace std;

const float FPS = 30.0;
const float T = 1 / FPS; //Period

const float ACCXLIMIT = 3;   //3.0
const float ACCYLIMIT = 10;  //35.0
const float ACC_LIMIT[2] = {ACCXLIMIT, ACCYLIMIT};
const float VELLIMIT = 10;   //10.0

//to determine when have we crossed a point
const float proximity_THRESHOLD_X = 0.1;
const float proximity_THRESHOLD_Y = 0.04;



//Class used to list the points selected by the "lasers_to_points" node
class SelectedPoints
{
public:
  float position[2] = {5, 0}; //first column aproximation
  float velocity[2];
  float acceleration[2];
  bool reached = false;
};
const int points_limit = 2048;
SelectedPoints Path[points_limit];

//Class used to emulate the Bird
class Flappy
{
public:
  float position[2];
  float velocity[2];
};
Flappy Bird;

// Class used to compute the accelerations
class Planned
{
public:
  float position[2];
  float velocity[2];
  float acceleration[2];
};
Planned Points[2];


void initNode()
{
  //Initialization of nodehandle
  nh_ = new ros::NodeHandle();
  //Init publishers and subscribers
  pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc", 1);
  sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, velCallback);
  pub_points = nh_->subscribe<geometry_msgs::Point>("/flappy_point_path", 1, PointsReceivedCallback);
}


void velCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  Bird.velocity[0] = msg->x;
  Bird.velocity[1] = msg->y;
  // simple time integrative
  Bird.position[0] = Bird.position[0] + Bird.velocity[0] * T;
  Bird.position[1] = Bird.position[1] + Bird.velocity[1] * T;

  path_planner();
}


float path_planner()
{
  geometry_msgs::Vector3 acc_cmd;
  for (int i = 0; i < points_limit; i++)
  {
    // X AXIS
    if (((std::abs(Path[i].position[0] - Bird.position[0]) < proximity_THRESHOLD_X) && (std::abs(Path[i].position[1] - Bird.position[1]) < proximity_THRESHOLD_X) &&
        (Path[i].reached == false)) //if we are close to the point, and the point has not been previously reached
        || //OR
        ((Path[i].reached == false) && (Path[i].position[0] < Bird.position[0])) //if we left the point behind
        
        ) 
    {
      Path[i].reached = true;
      break;
    }
    else if (Path[i].reached == false) //next non-reached point
    {
      // assign initial values
      Points[0].position[0] = Bird.position[0];
      Points[0].velocity[0] = Bird.velocity[0];
      for (int j = 0; j < 2; j++) //size_path
      {
        float crit[2]; //distance where we have to brake when vel is max
        crit[0] = minimum_braking_distance(VELLIMIT, ACC_LIMIT[0]);
        crit[1] = minimum_braking_distance(VELLIMIT, ACC_LIMIT[1]);

        float curr[2]; //distance where we have to brake for current velocity
        curr[0] = minimum_braking_distance(Points[j].velocity[0], ACC_LIMIT[0]);
        curr[1] = minimum_braking_distance(Points[j].velocity[1], ACC_LIMIT[1]);

        float dist[2]; //current distance with the objective
        dist[0] = Path[i].position[0] - Points[j].position[0];
        dist[1] = Path[i].position[1] - Points[j].position[1];

        if (dist[0] < proximity_THRESHOLD_X || Bird.position[0] > Path[i].position[0]) // we are in the STOP ZONE (near the target)
        {
          Points[j + 1].acceleration[0] = -Points[j + 1].velocity[0] / T;
          Points[j + 1].velocity[0] = Points[j].velocity[0] + Points[j + 1].acceleration[0] * T; // should be 0
          Points[j + 1].position[0] = Points[j].position[0] + Points[j + 1].velocity[0] * T;     // should be target
          break;
        }
        else if (dist[0] > crit[0]) //we have to accelerate
        {
          Points[j + 1].acceleration[0] = ACC_LIMIT[0]; //max acceleration
          Points[j + 1].velocity[0] = Points[j].velocity[0] + Points[j + 1].acceleration[0] * T;
          
          if (Points[j + 1].velocity[0] > VELLIMIT) //if we surpass the max acceleration, cut it down
          {
            Points[j + 1].acceleration[0] = (VELLIMIT - Points[j].velocity[0]) / T;
            Points[j + 1].velocity[0] = Points[j].velocity[0] + Points[j + 1].acceleration[0] * T;
          }
          Points[j + 1].position[0] = Points[j].position[0] + Points[j + 1].velocity[0] * T;
        }
        else if (dist[0] <= crit[0]) //we may be able to accelerate but NOT to reach the max velocity
        {
          if (dist[0] > curr[0]) //we still have margin not to deaaccelerate
          {
            float next_iter_curr;
            float next_iter_dist;
            float temp_acceleration = ACC_LIMIT[0];
            for (int m = 0; m < 15; m++)
            {
              temp_acceleration = ACC_LIMIT[0] - ACC_LIMIT[0] * m / 16;
              next_iter_curr = minimum_braking_distance(Points[j + 1].velocity[0], temp_acceleration);
              next_iter_dist = Path[j].position[0] - Points[j + 1].position[0];

              if (temp_acceleration < 0) //prevent the velocity from going negative
              {
                Points[j + 1].acceleration[0] = ACC_LIMIT[0] - ACC_LIMIT[0] * (m - 1) / 16;
              }

              if (next_iter_dist > next_iter_curr) //if the acceleration is acceptable assign it to out Point variable and stop iterating
              {
                Points[j + 1].acceleration[0] = temp_acceleration;
                break;
              }
              else //otherwhise, pick the smallest
              {
                Points[j + 1].acceleration[0] = temp_acceleration;
              }
            }
            Points[j + 1].velocity[0] = Points[j].velocity[0] + Points[j + 1].acceleration[0] * T;
            Points[j + 1].position[0] = Points[j].position[0] + Points[j + 1].velocity[0] * T;

          }
          else if (dist[0] <= curr[0]) //we have to deaccelerate
          {
            Points[j + 1].acceleration[0] = -ACC_LIMIT[0];
            Points[j + 1].velocity[0] = Points[j].velocity[0] + Points[j + 1].acceleration[0] * T;
            if (Points[j + 1].velocity[0] < 0)
            {
              Points[j + 1].acceleration[0] = (0 - Points[j].velocity[0]) / T;
              Points[j + 1].velocity[0] = Points[j].velocity[0] + Points[j + 1].acceleration[0] * T;
            }
            Points[j + 1].position[0] = Points[j].position[0] + Points[j + 1].velocity[0] * T;
          }
        }
      }

      acc_cmd.x = Points[1].acceleration[0];
      break;
    }
  }


  //Y AXIS

  for (int i = 0; i < points_limit; i++)
  {
    if (Path[i].reached == false)
    {
      Points[0].position[1] = Bird.position[1];
      Points[0].velocity[1] = Bird.velocity[1];

      for (int j = 0; j < 2; j++) 
      {
        float crit[2]; //distance where we have to brake when vel is max
        crit[1] = minimum_braking_distance(VELLIMIT, ACC_LIMIT[1]);

        float curr[2]; //distance where we have to brake for current velocity
        curr[1] = minimum_braking_distance(std::abs(Points[j].velocity[1]), ACC_LIMIT[1]);

        float dist[2]; //current distance with the objective
        dist[1] = std::abs(Path[i].position[1] - Points[j].position[1]);

        if (dist[1] < proximity_THRESHOLD_Y) // we are in the STOP ZONE
        {
          Points[j + 1].acceleration[1] = -Points[j + 1].velocity[1] / T;
          Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T; // should be 0
          Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;     //should be target
          break;
        }
        else if (dist[1] > crit[1]) // we have to accelerate
        {
          
          if (Bird.position[1] < Path[i].position[1]) // Bird under the goal
          {
            Points[j + 1].acceleration[1] = ACC_LIMIT[1]; //max acceleration
            Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
            if (Points[j + 1].velocity[1] > VELLIMIT) //if we are over the velocity limit
            {
              Points[j + 1].acceleration[1] = (+ VELLIMIT - Points[j].velocity[1]) / T;
              Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
            }
            Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;
          }
          else if (Bird.position[1] > Path[i].position[1]) //Bird over the goal
          {
            Points[j + 1].acceleration[1] = -ACC_LIMIT[1]; //max negative acceleration
            Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
            if (Points[j + 1].velocity[1] < -VELLIMIT)
            {
              Points[j + 1].acceleration[1] = (- VELLIMIT - Points[j].velocity[1]) / T;
              Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
            }
            Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;
          }
        }
        else if (dist[1] <= crit[1]) //we may be able to accelerate but NOT reach the max velocity
        {
          if (Bird.position[1] < Path[i].position[1]) //Bird under the goal
          {
            if (dist[1] > curr[1]) //we still have margin not to deaaccelerate
            {
              float next_iter_curr;
              float next_iter_dist;
              float temp_acceleration = ACC_LIMIT[1];
              for (int m = 0; m < 15; m++)
              {
                temp_acceleration = + ACC_LIMIT[1] - ACC_LIMIT[1] * m / 16;
                next_iter_curr = minimum_braking_distance(std::abs(Points[j + 1].velocity[1]), std::abs(temp_acceleration));
                next_iter_dist = std::abs(Path[j].position[1] - Points[j + 1].position[1]);
                if (temp_acceleration < 0)
                {
                  Points[j + 1].acceleration[1] = + ACC_LIMIT[1] - ACC_LIMIT[1] * (m - 1) / 16;
                }

                if (next_iter_dist > next_iter_curr)
                {
                  Points[j + 1].acceleration[1] = temp_acceleration;
                  break;
                }
                else
                {
                  Points[j + 1].acceleration[1] = temp_acceleration;
                }
              }
              //Points[j + 1].acceleration[1] = ACC_LIMIT[1]; //max acceleration
              Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;

              Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;

            }
            else if (dist[1] <= curr[1]) //we have to deaccelerate
            {
              //ROS_INFO("BRAKE GO UP");
              Points[j + 1].acceleration[1] = - ACC_LIMIT[1]; //max negative acceleration
              Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
              if (Points[j + 1].velocity[1] < 0)
              {
                Points[j + 1].acceleration[1] = (0 - Points[j].velocity[1]) / T;
                Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
              }
              Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;
            }
          }

          if (Bird.position[1] > Path[i].position[1]) //Bird over the goal
          {
            if (dist[1] > curr[1]) //we still have margin not to deaaccelerate
            {
              float next_iter_curr;
              float next_iter_dist;
              float temp_acceleration = - ACC_LIMIT[1];
              for (int m = 0; m < 15; m++)
              {
                temp_acceleration = - ACC_LIMIT[1] + ACC_LIMIT[1] * m / 16;
                next_iter_curr = minimum_braking_distance(std::abs(Points[j + 1].velocity[1]), std::abs(temp_acceleration));
                next_iter_dist = std::abs(Path[j].position[1] - Points[j + 1].position[1]);
                if (temp_acceleration > 0)
                {
                  Points[j + 1].acceleration[1] = - ACC_LIMIT[1] + ACC_LIMIT[1] * (m - 1) / 16;
                }

                if (next_iter_dist > next_iter_curr)
                {
                  Points[j + 1].acceleration[1] = temp_acceleration;
                  break;
                }
                else
                {
                  Points[j + 1].acceleration[1] = temp_acceleration;
                }
              }
              Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
              Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;
      
            }
            else if (dist[1] <= curr[1]) //we have to deaccelerate
            {
              Points[j + 1].acceleration[1] = + ACC_LIMIT[1]; //max acceleration
              Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
              if (Points[j + 1].velocity[1] > 0)
              {
                Points[j + 1].acceleration[1] = (0 - Points[j].velocity[1]) / T;
                Points[j + 1].velocity[1] = Points[j].velocity[1] + Points[j + 1].acceleration[1] * T;
              }
              Points[j + 1].position[1] = Points[j].position[1] + Points[j + 1].velocity[1] * T;
            }
          }
        }
      }
      
      
      ROS_INFO("AUTO: Path[%d].position[0]: %f", i, Path[i].position[0]);
      ROS_INFO("AUTO: Path[%d].position[1]: %f", i, Path[i].position[1]);
      ROS_INFO("Bird.position[0]: %f", Bird.position[0]);
      ROS_INFO("Bird.position[1]: %f", Bird.position[1]);
      ROS_INFO("Path[%d].acceleration[0]: %f", i, Points[1].acceleration[0]);
      ROS_INFO("Path[%d].acceleration[1]: %f", i, Points[1].acceleration[1]);
      

      acc_cmd.y = Points[1].acceleration[1];
      break;
    }
  }
  //publish the accelerations
  pub_acc_cmd.publish(acc_cmd);
}

void PointsReceivedCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  int path_number = msg->z;
  Path[path_number].position[0] = msg->x;
  Path[path_number].position[1] = msg->y;
}

float minimum_braking_distance(float V, float A)
{
  float X = 0;
  float n = V / (A * T);

  if (n == 0) //velocity = 0 (aboid infinite)
  {
    return 0;
  }
  n = round(n);
 
  A = V / (T * n);

  while (V > 0)
  {
    V = V - A * T;
    X = X + V * T;
  }

  return X;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "flappy_automation_code");
  initNode();

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
