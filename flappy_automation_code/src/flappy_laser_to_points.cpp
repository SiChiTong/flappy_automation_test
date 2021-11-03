#include "ros/ros.h"
#include "flappy_automation_code/flappy_laser_to_points.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <algorithm>

// Timming
float FPS = 30.0;
float T = 1 / FPS; //Period


// laser specs
float range_max = -1;
const float laser_length_THRESHOLD = -0.05; //the measures close to its total length will be ignored


const float X_TRESHOLD = 1.4;               //acceptable distance between 2 rocks to allow passing (found experimentally)
const float Y_TRESHOLD = 0.50;              //maximum width of a rock (found experimentally)

const float superior_limit = +2.40;         // Determine Dynamically
const float inferior_limit = -1.40;         // Determine Dynamically //-1.4

const float relyability_THRESHOLD = 80.0;   //determines at what point a path is relyable enough

const float X_negative_big_THRESHOLD = 1.2;   // used to determine the next non-relyable point previous to a column
const float X_negative_small_THRESHOLD = 0.6; // used to determine the next relyable point previous to a column
const float X_positive_THRESHOLD = 0.8;       // used to determine the next point after a column


// Class used by the 9 sensors
class LaserSensor
{
public:
  float length = 0;
  float angle = 0;
  float position[2] = {0, 0};
};
LaserSensor Laser[9];

// Class used to represent the bird properties
class Flappy
{
public:
  float position[2] = {0, 0};
  float velocity[2] = {0, 0};
};
Flappy Bird;


// Class to store the find points to be used as a path
const int column_limit = 1024;
const int splits_limit = 256;
class DetectedObject
{
public:
  float column_position;
  float points_max[splits_limit] = {superior_limit - Y_TRESHOLD};
  float points_min[splits_limit] = {inferior_limit + Y_TRESHOLD};
  int free[splits_limit] = {-1}; //free = -1, blocked = 1, unnasigned = 0
  float score[splits_limit] = {superior_limit - inferior_limit};
  float center[splits_limit] = {(superior_limit + inferior_limit) / 2};
  float relyability[splits_limit];
  int best;
};
DetectedObject Column[column_limit];


// Class to store the find points to be used as a path
class SelectedPoints
{
public:
  float position[2]; //simple method to force the bird to move forward once the program has started
};
SelectedPoints Path[column_limit * 2];


void update_lasers(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  for (int i = 0; i < 9; i++)
  {
    Laser[i].length = msg->ranges[i];
    Laser[i].angle = 0 + (msg->angle_increment) * (i - 4);
    range_max = msg->range_max + laser_length_THRESHOLD;

    Laser[i].position[0] = (Bird.position[0] + Laser[i].length * cos(Laser[i].angle));
    Laser[i].position[1] = (Bird.position[1] + Laser[i].length * sin(Laser[i].angle));
  }

}


void initNode()
{
  //Initialization of nodehandle
  nh_ = new ros::NodeHandle();
  //Init publishers and subscribers
  pub_points = nh_->advertise<geometry_msgs::Point>("/flappy_point_path", 1);
  sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, velCallback);
  sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, laserScanCallback);
}


void velCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback

  Bird.velocity[0] = msg->x;
  Bird.velocity[1] = msg->y;
  // simple time integrative
  Bird.position[0] = Bird.position[0] + Bird.velocity[0] * T;
  Bird.position[1] = Bird.position[1] + Bird.velocity[1] * T;
}


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  update_lasers(msg);
  column_detector();
  space_splitter();
  split_ranker();
  path_finder();
}


void column_detector()
{
  for (int k = 0; k < 9; k++) //for all nine lasers
  {
    if ((Laser[k].position[1] < superior_limit) && (Laser[k].position[1] > inferior_limit) && (Laser[k].length < range_max)) //if we detect anything that is not the ceiling and grownd
    {
      for (int l = 0; l < column_limit; l++) //for all columns determine what column is the laser pointing at
      {

        if (l == 0)
        {
          if (Laser[k].position[0] > (Column[l].column_position + X_TRESHOLD)) //
          {
            if (Column[l].column_position == 0) //if the column we are pointing at has a -1 assigned, assign it a new X value
            {
              Column[l].column_position = Laser[k].position[0];
              ROS_INFO("Laser[%d].position[0]: %f", k, Laser[k].position[0]);
              ROS_INFO("column %d has been assigned with: %f", l, Column[l].column_position);
              break;
            }
          }
        }
        else
        {
          if ((Laser[k].position[0] > (Column[l - 1].column_position + X_TRESHOLD)) && (Column[l - 1].column_position != 0)) //method to aboid overwriting all columns positions
          {
            if (Column[l].column_position == 0) //if the column we are pointing at has a -1 assigned, assign it a new X value
            {
              Column[l].column_position = Laser[k].position[0];
              ROS_INFO("Laser[%d].position[0]: %f", k, Laser[k].position[0]);
              ROS_INFO("column %d has been assigned with: %f", l, Column[l].column_position);
              break;
            }
          }
        }
      }
    }
  }
}


void space_splitter()
{
  for (int k = 0; k < 9; k++) //for all nine lasers
  {
    if ((Laser[k].position[1] < superior_limit) && (Laser[k].position[1] > inferior_limit) && (Laser[k].length < range_max)) //if we detect anything that is not the ceiling and grownd
    {
      for (int l = 0; l < column_limit; l++) //for all columns determine to what column does our point belong
      {
        if ((Laser[k].position[0] < (Column[l].column_position + X_TRESHOLD)) && (Laser[k].position[0] > (Column[l].column_position - X_TRESHOLD))) //belongs to the column
        {
          for (int m = 0; m < splits_limit; m++) //for all the splits
          {
            if ((Column[l].free[m] == -1) && (Laser[k].position[1] < Column[l].points_max[m]) && (Laser[k].position[1] > Column[l].points_min[m])) //if the split is free and the laser is between its range
            {
              for (int n = 0; n < splits_limit; n++) //look for the first unnasigned split
              {
                if (Column[l].free[n] == 0)
                {
                  //create and assign splits
                  //free = -1, blocked = 1, unnasigned = 0
                  //the new split
                  Column[l].points_max[n] = Column[l].points_max[m]; //the top part of the split is the original point_max
                  Column[l].points_min[n] = Laser[k].position[1];    //the lower part of the split is the laser_y position
                  Column[l].free[n] = -1;                            //we set it free as soon as it is assigned
                  Column[l].score[n] = Column[l].points_max[n] - Column[l].points_min[n];
                  Column[l].center[n] = (Column[l].points_max[n] + Column[l].points_min[n]) / 2;

                  //the original split
                  Column[l].points_max[m] = Laser[k].position[1];    //the top part of the split is the laser_y position
                  Column[l].points_min[m] = Column[l].points_min[m]; //the lower part of the split is the original point_min
                  Column[l].free[m] == -1;                           //should already be free
                  Column[l].score[m] = Column[l].points_max[m] - Column[l].points_min[m];
                  Column[l].center[m] = (Column[l].points_max[m] + Column[l].points_min[m]) / 2;

                  if (std::abs(Column[l].points_max[n] - Column[l].points_min[n]) < Y_TRESHOLD) //occupy the small splits
                  {
                    Column[l].free[n] = 1;
                  }
                  if (std::abs(Column[l].points_max[m] - Column[l].points_min[m]) < Y_TRESHOLD) //occupy the small splits
                  {
                    Column[l].free[m] = 1;
                  }
                  break; //stop once the split has been done
                }
              }
              break; //all the splits are mutually exclusive, so if we find a match, we stop the loop
            }

          }      
          break; //stop once we have found the belonging column
        }
      }
    }
  }
}


void split_ranker()
{
  float relyability[column_limit];
  float score_addition = 0;
  for (int l = 0; l < column_limit; l++) //for all columns
  {
    if (Column[l].column_position == 0) //unnasigned columns
    {
      break;
    }
    else if (Bird.position[0] > (Column[l].column_position)) //if the bird is past a column
    {
      //do nothing
    }
    else
    {
      int counter = 0;
      for (int m = 0; m < splits_limit; m++) //for all the splits
      {
        if (Column[l].free[m] == -1) //check if free
        {
          score_addition = score_addition + Column[l].score[m]; //we needed this value for the relyable calculator
          counter++;
          Column[l].best = m;
        } 
      }
      //we ran the same for loop again
      for (int m = 0; m < splits_limit; m++) //for all the splits
      {
        if (Column[l].free[m] == -1) //check if free
        {
          //first part checks best between all candidates, the second part checks it is not too big
          Column[l].relyability[m] = (Column[l].score[m] / score_addition) * (Y_TRESHOLD / Column[l].score[m]) * (Y_TRESHOLD / Column[l].score[m]) * 100;
          if (relyability[l] < Column[l].relyability[m]) //pick the best candidate
          {
            relyability[l] = Column[l].relyability[m];
            Column[l].best = m;
          }

        } //for all columns
      }
      
      
      ROS_INFO("Column[%d].center[%d]: %f", l, Column[l].best, Column[l].center[Column[l].best]);
      ROS_INFO("Column[%d].relyability[%d]: %f", l, l, Column[l].relyability[Column[l].best]);
      ROS_INFO("Bird.position[0]: %f", Bird.position[0]);
      ROS_INFO("Bird.position[1]: %f", Bird.position[1]);
      ROS_INFO("Column[%d].relyability[%d]: %f", l, l, Column[l].relyability[Column[l].best]);
      ROS_INFO("LASER: Path[%d].position[0]: %f",l*2, Path[l].position[0]);
      ROS_INFO("LASER: Path[%d].position[1]: %f",l*2, Path[l].position[1]);
      
      
    } //for all nine lasers
  }
}

void path_finder()
{
  geometry_msgs::Point published_point;
  for (int l = 0; l < column_limit; l++) //for all columns
  {
    if (Column[l].relyability[Column[l].best] <= relyability_THRESHOLD) //not relyable
    {
      Path[l * 2].position[1] = Column[l].center[Column[l].best];                     //we optain a y position for the path planner after all the for loops
      Path[l * 2].position[0] = Column[l].column_position - X_negative_big_THRESHOLD; //we optain a y position for the path planner after all the for loops
    }
    else //relyable
    {
      Path[l * 2].position[1] = Column[l].center[Column[l].best];                       //we optain a y position for the path planner after all the for loops
      Path[l * 2].position[0] = Column[l].column_position - X_negative_small_THRESHOLD; //we optain a y position for the path planner after all the for loops
    }
                                                                                       
    if (Bird.position[0] <= Path[l * 2].position[0]) //publish all the points ahead of the bird
    {
      published_point.x = Path[l * 2].position[0];
      published_point.y = Path[l * 2].position[1];
      published_point.z = l * 2; //we will use z to help keep the order of the points since it is unused
      pub_points.publish(published_point);
    }

    Path[l * 2 + 1].position[1] = Column[l].center[Column[l].best];                 //we optain a y position for the path planner after all the for loops
    Path[l * 2 + 1].position[0] = Column[l].column_position + X_positive_THRESHOLD; //we optain a y position for the path planner after all the for loops

    if (Bird.position[0] <= Path[l * 2].position[0])
    {
      published_point.x = Path[l * 2 + 1].position[0];
      published_point.y = Path[l * 2 + 1].position[1];
      published_point.z = l * 2 + 1;
      pub_points.publish(published_point);



    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "flappy_laser_to_points");
  initNode();

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}