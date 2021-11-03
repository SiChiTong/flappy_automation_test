# Flappy Bird Automation Game

This repository contains the Flappy Bird game modified to be controlled with ROS as well as the C++ algorithm to automatically control it.

![Flappy](flappy_cover.png)

## Getting Started

*This game has been tested with Ubuntu 16.04 running ROS Kinetic and Pygame (Python 2.7.).*

*If you don't have ROS installed in your system, you can install ROS and setup a workspace as covered here [[ROS install guide]](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).*

*If you don't have Pygame installed, you can do so by typing the following code in the Ubuntu Terminal:*
```
sudo apt-get install python-pygame
```

## Run the simulation

Open the Ubuntu Terminal and navigate to your workspace location:
```
cd ~/catkin_ws/
```
Clone the automation game,
```
git clone https://github.com/pep248/flappy_automation_test.git
```

Run the catkin_make command
```
catkin_make
```

Run the simulation:
```
roslaunch flappy_automation_code flappy_automation_code_cpp.launch
```

## Additional information
Scaling: 1 pixel = 0.01 meter  
Game and sensor update rates: 30 fps   
The velocity measurement is noise free   
Max acceleration x: 3.0 m/s^2  
Max acceleration y: 35.0 m/s^2  
Axis convention: x &rarr;, y &uarr;  


| Value         | Unit           | Topic  | Info |
| ------------- |:-------------:| :-----:| :-----:|
| Velocity      | meters/s           | /flappy_vel | [Vector3 message definition](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html) | 
| Acceleration  | meters/s^2         | /flappy_acc | [Vector3 message definition](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html) |
| LaserScan     | Radians, meters      | /flappy_laser_scan | [LaserScan message definition](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) |
| Points        | meters      | /flappy_point_path | [Point message definition](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) |
