# Flappy Bird Automation Game

This repository contains the Flappy Bird game modified to be controlled with ROS as well as the C++ algorithm to automatically control it.

![Flappy](flappy_cover.png)

## Getting Started

*This game has been tested with Ubuntu 16.04 running ROS Kinetic and Pygame (Python 2.7.)*
*If you don't have ROS installed in your system, you can install ROS and setup a workspace as covered here [[ROS install guide]](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
*If you don't have Pygame installed, you can do so by typing the following code in the Ubuntu Terminal:
```
sudo apt-get install python-pygame
```
*


To run the simulation, open the Ubuntu Terminal and navigate to your workspace location:
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

Run the catkin_make command
```
catkin_make
```

Run the simulation:
```
roslaunch flappy_automation_code flappy_automation_code_cpp.launch
```

## Other Information
I hope you will have fun solving this little game. If you have any questions or need other game information either write me or look around in the **flappy_main_game** folder. Here is some other helpful information for solving the task.

Scaling: 1 pixel = 0.01 meter  
Game and sensor update rates: 30 fps   
The velocity measurement is noise free   
Max acceleration x: 3.0 m/s^2  
Max acceleration y: 35.0 m/s^2  
Axis convention: x &rarr;, y &uarr;  
[LaserScan message definition](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)

| Value        | Unit           | Topic  |
| ------------- |:-------------:| :-----:|
| Velocity      | m/s           | /flappy_vel |
| Acceleration  | m/s^2         | /flappy_acc |
| LaserScan     | Radians, meters      | /flappy_laser_scan |
