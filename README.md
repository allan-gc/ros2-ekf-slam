# ME495 Sensing, Navigation and Machine Learning For Robotics
## Allan Garcia - Winter 2022

# Package List
This repository consists of several ROS packages:
- [nuturtle_description](nuturtle_description) - Used to visualize multiple turtlebot3 models in rviz
- [nusim](nusim) - Creates the simulator environment for the turtlebot3 models and allows for the manipulation and tracking of the robots position in rviz
- [nuturtle_control](nuturtle_control) - For controlling a simulated and physical turtlebot3. 
- [nuslam](nuslam) - Allows for control of turtlebot3 with added SLAM capabilites. 


# Library List
It also includes a C++ package library:

- [turtlelib](turtlelib) - Contains functions and classes for controlling a differential drive robot using forward and inverse kinematics, as well as general 2D transform calculations. It also contains a class for implementing an extended Kalman Filter for SLAM. More on the classes, functions and mathematical derivations can be found in the turtlelib package in /doc.

# Demo Videos
The result of using these packages is being able to control a turtlebot3. Here is a video of it in action both physically and in simulation. More videos and explanations can be found in the nuturtle_control package. 

[REAL_SIM.webm](https://user-images.githubusercontent.com/103614797/217753169-77573890-4c11-4eb9-9a48-e747d80c1bc5.webm)
