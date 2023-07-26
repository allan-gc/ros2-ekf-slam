# NuTurtle Control Package
The `nuturtle_control` package allows for the control of a Turtlebot3 using wheel commands. The main functionality of this package is for controlling a simulated and physical turtlebot3. It provdes nodes that can track the position of the robot in
simluation as well as odometry information. When in simulation, a blue turtlebot3 represents the followed odometry information while a red turtlebot3 represents the simulated kinematics. 

# Launch Files
This package contains the following launch files:

* `start_robot.launch.xml`: this launch file is capable of launching several different nodes that allow for different control and visualization of the turtlebot. The different arguments are described in the launch file. Some of the different launch commands are explained below:
    * `ros2 launch nuturtle_control start_robot.launch.xml robot:=nusim cmd_src:=circle` : starts the `nusim` simulator and the `circle` node. This command allows for virtual control and visualization of the turtlebot. 

    * `ros2 launch nuturtle_control start_robot.launch.xml robot:=localhost cmd_src:=circle` : starts the `numsr_turtlebot` node and the `circle` node. This command allows for control of the physical robot. You must be connected to the physcial turtlebot3 when `robot` is set to `localhost` in order for the `numsr_turtlebot` node to properly send and recieve commands to the robot. After launching this node on the robot, you can send wheel commands through `circle` or `teleop` to control the robot.

    * `ros2 launch nuturtle_control start_robot.launch.xml robot:=none use_rviz:=true`: starts an rviz node. This launch file can be used to visualize the blue robot in rviz. This launch file allows you to view the robot in simulation as it physically moves. To do so, first launch a launch file that connects to the robot with `robot=:localhost` (like the one above), and then launch this file locally. An example of this funcionality is seen at the bottom of the page. 


# Node Parameters
The package includes a .yaml file that contains a list of parameters that list some speficiations for the turtlebot3. These parameters are mainly used by the turtle_control node. The other nodes have parameters that are set and described in the launch file. Below is a list of the parameters from the .yaml file:

* `wheel_radius`: the radius of the turtlebot3 wheels

* `track_width`: the distance between the center of one wheel to the other

* `motor_cmd_max `: the max wheel command for the wheel encoders

* `motor_cmd_per_rad_sec `: the velocity of the wheels (rad/s) per 1 tick

* `encoder_ticks_per_rad `: the number of encoder ticks per radius that the wheel moves

* `collision_radius `: distance from the robot chassis that is used to detect a       collision

<br/>

# Simulation Demo
Here is a demo of using only the simulated turtlebots. The blue arrows represent the odometry. Both the blue and red robot are overlayed and follow the same circular path: 


[NUSIM.webm](https://github.com/allan-gc/ros2-ekf-slam/assets/103614797/f5fe8616-8335-4c08-bee6-ef3d9bb4163f)


<br/>

Here is a video of using the real robot while simultaneously visualizing its movement in rviz. The robot is being commanded with the services in the circle node to move forward and backward along an arc. At the end of the video, once it stops in the square marked on the floor, it is moved using teleop keys to position it in the square a bit better to try to get a smaller error in odometry. The error in odometry during this trial was 0.0025m.  


[REAL_SIM.webm](https://github.com/allan-gc/ros2-ekf-slam/assets/103614797/fb5aa3d3-0100-4201-880c-0aefc54b13a6)


