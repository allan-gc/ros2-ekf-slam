# Nusim Simulator
The `nusim` package creates a simulated environment for visualization, tracking, and control of multiple turtlebot3 robot models. It tracks the position of the robot models and can add obstacles to the environment.

# Launch Files
The package contains the following launch files:

* `nusim.launch.xml`: launches the simulator environment in rviz. It launches an rviz node, the `nusim` node, and it includes the `load_one.launch.py` launch file from the `nuturtle_description` package. 
Launch it using `ros2 launch nusim nusim.launch.xml `

# Simulator Parameters
The package includes a .yaml file that contains a list of parameters that can be used to change the setting and environment of the simulator. It also includes another .yaml parameter file that contains turtlebot3 specifications. Below is a list of the parameters:

* `rate`: controls the frequency of the main timer callback in the `nusim` node

* `draw_only`: Param that determines if the fake sensor data is published

* `max_range`: Max distance used to detect if the robot is near the obstacle

* `x0, y0, theta0`: used to set the initial position of the turtlebot3 in the simulator

* `obstacles`: used to set the dimensions of the cylindrical obstacles and to set the number of obstacles to add to the simulator. It includes four subfields:
    * `x`: a list of the x-coordinates of the obstacles
    * `y`: a list of the y-coordinates of the obstacles
    * `r`: the radius of the obstacles 
    * `h`: the height of the obstacles
<br/>
<br>

* `arena`: used to set the dimensions of walls that surround the simulation. It includes four subfields:
    * `x_length`: the length of arena world in the x-direction
    * `y_length`:the length of arena world in the y-direction
    * `wall_height`: the height of the walls 

<br/>

*  `basic_sensor_variance`: Variance used for zero mean Gaussian noise that is added to obstacle positions
*  `motor_cmd_per_rad_sec`: Conversion for motor command ticks to rad/s (the velocity of each motor command tick)
*  `encoder_ticks_per_rad`: Encoder ticks per radian
*  `collision_radius`: Collision circle around the turtlebot
*  `input_noise`: Variance used for zero mean Gaussian noise that is added to wheel commands
*  `slipping_fraction`: Slip added to wheel commands
*  `laser_max_range`: Max distance that the lidar laser can detect (meters)
*  `laser_min_range`: Min distance that the liadr laser can detect (meters)
*  `angle_increment`: Increment of lidar angle
*  `resolution` : Resolution of lidar data
*  `samples` : Amount of lidar data collected per reading
*  `noise_level`: Variance used for zero mean Gaussian noise that is added to lidar ranges


# Simulator Demo
Here is how the simulator looks with a red turtlebot3 and three red cylindrical obstacles:![nusim1](https://user-images.githubusercontent.com/103614797/213981691-ac2301e1-77bd-4edb-b0fc-29fd32649e30.png)

<br/>

Here is the simulator with the walls included: ![nusim_walls_demo](https://user-images.githubusercontent.com/103614797/217722222-4197bee5-2904-4f99-951d-b98158885624.png)


