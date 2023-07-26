# NuSlam Package
The `nuslam` package gives a turtlebot basic SLAM capabilities. An extended Kalman Filter (EKF) is used to estimate and correct the turtlebots position as it moves in the world. 

# Launch Files
This package contains the following launch files:
* `slam.launch.xml`: launches the `start_robot.launch.xml` launch file from the [nuturtle_control](nuturtle_control) package, `load_one.launch.xml` from [nuturtle_description](nuturtle_description), and the `slam` node from this package. The `start_robot.launch.xml` starts necessary nodes for turtlebot control and visualization, the  `load_one.launch.xml` launches a green robot in rviz, and the `slam` node handles the EKF calculations that ultimately control the green robot's position in the world. 
    * Use `ros2 launch nuslam slam.launch.xml cmd_src:=teleop robot:=nusim` to run the launch file. This will launch a red robot (ground truth), a blue robot (odometry estimation), and a green robot representing a corrected estimation of the red robots position. 
    
* `landmark_detect.launch.xml`: launches the `slam.launch.xml` launch file and the `landmarks` node. It applies data association for landmark detection as part of the SLAM implementaion. 
    * Use `ros2 launch nuslam landmark_detect.launch.xml cmd_src:=teleop robot:=nusim` to run the launch file. This will launch a red robot (ground truth), a blue robot (odometry estimation), and a green robot representing a corrected estimation of the red robots position, similar to the slam launch. 

# Demo Video
Here is a video of running the launch file. The red robot is being controlled with teleop keys, and the green robot can be seen following the red robots position due to the EKF corrections:

[SLAM_VID.webm](https://user-images.githubusercontent.com/103614797/222307589-0650ba46-c10c-4e59-903a-dc48b389d988.webm)

Here is a screenshot of the final positions and paths of the robots. The obstacles are in labeled in green to indicate that the green robot has detected its position in the world:

![SLAM_PIC](https://user-images.githubusercontent.com/103614797/222307740-25ac1c95-9d7c-4ef3-80f3-8f13d1d2999f.png)


Here is a video of the landmark detection and implementation of the data association within SLAM. The blue cylinder markers are the detected landmarks from the circle fitting and data association algorithms. The data association impelemtation lowered the accuracy of the original SLAM implementation, which can be seen in the video. The green robots position is inaccurately estimated, but the SLAM algorithm is still somewhat able to minimuze that error and align it with the red robot. The inaccuracies of landmark detecion and location estimation occur more as the robots are moved within the envorinment, which can be seen in the video. 


[slam_final_moving.webm](https://user-images.githubusercontent.com/103614797/226091603-12b97ab2-0d63-4ee0-b1e1-492c7ac95efe.webm)
















