# Nuturtle  Description
URDF files for Nuturtle Turtlebot
<br/>
<br/>
This package allows for the visualization of multiple turtlebot3 models in rviz. Below are instructions on how to use the package. 
* Use `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
* Use `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
  <br/>
  The robots in rviz:
  ![all_turtles](https://user-images.githubusercontent.com/103614797/211428005-d8ae91d1-9a26-4a94-ab3b-7167be626628.png)

 
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
   ![turtle_rqt](https://user-images.githubusercontent.com/103614797/211428656-cc04c386-bc27-4665-8842-3e61cefdced3.png)


# Launch File Details
The following command line inputs give more details about the launch files:
* `ros2 launch nuturtle_description load_one.launch.py --show-args`
  </br>

Output: 
  
    Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        Flag to enable joint_state_publisher_gui. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_rviz':
        Flag to launch rviz. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        Set turtle bot material. Valid choices are: ['red', 'blue', 'purple', 'green']
        (default: 'purple')
* `ros2 launch nuturtle_description load_all.launch.xml --show-args`
 </br>

Output: 
  
    Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        Flag to enable joint_state_publisher_gui. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_rviz':
        Flag to launch rviz. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        Set turtle bot material. Valid choices are: ['red', 'blue', 'purple', 'green']
        (default: 'purple')

Worked With Ava Zahedi, Rintaroh Shima, Marno Nel, James Oubre
