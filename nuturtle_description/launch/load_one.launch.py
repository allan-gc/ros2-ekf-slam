
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(name='use_jsp', default_value='true', choices=['true', 'false'],
                              description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument(name='use_rviz', default_value='true', choices=['true', 'false'],
                              description='Flag to launch rviz'),

        DeclareLaunchArgument(name='color',
                              default_value="purple",
                              choices=['red', 'blue', 'purple', 'green'],
                              description='Set turtle bot material'),

        SetLaunchConfiguration(name='rviz_config', value=[FindPackageShare('nuturtle_description'),
                                                          TextSubstitution(text='/config/basic_'),
                                                          LaunchConfiguration('color'),
                                                          TextSubstitution(text='.rviz')]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('color'),
            parameters=[
                {'frame_prefix': PathJoinSubstitution([LaunchConfiguration('color'), '']),
                    'robot_description':
                    Command([ExecutableInPackage("xacro", "xacro"), " ",
                            PathJoinSubstitution(
                            [FindPackageShare('nuturtle_description'),
                                "urdf/turtlebot3_burger.urdf.xacro"]),
                                " color:=",
                                LaunchConfiguration('color')])}
            ]
            ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=LaunchConfiguration('color'),
            condition=LaunchConfigurationEquals('use_jsp', 'true')
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace=LaunchConfiguration('color'),
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('nuturtle_description'),
                    LaunchConfiguration('rviz_config')
                    ])
            ],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        )

    ])
