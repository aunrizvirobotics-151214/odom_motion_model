"""
Launch file for the odometry_motion_model node.

Usage:
  ros2 launch odometry_motion_model motion_model.launch.py
  ros2 launch odometry_motion_model motion_model.launch.py alpha1:=0.5 alpha2:=0.5
  ros2 launch odometry_motion_model motion_model.launch.py with_rviz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('odometry_motion_model')
    rviz_config = os.path.join(pkg, 'rviz', 'motion_model.rviz')

    return LaunchDescription([
        # ── launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument('alpha1',         default_value='0.01',
                              description='rot noise from rotation'),
        DeclareLaunchArgument('alpha2',         default_value='0.01',
                              description='rot noise from translation'),
        DeclareLaunchArgument('alpha3',         default_value='0.001',
                              description='trans noise from translation'),
        DeclareLaunchArgument('alpha4',         default_value='0.001',
                              description='trans noise from rotation'),
        DeclareLaunchArgument('num_samples',    default_value='50',
                              description='number of particles'),
        DeclareLaunchArgument('marginalise_p3', default_value='false',
                              description='marginalise final rotation probability'),
        DeclareLaunchArgument('fixed_frame',    default_value='odom',
                              description='fixed frame for all published topics'),
        DeclareLaunchArgument('with_rviz',      default_value='true',
                              description='launch rviz2 automatically'),

        # ── motion model node ─────────────────────────────────────────────────
        Node(
            package    ='odometry_motion_model',
            executable ='odometry_motion_model_node',
            name       ='odometry_motion_model',
            output     ='screen',
            parameters =[{
                'alpha1':         LaunchConfiguration('alpha1'),
                'alpha2':         LaunchConfiguration('alpha2'),
                'alpha3':         LaunchConfiguration('alpha3'),
                'alpha4':         LaunchConfiguration('alpha4'),
                'num_samples':    LaunchConfiguration('num_samples'),
                'marginalise_p3': LaunchConfiguration('marginalise_p3'),
                'fixed_frame':    LaunchConfiguration('fixed_frame'),
            }],
        ),

        # ── optional RViz2 ────────────────────────────────────────────────────
        Node(
            package    ='rviz2',
            executable ='rviz2',
            name       ='rviz2',
            output     ='screen',
            arguments  =['-d', rviz_config],
            condition  =IfCondition(LaunchConfiguration('with_rviz')),
        ),
    ])
