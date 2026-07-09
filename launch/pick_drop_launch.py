#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bug_alg = LaunchConfiguration('bug_algorithm').perform(context)

    bug_node = Node(
        package='pick_drop_nav',
        executable=bug_alg,
        name='bug_algorithm',
        parameters=[{'standalone': False, 'goal_tolerance': 0.06}]
    )

    nodes = [

        Node(
            package='pick_drop_nav',
            executable='aruco_detector_node',
            name='aruco_detection',
            parameters=[{
                'yaml_path':          LaunchConfiguration('calib_file').perform(context),
                'cube_id':            int(LaunchConfiguration('target_id').perform(context)),
                'marker_length':      0.08,    
                'cube_marker_length': 0.045,    
                'camera_offset_x':    0.1241,  
            }]
        ),

        Node(
            package='pick_drop_nav',
            executable='localisation',
            name='localisation',
            #parameters=[{
             #   'use_clock_topic': False,
              #  'K_R':    0.1015834066,
              #  'K_L':    0.1110887664,
              #  'r_dd':   0.1,
              #  'r_aa':   0.1,
              #  'aruco_map': '{"811": [-1.0, 0.0], "871": [0.0, -1.6], "9": [1.0, 0.0], "8": [0.0, 1.6]}',
                
            #}]
        ),


        bug_node,


        Node(
            package='pick_drop_nav',
            executable='center_and_approach',
            name='center_and_approach',
            parameters=[{
                'standalone':    False,
                'stop_dist':     0.15,
                'brake_margin':  0.10,
                'w_min':         0.10,
                'yaw_thr':       0.20,
                'pick_advance_dist': 0.12,
            }]
        ),

        Node(
            package='pick_drop_nav',
            executable='mission_coordinator',
            name='mission_coordinator',
            parameters=[{
                'pickup_x':         float(LaunchConfiguration('pickup_x').perform(context)),
                'pickup_y':         float(LaunchConfiguration('pickup_y').perform(context)),
                'dropoff_x':        float(LaunchConfiguration('dropoff_x').perform(context)),
                'dropoff_y':        float(LaunchConfiguration('dropoff_y').perform(context)),
                'nav_tolerance':    0.10,
                'ca_trigger_dist':  1.0,
                'servo_wait':       2.0,
            }]
        ),
    ]

    return nodes


def generate_launch_description():

    args = [
        DeclareLaunchArgument('pickup_x',       default_value='0.0'),
        DeclareLaunchArgument('pickup_y',       default_value='1.25'),
        DeclareLaunchArgument('dropoff_x',      default_value='0.0'),
        DeclareLaunchArgument('dropoff_y',      default_value='-1.20'),
        DeclareLaunchArgument('bug_algorithm',  default_value='bug2',
                              description='bug0 o bug2'),
        DeclareLaunchArgument('calib_file',     default_value='~/.ros/camera_info/puzz_cam.yaml'),
        DeclareLaunchArgument('target_id',      default_value='17'),
        DeclareLaunchArgument('use_flip',       default_value='true'),

    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
