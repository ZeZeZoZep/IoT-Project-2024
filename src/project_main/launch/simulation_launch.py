import sys

from random import randint

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from project_main.sim_utils import spawn_sdf




WORLD_NAME = "iot_project_world"

NUMBER_OF_BALLOONS = 3
NUMBER_OF_SENSORS = 9


import numpy as np
from project_main.math_utils import random_point_in_circle



#-----------------------------------------------------------------------------------------------
# Launch file for the IoT Project. Launches all the nodes required to start the final solution
#-----------------------------------------------------------------------------------------------

def generate_launch_description():



    #------------------- Launch Gazebo here, with the iot_project_world world --------------------
    targets_to_spawn = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch/'
                'gz_sim.launch.py',
            ])
            ]),
            launch_arguments={
            'gz_args' : f"resources/{WORLD_NAME}.sdf"
            }.items()
        )
    ]



    #-------------------------------- Bridge for the world control -------------------------------

    targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/world/%s/control@ros_gz_interfaces/srv/ControlWorld" % WORLD_NAME
            ]
        )
    )

    #------------------------------ Bridge for the simulation clock ------------------------------
    targets_to_spawn.append(
        Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/world/{WORLD_NAME}/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"
        ]
        )
    )
    
    circles=[]
    #-------------------------- Spawn balloons and bridge their topics ---------------------------
    # if 5 above the ground effective range is 19, the distance between neighbour ballons should be 32 (+16, +27,71)
    for i in range(NUMBER_OF_BALLOONS):
        punto=tuple()
        if i==1:
            punto = (0, +32.0, 0)
        elif i==3:
            punto = (+27.71, +16, 0)
        elif i%3==0:
            punto=(0, (i/3)*32, 0)
        elif i%3==1:
            punto=(27.71, (((i-1)/3)*32)+16, 0)
        else:
            punto=(-27.71, (((i-2)/3)*32)+16, 0)
        circles.append((punto,10))
        targets_to_spawn.append(spawn_sdf("resources/balloon/balloon.sdf", id = i, pos = punto))
        targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/Balloon_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
            ]
        )
        )
        targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/Balloon_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ]
        )
        )

        targets_to_spawn.append(
            Node(
                package="project_main",
                executable="balloon_controller",
                namespace=f"Balloon_{i}",
                name=f"BalloonController{i}"
            )
        )

    print(circles)
    #-------------------------- Spawn sensors and bridge their topics ---------------------------
    '''
    for i in range(NUMBER_OF_SENSORS):
        circle=np.random.randint(0, NUMBER_OF_BALLOONS)
        targets_to_spawn.append(spawn_sdf("resources/active_sensor/active_sensor.sdf", id = i, pos = random_point_in_circle(circles[circle][0],circles[circle][1])))

        targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/ActiveSensor_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ]
        )
        )
        targets_to_spawn.append(
        Node(             
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/ActiveSensor_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
            ]
        )
        )

        targets_to_spawn.append(
            Node(
                package="project_main",
                executable="sensor_controller",
                namespace=f"ActiveSensor_{i}",
                parameters=[
                    {'id': i}
                ]
            )
        )
    '''
     #-------------------------- Spawn sensors and bridge their topics ---------------------------
    for i in range(NUMBER_OF_SENSORS):
        circle=np.random.randint(0, NUMBER_OF_BALLOONS)
        targets_to_spawn.append(spawn_sdf("resources/active_sensor/active_sensor.sdf", id = i, pos = random_point_in_circle(circles[circle][0],circles[circle][1])))

        targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/ActiveSensor_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ]
        )
        )

        targets_to_spawn.append(
        Node(             
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/ActiveSensor_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
            ]
        )
        )

        """ targets_to_spawn.append(
        Node(             
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/ActiveSensor_{i}/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"
            ]
        )
        ) """

        targets_to_spawn.append(
            Node(
                package="project_main",
                executable="sensor_controller",
                namespace=f"ActiveSensor_{i}",
                parameters=[
                    {'id': i}
                ]
            )
        )
    


    #------------------------------------ Spawn base station -------------------------------------
    targets_to_spawn.append(spawn_sdf("resources/base_station/base_station.sdf", pos = (-5, -40, 0)))
    targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/base_station/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ]
        )
    )

    targets_to_spawn.append(
        Node(
            package="project_main",
            executable="base_station_controller",
            arguments=[
                f"{NUMBER_OF_BALLOONS}",
                f"{NUMBER_OF_SENSORS}"
            ]
        )
    )


    targets_to_spawn.append(
        Node(
            package="project_main",
            executable="simulation_manager",
            arguments=[
                f"{NUMBER_OF_BALLOONS}",
                f"{NUMBER_OF_SENSORS}"
            ]
        )
    )

    targets_to_spawn.append(
        Node(
            package="project_main",
            executable="fleet_coordinator",
            arguments=[
                f"{NUMBER_OF_BALLOONS}",
                f"{NUMBER_OF_SENSORS}"
            ]
        )
    )


    return LaunchDescription(targets_to_spawn)
