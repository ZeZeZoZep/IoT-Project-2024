import sys
import math

from geometry_msgs.msg import Point

from random import randint

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from project_main.sim_utils import spawn_sdf


import numpy as np
from project_main.math_utils import random_point_in_circle


WORLD_NAME = "iot_project_world"

NUMBER_OF_BALLOONS = 3
NUMBER_OF_SENSORS = 6



points = {}
edges = []

#-----------------------------------------------------------------------------------------------
# Launch file for the IoT Project. Launches all the nodes required to start the final solution
#-----------------------------------------------------------------------------------------------
def polar_to_euclidian(module,phase,centre):
    # Converte le coordinate polari in cartesiane
    x = module * math.cos(phase)
    y = module * math.sin(phase)
    ret=Point()
    ret.x=centre[0]+x
    ret.y=centre[1]+y
    return ret
def point_to_tuple(point):
    return (point.x,point.y)
def object_to_point(object):
    ret=Point()
    ret.x=object[0]
    ret.y=object[1]
    return ret
def create_node(integer,point):
    points.update({integer:point})
def create_link(point1,point2):
    edges.append((point1,point2))

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
    for index in range(NUMBER_OF_BALLOONS):
        punto=tuple()
        if NUMBER_OF_BALLOONS<4:
            if index==0:
                punto=(0.0, 0.0, 0.0)
            elif index==1:
                punto = (+32.0, 0.0, 0.0)
            elif index==2:
                punto = (+16.0,-27.71, 0.0)
        else:
            if index%3==0:
                punto=((index/3)*32.0, 0.0)
            elif index%3==1:
                punto=((((index-1)/3)*32.0)+16.0, 27.71)
            else:
                punto=((((index-2)/3)*32.0)+16.0, -27.71)
                   
        create_node(index*7,punto)

        #CREA GRAFO ESAGONO
        for i in range(6):
            new_point=point_to_tuple(polar_to_euclidian(12,(i*math.pi/3),punto))
            create_node(index*7+(i+1),new_point)
            create_link(index*7,index*7+(i+1))
            if i != 0: create_link(index*7+(i+1),index*7+i)
            if i == 5: create_link(index*7+(i+1),index*7+1)
        circles.append((punto,10))
        targets_to_spawn.append(spawn_sdf("resources/balloon/balloon.sdf", id = index, pos = punto))
        targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/Balloon_{index}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
            ]
        )
        )
        targets_to_spawn.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            f"/Balloon_{index}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ]
        )
        )

        targets_to_spawn.append(
            Node(
                package="project_main",
                executable="balloon_controller",
                namespace=f"Balloon_{index}",
                name=f"BalloonController{index}"
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
