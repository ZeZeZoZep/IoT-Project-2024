import os
import math
import numpy as np
from dotenv import load_dotenv

from geometry_msgs.msg import Point
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from project_main.sim_utils import spawn_sdf
from project_main.math_utils import random_point_in_circle

load_dotenv()

WORLD_NAME = "iot_project_world"

number_of_balloons = int(os.getenv('NUMBER_OF_BALLOONS'))
number_of_sensors = int(os.getenv('NUMBER_OF_SENSORS'))
number_of_lidar_sensors = int(os.getenv('NUMBER_OF_LIDAR_SENSORS'))


points = []

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
    ret.z=centre[2]
    return ret
def point_to_tuple(point):
    return (point.x,point.y,point.z)
def object_to_point(object):
    ret=Point()
    ret.x=object[0]
    ret.y=object[1]
    ret.z=object[2]
    return ret
def create_node(integer,point):
    points.append((integer,point))


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
    for index in range(number_of_balloons):
        punto=tuple()
        if number_of_balloons<4:
            if index==0:
                punto=(0.0, 0.0, 0.0)
            elif index==1:
                punto = (+32.0, 0.0, 0.0)
            elif index==2:
                punto = (+16.0,-27.71, 0.0)
        else:
            if index%3==0:
                punto=((index/3)*32.0, 0.0,0.0)
            elif index%3==1:
                punto=((((index-1)/3)*32.0)+16.0, 27.71,0.0)
            else:
                punto=((((index-2)/3)*32.0)+16.0, -27.71,0.0)                   

        #CREA GRAFO ESAGONO
        for i in range(6):
            new_point=point_to_tuple(polar_to_euclidian(12,(i*math.pi/3),punto))
            create_node(index*7+(i+1),new_point)
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
    node_position={}
    
    for i in range(number_of_sensors):
        spawn_point=None
        spawn_point_index=None
        if i<number_of_lidar_sensors:
            circle=np.random.randint(0, number_of_balloons)
            spawn_point=random_point_in_circle(circles[circle][0],circles[circle][1])
            targets_to_spawn.append(spawn_sdf("resources/active_sensor_lidar/active_sensor_lidar.sdf", id = i, pos = spawn_point))
            targets_to_spawn.append(
                Node(             
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    arguments=[
                    f"/ActiveSensor_{i}/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"
                    ]
                )
            )
 
        else:
            spawn_point_index=np.random.randint(0, len(points))
            print(points[spawn_point_index])
            spawn_point=points[spawn_point_index][1]
            node_position.update({i:points[spawn_point_index][0]})
            points.pop(spawn_point_index)
            targets_to_spawn.append(spawn_sdf("resources/active_sensor/active_sensor.sdf", id = i, pos = spawn_point))    

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

        if i<number_of_lidar_sensors:

            targets_to_spawn.append(
                Node(
                    package="project_main",
                    executable="sensor_controller",
                    namespace=f"ActiveSensor_{i}",
                    parameters=[
                        {'id': i, 'nls':number_of_lidar_sensors}
                    ]
                )
            )
        else:

            targets_to_spawn.append(
                Node(
                    package="project_main",
                    executable="sensor_controller",
                    namespace=f"ActiveSensor_{i}",
                    parameters=[
                        {'id': i, 'nls':number_of_lidar_sensors}
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
                f"{number_of_balloons}",
                f"{number_of_sensors}"
            ]
        )
    )

    targets_to_spawn.append(
        Node(
            package="project_main",
            executable="simulation_manager",
            arguments=[
                f"{number_of_balloons}",
                f"{number_of_sensors}"
            ]
        )
    )

    targets_to_spawn.append(
        Node(
            package="project_main",
            executable="fleet_coordinator",
            arguments=[
                f"{node_position}",
                f"{number_of_balloons}",
                f"{number_of_sensors}",
                f"{number_of_lidar_sensors}"
            ]
        )
    )

    return LaunchDescription(targets_to_spawn)
