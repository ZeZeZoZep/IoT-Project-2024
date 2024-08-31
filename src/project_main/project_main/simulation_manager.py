import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import Data
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math_utils


NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

SENSORS_RANGE = 20

class SimulationManager(Node):

    def __init__(self):

        super().__init__('simulation_manager')


        self.sensor_positions = {}
        self.balloon_positions = {}
        

        for i in range(NUMBER_OF_SENSORS):
            
            self.create_subscription(
                Odometry,
                f'ActiveSensor_{i}/odometry',
                lambda odometry_msg, sensor_id = i: self.store_sensor_position(sensor_id, odometry_msg),
                10
                #self.store_sensor_position
            )
            
            self.create_subscription(
                Data,
                f'ActiveSensor_{i}/tx_data',
                lambda string_msg, sensor_id = i: self.forward_data_sb(sensor_id, string_msg),
                #self.forward_data,
                10
            )
            self.create_subscription(
                Odometry,
                f'ActiveSensor_{i}/odometry',
                lambda odometry_msg, sensor_id = i: self.store_sensor_position(sensor_id, odometry_msg),
                10
                #self.store_sensor_position
            )
            

        self.balloons_rx = {}
        for i in range(NUMBER_OF_BALLOONS):


            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id = i: self.store_balloon_position(balloon_id, odometry_msg),
                10
                #self.store_sensor_position
            )

            self.balloons_rx[i] = self.create_publisher(
                Data,
                f'Balloon_{i}/rx_data',
                10
            )
    
    def store_sensor_position(self, sensor_id, position : Odometry):
        
        self.sensor_positions[sensor_id] = position.pose.pose.position


    def store_balloon_position(self, balloon_id, position : Odometry):

        self.balloon_positions[balloon_id] = position.pose.pose.position

    def forward_data_sb(self, sensor_id, msg : Data):
        min=999999.99
        min_i=0
        for i in range(NUMBER_OF_BALLOONS):
            if sensor_id in self.sensor_positions and i in self.balloon_positions:
                distance=math_utils.point_distance(self.sensor_positions[sensor_id], self.balloon_positions[i])
                if distance < SENSORS_RANGE:
                    if distance < min:

                        min=distance
                        min_i=i

        self.balloons_rx[min_i].publish(msg)
        

        




def main():

    rclpy.init()

    simulationManager= SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulationManager)

    executor.spin()

    executor.shutdown()
    simulationManager.destroy_node()

    rclpy.shutdown()
    
