import os
import rclpy

from dotenv import load_dotenv
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import Data
from nav_msgs.msg import Odometry
import math_utils

load_dotenv()

number_of_balloons = int(os.getenv('NUMBER_OF_BALLOONS'))
number_of_sensors = int(os.getenv('NUMBER_OF_SENSORS'))

SENSORS_RANGE = 20

class SimulationManager(Node):

    def __init__(self):
        super().__init__('simulation_manager')

        self.sensor_positions = {}
        self.balloon_positions = {}        

        for i in range(number_of_sensors):
            
            self.create_subscription(
                Odometry,
                f'ActiveSensor_{i}/odometry',
                lambda odometry_msg, sensor_id = i: self.store_sensor_position(sensor_id, odometry_msg),
                10
            )
            
            self.create_subscription(
                Data,
                f'ActiveSensor_{i}/tx_data',
                lambda string_msg, sensor_id = i: self.forward_data_sb(sensor_id, string_msg),
                10
            )
            

        self.balloons_rx = {}
        for i in range(number_of_balloons):

            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id = i: self.store_balloon_position(balloon_id, odometry_msg),
                10
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

        for i in range(number_of_balloons):
            if sensor_id in self.sensor_positions and i in self.balloon_positions:
                distance=math_utils.point_distance(self.sensor_positions[sensor_id], self.balloon_positions[i])
                if distance < SENSORS_RANGE:
                        self.balloons_rx[i].publish(msg)
        
        

        




def main():

    rclpy.init()

    simulationManager= SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulationManager)

    executor.spin()

    executor.shutdown()
    simulationManager.destroy_node()

    rclpy.shutdown()
    
