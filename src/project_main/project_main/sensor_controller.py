import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import Data
from rosgraph_msgs.msg import Clock

from sim_utils import EventScheduler


WORLD_NAME = "iot_project_world"

class SensorController(Node):

    def __init__(self):
        super().__init__('sensor_controller')

        self.tx_topic = self.create_publisher(
            Data,
            'tx_data',
            10
        )

        self.id = self.declare_parameter('id', -1)

        self.generated_data = 0


        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )
        self.rate = 0.5 #average num of msg per second
        self.event_scheduler.schedule_event(np.random.exponential(1 / self.rate), self.simple_publish, False)

        #self.create_timer(1, self.simple_publish)


    def simple_publish(self):
        id = self.id.get_parameter_value().integer_value

        msg = Data()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.duration = 4 #one min
        msg.sensor_id = id
        msg.sqn = self.generate_data()
        msg.data = f"Sensor data: {id}_{msg.sqn}!"

        self.tx_topic.publish(msg)
        self.event_scheduler.schedule_event(np.random.exponential(1 / self.rate), self.simple_publish, False) 


    def generate_data(self):

        self.generated_data += 1
        return self.generated_data


def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor()

    sensor_controller = SensorController()
    executor.add_node(sensor_controller)

    executor.spin()

    sensor_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
